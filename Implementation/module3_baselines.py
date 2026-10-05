"""
Module 3 - Baseline Planners
============================
FYP B28: Payload-Aware Two-Tier UAV Route Planning

Baselines for Phase I evaluation:
  * Single-objective A*/Dijkstra variants for distance, energy, and risk.
  * A compact MOPSO-style approximate Pareto search over valid graph paths.

Single-objective A* returns one optimum for one scalar objective. MOPSO returns
an approximate Pareto archive. These are intentionally kept separate from the
NAMOA* Tier 1 engine, which returns a deterministic Pareto set.
"""

from __future__ import annotations

from dataclasses import dataclass
import random
import time
from typing import Iterable

import networkx as nx
import numpy as np

from module2_tier1_moastar import pareto_dominates, update_pareto_front


OBJECTIVES = ("distance", "energy", "risk")


@dataclass
class BaselineResult:
    method: str
    routes: list[tuple[list[float], list]]
    runtime_s: float
    metadata: dict


def route_cost(G: nx.DiGraph, path: list) -> list[float]:
    """Return [distance, energy, risk] cumulative cost for a valid path."""
    if len(path) < 2:
        return [0.0, 0.0, 0.0]

    total = np.zeros(3, dtype=float)
    for u, v in zip(path[:-1], path[1:]):
        if not G.has_edge(u, v):
            raise ValueError(f"Invalid route edge: {u!r} -> {v!r}")
        edge = G[u][v]
        total += np.array([edge["distance"], edge["energy"], edge["risk"]], dtype=float)
    return [round(float(x), 6) for x in total]


def is_valid_route(G: nx.DiGraph, path: Iterable, source, target) -> bool:
    """Check source, target, and edge continuity."""
    path = list(path)
    if not path or path[0] != source or path[-1] != target:
        return False
    return all(G.has_edge(u, v) for u, v in zip(path[:-1], path[1:]))


def single_objective_astar(
    G: nx.DiGraph,
    source,
    target,
    objective: str = "distance",
) -> BaselineResult:
    """
    Single-objective baseline.

    For distance, an admissible Euclidean lower-bound heuristic is used. Energy
    and risk use zero heuristic, making A* equivalent to Dijkstra while keeping
    the same scalar-search interface.
    """
    if objective not in OBJECTIVES:
        raise ValueError(f"Unknown objective {objective!r}; expected one of {OBJECTIVES}.")

    def weight(u, v, attrs):
        return float(attrs[objective])

    def heuristic(u, v):
        if objective != "distance":
            return 0.0
        try:
            dx = (G.nodes[u]["x"] - G.nodes[v]["x"]) * 111.0
            dy = (G.nodes[u]["y"] - G.nodes[v]["y"]) * 111.0
            return float(np.hypot(dx, dy))
        except (KeyError, TypeError):
            return 0.0

    t0 = time.perf_counter()
    path = nx.astar_path(G, source, target, heuristic=heuristic, weight=weight)
    runtime_s = time.perf_counter() - t0
    return BaselineResult(
        method=f"astar_{objective}",
        routes=[(route_cost(G, path), path)],
        runtime_s=runtime_s,
        metadata={"objective": objective},
    )


def _archive_add(archive: list[tuple[list[float], list]], item: tuple[list[float], list], limit: int):
    """Add path to non-dominated archive, dedupe by node sequence, trim by diversity."""
    g_new, path_new = item
    if any(tuple(path_new) == tuple(path) for _, path in archive):
        return
    if any(np.allclose(g_new, g, atol=1e-9) for g, _ in archive):
        return
    update_pareto_front(archive, g_new, path_new)
    if len(archive) <= limit:
        return

    costs = np.array([g for g, _ in archive], dtype=float)
    mins = costs.min(axis=0)
    spans = np.maximum(costs.max(axis=0) - mins, 1e-12)
    norm = (costs - mins) / spans
    centroid = norm.mean(axis=0)
    keep_scores = np.linalg.norm(norm - centroid, axis=1)
    keep_idx = np.argsort(keep_scores)[-limit:]
    archive[:] = [archive[int(i)] for i in keep_idx]


def _random_walk_path(
    G: nx.DiGraph,
    source,
    target,
    rng: random.Random,
    bias: np.ndarray | None = None,
    max_steps: int | None = None,
) -> list | None:
    """
    Construct a valid simple path by stochastic next-hop choice.

    bias is an objective weight vector used to prefer lower scalar edge cost.
    The function falls back to None if the walk reaches a dead end.
    """
    max_steps = max_steps or max(12, int(G.number_of_nodes() * 1.5))
    path = [source]
    visited = {source}
    current = source

    for _ in range(max_steps):
        if current == target:
            return path

        candidates = [v for v in G.successors(current) if v not in visited]
        if not candidates:
            return None

        scored = []
        for v in candidates:
            edge_cost = np.array([
                G[current][v]["distance"],
                G[current][v]["energy"],
                G[current][v]["risk"],
            ], dtype=float)
            scalar = float(edge_cost @ bias) if bias is not None else float(edge_cost.sum())
            try:
                dx = (G.nodes[v]["x"] - G.nodes[target]["x"]) * 111.0
                dy = (G.nodes[v]["y"] - G.nodes[target]["y"]) * 111.0
                scalar += 0.25 * float(np.hypot(dx, dy))
            except (KeyError, TypeError):
                pass
            scored.append((v, scalar))

        scores = np.array([s for _, s in scored], dtype=float)
        scores = scores - scores.min()
        temperature = max(float(scores.std()), 1e-6)
        probs = np.exp(-scores / temperature)
        probs = probs / probs.sum()
        pick = rng.choices([v for v, _ in scored], weights=probs.tolist(), k=1)[0]
        path.append(pick)
        visited.add(pick)
        current = pick

    return path if current == target else None


def _repair_path(G: nx.DiGraph, prefix: list, target, objective_weights: np.ndarray) -> list | None:
    """Append a shortest suffix from prefix end to target, avoiding existing nodes when possible."""
    if not prefix:
        return None
    current = prefix[-1]
    blocked = set(prefix[:-1])
    H = G.copy()
    H.remove_nodes_from(blocked)

    def weight(u, v, attrs):
        return float(np.array([attrs["distance"], attrs["energy"], attrs["risk"]]) @ objective_weights)

    try:
        suffix = nx.shortest_path(H, current, target, weight=weight)
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return None
    return prefix[:-1] + suffix


def _guided_particle_path(
    G: nx.DiGraph,
    source,
    target,
    rng: random.Random,
    pbest: list | None,
    leader: list | None,
    weights: np.ndarray,
) -> list | None:
    """Discrete PSO move: preserve a prefix, then repair with weighted shortest suffix."""
    guides = [p for p in (pbest, leader) if p]
    if guides and rng.random() < 0.70:
        guide = rng.choice(guides)
        cut = rng.randint(1, max(1, len(guide) - 1))
        repaired = _repair_path(G, guide[:cut], target, weights)
        if repaired and is_valid_route(G, repaired, source, target):
            return repaired
    return _random_walk_path(G, source, target, rng, bias=weights)


def mopso_baseline(
    G: nx.DiGraph,
    source,
    target,
    seed: int = 42,
    swarm_size: int = 30,
    iterations: int = 40,
    archive_size: int = 60,
) -> BaselineResult:
    """
    Compact MOPSO-style baseline for graph routes.

    Particles are valid source-target paths. Each iteration samples a random
    objective preference vector, follows personal/global guide paths when useful,
    repairs to the target with a weighted shortest suffix, and stores only
    non-dominated valid paths in the archive.
    """
    rng = random.Random(seed)
    t0 = time.perf_counter()

    objective_weights = [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 1.0, 1.0]) / 3.0,
    ]
    particles: list[list] = []

    for weights in objective_weights:
        try:
            path = _repair_path(G, [source], target, weights)
            if path and is_valid_route(G, path, source, target):
                particles.append(path)
        except nx.NetworkXException:
            pass

    attempts = 0
    while len(particles) < swarm_size and attempts < swarm_size * 20:
        attempts += 1
        weights = np.array([rng.random(), rng.random(), rng.random()], dtype=float)
        weights = weights / weights.sum()
        path = _random_walk_path(G, source, target, rng, bias=weights)
        if path and is_valid_route(G, path, source, target):
            particles.append(path)

    if not particles:
        fallback = nx.shortest_path(G, source, target, weight="distance")
        particles = [fallback]

    pbest = list(particles)
    pbest_costs = [route_cost(G, p) for p in pbest]
    archive: list[tuple[list[float], list]] = []
    for path in particles:
        _archive_add(archive, (route_cost(G, path), path), archive_size)

    for _ in range(iterations):
        next_particles = []
        for idx, path in enumerate(particles):
            leader = rng.choice(archive)[1] if archive else path
            weights = np.array([rng.random(), rng.random(), rng.random()], dtype=float)
            weights = weights / weights.sum()
            candidate = _guided_particle_path(
                G, source, target, rng, pbest[idx % len(pbest)], leader, weights
            )
            if not candidate or not is_valid_route(G, candidate, source, target):
                candidate = path

            cand_cost = route_cost(G, candidate)
            old_cost = pbest_costs[idx % len(pbest)]
            if pareto_dominates(cand_cost, old_cost) or (
                not pareto_dominates(old_cost, cand_cost) and rng.random() < 0.35
            ):
                pbest[idx % len(pbest)] = candidate
                pbest_costs[idx % len(pbest)] = cand_cost

            _archive_add(archive, (cand_cost, candidate), archive_size)
            next_particles.append(candidate)
        particles = next_particles

    runtime_s = time.perf_counter() - t0
    return BaselineResult(
        method="mopso",
        routes=archive,
        runtime_s=runtime_s,
        metadata={
            "seed": seed,
            "swarm_size": swarm_size,
            "iterations": iterations,
            "archive_size": archive_size,
        },
    )
