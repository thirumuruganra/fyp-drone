"""
Module 5 - Evaluation Metrics
=============================
Metrics for comparing NAMOA* and baseline route sets.

Hypervolume and IGD use a reference front constructed from the union of all
candidate method outputs for one scenario, then filtered to non-dominated
points. The HV reference point is the scenario-wise nadir point padded by 10%.
All HV/IGD/spread values are computed in normalized objective space.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import time

import numpy as np

from module2_tier1_moastar import pareto_dominates, update_pareto_front
from module4_tier2_payload import PAYLOAD_PRESETS, TierTwoSelector


def ensure_finite(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError("Expected objective matrix with shape (n, 3).")
    if not np.isfinite(arr).all():
        raise ValueError("Objective matrix contains NaN or Inf.")
    return arr


def nondominated_routes(routes: list[tuple[list[float], list]]) -> list[tuple[list[float], list]]:
    front: list[tuple[list[float], list]] = []
    seen_costs: list[list[float]] = []
    for costs, path in routes:
        cost_vec = [float(x) for x in costs]
        if any(np.allclose(cost_vec, seen, atol=1e-9) for seen in seen_costs):
            continue
        seen_costs.append(cost_vec)
        update_pareto_front(front, cost_vec, list(path))
    return front


def nondominated_points(points: np.ndarray) -> np.ndarray:
    arr = ensure_finite(points)
    front: list[tuple[list[float], list]] = []
    for idx, p in enumerate(arr):
        update_pareto_front(front, p.tolist(), [idx])
    return np.array([g for g, _ in front], dtype=float)


def objective_matrix(routes: list[tuple[list[float], list]]) -> np.ndarray:
    return ensure_finite(np.array([g for g, _ in routes], dtype=float))


def normalization_bounds(reference_front: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref = ensure_finite(reference_front)
    ideal = ref.min(axis=0)
    nadir = ref.max(axis=0)
    span = np.maximum(nadir - ideal, 1e-12)
    return ideal, ideal + span


def normalize(points: np.ndarray, ideal: np.ndarray, nadir: np.ndarray) -> np.ndarray:
    arr = ensure_finite(points)
    span = np.maximum(nadir - ideal, 1e-12)
    return np.clip((arr - ideal) / span, 0.0, None)


def reference_point_for(normalized_reference_front: np.ndarray, pad: float = 0.10) -> np.ndarray:
    ref = ensure_finite(normalized_reference_front)
    return ref.max(axis=0) + pad


def hypervolume(points: np.ndarray, reference_point: np.ndarray) -> float:
    """
    Exact dominated hypervolume for minimization in 3D via recursive sweep.
    Points outside the reference box are ignored.
    """
    pts = ensure_finite(points)
    ref = np.asarray(reference_point, dtype=float)
    if ref.shape != (3,):
        raise ValueError("Reference point must have shape (3,).")
    pts = pts[np.all(pts <= ref, axis=1)]
    if len(pts) == 0:
        return 0.0
    pts = nondominated_points(pts)
    return float(_hv_recursive(pts, ref))


def _hv_recursive(points: np.ndarray, ref: np.ndarray) -> float:
    if points.size == 0:
        return 0.0
    dim = points.shape[1]
    if dim == 1:
        return max(0.0, float(ref[0] - points[:, 0].min()))

    xs = np.unique(points[:, 0])
    xs.sort()
    volume = 0.0
    for i, x in enumerate(xs):
        next_x = xs[i + 1] if i + 1 < len(xs) else ref[0]
        width = max(0.0, float(next_x - x))
        if width == 0.0:
            continue
        active = points[points[:, 0] <= x][:, 1:]
        volume += width * _hv_recursive(active, ref[1:])
    return volume


def igd(approx_points: np.ndarray, reference_front: np.ndarray) -> float:
    """Inverted Generational Distance: mean distance ref point -> nearest approx point."""
    approx = ensure_finite(approx_points)
    ref = ensure_finite(reference_front)
    if len(approx) == 0:
        return float("inf")
    distances = []
    for point in ref:
        distances.append(float(np.linalg.norm(approx - point, axis=1).min()))
    return float(np.mean(distances))


def spread_delta(points: np.ndarray, reference_front: np.ndarray | None = None) -> float:
    """
    NSGA-II-style spread metric on normalized points sorted by distance.
    Lower is better; 0 means evenly spaced. Returns 0 for fronts with <3 points.
    """
    pts = ensure_finite(points)
    if len(pts) < 3:
        return 0.0
    order = np.argsort(pts[:, 0])
    pts = pts[order]
    consecutive = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    mean_d = float(consecutive.mean())
    if mean_d <= 1e-12:
        return 0.0

    df = dl = 0.0
    if reference_front is not None and len(reference_front) >= 2:
        ref = ensure_finite(reference_front)
        ideal_idx = int(np.argmin(ref[:, 0]))
        risk_idx = int(np.argmin(ref[:, 2]))
        df = float(np.linalg.norm(pts[0] - ref[ideal_idx]))
        dl = float(np.linalg.norm(pts[-1] - ref[risk_idx]))

    numerator = df + dl + float(np.abs(consecutive - mean_d).sum())
    denominator = df + dl + (len(pts) - 1) * mean_d
    return float(numerator / denominator) if denominator > 0 else 0.0


def diversity_extent(points: np.ndarray) -> float:
    """Diagonal extent covered by a front in objective space. Higher means broader front."""
    pts = ensure_finite(points)
    if len(pts) < 2:
        return 0.0
    return float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))


def build_reference_front(method_routes: dict[str, list[tuple[list[float], list]]]) -> np.ndarray:
    all_routes: list[tuple[list[float], list]] = []
    for routes in method_routes.values():
        all_routes.extend(routes)
    if not all_routes:
        raise ValueError("Cannot build reference front from empty route sets.")
    return objective_matrix(nondominated_routes(all_routes))


def evaluate_method_set(
    method_routes: dict[str, list[tuple[list[float], list]]],
    runtimes: dict[str, float],
) -> tuple[list[dict], dict]:
    """Return per-method metric rows plus reference metadata."""
    reference_front = build_reference_front(method_routes)
    ideal, nadir = normalization_bounds(reference_front)
    ref_norm = normalize(reference_front, ideal, nadir)
    hv_ref = reference_point_for(ref_norm)

    rows = []
    for method, routes in method_routes.items():
        front = nondominated_routes(routes)
        costs = objective_matrix(front)
        norm = normalize(costs, ideal, nadir)
        rows.append({
            "method": method,
            "front_size": len(front),
            "runtime_s": float(runtimes.get(method, 0.0)),
            "hypervolume": hypervolume(norm, hv_ref),
            "igd": igd(norm, ref_norm),
            "spread_delta": spread_delta(norm, ref_norm),
            "diversity_extent": diversity_extent(norm),
        })

    metadata = {
        "reference_front_size": int(len(reference_front)),
        "ideal": ideal.tolist(),
        "nadir": nadir.tolist(),
        "hv_reference_point_normalized": hv_ref.tolist(),
        "notes": (
            "Reference front = non-dominated union of all method outputs for the scenario. "
            "HV/IGD/spread use normalized objective space; HV reference point = normalized "
            "scenario nadir padded by 10%."
        ),
    }
    return rows, metadata


def objective_ranks(costs: np.ndarray, selected_cost: list[float]) -> dict[str, int]:
    arr = ensure_finite(costs)
    selected = np.asarray(selected_cost, dtype=float)
    names = ("distance", "energy", "risk")
    ranks = {}
    for idx, name in enumerate(names):
        order = np.argsort(arr[:, idx])
        selected_matches = np.where(np.all(np.isclose(arr, selected, atol=1e-9), axis=1))[0]
        if len(selected_matches) == 0:
            ranks[name] = int(np.searchsorted(np.sort(arr[:, idx]), selected[idx]) + 1)
            continue
        selected_idx = int(selected_matches[0])
        ranks[name] = int(np.where(order == selected_idx)[0][0] + 1)
    return ranks


def payload_alignment_score(weights: dict[str, float], ranks: dict[str, int], front_size: int) -> float:
    """
    Rank-based payload-alignment score.

    A = Σ_k w_k * (1 - (rank_k - 1)/(K - 1)). For K=1 every rank is best.
    """
    if front_size <= 1:
        return 1.0
    return float(sum(
        weights[name] * (1.0 - (ranks[name] - 1) / (front_size - 1))
        for name in ("distance", "energy", "risk")
    ))


def _normalized_costs_for_front(pareto_front: list[tuple[list[float], list]]) -> np.ndarray:
    costs = objective_matrix(pareto_front)
    ideal = costs.min(axis=0)
    span = np.maximum(costs.max(axis=0) - ideal, 1e-12)
    return (costs - ideal) / span


def knee_point_index(pareto_front: list[tuple[list[float], list]]) -> int:
    """Knee = point farthest from line through min-distance and min-risk extremes."""
    norm = _normalized_costs_for_front(pareto_front)
    if len(norm) <= 2:
        return 0
    a = norm[int(np.argmin(norm[:, 0]))]
    b = norm[int(np.argmin(norm[:, 2]))]
    ab = b - a
    denom = float(np.linalg.norm(ab))
    if denom <= 1e-12:
        centroid = norm.mean(axis=0)
        return int(np.argmax(np.linalg.norm(norm - centroid, axis=1)))
    distances = np.linalg.norm(np.cross(norm - a, ab), axis=1) / denom
    return int(np.argmax(distances))


def crowding_distance_index(pareto_front: list[tuple[list[float], list]]) -> int:
    """Crowding picker = point with largest NSGA-II crowding distance."""
    norm = _normalized_costs_for_front(pareto_front)
    n = len(norm)
    if n <= 2:
        return 0
    distances = np.zeros(n, dtype=float)
    for obj_idx in range(3):
        order = np.argsort(norm[:, obj_idx])
        distances[order[0]] = np.inf
        distances[order[-1]] = np.inf
        for pos in range(1, n - 1):
            distances[order[pos]] += norm[order[pos + 1], obj_idx] - norm[order[pos - 1], obj_idx]
    return int(np.argmax(distances))


def fixed_weight_index(pareto_front: list[tuple[list[float], list]]) -> int:
    """Fixed Chokkalingam-style picker = equal weights over normalized utility."""
    norm = _normalized_costs_for_front(pareto_front)
    scores = (1.0 - norm).mean(axis=1)
    return int(np.argmax(scores))


def _rows_for_picker(
    pareto_front: list[tuple[list[float], list]],
    weights: dict[str, float],
    picker: str,
    route_idx: int,
) -> dict:
    costs = objective_matrix(pareto_front)
    g, path = pareto_front[route_idx]
    ranks = objective_ranks(costs, g)
    return {
        f"{picker}_index": int(route_idx),
        f"{picker}_rank_distance": ranks["distance"],
        f"{picker}_rank_energy": ranks["energy"],
        f"{picker}_rank_risk": ranks["risk"],
        f"{picker}_alignment": payload_alignment_score(weights, ranks, len(pareto_front)),
        f"{picker}_point_type": TierTwoSelector(pareto_front).classify_point(route_idx),
    }


def tier2_evidence(pareto_front: list[tuple[list[float], list]]) -> list[dict]:
    """Payload-alignment evidence, no fake accuracy labels."""
    selector = TierTwoSelector(pareto_front)
    costs = selector.costs
    rows = []
    selected_paths = set()

    for name, tags in PAYLOAD_PRESETS.items():
        t0 = time.perf_counter()
        selected, wr, wd, we, scores = selector.select(tags["F"], tags["W"], tags["U"], tags["P"])
        latency_s = time.perf_counter() - t0
        g, path = selected
        route_idx = selector.selected_index(selected)
        ranks = objective_ranks(costs, g)
        weights = {"risk": wr, "distance": wd, "energy": we}
        dominant = max(weights, key=weights.get)
        point_type = selector.classify_point(route_idx)
        alignment = payload_alignment_score(weights, ranks, len(pareto_front))
        shortest_idx = int(np.argmin(costs[:, 0]))
        knee_idx = knee_point_index(pareto_front)
        crowd_idx = crowding_distance_index(pareto_front)
        fixed_idx = fixed_weight_index(pareto_front)
        selected_paths.add(tuple(path))
        row = {
            "preset": name,
            "F": tags["F"],
            "W": tags["W"],
            "U": tags["U"],
            "P": tags["P"],
            "w_risk": float(wr),
            "w_distance": float(wd),
            "w_energy": float(we),
            "dominant_weight": dominant,
            "selected_index": int(route_idx),
            "distance": float(g[0]),
            "energy": float(g[1]),
            "risk": float(g[2]),
            "rank_distance": ranks["distance"],
            "rank_energy": ranks["energy"],
            "rank_risk": ranks["risk"],
            "dominant_objective_rank": ranks[dominant],
            "payload_alignment": alignment,
            "point_type": point_type,
            "selection_latency_ms": float(latency_s * 1000.0),
            "score": float(scores[route_idx]),
        }
        row.update(_rows_for_picker(pareto_front, weights, "astar_shortest", shortest_idx))
        row.update(_rows_for_picker(pareto_front, weights, "knee", knee_idx))
        row.update(_rows_for_picker(pareto_front, weights, "crowding", crowd_idx))
        row.update(_rows_for_picker(pareto_front, weights, "fixed_equal", fixed_idx))
        rows.append(row)

    for row in rows:
        row["unique_routes_across_presets"] = len(selected_paths)
    return rows


def tier2_sweep(pareto_front: list[tuple[list[float], list]]) -> list[dict]:
    """Evaluate all 225 F/W/U/P tag combinations on one front."""
    selector = TierTwoSelector(pareto_front)
    costs = selector.costs
    rows = []
    for F in range(1, 6):
        for W in range(1, 4):
            for U in range(1, 4):
                for P in range(1, 6):
                    t0 = time.perf_counter()
                    selected, wr, wd, we, scores = selector.select(F, W, U, P)
                    latency_s = time.perf_counter() - t0
                    g, path = selected
                    idx = selector.selected_index(selected)
                    ranks = objective_ranks(costs, g)
                    weights = {"risk": wr, "distance": wd, "energy": we}
                    rows.append({
                        "F": F,
                        "W": W,
                        "U": U,
                        "P": P,
                        "selected_index": int(idx),
                        "distance": float(g[0]),
                        "energy": float(g[1]),
                        "risk": float(g[2]),
                        "rank_distance": ranks["distance"],
                        "rank_energy": ranks["energy"],
                        "rank_risk": ranks["risk"],
                        "payload_alignment": payload_alignment_score(weights, ranks, len(pareto_front)),
                        "point_type": selector.classify_point(idx),
                        "selection_latency_ms": float(latency_s * 1000.0),
                        "score": float(scores[idx]),
                        "path": " ".join(str(node) for node in path),
                    })
    return rows


def monotonicity_violations(sweep_rows: list[dict]) -> list[dict]:
    """Find simple adjacent monotonicity violations across the full tag sweep."""
    by_tags = {(r["F"], r["W"], r["U"], r["P"]): r for r in sweep_rows}
    violations = []
    for row in sweep_rows:
        F, W, U, P = row["F"], row["W"], row["U"], row["P"]
        checks = [
            ("F", F < 5, (F + 1, W, U, P), "risk"),
            ("W", W < 3, (F, W + 1, U, P), "energy"),
            ("U", U < 3, (F, W, U + 1, P), "distance"),
        ]
        for field, enabled, next_key, objective in checks:
            if not enabled:
                continue
            nxt = by_tags[next_key]
            if nxt[objective] > row[objective] + 1e-9:
                violations.append({
                    "field": field,
                    "from_tags": f"F={F},W={W},U={U},P={P}",
                    "to_tags": f"F={next_key[0]},W={next_key[1]},U={next_key[2]},P={next_key[3]}",
                    "objective": objective,
                    "from_value": row[objective],
                    "to_value": nxt[objective],
                })
    return violations


def risk_component_shares(
    G,
    route_rows: list[tuple[str, tuple[list[float], list]]],
) -> list[dict]:
    """
    Mean risk-component shares for routes.

    Current checked-in graph uses a synthetic scalar risk. If edge-level
    risk_components exist, use them; otherwise emit synthetic_risk=1.0.
    """
    rows = []
    for label, (_cost, path) in route_rows:
        totals: dict[str, float] = {}
        total = 0.0
        for u, v in zip(path[:-1], path[1:]):
            edge = G[u][v]
            exposure = float(edge.get("risk", 0.0))
            components = edge.get("risk_components")
            if isinstance(components, dict) and components:
                comp_sum = max(sum(float(x) for x in components.values()), 1e-12)
                for name, value in components.items():
                    contribution = exposure * float(value) / comp_sum
                    totals[name] = totals.get(name, 0.0) + contribution
                    total += contribution
            else:
                totals["synthetic_risk"] = totals.get("synthetic_risk", 0.0) + exposure
                total += exposure
        if total <= 1e-12:
            totals["synthetic_risk"] = 1.0
            total = 1.0
        for component, value in sorted(totals.items()):
            rows.append({
                "route": label,
                "component": component,
                "share": float(value / total),
            })
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as fh:
        json.dump(data, fh, indent=2)
