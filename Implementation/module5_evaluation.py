"""
Module 5 - Evaluation Metrics
=============================
Metrics for comparing NAMOA* and baseline route sets.

Hypervolume and IGD use a reference front constructed from the union of all
candidate method outputs for one scenario, then filtered to non-dominated
points. The HV reference point is the scenario-wise nadir point padded by 10%.
All HV/IGD/spread values are computed in normalized objective space.

Extra indicators: GD, IGD+ (Pareto-compliant IGD), additive epsilon I_eps+,
Zitzler set coverage C(A, B), HV ratio and the share of the reference front a
method found.  Stochastic methods are scored per run against the same
reference (evaluate_runs / summarize_runs -> mean +- std).
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import time

import numpy as np

from module2_tier1_moastar import pareto_dominates, update_pareto_front, weakly_dominates
from module3_baselines import route_cost
from module4_tier2_payload import PAYLOAD_PRESETS, TierTwoSelector


def ensure_finite(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError("Expected objective matrix with shape (n, 3).")
    if not np.isfinite(arr).all():
        raise ValueError("Objective matrix contains NaN or Inf.")
    return arr


def nondominated_routes(routes: list[tuple[list[float], list]],
                        tol: float = 1e-9) -> list[tuple[list[float], list]]:
    """
    Non-dominated, de-duplicated routes.  Dominance is tested within `tol`, so a
    route that ties another up to float noise (costs summed in another order,
    e.g. MOPSO re-costing vs NAMOA labels) cannot survive next to a route that
    beats it in the other objectives.
    """
    front: list[tuple[list[float], list]] = []
    for costs, path in routes:
        cost_vec = [float(x) for x in costs]
        if any(weakly_dominates(g, cost_vec, tol) for g, _ in front):
            continue                                   # dominated or duplicate
        front = [(g, p) for g, p in front if not weakly_dominates(cost_vec, g, tol)]
        front.append((cost_vec, list(path)))
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


def gd(approx_points: np.ndarray, reference_front: np.ndarray) -> float:
    """Generational Distance: mean distance approx point -> nearest reference point."""
    approx = ensure_finite(approx_points)
    ref = ensure_finite(reference_front)
    if len(approx) == 0:
        return float("inf")
    return float(np.mean([np.linalg.norm(ref - p, axis=1).min() for p in approx]))


def igd_plus(approx_points: np.ndarray, reference_front: np.ndarray) -> float:
    """
    IGD+ (Ishibuchi et al. 2015): like IGD, but only the part of a that is worse
    than r counts, d+(r, a) = ||max(a - r, 0)||.  Weakly Pareto compliant.
    """
    approx = ensure_finite(approx_points)
    ref = ensure_finite(reference_front)
    if len(approx) == 0:
        return float("inf")
    return float(np.mean([
        np.linalg.norm(np.maximum(approx - r, 0.0), axis=1).min() for r in ref
    ]))


def additive_epsilon(approx_points: np.ndarray, reference_front: np.ndarray) -> float:
    """
    Additive epsilon indicator I_eps+(A, R): smallest shift eps so that every
    reference point is weakly dominated by some a - eps.  0 = A covers R.
    """
    approx = ensure_finite(approx_points)
    ref = ensure_finite(reference_front)
    if len(approx) == 0:
        return float("inf")
    return float(max((approx - r).max(axis=1).min() for r in ref))


def set_coverage(a_points: np.ndarray, b_points: np.ndarray) -> float:
    """Zitzler C(A, B): share of B weakly dominated by at least one point of A."""
    a = ensure_finite(a_points)
    b = ensure_finite(b_points)
    if len(b) == 0:
        return 0.0
    covered = sum(1 for q in b if np.any(np.all(a <= q + 1e-12, axis=1)))
    return float(covered / len(b))


def reference_share(points: np.ndarray, reference_front: np.ndarray, atol: float = 1e-9) -> float:
    """Share of reference-front points a method found exactly."""
    pts = ensure_finite(points)
    ref = ensure_finite(reference_front)
    found = sum(1 for r in ref if np.any(np.all(np.isclose(pts, r, atol=atol), axis=1)))
    return float(found / len(ref))


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


def reference_context(method_routes: dict[str, list[tuple[list[float], list]]]) -> dict:
    """Reference front, normalization bounds and HV reference point for one scenario."""
    reference_front = build_reference_front(method_routes)
    ideal, nadir = normalization_bounds(reference_front)
    ref_norm = normalize(reference_front, ideal, nadir)
    hv_ref = reference_point_for(ref_norm)
    return {
        "reference_front": reference_front,
        "ideal": ideal,
        "nadir": nadir,
        "ref_norm": ref_norm,
        "hv_ref": hv_ref,
        "hv_reference_front": hypervolume(ref_norm, hv_ref),
    }


def metric_row(routes: list[tuple[list[float], list]], ctx: dict) -> dict:
    """Every front-quality indicator of one route set against a reference context."""
    front = nondominated_routes(routes)
    norm = normalize(objective_matrix(front), ctx["ideal"], ctx["nadir"])
    ref_norm = ctx["ref_norm"]
    hv = hypervolume(norm, ctx["hv_ref"])
    return {
        "front_size": len(front),
        "hypervolume": hv,
        "hv_ratio": hv / ctx["hv_reference_front"] if ctx["hv_reference_front"] > 0 else 0.0,
        "igd": igd(norm, ref_norm),
        "igd_plus": igd_plus(norm, ref_norm),
        "gd": gd(norm, ref_norm),
        "epsilon_add": additive_epsilon(norm, ref_norm),
        "reference_share": reference_share(norm, ref_norm),
        "spread_delta": spread_delta(norm, ref_norm),
        "diversity_extent": diversity_extent(norm),
    }


def evaluate_method_set(
    method_routes: dict[str, list[tuple[list[float], list]]],
    runtimes: dict[str, float],
) -> tuple[list[dict], dict]:
    """Return per-method metric rows plus reference metadata."""
    ctx = reference_context(method_routes)

    rows = []
    for method, routes in method_routes.items():
        row = {"method": method, "runtime_s": float(runtimes.get(method, 0.0))}
        row.update(metric_row(routes, ctx))
        rows.append(row)

    metadata = {
        "reference_front_size": int(len(ctx["reference_front"])),
        "ideal": ctx["ideal"].tolist(),
        "nadir": ctx["nadir"].tolist(),
        "hv_reference_point_normalized": ctx["hv_ref"].tolist(),
        "hv_reference_front": ctx["hv_reference_front"],
        "notes": (
            "Reference front = non-dominated union of all method outputs for the scenario. "
            "HV/IGD/IGD+/GD/eps/spread use normalized objective space; HV reference point = "
            "normalized scenario nadir padded by 10%."
        ),
    }
    return rows, metadata


def evaluate_runs(
    method_routes: dict[str, list[tuple[list[float], list]]],
    run_groups: dict[str, list[list[tuple[list[float], list]]]],
    run_runtimes: dict[str, list[float]] | None = None,
) -> list[dict]:
    """
    Score every run of a stochastic method on its own, against the same reference
    as evaluate_method_set(method_routes); method_routes must already hold the
    pooled runs so the reference covers them.
    """
    ctx = reference_context(method_routes)
    rows = []
    for method, runs in run_groups.items():
        times = (run_runtimes or {}).get(method, [])
        for idx, routes in enumerate(runs):
            if not routes:
                continue
            row = {"method": method, "run": idx,
                   "runtime_s": float(times[idx]) if idx < len(times) else 0.0}
            row.update(metric_row(routes, ctx))
            rows.append(row)
    return rows


RUN_SUMMARY_METRICS = ("front_size", "runtime_s", "hypervolume", "hv_ratio", "igd", "igd_plus",
                       "gd", "epsilon_add", "reference_share", "spread_delta")


def summarize_runs(run_rows: list[dict], group_keys: tuple[str, ...] = ("method",)) -> list[dict]:
    """Mean, std (ddof=1), min, max of every run metric per group."""
    groups: dict[tuple, list[dict]] = {}
    for row in run_rows:
        groups.setdefault(tuple(row.get(k) for k in group_keys), []).append(row)
    out = []
    for key, rows in groups.items():
        summary = dict(zip(group_keys, key))
        summary["runs"] = len(rows)
        for metric in RUN_SUMMARY_METRICS:
            vals = np.array([float(r[metric]) for r in rows if metric in r], dtype=float)
            if len(vals) == 0:
                continue
            summary[f"{metric}_mean"] = float(vals.mean())
            summary[f"{metric}_std"] = float(vals.std(ddof=1)) if len(vals) > 1 else 0.0
            summary[f"{metric}_min"] = float(vals.min())
            summary[f"{metric}_max"] = float(vals.max())
        out.append(summary)
    return out


def coverage_rows(method_routes: dict[str, list[tuple[list[float], list]]],
                  methods: tuple[str, ...] | None = None) -> list[dict]:
    """Pairwise Zitzler coverage C(A, B) for the given methods (default: all)."""
    names = list(methods) if methods else list(method_routes)
    fronts = {m: objective_matrix(nondominated_routes(method_routes[m]))
              for m in names if method_routes.get(m)}
    return [
        {"method_a": a, "method_b": b, "coverage": set_coverage(fronts[a], fronts[b])}
        for a in fronts for b in fronts if a != b
    ]


def front_robustness(
    G_eval,
    routes: list[tuple[list[float], list]],
    eval_front: list[tuple[list[float], list]],
) -> dict:
    """
    How a front built under one weather holds up under another.

    routes are re-costed on G_eval (same topology, other edge costs) and compared
    with eval_front, the front built on G_eval.  Returns the share of re-costed
    routes still non-dominated, the HV they keep (ratio, eval-front bounds) and
    the additive epsilon gap.
    """
    recosted = []
    for _g, path in routes:
        try:
            recosted.append((route_cost(G_eval, list(path)), list(path)))
        except ValueError:
            continue                          # an edge missing in G_eval: route infeasible
    if not recosted or not eval_front:
        return {"routes": len(routes), "feasible": len(recosted), "still_nondominated": 0.0,
                "hv_kept": 0.0, "epsilon_add": float("inf")}
    eval_costs = objective_matrix(eval_front)
    rec_costs = objective_matrix(recosted)
    nondom = sum(
        1 for c in rec_costs
        if not any(pareto_dominates(e.tolist(), c.tolist()) for e in eval_costs)
    )
    ideal, nadir = normalization_bounds(eval_costs)
    eval_norm = normalize(eval_costs, ideal, nadir)
    rec_norm = normalize(rec_costs, ideal, nadir)
    hv_ref = reference_point_for(eval_norm)
    hv_eval = hypervolume(eval_norm, hv_ref)
    return {
        "routes": len(routes),
        "feasible": len(recosted),
        "still_nondominated": float(nondom / len(recosted)),
        "hv_kept": float(hypervolume(rec_norm, hv_ref) / hv_eval) if hv_eval > 0 else 0.0,
        "epsilon_add": additive_epsilon(rec_norm, eval_norm),
    }


def tier2_vs_weighted(
    pareto_front: list[tuple[list[float], list]],
    weighted_routes: dict[str, list[tuple[list[float], list]]],
) -> list[dict]:
    """
    Tier 2 pick vs a weighted-sum A* re-search with the same preset weights.

    weighted_routes maps preset name -> that preset's weighted-sum A* routes.
    The gap is measured in the front's own normalized objective space.
    """
    selector = TierTwoSelector(pareto_front)
    costs = selector.costs
    ideal, nadir = normalization_bounds(costs)
    rows = []
    for name, tags in PAYLOAD_PRESETS.items():
        routes = weighted_routes.get(name)
        if not routes:
            continue
        selected, *_ = selector.select(tags["F"], tags["W"], tags["U"], tags["P"])
        w_cost, w_path = routes[0]
        t2_norm = normalize(np.array([selected[0]]), ideal, nadir)[0]
        w_norm = normalize(np.array([w_cost]), ideal, nadir)[0]
        rows.append({
            "preset": name,
            "same_path": list(selected[1]) == list(w_path),
            "objective_gap": float(np.linalg.norm(t2_norm - w_norm)),
            "weighted_on_front": bool(np.any(np.all(np.isclose(costs, w_cost, atol=1e-6), axis=1))),
            "weighted_dominated_by_front": any(pareto_dominates(c.tolist(), list(w_cost)) for c in costs),
        })
    return rows


def amortization_rows(t_tier1: float, t_tier2: float, t_weighted: float, t_mopso: float,
                      n_max: int = 100) -> list[dict]:
    """
    Cumulative planning time for n missions on one origin-destination pair:
    two-tier = one Tier 1 front + n Tier 2 picks; weighted-sum A* and MOPSO
    re-search once per mission.
    """
    return [
        {"missions": n,
         "two_tier_s": t_tier1 + n * t_tier2,
         "weighted_astar_s": n * t_weighted,
         "mopso_s": n * t_mopso}
        for n in range(1, n_max + 1)
    ]


def area_profile(G) -> dict:
    """Descriptive statistics of a lattice graph (real-area characterization)."""
    nodes = list(G.nodes(data=True))
    edges = [d for *_, d in G.edges(data=True)]

    def node_vals(key):
        return np.array([float(d.get(key, 0.0)) for _, d in nodes], dtype=float)

    def edge_vals(key):
        return np.array([float(d.get(key, 0.0)) for d in edges], dtype=float)

    def stat(arr, fn):
        return float(fn(arr)) if len(arr) else 0.0

    info = G.graph
    return {
        "nodes": G.number_of_nodes(),
        "edges": G.number_of_edges(),
        "n_buildings": int(info.get("n_buildings", 0)),
        "n_restricted_zones": int(info.get("n_restricted_zones", 0)),
        "n_water_zones": int(info.get("n_water_zones", 0)),
        "blocked_nodes": int(info.get("blocked_nodes", 0)),
        "nfz_nodes": int(info.get("nfz_nodes", 0)),
        "nfz_edges": int(info.get("nfz_edges", 0)),
        "elevation_provider": str(info.get("elevation_provider", "")),
        "ground_min_m": stat(node_vals("ground_elev_m"), np.min),
        "ground_max_m": stat(node_vals("ground_elev_m"), np.max),
        "bldg_density_mean": stat(node_vals("bldg_density"), np.mean),
        "bldg_h_max_m": stat(node_vals("bldg_h_m"), np.max),
        "agl_mean_m": stat(node_vals("agl_m"), np.mean),
        "water_edge_mean": stat(edge_vals("water"), np.mean),
        "population_edge_mean": stat(edge_vals("population"), np.mean),
        "rho_mean": stat(edge_vals("rho"), np.mean),
        "rho_max": stat(edge_vals("rho"), np.max),
    }


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


def monotonicity_violations(sweep_rows: list[dict],
                            group_keys: tuple[str, ...] = ("scenario",)) -> list[dict]:
    """
    Find simple adjacent monotonicity violations across the full tag sweep.

    Rows are compared only within one group (default: one scenario front), so a
    sweep that mixes several fronts gives no false violations.  Missing group
    keys count as one group.
    """
    def group_of(row):
        return tuple(row.get(k) for k in group_keys)

    by_tags = {group_of(r) + (r["F"], r["W"], r["U"], r["P"]): r for r in sweep_rows}
    violations = []
    for row in sweep_rows:
        F, W, U, P = row["F"], row["W"], row["U"], row["P"]
        group = group_of(row)
        checks = [
            ("F", F < 5, (F + 1, W, U, P), "risk"),
            ("W", W < 3, (F, W + 1, U, P), "energy"),
            ("U", U < 3, (F, W, U + 1, P), "distance"),
        ]
        for field, enabled, next_key, objective in checks:
            if not enabled:
                continue
            nxt = by_tags.get(group + next_key)
            if nxt is None:
                continue
            if nxt[objective] > row[objective] + 1e-9:
                violations.append({
                    **dict(zip(group_keys, group)),
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
    fieldnames = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, restval="")
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as fh:
        json.dump(data, fh, indent=2)
