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
        selected_paths.add(tuple(path))
        rows.append({
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
            "point_type": point_type,
            "selection_latency_ms": float(latency_s * 1000.0),
            "score": float(scores[route_idx]),
        })

    for row in rows:
        row["unique_routes_across_presets"] = len(selected_paths)
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
