"""
Module 4 — Payload Metadata Interpreter  (Tier 2)
===================================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery
Phase I — Software Simulation

Implements Algorithm 2 from Section 3.4 of the report.

What this module does:
  1. Reads the four operator-assigned payload metadata tags:
       F  — Fragility class        {1..5}
       W  — Payload weight class   {1..3}
       U  — Delivery urgency        {1..3}
       P  — Priority level          {1..5}

  2. Derives three routing objective weights via the ALGEBRAIC BRIDGE
     (Eqs. 3.2 – 3.7, Section 3.3.2):
         r_raw    = F × 2
         d_raw    = U × 3
         e_raw    = W × 2
         w_risk   = r_raw  / (r_raw + d_raw + e_raw)
         w_dist   = d_raw  / (r_raw + d_raw + e_raw)
         w_energy = e_raw  / (r_raw + d_raw + e_raw)

  3. Min-max normalises each Pareto point's objectives (Eq. 3.8).

  4. High-priority amplification: if P ≥ 4, the dominant weight (whichever
     of w_risk / w_dist / w_energy is largest) is amplified by 0.1× itself,
     then all three weights are renormalised back to sum to 1.0.

  5. Computes a utility score for every Pareto point (Eq. 3.9):
         score(p_i) = w_dist × (1 − d̃_i) + w_energy × (1 − ẽ_i) + w_risk × (1 − ρ̃_i)

  6. Returns p* = argmax score(p_i) — the mission route.

NO re-planning is done.  The Pareto front from Tier 1 is loaded once and
re-used across ALL mission types instantly.

HOW TO RUN:
    python module4_tier2_payload.py

PREREQUISITES:
    module1_graph_builder.py and module2_tier1_moastar.py must have been run.

OUTPUT:
    outputs/module4_route_comparison.png    — bar chart of selected routes
    outputs/module4_pareto_selections.png   — 3D Pareto + selections overlay
    Console table showing all four payload preset results.
"""

import csv
import json
import os
import pickle
import time
from dataclasses import asdict, dataclass
from typing import Any, cast
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D   # needed for 3D projection

import config


# ─────────────────────────────────────────────────────────────────────────────
#  Four reference payload presets  (Table 3.2 of the report)
# ─────────────────────────────────────────────────────────────────────────────

PAYLOAD_PRESETS = {
    "fragile_medical":     {"F": 5, "W": 1, "U": 1, "P": 3,
                            "label": "Fragile\nMedical"},
    "emergency_medicine":  {"F": 1, "W": 1, "U": 3, "P": 5,
                            "label": "Emergency\nMedicine"},
    "disaster_relief":     {"F": 1, "W": 3, "U": 1, "P": 2,
                            "label": "Disaster\nRelief"},
    "mixed_cargo":         {"F": 3, "W": 2, "U": 2, "P": 3,
                            "label": "Mixed\nCargo"},
}

# Colours for each preset (consistent across all plots)
PRESET_COLOURS = {
    "fragile_medical":    "#2196F3",   # blue
    "emergency_medicine": "#F44336",   # red
    "disaster_relief":    "#FF9800",   # orange
    "mixed_cargo":        "#4CAF50",   # green
}


@dataclass
class SelectionResult:
    """Detailed Tier 2 decision record for audit/export."""

    tags: dict
    weights: dict
    selected_index: int | None
    cost_vector: list | None
    path: list
    normalised_scores: list
    objective_ranks: dict | None
    point_type: str | None
    latency_s: float
    feasible_count: int
    hold_advisory: str | None


class WeatherNoGo(Exception):
    """Raised by callers when weather gate says HOLD."""


# ─────────────────────────────────────────────────────────────────────────────
#  Weight derivation  (Algebraic Bridge — Section 3.3.2)
# ─────────────────────────────────────────────────────────────────────────────

def _validate_integer_tag(name: str, value: Any, lo: int, hi: int) -> int:
    """Validate one payload tag. Accept numpy integers; reject bool/floats."""
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{name} must be an integer in {lo}..{hi}")
    value = int(value)
    if value < lo or value > hi:
        raise ValueError(f"{name} must be an integer in {lo}..{hi}")
    return value


def validate_payload_tags(F: int, W: int, U: int, P: int) -> tuple[int, int, int, int]:
    """Validate Tier 2 payload tags and return plain Python ints."""
    return (
        _validate_integer_tag("F", F, 1, 5),
        _validate_integer_tag("W", W, 1, 3),
        _validate_integer_tag("U", U, 1, 3),
        _validate_integer_tag("P", P, 1, 5),
    )


def derive_weights(F: int, W: int, U: int) -> tuple:
    """
    Algebraic Bridge: payload tags → routing objective weights.

    Equations 3.2 – 3.7:
        r_raw    = F × 2   (fragility     → risk weight)
        d_raw    = U × 3   (urgency       → distance weight)
        e_raw    = W × 2   (payload weight → energy weight)
        w_risk   = r_raw  / total
        w_dist   = d_raw  / total
        w_energy = e_raw  / total

    The multipliers (2 for fragility/payload weight, 3 for urgency) reflect the
    domain insight that urgency has a higher penalty sensitivity in UAV medical
    delivery (Section 3.3.2). Payload weight maps to the energy objective
    because heavier payload directly increases hover/lift power draw
    (rotorcraft power scales with weight), unlike risk or distance, which are
    not physically driven by payload mass.

    Returns
    -------
    (w_risk, w_dist, w_energy) : floats that sum to 1.0
    """
    F = _validate_integer_tag("F", F, 1, 5)
    W = _validate_integer_tag("W", W, 1, 3)
    U = _validate_integer_tag("U", U, 1, 3)
    r_raw = F * 2
    d_raw = U * 3
    e_raw = W * 2
    total = r_raw + d_raw + e_raw
    return r_raw / total, d_raw / total, e_raw / total


def route_energy_wh(
    energy_objective: float,
    payload_weight_class: int,
    *,
    m_uav_kg: float = config.M_UAV_KG,
    k_cal_wh_per_km: float = config.K_CAL_WH_PER_KM,
    payload_mass_kg: dict[int, float] | None = None,
) -> float:
    """Convert route energy objective into battery Wh for one payload class."""
    W = _validate_integer_tag("W", payload_weight_class, 1, 3)
    masses = payload_mass_kg if payload_mass_kg is not None else config.PAYLOAD_MASS_KG
    m_payload = masses[W]
    return float(k_cal_wh_per_km * energy_objective * ((m_uav_kg + m_payload) / m_uav_kg) ** 1.5)


# ─────────────────────────────────────────────────────────────────────────────
#  Tier 2 selector
# ─────────────────────────────────────────────────────────────────────────────

class TierTwoSelector:
    """
    Payload Metadata Interpreter — Tier 2 Pareto Selection.

    Parameters
    ----------
    pareto_front : list of ([d, ε, ρ], path)
        Pre-computed Pareto front from Module 2 (Tier 1).
    """

    def __init__(self, pareto_front: list):
        if not pareto_front:
            raise ValueError("Pareto front is empty — run Module 2 first.")
        self.pareto_front = pareto_front
        self.costs = np.array([g for (g, _) in pareto_front])   # shape (K, 3)
        self._compute_minmax()

    # ------------------------------------------------------------------
    #  Preprocessing
    # ------------------------------------------------------------------

    def _compute_minmax(self) -> None:
        """Precompute min/max for min-max normalisation (Eq. 3.8)."""
        self.d_min, self.d_max = self.costs[:, 0].min(), self.costs[:, 0].max()
        self.e_min, self.e_max = self.costs[:, 1].min(), self.costs[:, 1].max()
        self.r_min, self.r_max = self.costs[:, 2].min(), self.costs[:, 2].max()

    def _normalise(self, g: list) -> tuple:
        """
        Min-max normalise a single Pareto point (Eq. 3.8).

        d̃_i = (d_i − d_min) / (d_max − d_min)
        ẽ_i = (ε_i − ε_min) / (ε_max − ε_min)
        ρ̃_i = (ρ_i − ρ_min) / (ρ_max − ρ_min)

        Handles the degenerate case where min == max (returns 0).
        """
        def safe_norm(val, mn, mx):
            return (val - mn) / (mx - mn) if mx > mn else 0.0

        return (safe_norm(g[0], self.d_min, self.d_max),
                safe_norm(g[1], self.e_min, self.e_max),
                safe_norm(g[2], self.r_min, self.r_max))

    def _score_all(self, w_risk: float, w_dist: float, w_energy: float) -> np.ndarray:
        scores = np.empty(len(self.pareto_front))
        for i, (g, _) in enumerate(self.pareto_front):
            d_t, e_t, r_t = self._normalise(g)
            scores[i] = (w_dist   * (1.0 - d_t) +
                         w_energy * (1.0 - e_t) +
                         w_risk   * (1.0 - r_t))
        return scores

    def _derive_amplified_weights(self, F: int, W: int, U: int, P: int) -> tuple[float, float, float]:
        w_risk, w_dist, w_energy = derive_weights(F, W, U)
        if P >= 4:
            w_max = max(w_risk, w_dist, w_energy)
            bonus = 0.1 * w_max
            if w_risk == w_max:
                w_risk += bonus
            elif w_dist == w_max:
                w_dist += bonus
            else:
                w_energy += bonus
            total = w_risk + w_dist + w_energy
            w_risk, w_dist, w_energy = w_risk / total, w_dist / total, w_energy / total
        return w_risk, w_dist, w_energy

    def _objective_ranks(self, route_index: int) -> dict:
        ranks = {}
        for obj_idx, name in enumerate(("distance", "energy", "risk")):
            order = sorted(range(len(self.pareto_front)), key=lambda idx: (self.costs[idx, obj_idx], idx))
            ranks[name] = order.index(route_index) + 1
        return ranks

    def _waypoints_for_path(self, path: list, graph=None) -> list:
        waypoints = []
        for node in path:
            data = graph.nodes[node] if graph is not None and node in graph else {}
            waypoints.append({
                "node": node,
                "lat": data.get("lat"),
                "lon": data.get("lon"),
                "alt_amsl_m": data.get("alt_amsl_m"),
                "agl_m": data.get("agl_m"),
            })
        return waypoints

    def _write_selection_outputs(self, result: SelectionResult, output_dir: str, name: str) -> None:
        os.makedirs(output_dir, exist_ok=True)
        json_path = os.path.join(output_dir, f"selection_{name}.json")
        csv_path = os.path.join(output_dir, f"selection_{name}.csv")

        with open(json_path, "w", encoding="utf-8") as fh:
            json.dump(asdict(result), fh, indent=2)

        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=["seq", "node", "lat", "lon", "alt_amsl_m", "agl_m"])
            writer.writeheader()
            for seq, waypoint in enumerate(result.path):
                row = {"seq": seq}
                row.update(waypoint)
                writer.writerow(row)

    # ------------------------------------------------------------------
    #  Algorithm 2 — single selection
    # ------------------------------------------------------------------

    def select(self, F: int, W: int, U: int, P: int) -> tuple:
        """
        Algorithm 2 — Payload-Driven Pareto Route Selection.

        Parameters
        ----------
        F : Fragility class         {1..5}
        W : Payload weight class    {1..3}
        U : Delivery urgency        {1..3}
        P : Priority level          {1..5}

        Returns
        -------
        selected_route : (g_vector, path)
        w_risk, w_dist, w_energy : derived weights
        scores : np.ndarray of all utility scores
        """
        F, W, U, P = validate_payload_tags(F, W, U, P)
        w_risk, w_dist, w_energy = self._derive_amplified_weights(F, W, U, P)
        scores = self._score_all(w_risk, w_dist, w_energy)

        # argmax → selected mission route (Algorithm 2, lines 21–22)
        best_idx       = int(np.argmax(scores))
        selected_route = self.pareto_front[best_idx]

        return selected_route, w_risk, w_dist, w_energy, scores

    def select_detailed(
        self,
        F: int,
        W: int,
        U: int,
        P: int,
        *,
        graph=None,
        output_name: str = "custom",
        output_dir: str = "outputs/tier2",
        write_outputs: bool = True,
        weather_no_go: bool = False,
        weather_advisory: str | None = None,
        battery_wh: float = config.BATTERY_WH,
        battery_reserve: float = config.BATTERY_RESERVE,
        m_uav_kg: float = config.M_UAV_KG,
        k_cal_wh_per_km: float = config.K_CAL_WH_PER_KM,
        payload_mass_kg: dict[int, float] | None = None,
    ) -> SelectionResult:
        """Detailed Tier 2 selection with validation, feasibility, and export."""
        t0 = time.perf_counter()
        F, W, U, P = validate_payload_tags(F, W, U, P)
        w_risk, w_dist, w_energy = self._derive_amplified_weights(F, W, U, P)
        scores = self._score_all(w_risk, w_dist, w_energy)
        usable_wh = float(battery_wh * (1.0 - battery_reserve))
        feasible_indices = [
            idx for idx, (g, _) in enumerate(self.pareto_front)
            if route_energy_wh(
                float(g[1]), W,
                m_uav_kg=m_uav_kg,
                k_cal_wh_per_km=k_cal_wh_per_km,
                payload_mass_kg=payload_mass_kg,
            ) <= usable_wh
        ]

        hold_advisory = None
        if weather_no_go:
            hold_advisory = weather_advisory or "HOLD: weather gate is NO-GO."
        elif not feasible_indices:
            hold_advisory = (
                f"HOLD: no feasible route for payload W={W}; "
                f"usable battery {usable_wh:.2f} Wh."
            )

        if hold_advisory:
            result = SelectionResult(
                tags={"F": F, "W": W, "U": U, "P": P},
                weights={"risk": w_risk, "distance": w_dist, "energy": w_energy},
                selected_index=None,
                cost_vector=None,
                path=[],
                normalised_scores=[float(x) for x in scores],
                objective_ranks=None,
                point_type=None,
                latency_s=time.perf_counter() - t0,
                feasible_count=len(feasible_indices),
                hold_advisory=hold_advisory,
            )
        else:
            best_idx = max(feasible_indices, key=lambda idx: (scores[idx], -idx))
            g, path = self.pareto_front[best_idx]
            result = SelectionResult(
                tags={"F": F, "W": W, "U": U, "P": P},
                weights={"risk": w_risk, "distance": w_dist, "energy": w_energy},
                selected_index=int(best_idx),
                cost_vector=[float(x) for x in g],
                path=self._waypoints_for_path(path, graph=graph),
                normalised_scores=[float(x) for x in scores],
                objective_ranks=self._objective_ranks(best_idx),
                point_type=self.classify_point(best_idx),
                latency_s=time.perf_counter() - t0,
                feasible_count=len(feasible_indices),
                hold_advisory=None,
            )

        if write_outputs:
            self._write_selection_outputs(result, output_dir, output_name)
        return result

    # ------------------------------------------------------------------
    #  Run all four reference presets
    # ------------------------------------------------------------------

    def selected_index(self, selected_route: tuple) -> int:
        """Return the index of a selected route in the current Pareto front."""
        return self.pareto_front.index(selected_route)

    def classify_point(self, route_index: int) -> str:
        """
        Classify a Pareto point for demo diagnostics.

        A point is an objective extreme only if it is the best route for at
        least one individual objective. Otherwise it is non-extreme.
        """
        objective_names = ("min-distance", "min-energy", "min-risk")
        matches = [
            objective_names[obj_idx]
            for obj_idx in range(3)
            if route_index == int(np.argmin(self.costs[:, obj_idx]))
        ]
        return "/".join(matches) if matches else "non-extreme"

    def run_all_presets(self) -> dict:
        """
        Run Tier 2 for all four payload presets from Table 3.2.

        Returns
        -------
        results : dict  name → (g_vector, path, w_risk, w_dist, w_energy, scores)
        """
        print("\n" + "="*118)
        print("  MODULE 4 — Tier 2 Payload-Aware Route Selection  (all presets)")
        print("="*118)
        header = (f"{'Preset':<26} {'F':>2} {'W':>2} {'U':>2} {'P':>2}"
                  f"  {'w_risk':>7} {'w_dist':>7} {'w_enrg':>7}"
                  f"  {'Idx':>4} {'Dist(km)':>9} {'Energy':>8}"
                  f" {'CumRisk':>8} {'Score':>8} {'Point type':>13}")
        print(header)
        print("-"*118)

        results = {}
        for name, tags in PAYLOAD_PRESETS.items():
            F, W, U, Plvl = tags["F"], tags["W"], tags["U"], tags["P"]
            selected, wr, wd, we, scores = self.select(F, W, U, Plvl)
            g, path = selected
            route_idx = self.selected_index(selected)
            point_type = self.classify_point(route_idx)
            results[name] = (g, path, wr, wd, we, scores)
            print(f"  {name:<24} {F:>2} {W:>2} {U:>2} {Plvl:>2}"
                  f"  {wr:>7.3f} {wd:>7.3f} {we:>7.3f}"
                  f"  {route_idx:>4} {g[0]:>9.4f} {g[1]:>8.4f}"
                  f" {g[2]:>8.4f} {scores[route_idx]:>8.4f}"
                  f" {point_type:>13}")

        unique_routes = len({tuple(result[1]) for result in results.values()})
        print("="*118)
        print(f"  ↳  Unique selected routes: {unique_routes}/{len(results)}")
        print("     Route reuse is valid when one Pareto point maximises utility for multiple payloads.")
        print("="*118 + "\n")
        return results

    # ------------------------------------------------------------------
    #  Visualisations
    # ------------------------------------------------------------------

    def visualise_route_comparison(self, results: dict) -> Figure:
        """
        Bar chart comparing Distance, Energy, and Risk for each selected route.
        """
        names  = list(results.keys())
        labels = [PAYLOAD_PRESETS[n]["label"] for n in names]
        colors = [PRESET_COLOURS[n] for n in names]

        metrics     = ["Distance (km)", "Energy (proxy)", "Cumulative Risk"]
        metric_idx  = [0, 1, 2]

        fig, axes = plt.subplots(1, 3, figsize=(14, 6))
        for ax, m_label, m_idx in zip(axes, metrics, metric_idx):
            vals = [results[n][0][m_idx] for n in names]
            bars = ax.bar(labels, vals, color=colors,
                          edgecolor="black", linewidth=0.8)
            ax.set_title(m_label, fontweight="bold", fontsize=12)
            ax.set_ylabel(m_label, fontsize=10)
            ax.grid(axis="y", linestyle="--", alpha=0.4)
            for bar, v in zip(bars, vals):
                ax.text(bar.get_x() + bar.get_width() / 2.0,
                        bar.get_height() + max(vals) * 0.01,
                        f"{v:.3f}", ha="center", va="bottom", fontsize=8.5)

        fig.suptitle(
            "Module 4 — Tier 2 Payload-Aware Route Selection\n"
            "Payload tags score the Pareto front; routes may be reused when utility agrees",
            fontweight="bold", fontsize=13
        )
        plt.tight_layout()
        return fig

    def visualise_pareto_with_selections(self, results: dict) -> Figure:
        """
        3D scatter: all Pareto points (grey) + selected point per preset (coloured).
        """
        costs = self.costs
        fig   = plt.figure(figsize=(11, 9))
        ax    = cast(Axes3D, fig.add_subplot(111, projection="3d"))

        # Full Pareto front — grey
        ax.scatter(costs[:, 0], costs[:, 1], costs[:, 2],
                   c="lightgray", s=35, alpha=0.55,
                   edgecolors="none", label="Pareto front")

        # Selected point for each preset — coloured & large
        for name, (g, _, wr, wd, we, _) in results.items():
            label = PAYLOAD_PRESETS[name]["label"].replace("\n", " ")
            ax.scatter([g[0]], [g[1]], [g[2]],  # type: ignore[arg-type]
                       s=220, zorder=6,
                       color=PRESET_COLOURS[name],
                       edgecolors="black", linewidths=1.5,
                       label=f"{label}  (w_r={wr:.2f}, w_d={wd:.2f}, w_e={we:.2f})")

        ax.set_xlabel("Distance (km)", labelpad=10)
        ax.set_ylabel("Energy (proxy)", labelpad=10)
        ax.set_zlabel("Cumulative Risk", labelpad=10)
        ax.set_title("Module 4 — Pareto Front with Tier 2 Payload-Aware Selections",
                     fontsize=11, fontweight="bold")
        ax.legend(loc="upper left", fontsize=8, framealpha=0.85)
        plt.tight_layout()
        return fig

    def visualise_selected_routes_on_graph(
            self, G, results: dict, source, target) -> Figure:
        """
        2×2 figure — each panel shows the UAV airspace graph with the
        selected route for one payload preset drawn on top.

        Parameters
        ----------
        G      : nx.DiGraph — airspace graph from Module 1.
        results: dict from run_all_presets().
        source : source node ID.
        target : target node ID.
        """
        import networkx as nx
        names = list(results.keys())
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.patch.set_facecolor("#F5F5F5")
        axes_flat = axes.flatten()
        pos = {n: (d["x"], d["y"]) for n, d in G.nodes(data=True)}

        for ax, name in zip(axes_flat, names):
            g_vec, path, wr, wd, we, _ = results[name]
            color  = PRESET_COLOURS[name]
            label  = PAYLOAD_PRESETS[name]["label"].replace("\n", " ")

            ax.set_facecolor("#FAFAFA")

            # ── Full airspace graph (grey background) ────────────────────
            nx.draw_networkx_edges(G, pos, ax=ax, edge_color="#DDDDDD",
                                   arrows=False, width=0.5, alpha=0.6)
            nx.draw_networkx_nodes(G, pos, ax=ax, node_size=12,
                                   node_color="#BBBBBB", alpha=0.75)

            # ── Selected route ───────────────────────────────────────────
            if path and len(path) > 1:
                route_edges = list(zip(path[:-1], path[1:]))
                nx.draw_networkx_edges(G, pos, edgelist=route_edges, ax=ax,
                                       edge_color=color, width=3.2,
                                       arrows=True, arrowsize=14,
                                       connectionstyle="arc3,rad=0.08",
                                       alpha=0.95)
                # Intermediate waypoints in preset colour
                waypoints = path[1:-1]
                if waypoints:
                    nx.draw_networkx_nodes(G, pos, nodelist=waypoints, ax=ax,
                                           node_size=28, node_color=color,
                                           alpha=0.9, edgecolors="white",
                                           linewidths=0.5)

            # ── Source (green) and Target (red) ──────────────────────────
            nx.draw_networkx_nodes(G, pos, nodelist=[source], ax=ax,
                                   node_size=180, node_color="#27AE60",
                                   edgecolors="black", linewidths=1.0)
            nx.draw_networkx_nodes(G, pos, nodelist=[target], ax=ax,
                                   node_size=180, node_color="#E74C3C",
                                   edgecolors="black", linewidths=1.0)
            sx, sy = pos[source]
            tx, ty = pos[target]
            offset = (max(p[0] for p in pos.values()) -
                      min(p[0] for p in pos.values())) * 0.02
            ax.text(sx, sy + offset, "S", ha="center", va="bottom",
                    fontsize=9, fontweight="bold", color="#27AE60")
            ax.text(tx, ty + offset, "T", ha="center", va="bottom",
                    fontsize=9, fontweight="bold", color="#E74C3C")

            hop_count = len(path) - 1 if path else 0
            ax.set_title(
                f"{label}\n"
                f"Dist = {g_vec[0]:.1f} km  |  Energy = {g_vec[1]:.1f}"
                f"  |  CumRisk = {g_vec[2]:.2f}  |  Hops = {hop_count}\n"
                f"w_risk={wr:.2f}  w_dist={wd:.2f}  w_energy={we:.2f}",
                fontsize=10, fontweight="bold", color=color, pad=6
            )
            ax.axis("off")

        fig.suptitle(
            "Module 4 — Tier 2 Selected UAV Flight Routes per Payload Preset\n"
            "■ Green = Source   ■ Red = Target   Coloured path = selected route",
            fontsize=13, fontweight="bold", y=1.01
        )
        plt.tight_layout()
        return fig

    def visualise_weight_derivation(self, results: dict) -> Figure:
        """
        Stacked bar showing how the algebraic bridge maps each payload to weights.
        """
        names  = list(results.keys())
        labels = [PAYLOAD_PRESETS[n]["label"] for n in names]
        w_risk   = [results[n][2] for n in names]
        w_dist   = [results[n][3] for n in names]
        w_energy = [results[n][4] for n in names]

        x = np.arange(len(names))
        fig, ax = plt.subplots(figsize=(9, 6))

        bars_r = ax.bar(x, w_risk,   label="w_risk",   color="#E74C3C", edgecolor="k")
        bars_d = ax.bar(x, w_dist,   bottom=w_risk,
                        label="w_dist",   color="#3498DB", edgecolor="k")
        bottom2 = [a + b for a, b in zip(w_risk, w_dist)]
        bars_e = ax.bar(x, w_energy, bottom=bottom2,
                        label="w_energy", color="#2ECC71", edgecolor="k")

        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=11)
        ax.set_ylabel("Weight (sums to 1.0)")
        ax.set_ylim(0, 1.0)
        ax.set_title("Module 4 — Algebraic Bridge: Payload Tags → Routing Weights\n"
                     "(Eqs. 3.5 – 3.7)", fontweight="bold")
        ax.legend(fontsize=10)
        ax.grid(axis="y", linestyle="--", alpha=0.4)

        # Annotate each segment
        for bars, vals in [(bars_r, w_risk), (bars_d, w_dist), (bars_e, w_energy)]:
            for bar, v in zip(bars, vals):
                if v > 0.04:
                    ax.text(bar.get_x() + bar.get_width() / 2.0,
                            bar.get_y() + v / 2.0,
                            f"{v:.2f}", ha="center", va="center",
                            fontsize=9, color="white", fontweight="bold")
        plt.tight_layout()
        return fig


# ─────────────────────────────────────────────────────────────────────────────
#  Script entry-point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    from module2_tier1_moastar import TierOneEngine

    os.makedirs("outputs", exist_ok=True)

    # ── Load pre-computed Pareto front ───────────────────────────────────
    pf_path = "pareto_front_storage/pareto_front.pkl"
    if not os.path.exists(pf_path):
        raise FileNotFoundError(
            "Pareto front not found. Run module2_tier1_moastar.py first."
        )
    pareto_front = TierOneEngine.load(pf_path)
    print(f"[Module 4] Loaded Pareto front: {len(pareto_front)} routes")

    # ── Instantiate and run Tier 2 ───────────────────────────────────────
    selector = TierTwoSelector(pareto_front)
    results  = selector.run_all_presets()

    # ── Save visualisations ──────────────────────────────────────────────
    fig1 = selector.visualise_route_comparison(results)
    fig1.savefig("outputs/module4_route_comparison.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig1)
    print("[Module 4] Route comparison saved → outputs/module4_route_comparison.png")

    fig2 = selector.visualise_pareto_with_selections(results)
    fig2.savefig("outputs/module4_pareto_selections.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print("[Module 4] Pareto selections saved → outputs/module4_pareto_selections.png")

    fig3 = selector.visualise_weight_derivation(results)
    fig3.savefig("outputs/module4_weight_derivation.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig3)
    print("[Module 4] Weight derivation saved → outputs/module4_weight_derivation.png")

    print("[Module 4] DONE ✓")
