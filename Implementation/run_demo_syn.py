"""
run_demo_syn.py — Phase I Software Simulation  (Review 1 Demo)
===============================================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery

This script runs the FULL end-to-end pipeline for the 30% Review 1
implementation demo:

  Step 1 → Module 1: Build the seeded synthetic airspace graph (wind + terrain + obstacles)
  Step 2 → Module 2: Run Multi-Objective A* to compute Pareto front (Tier 1)
  Step 3 → Module 4: Run Payload-Aware selection for 4 payload presets (Tier 2)
  Step 4 → Save all output plots to outputs/

HOW TO RUN:
  ┌─────────────────────────────────────────────────────────────┐
  │  Install dependencies first (one-time):                     │
  │    pip install networkx numpy matplotlib osmnx              │
  │                                                             │
  │  Then run:                                                  │
  │    python run_demo_syn.py                                   │
  └─────────────────────────────────────────────────────────────┘

OUTPUT FILES (all in  outputs/ folder):
  module1_airspace_graph.png      — the UAV airspace graph
  module2_pareto_front_3d.png     — Pareto front in 3D objective space
  module2_pareto_front_2d.png     — 2D projections of Pareto front
  module4_route_comparison.png    — bar chart: selected routes per payload
  module4_pareto_selections.png   — Pareto + coloured selection markers
  module4_weight_derivation.png   — stacked bar: algebraic bridge weights
  module_all_combined.png         — single figure summarising all steps
"""

import os
import sys
import time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── Make sure sibling modules are importable ────────────────────────────────
sys.path.insert(0, os.path.dirname(__file__))

from module1_graph_builder import AirspaceGraphBuilder
from module2_tier1_moastar  import TierOneEngine
from module4_tier2_payload   import TierTwoSelector, PAYLOAD_PRESETS, PRESET_COLOURS

os.makedirs("outputs",              exist_ok=True)
os.makedirs("pareto_front_storage", exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
def print_banner(step: int, title: str) -> None:
    bar = "─" * 62
    print(f"\n╔{bar}╗")
    print(f"║  STEP {step}: {title:<53}║")
    print(f"╚{bar}╝")


# ─────────────────────────────────────────────────────────────────────────────
def run_module1(location="Thiruvanmiyur, Chennai, Tamil Nadu, India",
                dist=2000, seed=42):
    """Build and save the airspace graph, return G + (source, target)."""
    print_banner(1, "Airspace Graph Builder  [Module 1]")
    t0 = time.time()

    builder = AirspaceGraphBuilder(location=location, dist=dist, seed=seed)
    G = builder.fetch_graph()
    builder.save()

    # Pick source (first node) and target (last node — opposite corner for
    # maximum path diversity across the spatial hazard zones)
    nodes  = list(G.nodes())
    source = nodes[0]
    target = nodes[-1]
    print(f"[Demo]   Source node : {source}")
    print(f"[Demo]   Target node : {target}")

    fig = builder.visualise(source=source, target=target,
                             title="Module 1 — Airspace Graph (UAV delivery region)")
    fig.savefig("outputs/module1_airspace_graph.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Demo] Module 1 done in {time.time()-t0:.1f}s ✓")
    return G, source, target


# ─────────────────────────────────────────────────────────────────────────────
def run_module2(G, source, target):
    """Run Tier 1 Multi-Objective A* and return the Pareto front."""
    print_banner(2, "Multi-Objective A* — Tier 1 Pareto Front  [Module 2]")
    t0 = time.time()

    engine = TierOneEngine(G, source, target)
    pareto_front = engine.run()
    engine.print_summary()
    engine.save()

    fig3d = engine.visualise_3d()
    if fig3d:
        fig3d.savefig("outputs/module2_pareto_front_3d.png",
                      dpi=150, bbox_inches="tight")
        plt.close(fig3d)

    fig2d = engine.visualise_2d_projections()
    if fig2d:
        fig2d.savefig("outputs/module2_pareto_front_2d.png",
                      dpi=150, bbox_inches="tight")
        plt.close(fig2d)

    print(f"[Demo] Module 2 done in {time.time()-t0:.1f}s ✓")
    return pareto_front


# ─────────────────────────────────────────────────────────────────────────────
def run_module4(pareto_front, G=None, source=None, target=None):
    """Run Tier 2 payload-aware selection for all four presets."""
    print_banner(4, "Payload-Aware Route Selection — Tier 2  [Module 4]")
    t0 = time.time()

    selector = TierTwoSelector(pareto_front, no_go_reasons=(G.graph.get("no_go_reasons") if G is not None else None))
    results  = selector.run_all_presets()
    if not results:                     # weather NO-GO, or no preset fits the battery
        print("[Demo] Tier 2 HOLD for every preset — Tier 2 plots skipped")
        return selector, results

    fig1 = selector.visualise_route_comparison(results)
    fig1.savefig("outputs/module4_route_comparison.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig1)

    fig2 = selector.visualise_pareto_with_selections(results)
    fig2.savefig("outputs/module4_pareto_selections.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig2)

    fig3 = selector.visualise_weight_derivation(results)
    fig3.savefig("outputs/module4_weight_derivation.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig3)

    # ── NEW: draw each selected route on the airspace graph ─────────────
    if G is not None and source is not None and target is not None:
        fig4 = selector.visualise_selected_routes_on_graph(
            G, results, source, target)
        fig4.savefig("outputs/module4_routes_on_graph.png",
                     dpi=150, bbox_inches="tight")
        plt.close(fig4)
        print("[Demo] Route-on-graph figure saved → outputs/module4_routes_on_graph.png")

    print(f"[Demo] Module 4 done in {time.time()-t0:.1f}s ✓")
    return selector, results


# ─────────────────────────────────────────────────────────────────────────────
def build_summary_figure(G, source, target, engine, selector, results):
    """
    Produce a single combined figure with 4 panels:
      [A] Airspace graph
      [B] 3D Pareto front
      [C] Weight derivation (algebraic bridge)
      [D] Selected routes per payload preset
    """
    print_banner(0, "Building combined summary figure ...")
    from mpl_toolkits.mplot3d import Axes3D
    from typing import cast

    fig = plt.figure(figsize=(20, 14))
    fig.patch.set_facecolor("#FAFAFA")

    # ── Panel A: Airspace graph ──────────────────────────────────────────
    import networkx as nx
    ax_a = fig.add_subplot(2, 2, 1)
    pos = {n: (d["x"], d["y"]) for n, d in G.nodes(data=True)}
    nx.draw_networkx_edges(G, pos, ax=ax_a, edge_color="#CCCCCC",
                           arrows=False, width=0.5, alpha=0.7)
    nx.draw_networkx_nodes(G, pos, ax=ax_a, node_size=20,
                           node_color="#4A90D9", alpha=0.9)
    nx.draw_networkx_nodes(G, pos, nodelist=[source], ax=ax_a,
                           node_size=150, node_color="#27AE60")
    nx.draw_networkx_nodes(G, pos, nodelist=[target], ax=ax_a,
                           node_size=150, node_color="#E74C3C")
    ax_a.set_title("(A)  Airspace Graph — Module 1\n"
                   f"{G.number_of_nodes()} waypoints  |  "
                   f"{G.number_of_edges()} flight segments",
                   fontweight="bold", fontsize=11)
    ax_a.axis("off")

    # ── Panel B: 3D Pareto front ─────────────────────────────────────────
    ax_b = cast(Axes3D, fig.add_subplot(2, 2, 2, projection="3d"))
    costs = engine.costs if hasattr(engine, "costs") else \
            selector.costs
    ax_b.scatter(costs[:, 0], costs[:, 1], costs[:, 2],
                 c=costs[:, 2], cmap="RdYlGn_r", s=50,
                 edgecolors="k", linewidths=0.4, alpha=0.8)
    # Overlay selections
    for name, (g, _, wr, wd, we, _) in results.items():
        lbl = PAYLOAD_PRESETS[name]["label"].replace("\n", " ")
        ax_b.scatter([g[0]], [g[1]], [g[2]], s=180, zorder=6,  # type: ignore[arg-type]
                     color=PRESET_COLOURS[name],
                     edgecolors="k", linewidths=1.5, label=lbl)
    ax_b.set_xlabel("Distance (km)", fontsize=8, labelpad=5)
    ax_b.set_ylabel("Energy",        fontsize=8, labelpad=5)
    ax_b.set_zlabel("Risk",          fontsize=8, labelpad=5)
    ax_b.set_title("(B)  Pareto Front + Tier 2 Selections — Module 2 & 4",
                   fontweight="bold", fontsize=11)
    ax_b.legend(fontsize=7, loc="upper left")

    # ── Panel C: Weight derivation stacked bar ───────────────────────────
    ax_c = fig.add_subplot(2, 2, 3)
    names  = list(results.keys())
    xlabels = [PAYLOAD_PRESETS[n]["label"] for n in names]
    x = range(len(names))
    w_r = [results[n][2] for n in names]
    w_d = [results[n][3] for n in names]
    w_e = [results[n][4] for n in names]
    ax_c.bar(x, w_r, label="w_risk",   color="#E74C3C", edgecolor="k")
    ax_c.bar(x, w_d, bottom=w_r,
             label="w_dist",   color="#3498DB", edgecolor="k")
    b2 = [a + b for a, b in zip(w_r, w_d)]
    ax_c.bar(x, w_e, bottom=b2,
             label="w_energy", color="#2ECC71", edgecolor="k")
    ax_c.set_xticks(list(x))
    ax_c.set_xticklabels(xlabels, fontsize=10)
    ax_c.set_ylabel("Weight")
    ax_c.set_ylim(0, 1.0)
    ax_c.set_title("(C)  Algebraic Bridge: Payload → Weights — Module 4",
                   fontweight="bold", fontsize=11)
    ax_c.legend(fontsize=9)
    ax_c.grid(axis="y", linestyle="--", alpha=0.4)

    # ── Panel D: Route metric comparison ────────────────────────────────
    ax_d = fig.add_subplot(2, 2, 4)
    import numpy as np
    bar_w = 0.25
    x_pos = np.arange(len(names))
    colors = [PRESET_COLOURS[n] for n in names]
    dist_vals = [results[n][0][0] for n in names]
    enrg_vals = [results[n][0][1] for n in names]
    risk_vals = [results[n][0][2] for n in names]

    # Normalise to [0,1] for display on same axis
    def norm(v):
        mn, mx = min(v), max(v)
        return [(x - mn) / (mx - mn + 1e-9) for x in v]

    ax_d.bar(x_pos - bar_w, norm(dist_vals), bar_w,
             label="Distance", color="#3498DB", edgecolor="k", alpha=0.85)
    ax_d.bar(x_pos,          norm(enrg_vals), bar_w,
             label="Energy",   color="#E67E22", edgecolor="k", alpha=0.85)
    ax_d.bar(x_pos + bar_w,  norm(risk_vals), bar_w,
             label="Cum. Risk", color="#E74C3C", edgecolor="k", alpha=0.85)
    ax_d.set_xticks(list(x_pos))
    ax_d.set_xticklabels(xlabels, fontsize=10)
    ax_d.set_ylabel("Normalised objective value")
    ax_d.set_title("(D)  Selected Route Objectives per Payload — Module 4",
                   fontweight="bold", fontsize=11)
    ax_d.legend(fontsize=9)
    ax_d.grid(axis="y", linestyle="--", alpha=0.4)

    fig.suptitle("Payload-Aware Two-Tier UAV Route Planning — Review 1 Demo\n"
                 "Phase I Software Simulation  |  30% Implementation",
                 fontsize=15, fontweight="bold", y=1.01)
    plt.tight_layout()
    out = "outputs/module_all_combined.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Demo] Combined summary figure saved → {out}")
    return out


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    t_start = time.time()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║   FYP B28 — Payload-Aware Two-Tier UAV Route Planning       ║")
    print("║   Phase I Software Simulation  |  Review 1 Demo             ║")
    print("╚══════════════════════════════════════════════════════════════╝")

    # ── Step 1: Module 1 ─────────────────────────────────────────────────
    G, source, target = run_module1(location="Thiruvanmiyur, Chennai, Tamil Nadu, India", dist=4000, seed=42)

    # ── Step 2: Module 2 ─────────────────────────────────────────────────
    pareto_front = run_module2(G, source, target)

    if not pareto_front:
        print("\n[ERROR] No Pareto front found for the chosen source→target pair.")
        print("        Try increasing graph size or choosing different nodes.")
        sys.exit(1)

    # ── Step 3: Module 4 ─────────────────────────────────────────────────
    selector, results = run_module4(pareto_front, G=G, source=source, target=target)

    # ── Step 4: Combined figure ──────────────────────────────────────────
    # Attach costs attribute to engine for summary figure
    import numpy as np
    class _FakeEngine:
        def __init__(self, costs: np.ndarray):
            self.costs = costs
    eng = _FakeEngine(np.array([g for (g, _) in pareto_front]))

    if results:                                   # empty after a weather NO-GO hold
        build_summary_figure(G, source, target, eng, selector, results)

    # ── Final summary ────────────────────────────────────────────────────
    elapsed = time.time() - t_start
    print("\n" + "═"*62)
    print("  ALL STEPS COMPLETE")
    print(f"  Total time : {elapsed:.1f} s")
    print("  Output files in: outputs/")
    print("    • module1_airspace_graph.png")
    print("    • module2_pareto_front_3d.png")
    print("    • module2_pareto_front_2d.png")
    print("    • module4_route_comparison.png")
    print("    • module4_pareto_selections.png")
    print("    • module4_weight_derivation.png")
    print("    • module4_routes_on_graph.png  ← selected flight paths on the map")
    print("    • module_all_combined.png  ← show this in your Review 1 demo")
    print("═"*62)


if __name__ == "__main__":
    main()
