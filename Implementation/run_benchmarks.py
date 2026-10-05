"""
Run benchmark suite for Phase I algorithm evaluation.

Outputs:
  outputs/benchmarks/benchmark_metrics.csv
  outputs/benchmarks/benchmark_metrics.json
  outputs/benchmarks/tier2_evidence.csv
  outputs/benchmarks/*.png
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from module1_graph_builder import AirspaceGraphBuilder
from module2_tier1_moastar import TierOneEngine
from module3_baselines import (
    mopso_baseline,
    objective_scales_from_extremes,
    single_objective_astar,
    weighted_sum_astar,
)
from module4_tier2_payload import PAYLOAD_PRESETS, TierTwoSelector
from module5_evaluation import (
    evaluate_method_set,
    monotonicity_violations,
    risk_component_shares,
    tier2_evidence,
    tier2_sweep,
    write_csv,
    write_json,
)


SCENARIOS = [
    {"name": "S1_50_d05", "nodes": 50, "density": 0.05, "seed": 101, "max_explored": 80_000},
    {"name": "S1_50_d20", "nodes": 50, "density": 0.20, "seed": 102, "max_explored": 80_000},
    {"name": "S1_50_d40", "nodes": 50, "density": 0.40, "seed": 103, "max_explored": 80_000},
    {"name": "S2_200_d05", "nodes": 200, "density": 0.05, "seed": 201, "max_explored": 180_000},
    {"name": "S2_200_d20", "nodes": 200, "density": 0.20, "seed": 202, "max_explored": 180_000},
    {"name": "S2_200_d40", "nodes": 200, "density": 0.40, "seed": 203, "max_explored": 180_000},
    {"name": "S3_500_d05", "nodes": 500, "density": 0.05, "seed": 301, "max_explored": 260_000},
    {"name": "S3_500_d20", "nodes": 500, "density": 0.20, "seed": 302, "max_explored": 260_000},
    {"name": "S3_500_d40", "nodes": 500, "density": 0.40, "seed": 303, "max_explored": 260_000},
]


def _apply_density_risk(G, density: float) -> None:
    """
    Adjust synthetic risk intensity for benchmark density scenarios.

    The current graph builder has one synthetic risk field, not a full obstacle
    layer. This deterministic scaling gives sparse/medium/dense scenarios
    without changing topology or adding fake live data.
    """
    multiplier = 0.55 + 2.25 * density
    for _u, _v, data in G.edges(data=True):
        risk = float(np.clip(data["risk"] * multiplier, 0.0, 1.0))
        data["risk"] = round(risk, 5)
        data["cost"] = [data["distance"], data["energy"], data["risk"]]


def build_scenario(n_nodes: int, seed: int, density: float):
    builder = AirspaceGraphBuilder(seed=seed)
    G = builder._build_synthetic_graph(n_nodes=n_nodes)
    _apply_density_risk(G, density)
    builder.G = G
    nodes = list(G.nodes())
    return G, nodes[0], nodes[-1]


def run_namoa(G, source, target, max_explored: int):
    t0 = time.perf_counter()
    engine = TierOneEngine(G, source, target, max_explored=max_explored)
    routes = engine.run()
    return routes, time.perf_counter() - t0, engine


def plot_metric_bars(rows: list[dict], out_dir: Path) -> None:
    methods = sorted({row["method"] for row in rows})
    scenarios = list(dict.fromkeys(row["scenario"] for row in rows))
    metrics = [
        ("runtime_s", "Runtime (s)", "benchmark_runtime.png"),
        ("front_size", "Front size", "benchmark_front_size.png"),
        ("hypervolume", "Hypervolume (higher better)", "benchmark_hv.png"),
        ("igd", "IGD (lower better)", "benchmark_igd.png"),
        ("spread_delta", "Spread delta (lower better)", "benchmark_spread.png"),
    ]
    out_dir.mkdir(parents=True, exist_ok=True)

    for metric, title, filename in metrics:
        fig, ax = plt.subplots(figsize=(10, 5.5))
        x = np.arange(len(scenarios))
        width = 0.8 / max(1, len(methods))
        for idx, method in enumerate(methods):
            vals = []
            for scenario in scenarios:
                match = next(
                    row for row in rows
                    if row["scenario"] == scenario and row["method"] == method
                )
                vals.append(match[metric])
            ax.bar(x + (idx - (len(methods) - 1) / 2) * width, vals, width, label=method)
        ax.set_title(title, fontweight="bold")
        ax.set_xticks(x)
        ax.set_xticklabels(scenarios, rotation=15, ha="right")
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out_dir / filename, dpi=150, bbox_inches="tight")
        plt.close(fig)


def run_benchmarks(args):
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    metric_rows = []
    tier2_rows = []
    sweep_rows = []
    risk_share_rows = []
    metadata = {"scenarios": [], "configuration": vars(args)}

    scenarios = SCENARIOS if args.scenario == "all" else [
        s for s in SCENARIOS if s["name"].startswith(args.scenario)
    ]
    if not scenarios:
        raise ValueError(f"No matching scenario for {args.scenario!r}")

    for scenario in scenarios:
        print(f"\n[Benchmark] {scenario['name']} | nodes={scenario['nodes']} density={scenario['density']}")
        G, source, target = build_scenario(scenario["nodes"], scenario["seed"], scenario["density"])
        method_routes = {}
        runtimes = {}
        diagnostics = {}

        namoa_routes, namoa_runtime, namoa_engine = run_namoa(
            G, source, target, max_explored=scenario["max_explored"]
        )
        method_routes["namoa"] = namoa_routes
        runtimes["namoa"] = namoa_runtime
        diagnostics["namoa"] = {
            "expanded": namoa_engine.expanded,
            "truncated": namoa_engine.truncated,
        }

        for objective in ("distance", "energy", "risk"):
            result = single_objective_astar(G, source, target, objective)
            method_routes[result.method] = result.routes
            runtimes[result.method] = result.runtime_s
            diagnostics[result.method] = {"expanded": "", "truncated": False}

        if namoa_routes:
            scales = objective_scales_from_extremes(G, source, target)
            selector = TierTwoSelector(namoa_routes)
            for preset_name, tags in PAYLOAD_PRESETS.items():
                wr, wd, we = selector._derive_amplified_weights(tags["F"], tags["W"], tags["U"], tags["P"])
                result = weighted_sum_astar(
                    G,
                    source,
                    target,
                    {"distance": wd, "energy": we, "risk": wr},
                    method_name=f"astar_weighted_{preset_name}",
                    scales=scales,
                )
                method_routes[result.method] = result.routes
                runtimes[result.method] = result.runtime_s
                diagnostics[result.method] = {"expanded": "", "truncated": False}

        mopso_runs = []
        mopso_runtime = 0.0
        for rep in range(args.mopso_repetitions):
            result = mopso_baseline(
                G,
                source,
                target,
                seed=scenario["seed"] + rep,
                swarm_size=args.swarm_size,
                iterations=args.iterations,
                archive_size=args.archive_size,
            )
            mopso_runs.extend(result.routes)
            mopso_runtime += result.runtime_s
        method_routes["mopso"] = mopso_runs
        runtimes["mopso"] = mopso_runtime / max(1, args.mopso_repetitions)
        diagnostics["mopso"] = {"expanded": "", "truncated": False}

        rows, ref_meta = evaluate_method_set(method_routes, runtimes)
        for row in rows:
            row["scenario"] = scenario["name"]
            row["nodes"] = scenario["nodes"]
            row["density"] = scenario["density"]
            row["expanded"] = diagnostics.get(row["method"], {}).get("expanded", "")
            row["truncated"] = diagnostics.get(row["method"], {}).get("truncated", False)
            metric_rows.append(row)

        if namoa_routes:
            for row in tier2_evidence(namoa_routes):
                row["scenario"] = scenario["name"]
                row["nodes"] = scenario["nodes"]
                row["density"] = scenario["density"]
                tier2_rows.append(row)
            for row in tier2_sweep(namoa_routes):
                row["scenario"] = scenario["name"]
                row["nodes"] = scenario["nodes"]
                row["density"] = scenario["density"]
                sweep_rows.append(row)

            selector = TierTwoSelector(namoa_routes)
            share_routes = [(f"{scenario['name']}:front_{idx}", route) for idx, route in enumerate(namoa_routes)]
            for preset_name, tags in PAYLOAD_PRESETS.items():
                selected, *_ = selector.select(tags["F"], tags["W"], tags["U"], tags["P"])
                share_routes.append((f"{scenario['name']}:preset_{preset_name}", selected))
            risk_share_rows.extend(risk_component_shares(G, share_routes))

        metadata["scenarios"].append({
            **scenario,
            "source": source,
            "target": target,
            "reference": ref_meta,
            "namoa_diagnostics": diagnostics["namoa"],
            "method_route_counts_raw": {k: len(v) for k, v in method_routes.items()},
        })

    write_csv(out_dir / "benchmark_metrics.csv", metric_rows)
    write_csv(out_dir / "tier2_evidence.csv", tier2_rows)
    write_csv(out_dir / "tier2_sweep.csv", sweep_rows)
    write_csv(out_dir / "risk_component_shares.csv", risk_share_rows)
    violations = monotonicity_violations(sweep_rows)
    write_csv(out_dir / "tier2_monotonicity_violations.csv", violations)
    write_json(out_dir / "benchmark_metrics.json", {
        "metadata": metadata,
        "metrics": metric_rows,
        "tier2_evidence": tier2_rows,
        "tier2_sweep_summary": {
            "rows": len(sweep_rows),
            "distinct_selected_points": len({(r["scenario"], r["selected_index"]) for r in sweep_rows}),
            "non_extreme_picks": sum(1 for r in sweep_rows if r["point_type"] == "non-extreme"),
            "monotonicity_violations": len(violations),
            "latency_ms_median": float(np.median([r["selection_latency_ms"] for r in sweep_rows])) if sweep_rows else 0.0,
            "latency_ms_p95": float(np.percentile([r["selection_latency_ms"] for r in sweep_rows], 95)) if sweep_rows else 0.0,
        },
    })
    plot_metric_bars(metric_rows, out_dir)

    print(f"\n[Benchmark] Wrote outputs -> {out_dir}")
    return metric_rows, tier2_rows


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="all", help="all, S1, S2, or S3")
    parser.add_argument("--mopso-repetitions", type=int, default=30)
    parser.add_argument("--swarm-size", type=int, default=100)
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--archive-size", type=int, default=100)
    parser.add_argument("--output-dir", default="outputs/benchmarks")
    parser.add_argument("--smoke", action="store_true", help="small deterministic settings for quick local checks")
    args = parser.parse_args()
    if args.smoke:
        args.mopso_repetitions = 1
        args.swarm_size = 12
        args.iterations = 10
        args.archive_size = 30
        if args.scenario == "all":
            args.scenario = "S1"
    return args


if __name__ == "__main__":
    run_benchmarks(parse_args())
