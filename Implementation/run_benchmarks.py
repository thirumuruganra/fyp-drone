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
from module3_baselines import mopso_baseline, single_objective_astar
from module5_evaluation import (
    evaluate_method_set,
    tier2_evidence,
    write_csv,
    write_json,
)


SCENARIOS = [
    {"name": "S1_50_sparse", "nodes": 50, "seed": 101, "max_explored": 80_000},
    {"name": "S2_200_medium", "nodes": 200, "seed": 202, "max_explored": 180_000},
    {"name": "S3_500_dense", "nodes": 500, "seed": 303, "max_explored": 260_000},
]


def build_scenario(n_nodes: int, seed: int):
    builder = AirspaceGraphBuilder(seed=seed)
    G = builder._build_synthetic_graph(n_nodes=n_nodes)
    builder.G = G
    nodes = list(G.nodes())
    return G, nodes[0], nodes[-1]


def run_namoa(G, source, target, max_explored: int):
    t0 = time.perf_counter()
    engine = TierOneEngine(G, source, target, max_explored=max_explored)
    routes = engine.run()
    return routes, time.perf_counter() - t0, engine.diagnostics()


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
    metadata = {"scenarios": [], "configuration": vars(args)}

    scenarios = SCENARIOS if args.scenario == "all" else [
        s for s in SCENARIOS if s["name"].startswith(args.scenario)
    ]
    if not scenarios:
        raise ValueError(f"No matching scenario for {args.scenario!r}")

    for scenario in scenarios:
        print(f"\n[Benchmark] {scenario['name']} | nodes={scenario['nodes']}")
        G, source, target = build_scenario(scenario["nodes"], scenario["seed"])
        method_routes = {}
        runtimes = {}

        namoa_routes, namoa_runtime, namoa_diag = run_namoa(
            G, source, target, max_explored=scenario["max_explored"]
        )
        method_routes["namoa"] = namoa_routes
        runtimes["namoa"] = namoa_runtime

        for objective in ("distance", "energy", "risk"):
            result = single_objective_astar(G, source, target, objective)
            method_routes[result.method] = result.routes
            runtimes[result.method] = result.runtime_s

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

        rows, ref_meta = evaluate_method_set(method_routes, runtimes)
        for row in rows:
            row["scenario"] = scenario["name"]
            row["nodes"] = scenario["nodes"]
            metric_rows.append(row)

        if namoa_routes:
            for row in tier2_evidence(namoa_routes):
                row["scenario"] = scenario["name"]
                row["nodes"] = scenario["nodes"]
                tier2_rows.append(row)

        metadata["scenarios"].append({
            **scenario,
            "source": source,
            "target": target,
            "namoa": namoa_diag,
            "reference": ref_meta,
            "method_route_counts_raw": {k: len(v) for k, v in method_routes.items()},
        })

    write_csv(out_dir / "benchmark_metrics.csv", metric_rows)
    write_csv(out_dir / "tier2_evidence.csv", tier2_rows)
    write_json(out_dir / "benchmark_metrics.json", {
        "metadata": metadata,
        "metrics": metric_rows,
        "tier2_evidence": tier2_rows,
    })
    plot_metric_bars(metric_rows, out_dir)

    print(f"\n[Benchmark] Wrote outputs -> {out_dir}")
    return metric_rows, tier2_rows


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="all", help="all, S1, S2, or S3")
    parser.add_argument("--mopso-repetitions", type=int, default=2)
    parser.add_argument("--swarm-size", type=int, default=24)
    parser.add_argument("--iterations", type=int, default=30)
    parser.add_argument("--archive-size", type=int, default=60)
    parser.add_argument("--output-dir", default="outputs/benchmarks")
    return parser.parse_args()


if __name__ == "__main__":
    run_benchmarks(parse_args())
