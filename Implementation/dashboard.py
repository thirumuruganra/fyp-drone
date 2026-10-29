"""
Minimal operator dashboard for Phase 7.

Tkinter chosen per D5 suggested default: no new web dependency, one desktop file.
Loads stored Tier 1 artifacts, selects a Tier 2 route from F/W/U/P tags, shows a
route map, and exports a QGC WPL mission file.
"""

from __future__ import annotations

from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import networkx as nx

from mission_export import export_qgc_wpl
from module1_graph_builder import AirspaceGraphBuilder
from module2_tier1_moastar import TierOneEngine
from module4_tier2_payload import SelectionResult, TierTwoSelector, validate_payload_tags


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_GRAPH_PATH = BASE_DIR / "pareto_front_storage" / "airspace_graph.pkl"
DEFAULT_FRONT_PATH = BASE_DIR / "pareto_front_storage" / "pareto_front.pkl"
DEFAULT_EXPORT_PATH = BASE_DIR / "outputs" / "missions" / "selected_route.waypoints"


def load_dashboard_artifacts(
    graph_path: str | Path = DEFAULT_GRAPH_PATH,
    front_path: str | Path = DEFAULT_FRONT_PATH,
):
    graph = AirspaceGraphBuilder.load(str(graph_path))
    front = TierOneEngine.load(str(front_path))
    return graph, front


def select_payload_route(
    G,
    pareto_front,
    F: int,
    W: int,
    U: int,
    P: int,
    *,
    write_outputs: bool = True,
) -> SelectionResult:
    validate_payload_tags(F, W, U, P)
    selector = TierTwoSelector(pareto_front, no_go_reasons=G.graph.get("no_go_reasons"))
    return selector.select_detailed(
        F,
        W,
        U,
        P,
        graph=G,
        output_name="dashboard",
        write_outputs=write_outputs,
    )


def route_node_ids(result: SelectionResult) -> list:
    return [wp["node"] for wp in result.path if wp.get("node") is not None]


def build_route_figure(G: nx.DiGraph, result: SelectionResult | None = None) -> Figure:
    fig = Figure(figsize=(6.5, 5.0), dpi=100)
    ax = fig.add_subplot(111)
    ax.set_title("Selected UAV Route")
    ax.axis("off")
    pos = {n: (data.get("x", data.get("lon", 0.0)), data.get("y", data.get("lat", 0.0))) for n, data in G.nodes(data=True)}
    nx.draw_networkx_edges(G, pos, ax=ax, edge_color="#D0D0D0", arrows=False, width=0.6, alpha=0.7)
    nx.draw_networkx_nodes(G, pos, ax=ax, node_size=12, node_color="#AAB7C4", alpha=0.9)
    if result and not result.hold_advisory:
        nodes = route_node_ids(result)
        if len(nodes) > 1:
            nx.draw_networkx_edges(
                G,
                pos,
                edgelist=list(zip(nodes[:-1], nodes[1:])),
                ax=ax,
                edge_color="#E67E22",
                arrows=True,
                arrowsize=12,
                width=2.8,
            )
            nx.draw_networkx_nodes(G, pos, nodelist=[nodes[0]], ax=ax, node_color="#27AE60", node_size=80)
            nx.draw_networkx_nodes(G, pos, nodelist=[nodes[-1]], ax=ax, node_color="#C0392B", node_size=80)
    fig.tight_layout()
    return fig


class DashboardApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("FYP B28 Payload Route Dashboard")
        self.G = None
        self.front = None
        self.result: SelectionResult | None = None
        self.canvas: FigureCanvasTkAgg | None = None

        self.status_var = tk.StringVar(value="Load graph/front, enter tags, select route.")
        self.tag_vars = {
            "F": tk.IntVar(value=3),
            "W": tk.IntVar(value=1),
            "U": tk.IntVar(value=2),
            "P": tk.IntVar(value=3),
        }

        self._build_ui()
        self._load_default_artifacts()

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        controls = ttk.Frame(frame)
        controls.grid(row=0, column=0, sticky="ew")

        ranges = {"F": (1, 5), "W": (1, 3), "U": (1, 3), "P": (1, 5)}
        for col, (name, (lo, hi)) in enumerate(ranges.items()):
            ttk.Label(controls, text=name).grid(row=0, column=col * 2, sticky="w")
            ttk.Spinbox(controls, from_=lo, to=hi, textvariable=self.tag_vars[name], width=4).grid(
                row=0, column=col * 2 + 1, padx=(2, 10)
            )

        ttk.Button(controls, text="Select Route", command=self.select_route).grid(row=0, column=8, padx=4)
        ttk.Button(controls, text="Export Mission", command=self.export_mission).grid(row=0, column=9, padx=4)

        self.plot_holder = ttk.Frame(frame)
        self.plot_holder.grid(row=1, column=0, sticky="nsew", pady=10)
        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)

        ttk.Label(frame, textvariable=self.status_var, foreground="#2C3E50").grid(row=2, column=0, sticky="ew")

    def _load_default_artifacts(self) -> None:
        try:
            self.G, self.front = load_dashboard_artifacts()
            self.status_var.set(f"Loaded graph {self.G.number_of_nodes()} nodes, front {len(self.front)} routes.")
            self._draw(build_route_figure(self.G))
        except Exception as exc:
            self.status_var.set(f"Artifact load failed: {exc}")

    def _draw(self, fig: Figure) -> None:
        if self.canvas is not None:
            self.canvas.get_tk_widget().destroy()
        self.canvas = FigureCanvasTkAgg(fig, master=self.plot_holder)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def select_route(self) -> None:
        if self.G is None or self.front is None:
            messagebox.showerror("Artifacts missing", "Graph/front artifacts are not loaded.")
            return
        try:
            tags = {name: var.get() for name, var in self.tag_vars.items()}
            self.result = select_payload_route(self.G, self.front, tags["F"], tags["W"], tags["U"], tags["P"])
            if self.result.hold_advisory:
                self.status_var.set(self.result.hold_advisory)
            else:
                self.status_var.set(
                    f"Selected route {self.result.selected_index}; "
                    f"cost={self.result.cost_vector}; feasible={self.result.feasible_count}"
                )
            self._draw(build_route_figure(self.G, self.result))
        except Exception as exc:
            self.result = None
            self.status_var.set(f"Selection failed: {exc}")
            messagebox.showerror("Selection failed", str(exc))

    def export_mission(self) -> None:
        if self.result is None:
            messagebox.showerror("No route", "Select a route before exporting.")
            return
        try:
            path = filedialog.asksaveasfilename(
                initialfile=DEFAULT_EXPORT_PATH.name,
                defaultextension=".waypoints",
                filetypes=[("QGC waypoint files", "*.waypoints"), ("All files", "*.*")],
            )
            if not path:
                return
            out = export_qgc_wpl(self.result, path)
            self.status_var.set(f"Mission exported: {out}")
        except Exception as exc:
            self.status_var.set(f"Export failed: {exc}")
            messagebox.showerror("Export failed", str(exc))


def main() -> int:
    root = tk.Tk()
    DashboardApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
