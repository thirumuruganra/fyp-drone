"""
plot_airspace_3d.py — 3-D view of the airspace
==============================================
Two styles:

  world (default)  Gazebo-like scene.  Terrain mesh from the DEM (interpolated to a
                   30 m display mesh), OSM building footprints extruded at their real
                   coordinates, OSM roads draped on the terrain.  Building colour =
                   where its height came from (grey tagged height, beige levels,
                   pink per-type default), same palette as the Gazebo world.
  cells            Terrain mesh + one bar per 200 m lattice cell (the tallest building
                   in the cell, `bldg_h_m`, which sets node altitude in the planner).

Optional in both: --flight-surface = translucent mesh at the planned flight altitude.

    python plot_airspace_3d.py --offline                       # 2 km window at the centre
    python plot_airspace_3d.py --offline --window-km 3 --azim -30
    python plot_airspace_3d.py --offline --window-km 6         # whole box: slow (~30k buildings)
    python plot_airspace_3d.py --offline --style cells --flight-surface
    python plot_airspace_3d.py --offline --show                # interactive window, rotate with mouse

Output: outputs/airspace_3d.png
"""

import argparse
import os
import sys
import time

import numpy as np
import matplotlib

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as cfg
from env_data import (FIXTURE_DEM, LocalFrame, LocalRasterProvider, _first_number, _local_rows,
                      build_layer_indexes, building_height_m, default_elevation_providers,
                      fetch_osm_layers, fetch_osm_roads)
from module1_graph_builder import LatticeAirspaceBuilder

FIXTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests", "fixtures")

# Gazebo palette (Simulation/gazebo_world_builder.py, sim_config.py)
SKY_RGB = (0.62, 0.78, 0.94)
TERRAIN_RGB = (0.55, 0.62, 0.45)
ROAD_RGB = (0.25, 0.25, 0.27)
HEIGHT_SOURCE_RGB = {"tagged": (0.62, 0.62, 0.66), "levels": (0.86, 0.78, 0.62), "default": (0.90, 0.60, 0.58)}
ROAD_WIDTH_M = {"motorway": 14.0, "trunk": 12.0, "primary": 10.0, "secondary": 8.0,
                "tertiary": 7.0, "residential": 5.0}
ROAD_WIDTH_DEFAULT_M = 4.0
LIGHT = np.array([-0.45, -0.55, 0.70])
LIGHT /= np.linalg.norm(LIGHT)


# ── data ─────────────────────────────────────────────────────────────────────

def load_layers(args):
    if args.no_buildings:
        return {}
    return fetch_osm_layers(args.lat, args.lon, offline=args.offline,
                            fixture_dir=FIXTURE_DIR if args.offline else None)


def build_graph(args, layers):
    center = (args.lat, args.lon)
    providers = None
    if not args.no_terrain:
        providers = default_elevation_providers()
        if args.offline:
            providers.append(LocalRasterProvider(os.path.join(FIXTURE_DIR, FIXTURE_DEM)))
    layer_kwargs = {}
    if layers:
        layer_kwargs = build_layer_indexes(layers, LocalFrame(*center), buildings=True, risk_layers=False)
    builder = LatticeAirspaceBuilder(
        center=center, box_m=args.box_km * 1000.0, spacing_m=args.spacing_m, seed=42,
        offline=args.offline, wind_override=(0.0, 0.0) if args.offline else None,
        elevation_providers=providers, **layer_kwargs)
    return builder.fetch_graph()


def lattice_arrays(G):
    """Node attributes → (n × n) grids indexed [row, col]; blocked/removed nodes = NaN."""
    n = int(round(np.sqrt(max(G.nodes) + 1)))
    names = ("ground_elev_m", "bldg_h_m", "alt_amsl_m")
    grids = {k: np.full((n, n), np.nan) for k in names}
    for node, d in G.nodes(data=True):
        r, c = divmod(node, n)
        for k in names:
            grids[k][r, c] = d[k]
    spacing = G.graph["spacing_km"] * 1000.0
    half = (n - 1) / 2.0
    cols, rows = np.meshgrid(np.arange(n), np.arange(n))
    grids["x_m"] = (cols - half) * spacing         # blocked nodes have no attributes: regular lattice
    grids["y_m"] = (rows - half) * spacing
    return grids, spacing


def ground_interpolator(grids):
    """Bilinear ground height (m AMSL) at local (x, y); blocked nodes take the nearest valid one."""
    from scipy.interpolate import RegularGridInterpolator
    from scipy.ndimage import distance_transform_edt
    z = grids["ground_elev_m"].copy()
    bad = np.isnan(z)
    if bad.any():
        idx = distance_transform_edt(bad, return_distances=False, return_indices=True)
        z = z[tuple(idx)]
    xs, ys = grids["x_m"][0], grids["y_m"][:, 0]
    f = RegularGridInterpolator((ys, xs), z, bounds_error=False, fill_value=None)
    return lambda x, y: f(np.column_stack([np.asarray(y, float).ravel(), np.asarray(x, float).ravel()])
                          ).reshape(np.shape(x))


def building_footprints(layers, frame, half_m):
    """[(shapely Polygon in local m, height m, height source)] with centroid inside the window."""
    out = []
    gdf = layers.get("buildings")
    for geom, row in _local_rows(gdf, frame, ("Polygon", "MultiPolygon")):
        c = geom.centroid
        if abs(c.x) > half_m or abs(c.y) > half_m:
            continue
        h = building_height_m(row)
        tagged = _first_number(row.get("height"))
        levels = _first_number(row.get("building:levels"))
        src = "tagged" if tagged and tagged > 0 else "levels" if levels and levels > 0 else "default"
        for poly in getattr(geom, "geoms", [geom]):
            out.append((poly.simplify(0.4), h, src))
    return out


def road_lines(args, frame, half_m):
    """[(N×2 local-metre vertex array, width m)] clipped to the window."""
    import shapely
    from shapely.geometry import box
    roads = fetch_osm_roads(args.lat, args.lon, offline=args.offline)
    window = box(-half_m, -half_m, half_m, half_m)
    out = []
    for geom, (_, row) in zip(roads.geometry, roads.iterrows()):
        if geom is None or geom.is_empty:
            continue
        local = shapely.transform(geom, lambda xy: np.column_stack(frame.to_local(xy[:, 1], xy[:, 0])))
        local = shapely.segmentize(local.intersection(window), 30.0)
        hw = str(row.get("highway") or "").split("_")[0]
        width = ROAD_WIDTH_M.get(hw, ROAD_WIDTH_DEFAULT_M)
        for part in getattr(local, "geoms", [local]):
            if part.geom_type == "LineString" and len(part.coords) > 1:
                out.append((np.asarray(part.coords), width))
    return out


# ── world style ──────────────────────────────────────────────────────────────

def _building_polys(footprints, ground):
    """Roof + wall quads (world metres) with pre-lit colours."""
    from shapely.geometry.polygon import orient
    verts, colours = [], []
    for poly, h, src in footprints:
        poly = orient(poly, 1.0)                                  # exterior CCW → outward normal = (dy, -dx)
        ring = np.asarray(poly.exterior.coords)
        z0 = float(ground(ring[:-1, 0].mean(), ring[:-1, 1].mean()))
        z1 = z0 + h
        base = np.array(HEIGHT_SOURCE_RGB[src])
        verts.append(np.column_stack([ring[:-1], np.full(len(ring) - 1, z1)]))
        colours.append(base * (0.62 + 0.38 * LIGHT[2]))           # roof
        d = np.diff(ring, axis=0)
        for (x0, y0), (x1, y1), (dx, dy) in zip(ring[:-1], ring[1:], d):
            ln = np.hypot(dx, dy)
            if ln < 1e-6:
                continue
            nrm = np.array([dy, -dx, 0.0]) / ln
            verts.append([(x0, y0, z0), (x1, y1, z0), (x1, y1, z1), (x0, y0, z1)])
            colours.append(base * (0.45 + 0.55 * max(0.0, float(nrm @ LIGHT))))
    return verts, np.clip(colours, 0, 1)


def plot_world(G, args, layers):
    grids, _ = lattice_arrays(G)
    ground = ground_interpolator(grids)
    frame = LocalFrame(args.lat, args.lon)
    win = min(args.window_km, args.box_km) * 1000.0
    half = win / 2.0

    fig = plt.figure(figsize=(14, 11), facecolor=SKY_RGB)
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    ax.set_facecolor(SKY_RGB)
    ax.set_axis_off()

    # terrain: 30 m display mesh
    n = int(np.ceil(win / args.terrain_res_m)) + 1
    gx = np.linspace(-half, half, n)
    X, Y = np.meshgrid(gx, gx)
    Z = ground(X, Y)
    ax.plot_surface(X, Y, Z, color=TERRAIN_RGB, shade=True, lightsource=matplotlib.colors.LightSource(
        azdeg=300, altdeg=45), linewidth=0, antialiased=False, rstride=1, cstride=1, zorder=1)
    zmax = float(Z.max())

    if not args.no_roads:
        segs, widths = [], []
        for pts, w in road_lines(args, frame, half):
            z = ground(pts[:, 0], pts[:, 1]) + 0.15
            segs.append(np.column_stack([pts, z]))
            widths.append(w)
        if segs:
            pts_per_m = 8.0 * 72 / win                           # ~8 in of plot width → points per metre
            ax.add_collection3d(Line3DCollection(segs, colors=[ROAD_RGB], linewidths=np.array(widths) * pts_per_m,
                                                 capstyle="butt", zorder=2), autolim=False)
        print(f"[3D] {len(segs)} road polylines")

    if layers:
        fps = building_footprints(layers, frame, half)
        if fps:
            t0 = time.time()
            verts, colours = _building_polys(fps, ground)
            ax.add_collection3d(Poly3DCollection(verts, facecolors=colours, edgecolors=colours * 0.7,
                                                 linewidths=0.15, zorder=3), autolim=False)
            zmax = max(zmax, float(Z.min()) + max(h for _, h, _ in fps))
            print(f"[3D] {len(fps)} buildings, {len(verts)} faces ({time.time() - t0:.1f}s)")

    if args.flight_surface:
        flat = ax.plot_surface(grids["x_m"], grids["y_m"], grids["alt_amsl_m"], color="tab:blue",
                               alpha=0.15, linewidth=0, shade=False, zorder=4)
        zmax = max(zmax, float(np.nanmax(grids["alt_amsl_m"])))

    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    zmin = float(Z.min())
    ax.set_zlim(zmin, zmax)
    aspect_z = max(args.z_exag * (zmax - zmin) / win, 0.06)
    print(f"[3D] vertical exaggeration x{aspect_z * win / max(zmax - zmin, 1e-9):.1f}")
    ax.set_box_aspect((1, 1, aspect_z), zoom=args.zoom)
    ax.view_init(elev=args.elev, azim=args.azim)
    title = (f"Airspace world — OSM buildings + roads on DEM terrain ({win / 1000:g} km window, "
             f"{args.lat:.4f}, {args.lon:.4f})")
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    return fig, title


# ── cells style (one bar per lattice cell) ───────────────────────────────────

def plot_cells(G, args):
    g, spacing = lattice_arrays(G)
    X, Y = g["x_m"] / 1000.0, g["y_m"] / 1000.0
    ground, bldg, flight = g["ground_elev_m"], g["bldg_h_m"], g["alt_amsl_m"]

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)   # explicit draw order: bars over terrain

    surf = ax.plot_surface(X, Y, ground, cmap="terrain", edgecolor=(0, 0, 0, 0.25),
                           linewidth=0.3, alpha=0.9, antialiased=True, rstride=1, cstride=1, zorder=1)
    fig.colorbar(surf, cax=fig.add_axes([0.52, 0.10, 0.28, 0.018]), orientation="horizontal",
                 label="Ground elevation (m AMSL)")

    if args.flight_surface:
        ax.plot_surface(X, Y, flight, color="tab:blue", alpha=0.18, linewidth=0, shade=False, zorder=3)

    mask = np.nan_to_num(bldg) > 0.5
    if mask.any():
        frac = 0.8
        bx = X[mask] - frac * spacing / 2000.0
        by = Y[mask] - frac * spacing / 2000.0
        bz = ground[mask]
        bh = bldg[mask]
        w = frac * spacing / 1000.0
        norm = Normalize(vmin=0, vmax=max(bh.max(), 1.0))
        ax.bar3d(bx, by, bz, w, w, bh, color=cm.autumn_r(norm(bh)), shade=True,
                 edgecolor=(0, 0, 0, 0.3), linewidth=0.2, zorder=2)
        mappable = cm.ScalarMappable(norm=norm, cmap="autumn_r")
        fig.colorbar(mappable, cax=fig.add_axes([0.20, 0.10, 0.28, 0.018]), orientation="horizontal",
                     label="Building height (m)")

    ax.set_xlabel("East (km)")
    ax.set_ylabel("North (km)")
    ax.set_zlabel("Height (m AMSL)")
    zmax = np.nanmax(np.where(mask, ground + np.nan_to_num(bldg), ground))
    zmax = max(zmax, np.nanmax(flight) if args.flight_surface else 0.0)
    ax.set_zlim(np.nanmin(ground), zmax)
    ax.set_box_aspect((1, 1, args.z_ratio))                      # true-scale z is flat: z = z_ratio × plan width
    ax.view_init(elev=args.elev, azim=args.azim)
    title = (f"Airspace lattice — terrain mesh + buildings "
             f"({args.box_km:g} km box, {args.spacing_m:g} m spacing, {mask.sum()} occupied cells)")
    fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.18)
    return fig, title


def finish(fig, dpi, title=None, margin=0.05):
    """Crop the render to its content with equal margins on all sides, so the scene is centred."""
    fig.set_dpi(dpi)
    fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[..., :3].astype(int)
    bg = np.array(matplotlib.colors.to_rgb(fig.get_facecolor())) * 255
    ys, xs = np.where(np.abs(img - bg).max(axis=2) > 6)
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    pad = int(margin * max(x1 - x0, y1 - y0))
    img = np.pad(img, ((pad, pad), (pad, pad), (0, 0)), constant_values=0)
    img[:pad], img[-pad:], img[:, :pad], img[:, -pad:] = bg, bg, bg, bg     # padding in background colour
    crop = img[y0:y1 + 2 * pad, x0:x1 + 2 * pad].astype(np.uint8)
    h, w = crop.shape[:2]
    head = int(0.05 * h) if title else 0
    out = plt.figure(figsize=(w / dpi, (h + head) / dpi), dpi=dpi, facecolor=fig.get_facecolor())
    ax = out.add_axes([0, 0, 1, h / (h + head)])
    ax.imshow(crop, aspect="auto", interpolation="none")
    ax.axis("off")
    if title:
        out.text(0.5, 1 - head / (2 * (h + head)), title, ha="center", va="center", fontsize=11)
    plt.close(fig)
    return out


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--style", choices=("world", "cells"), default="world")
    p.add_argument("--lat", type=float, default=cfg.CENTER_LAT)
    p.add_argument("--lon", type=float, default=cfg.CENTER_LON)
    p.add_argument("--box-km", type=float, default=cfg.BOX_M / 1000.0)
    p.add_argument("--spacing-m", type=float, default=cfg.LATTICE_SPACING_M)
    p.add_argument("--offline", action="store_true", help="cached OSM/DEM, else tests/fixtures")
    p.add_argument("--no-terrain", action="store_true", help="flat ground")
    p.add_argument("--no-buildings", action="store_true")
    p.add_argument("--no-roads", action="store_true")
    p.add_argument("--flight-surface", action="store_true", help="also draw planned flight altitude mesh")
    p.add_argument("--window-km", type=float, default=2.0, help="world style: square window around the centre")
    p.add_argument("--terrain-res-m", type=float, default=30.0, help="world style: terrain display mesh")
    p.add_argument("--z-exag", type=float, default=2.0, help="world style: vertical exaggeration (1 = true scale)")
    p.add_argument("--zoom", type=float, default=1.0, help="world style: scene zoom")
    p.add_argument("--z-ratio", type=float, default=0.45, help="cells style: z axis length / plan width")
    p.add_argument("--elev", type=float, default=32.0)
    p.add_argument("--azim", type=float, default=-55.0)
    p.add_argument("--dpi", type=int, default=200)
    p.add_argument("--show", action="store_true", help="open interactive window")
    p.add_argument("--out", default=os.path.join("outputs", "airspace_3d.png"))
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    layers = load_layers(args)
    G = build_graph(args, layers)
    fig, title = plot_world(G, args, layers) if args.style == "world" else plot_cells(G, args)
    fig = finish(fig, args.dpi, title)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, facecolor=fig.get_facecolor())
    print(f"[3D] saved {args.out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
