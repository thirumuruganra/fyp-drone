"""
Phase 2: Gazebo (Harmonic+) city world from a region bundle.

    python gazebo_world_builder.py thiru_demo            # → worlds/thiru_demo/

Output (git-ignored):

    worlds/<id>/<world_name>.sdf          world: ArduPilot iris + systems + pads
    worlds/<id>/models/<id>_terrain/       static terrain mesh (visual = collision)
    worlds/<id>/models/<id>_bldg_<i>_<j>/  building tiles, one link per height source
    worlds/<id>/models/<id>_roads/         flat road ribbons (visual only)
    worlds/<id>/gui.config                GUI layout + camera following the vehicle
    worlds/<id>/sitl_command.txt          matching sim_vehicle.py line
    worlds/<id>/world_manifest.json       bundle hash, counts, extents

Every height comes from the bundle: terrain z = DEM − H_home; building walls run
from base_amsl_m to roof_amsl_m (the same roof the planner clearance check uses).

route_markers_sdf() renders Tier 1 front routes (thin) and the Tier 2 pick (thick)
as a visual-only model, spawned by gazebo/launch_region.sh.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import _paths
import sim_config as scfg
from sim_region import RegionBundle, sample_grid

TERRAIN_RGBA = (0.55, 0.62, 0.45, 1.0)
ROAD_RGBA = (0.25, 0.25, 0.27, 1.0)
PAD_RGBA = {"depot": (0.15, 0.45, 0.95, 1.0), "dest": (1.0, 0.55, 0.05, 1.0)}
SELECTED_RGBA = (0.95, 0.10, 0.10, 0.95)
FRONT_RGBA = (0.20, 0.55, 1.00, 0.45)


def world_name(region_id: str) -> str:
    return f"fyp_{region_id}"


def _rgba(c) -> str:
    return " ".join(f"{v:.3f}" for v in c)


def world_extent(bundle: RegionBundle):
    """(xmin, xmax, ymin, ymax) of the bundle bbox in the world (depot) frame."""
    bb = bundle.manifest["bbox_latlon"]
    xs, ys = bundle.to_world([bb["south"], bb["north"], bb["south"], bb["north"]],
                             [bb["west"], bb["west"], bb["east"], bb["east"]])
    return float(np.min(xs)), float(np.max(xs)), float(np.min(ys)), float(np.max(ys))


def world_ground_z(bundle: RegionBundle, xs, ys) -> np.ndarray:
    lat, lon = bundle.world_frame.to_latlon(np.asarray(xs, float), np.asarray(ys, float))
    return sample_grid(bundle.terrain, lat, lon) - bundle.h_home


# ── meshes ───────────────────────────────────────────────────────────────────

def terrain_mesh(bundle: RegionBundle, res_m: float = scfg.SIM_TERRAIN_RES_M):
    """Regular triangulated terrain over the bundle extent (world frame)."""
    import trimesh
    xmin, xmax, ymin, ymax = world_extent(bundle)
    nx_ = int(math.ceil((xmax - xmin) / res_m)) + 1
    ny_ = int(math.ceil((ymax - ymin) / res_m)) + 1
    xs = np.linspace(xmin, xmax, nx_)
    ys = np.linspace(ymin, ymax, ny_)
    X, Y = np.meshgrid(xs, ys)                    # row = y index, col = x index
    Z = world_ground_z(bundle, X.ravel(), Y.ravel()).reshape(X.shape)
    verts = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    idx = np.arange(nx_ * ny_).reshape(ny_, nx_)
    a, b = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel()
    c, d = idx[1:, :-1].ravel(), idx[1:, 1:].ravel()
    faces = np.concatenate([np.column_stack([a, b, d]), np.column_stack([a, d, c])])   # CCW from above
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)


def building_meshes(bundle: RegionBundle, tile_m: float = scfg.SIM_TILE_M):
    """{(i, j, height_source): Trimesh} of LoD1 extrusions, plus build stats."""
    import trimesh
    xmin, _, ymin, _ = world_extent(bundle)
    parts: dict = {}
    stats = {"extruded": 0, "failed": 0, "max_roof_z": -math.inf}
    for poly, row in bundle.local_footprints(bundle.world_frame):
        base_z = float(row["base_amsl_m"]) - bundle.h_home
        height = float(row["roof_amsl_m"]) - float(row["base_amsl_m"])
        if height <= 0:
            stats["failed"] += 1
            continue
        try:
            m = trimesh.creation.extrude_polygon(poly.buffer(0), height)
        except Exception:
            stats["failed"] += 1
            continue
        m.apply_translation([0.0, 0.0, base_z])
        c = poly.centroid
        key = (int((c.x - xmin) // tile_m), int((c.y - ymin) // tile_m), str(row["height_source"]))
        parts.setdefault(key, []).append(m)
        stats["extruded"] += 1
        stats["max_roof_z"] = max(stats["max_roof_z"], base_z + height)
    return {k: trimesh.util.concatenate(v) for k, v in parts.items()}, stats


def road_mesh(bundle: RegionBundle, tile_m: float = scfg.SIM_TILE_M, lift_m: float = 0.08):
    """Flat road ribbons draped per tile piece (visual only); None when no roads."""
    import shapely
    import trimesh
    from shapely.geometry import box
    from shapely.ops import unary_union
    roads = bundle.layer("roads")
    if len(roads) == 0:
        return None
    frame = bundle.world_frame
    ribbons = []
    for geom, (_, row) in zip(roads.geometry, roads.iterrows()):
        if geom is None or geom.is_empty:
            continue
        local = shapely.transform(geom, lambda xy: np.column_stack(frame.to_local(xy[:, 1], xy[:, 0])))
        hw = str(row.get("highway") or "").split("_")[0]
        width = scfg.ROAD_WIDTH_M.get(hw, scfg.ROAD_WIDTH_DEFAULT_M)
        ribbons.append(local.buffer(width / 2.0, cap_style="flat"))
    if not ribbons:
        return None
    merged = unary_union(ribbons)
    xmin, xmax, ymin, ymax = world_extent(bundle)
    meshes = []
    for x0 in np.arange(xmin, xmax, tile_m):
        for y0 in np.arange(ymin, ymax, tile_m):
            piece = merged.intersection(box(x0, y0, x0 + tile_m, y0 + tile_m))
            for poly in getattr(piece, "geoms", [piece]):
                if poly.is_empty or poly.geom_type != "Polygon" or poly.area < 1.0:
                    continue
                ext = np.asarray(poly.exterior.coords)
                z = float(np.max(world_ground_z(bundle, ext[:, 0], ext[:, 1]))) + lift_m
                try:
                    m = trimesh.creation.extrude_polygon(poly, 0.02)
                except Exception:
                    continue
                m.apply_translation([0.0, 0.0, z])
                meshes.append(m)
    return trimesh.util.concatenate(meshes) if meshes else None


# ── SDF writers ──────────────────────────────────────────────────────────────

def _model_config(name: str, description: str) -> str:
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
            f'  <sdf version="1.9">model.sdf</sdf>\n  <description>{description}</description>\n</model>\n')


def _mesh_link(link: str, mesh_rel: str, rgba, collision: bool) -> str:
    col = (f'      <collision name="collision"><geometry><mesh><uri>{mesh_rel}</uri></mesh></geometry></collision>\n'
           if collision else "")
    return (f'    <link name="{link}">\n'
            f'      <visual name="visual"><cast_shadows>false</cast_shadows>'
            f'<geometry><mesh><uri>{mesh_rel}</uri></mesh></geometry>\n'
            f'        <material><ambient>{_rgba(rgba)}</ambient><diffuse>{_rgba(rgba)}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>\n      </visual>\n{col}    </link>\n')


def write_mesh_model(models_dir: Path, name: str, links: Sequence[tuple], description: str) -> Path:
    """links: (link name, Trimesh, rgba, collision?[, flat_shading?]).  Meshes saved as OBJ under meshes/."""
    mdir = models_dir / name
    (mdir / "meshes").mkdir(parents=True, exist_ok=True)
    body = []
    for link, mesh, rgba, collision, *flat in links:
        fname = f"{link}.obj"
        if flat and flat[0]:
            mesh = mesh.copy()
            mesh.unmerge_vertices()       # one vertex per face corner → flat-shaded walls/roofs
        mesh.export(str(mdir / "meshes" / fname), file_type="obj", include_normals=True)
        body.append(_mesh_link(link, f"model://{name}/meshes/{fname}", rgba, collision))
    sdf = (f'<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="{name}">\n    <static>true</static>\n'
           + "".join(body) + "  </model>\n</sdf>\n")
    (mdir / "model.sdf").write_text(sdf, encoding="utf-8")
    (mdir / "model.config").write_text(_model_config(name, description), encoding="utf-8")
    return mdir


def sitl_command(bundle: RegionBundle) -> str:
    lat, lon = bundle.depot
    # ArduPilot >= 4.6 dev: the SITL binary picks per-frame defaults from --model, and --model JSON
    # matches none, so FRAME_CLASS stays 0 ("Arm: Motors: Check frame class and type"). Layer them back.
    pdir = "${FYP_ARDUPILOT_DIR:-$HOME/ardupilot}/Tools/autotest/default_params"
    # Project overrides last (precision landing off: no IR-LOCK beacon in the city world).
    return (f"sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console "
            f"--add-param-file={pdir}/copter.parm --add-param-file={pdir}/gazebo-iris.parm "
            f"--add-param-file={_paths.SITL_OVERRIDES} "
            f"--custom-location={lat:.7f},{lon:.7f},{bundle.h_home:.3f},0")


def build_world(region_id: str, *, regions_root: Optional[Path] = None, worlds_root: Optional[Path] = None,
                terrain_res_m: float = scfg.SIM_TERRAIN_RES_M, tile_m: float = scfg.SIM_TILE_M,
                vehicle_model: str = scfg.SIM_VEHICLE_MODEL, roads: bool = True, sky: bool = True,
                contact_system: bool = False, gui_view: str = "follow") -> Path:
    import jinja2
    offset = scfg.SIM_CAMERA_VIEWS[gui_view]
    follow = gui_view != "takeoff"      # takeoff: fixed pad shot; camera_director.py switches to follow
    # Start the GUI camera at the follow offset from the depot (world origin), looking at it.
    camera_pose = (*offset, 0.0, math.atan2(offset[2], math.hypot(*offset[:2])),
                   math.atan2(-offset[1], -offset[0]))
    bundle = RegionBundle.load(region_id, regions_root)
    out = Path(worlds_root or _paths.WORLDS_DIR) / region_id
    if out.exists():
        shutil.rmtree(out)
    models_dir = out / "models"
    models_dir.mkdir(parents=True)

    models = []
    terr = terrain_mesh(bundle, terrain_res_m)
    name = f"{region_id}_terrain"
    write_mesh_model(models_dir, name, [("terrain", terr, TERRAIN_RGBA, True)], "terrain from bundle DEM")
    models.append(name)

    tiles, bstats = building_meshes(bundle, tile_m)
    by_tile: dict = {}
    for (i, j, src), mesh in tiles.items():
        by_tile.setdefault((i, j), []).append(
            (f"bldg_{src}", mesh, scfg.HEIGHT_SOURCE_COLOURS.get(src, (0.7, 0.7, 0.7, 1.0)), True, True))
    for (i, j), links in sorted(by_tile.items()):
        name = f"{region_id}_bldg_{i}_{j}"
        write_mesh_model(models_dir, name, links, "OSM building tile (LoD1 extrusion)")
        models.append(name)

    n_road_tris = 0
    if roads:
        rm = road_mesh(bundle, tile_m)
        if rm is not None:
            name = f"{region_id}_roads"
            write_mesh_model(models_dir, name, [("roads", rm, ROAD_RGBA, False, True)], "OSM roads (visual only)")
            models.append(name)
            n_road_tris = int(len(rm.faces))

    pads = []
    for kind, plist in (("depot", [bundle.pads["depot"]]), ("dest", bundle.pads["destinations"])):
        for p in plist:
            x, y, z = bundle.to_world(p["lat"], p["lon"], p["ground_amsl_m"])
            pads.append({"name": p["name"], "x": float(x), "y": float(y), "z": float(z),
                         "rgba": _rgba(PAD_RGBA[kind])})

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(_paths.TEMPLATES_DIR)),
                             undefined=jinja2.StrictUndefined, keep_trailing_newline=True)
    wname = world_name(region_id)
    lat, lon = bundle.depot
    cmd = sitl_command(bundle)
    sdf = env.get_template("city.sdf.j2").render(
        region_id=region_id, bundle_hash=bundle.bundle_hash, world_name=wname, lat=lat, lon=lon,
        h_home=bundle.h_home, custom_location=cmd.split("=")[-1], models=models, pads=pads,
        pad_radius=2.0, pole_height=6.0, vehicle_model=vehicle_model,
        spawn_z=scfg.SIM_VEHICLE_SPAWN_Z, sky=sky, contact_system=contact_system)
    world_path = out / f"{wname}.sdf"
    world_path.write_text(sdf, encoding="utf-8")
    (out / "gui.config").write_text(env.get_template("gui.config.j2").render(
        vehicle_model=vehicle_model, camera_pose=" ".join(f"{v:.4f}" for v in camera_pose),
        camera_offset=" ".join(f"{v:.1f}" for v in offset), follow=follow), encoding="utf-8")
    (out / "sitl_command.txt").write_text(cmd + "\n", encoding="utf-8")

    xmin, xmax, ymin, ymax = world_extent(bundle)
    wm = {
        "region_id": region_id, "bundle_hash": bundle.bundle_hash, "world_name": wname,
        "world_sdf": world_path.name, "vehicle_model": vehicle_model, "sitl_command": cmd,
        "gui": {"config": "gui.config", "view": gui_view, "camera_offset": list(offset),
                "mode": "follow" if follow else "fixed"},
        "extent_m": {"xmin": xmin, "xmax": xmax, "ymin": ymin, "ymax": ymax},
        "terrain": {"res_m": terrain_res_m, "vertices": int(len(terr.vertices)), "faces": int(len(terr.faces)),
                    "z_min": float(terr.vertices[:, 2].min()), "z_max": float(terr.vertices[:, 2].max())},
        "buildings": {"tiles": len(by_tile), **bstats,
                      "faces": int(sum(len(m.faces) for m in tiles.values()))},
        "road_faces": n_road_tris, "models": models, "fixture": bundle.is_fixture,
    }
    (out / "world_manifest.json").write_text(json.dumps(wm, indent=2), encoding="utf-8")
    print(f"[World] {wname} → {world_path}")
    print(f"[World]   terrain {wm['terrain']['faces']} faces (z {wm['terrain']['z_min']:.1f}…"
          f"{wm['terrain']['z_max']:.1f} m), buildings {bstats['extruded']} in {len(by_tile)} tiles "
          f"({bstats['failed']} failed), road faces {n_road_tris}")
    print(f"[World]   SITL: {cmd}")
    return world_path


# ── route markers ────────────────────────────────────────────────────────────

def _segment_visual(name: str, p0, p1, width: float, rgba, transparency: float = 0.0) -> str:
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    if length < 1e-3:
        return ""
    mid = (p0 + p1) / 2.0
    yaw = math.atan2(d[1], d[0])
    pitch = -math.asin(max(-1.0, min(1.0, d[2] / length)))
    return (f'      <visual name="{name}"><cast_shadows>false</cast_shadows>'
            f'<transparency>{transparency:.2f}</transparency>'
            f'<pose>{mid[0]:.3f} {mid[1]:.3f} {mid[2]:.3f} 0 {pitch:.5f} {yaw:.5f}</pose>'
            f'<geometry><box><size>{length:.3f} {width:.3f} {width:.3f}</size></box></geometry>'
            f'<material><ambient>{_rgba(rgba)}</ambient><diffuse>{_rgba(rgba)}</diffuse>'
            f'<emissive>{_rgba(rgba)}</emissive></material></visual>\n')


def route_markers_sdf(selected_xyz: Sequence, front_xyz: Sequence[Sequence] = (), *,
                      name: str = "route_markers", selected_width: float = scfg.SIM_SELECTED_MARKER_WIDTH_M,
                      front_width: float = scfg.SIM_FRONT_MARKER_WIDTH_M,
                      max_front: int = scfg.SIM_MAX_FRONT_MARKERS) -> str:
    """Visual-only static model: selected route (red) + up to max_front front routes (blue).

    Lines are thinner than the vehicle and translucent: the drone flies along the line's axis, so a
    wide opaque marker would hide it."""
    vis = []
    for k, route in enumerate(list(front_xyz)[:max_front]):
        for s in range(len(route) - 1):
            vis.append(_segment_visual(f"front{k}_{s}", route[s], route[s + 1], front_width, FRONT_RGBA,
                                       scfg.SIM_FRONT_MARKER_TRANSPARENCY))
    for s in range(len(selected_xyz) - 1):
        vis.append(_segment_visual(f"sel_{s}", selected_xyz[s], selected_xyz[s + 1], selected_width, SELECTED_RGBA,
                                   scfg.SIM_SELECTED_MARKER_TRANSPARENCY))
    return (f'<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="{name}">\n    <static>true</static>\n'
            f'    <link name="link">\n' + "".join(vis) + "    </link>\n  </model>\n</sdf>\n")


def main(argv=None):
    p = argparse.ArgumentParser(description="Build the Gazebo city world from a region bundle.")
    p.add_argument("region_id", nargs="?", default=scfg.SIM_REGION_ID)
    p.add_argument("--terrain-res-m", type=float, default=scfg.SIM_TERRAIN_RES_M)
    p.add_argument("--tile-m", type=float, default=scfg.SIM_TILE_M)
    p.add_argument("--vehicle-model", default=scfg.SIM_VEHICLE_MODEL)
    p.add_argument("--no-roads", action="store_true")
    p.add_argument("--no-sky", action="store_true", help="lighter rendering (WSLg / software GL)")
    p.add_argument("--contact-system", action="store_true", help="add gz Contact system (collision evidence)")
    p.add_argument("--view", choices=sorted(scfg.SIM_CAMERA_VIEWS), default="follow",
                   help="GUI camera in gui.config: follow / wide track the vehicle, takeoff is a fixed pad shot")
    a = p.parse_args(argv)
    build_world(a.region_id, terrain_res_m=a.terrain_res_m, tile_m=a.tile_m, vehicle_model=a.vehicle_model,
                roads=not a.no_roads, sky=not a.no_sky, contact_system=a.contact_system, gui_view=a.view)


if __name__ == "__main__":
    main()
