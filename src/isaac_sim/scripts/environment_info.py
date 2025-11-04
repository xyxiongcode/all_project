#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Extract geometric environment info from a USD scene:
- Scene XY bounding box and outer-rect area (in meters)
- Per-object world-aligned bounding boxes (AABB), centers, sizes
- Optional 2D occupancy raster (AABB projected onto XY)

Usage:
  python usd_geoinfo.py scene.usd [--mesh-only] [--min-size 0.02]
                                  [--csv obstacles.csv] [--json obstacles.json]
                                  [--grid-res 0.1] [--grid-png occupancy.png]

Requires:
  - pxr (USD Python)
Optional:
  - numpy, pillow (for occupancy raster)
"""

import argparse
import json
import math
import sys
from pathlib import Path

try:
    import numpy as np
except Exception:
    np = None

try:
    from PIL import Image
except Exception:
    Image = None

from pxr import Usd, UsdGeom, Gf, Tf


def get_stage_meters_per_unit(stage: Usd.Stage) -> float:
    """Return metersPerUnit (default 0.01 means centimeters)."""
    meters = UsdGeom.GetStageMetersPerUnit(stage)
    try:
        return float(meters)
    except Exception:
        # Fallback: many DCC scenes implicitly use meters=1.0
        return 1.0


def is_candidate_geom(prim: Usd.Prim, mesh_only: bool) -> bool:
    if not prim.IsActive() or not prim.IsDefined():
        return False
    if mesh_only:
        return prim.IsA(UsdGeom.Mesh)
    # Broader: any Boundable (Mesh, Cube, Sphere, Capsule, Points with extents, etc.)
    return UsdGeom.Boundable(prim) is not None or prim.IsA(UsdGeom.Mesh)


def compute_world_aabb(bbox_cache: UsdGeom.BBoxCache, prim: Usd.Prim) -> Gf.Range3d:
    # Use world-space aligned box
    bound = bbox_cache.ComputeWorldBound(prim)
    return bound.ComputeAlignedBox()


def range3d_to_center_size(r: Gf.Range3d):
    mn = r.GetMin()
    mx = r.GetMax()
    cx = 0.5 * (mn[0] + mx[0])
    cy = 0.5 * (mn[1] + mx[1])
    cz = 0.5 * (mn[2] + mx[2])
    sx = max(0.0, mx[0] - mn[0])
    sy = max(0.0, mx[1] - mn[1])
    sz = max(0.0, mx[2] - mn[2])
    return (cx, cy, cz), (sx, sy, sz)


def meters_scale_tuple(t, scale):
    return tuple(v * scale for v in t)


def collect_geometry(stage_path: str, mesh_only: bool, min_size_m: float):
    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {stage_path}")

    # Force default prim as pseudo-root if present
    if stage.GetDefaultPrim() is None:
        # Attempt to set default prim (optional)
        pass

    meters_per_unit = get_stage_meters_per_unit(stage)
    # BBox cache (default, render, proxy)
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, useExtentsHint=True)

    objects = []
    scene_min = Gf.Vec3d(float('inf'), float('inf'), float('inf'))
    scene_max = Gf.Vec3d(float('-inf'), float('-inf'), float('-inf'))

    for prim in stage.Traverse():
        if not is_candidate_geom(prim, mesh_only):
            continue
        try:
            aabb = compute_world_aabb(bbox_cache, prim)
            if not aabb.IsEmpty():
                (c, s) = range3d_to_center_size(aabb)

                # Convert to meters
                c_m = tuple(v * meters_per_unit for v in c)
                s_m = tuple(v * meters_per_unit for v in s)

                # Filter tiny speckles
                if max(s_m[0], s_m[1], s_m[2]) < min_size_m:
                    continue

                mn = aabb.GetMin()
                mx = aabb.GetMax()
                mn_m = (mn[0] * meters_per_unit, mn[1] * meters_per_unit, mn[2] * meters_per_unit)
                mx_m = (mx[0] * meters_per_unit, mx[1] * meters_per_unit, mx[2] * meters_per_unit)

                # Update scene bounds
                scene_min = Gf.Vec3d(
                    min(scene_min[0], mn[0]),
                    min(scene_min[1], mn[1]),
                    min(scene_min[2], mn[2]),
                )
                scene_max = Gf.Vec3d(
                    max(scene_max[0], mx[0]),
                    max(scene_max[1], mx[1]),
                    max(scene_max[2], mx[2]),
                )

                objects.append({
                    "name": prim.GetName(),
                    "path": prim.GetPath().pathString,
                    "center_m": {"x": c_m[0], "y": c_m[1], "z": c_m[2]},
                    "size_m": {"x": s_m[0], "y": s_m[1], "z": s_m[2]},
                    "min_m": {"x": mn_m[0], "y": mn_m[1], "z": mn_m[2]},
                    "max_m": {"x": mx_m[0], "y": mx_m[1], "z": mx_m[2]},
                    "type": prim.GetTypeName(),
                })
        except Exception as e:
            # Some prims (lights/cameras) may fail bounds; ignore
            Tf.Warn(f"Skipping prim {prim.GetPath()} due to error: {e}")

    if math.isinf(scene_min[0]) or math.isinf(scene_max[0]):
        # No geometry found
        scene_bounds_m = None
        area_xy_m2 = 0.0
    else:
        # Convert scene bounds to meters
        scene_min_m = (scene_min[0] * meters_per_unit, scene_min[1] * meters_per_unit, scene_min[2] * meters_per_unit)
        scene_max_m = (scene_max[0] * meters_per_unit, scene_max[1] * meters_per_unit, scene_max[2] * meters_per_unit)
        scene_bounds_m = {"min_m": {"x": scene_min_m[0], "y": scene_min_m[1], "z": scene_min_m[2]},
                          "max_m": {"x": scene_max_m[0], "y": scene_max_m[1], "z": scene_max_m[2]}}

        # XY outer rectangle area
        width_x = max(0.0, scene_max_m[0] - scene_min_m[0])
        depth_y = max(0.0, scene_max_m[1] - scene_min_m[1])
        area_xy_m2 = width_x * depth_y

    summary = {
        "usd": str(stage_path),
        "meters_per_unit": meters_per_unit,
        "scene_bounds_m": scene_bounds_m,
        "scene_area_xy_m2": area_xy_m2,
        "object_count": len(objects),
        "objects": objects,
    }
    return summary


def save_csv(objs, csv_path: str):
    import csv
    keys = ["name", "path",
            "center_x_m", "center_y_m", "center_z_m",
            "size_x_m", "size_y_m", "size_z_m",
            "min_x_m", "min_y_m", "min_z_m",
            "max_x_m", "max_y_m", "max_z_m",
            "type"]
    with open(csv_path, "w", newline='', encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for o in objs:
            row = [
                o["name"], o["path"],
                o["center_m"]["x"], o["center_m"]["y"], o["center_m"]["z"],
                o["size_m"]["x"], o["size_m"]["y"], o["size_m"]["z"],
                o["min_m"]["x"], o["min_m"]["y"], o["min_m"]["z"],
                o["max_m"]["x"], o["max_m"]["y"], o["max_m"]["z"],
                o["type"],
            ]
            w.writerow(row)


def rasterize_occupancy(summary: dict, grid_res_m: float):
    """Make a 2D occupancy (XY) by painting each object's AABB projection."""
    if np is None:
        raise RuntimeError("numpy is required for occupancy rasterization")
    if summary["scene_bounds_m"] is None:
        raise RuntimeError("No scene bounds to rasterize")

    mn = summary["scene_bounds_m"]["min_m"]
    mx = summary["scene_bounds_m"]["max_m"]
    xmin, ymin = mn["x"], mn["y"]
    xmax, ymax = mx["x"], mx["y"]

    width_m = max(0.0, xmax - xmin)
    height_m = max(0.0, ymax - ymin)

    # Avoid zero-sized
    width_px = max(1, int(math.ceil(width_m / grid_res_m)))
    height_px = max(1, int(math.ceil(height_m / grid_res_m)))

    occ = np.zeros((height_px, width_px), dtype=np.uint8)  # 0 free, 255 occupied for output

    def world_to_px(x, y):
        # Origin at scene_min (top-left in image coordinates with y increasing downward)
        ix = int((x - xmin) / grid_res_m)
        iy = int((y - ymin) / grid_res_m)
        # Convert to image row index (flip Y to have upward-positive become top->down)
        row = height_px - 1 - iy
        col = ix
        return row, col

    for o in summary["objects"]:
        # Project AABB to XY
        bx0, by0 = o["min_m"]["x"], o["min_m"]["y"]
        bx1, by1 = o["max_m"]["x"], o["max_m"]["y"]

        # Clamp to scene bounds
        bx0 = max(bx0, xmin); by0 = max(by0, ymin)
        bx1 = min(bx1, xmax); by1 = min(by1, ymax)
        if bx1 <= bx0 or by1 <= by0:
            continue

        r0, c0 = world_to_px(bx0, by0)
        r1, c1 = world_to_px(bx1, by1)

        # Ensure proper ordering
        rmin, rmax = sorted([r0, r1])
        cmin, cmax = sorted([c0, c1])

        # Paint occupied
        rmin = max(0, rmin); cmin = max(0, cmin)
        rmax = min(height_px - 1, rmax); cmax = min(width_px - 1, cmax)
        if rmax >= rmin and cmax >= cmin:
            occ[rmin:rmax+1, cmin:cmax+1] = 255

    return occ


def maybe_save_png(arr: "np.ndarray", png_path: str):
    if Image is None:
        raise RuntimeError("Pillow is required to save PNG images")
    img = Image.fromarray(arr, mode="L")
    img.save(png_path)


def main():
    ap = argparse.ArgumentParser(description="Extract geometric info from USD.")
    ap.add_argument("usd", type=str, help="Path to USD/USDZ/USDA file")
    ap.add_argument("--mesh-only", action="store_true", help="Count only UsdGeomMesh")
    ap.add_argument("--min-size", type=float, default=0.0, help="Filter tiny objects (meters)")
    ap.add_argument("--csv", type=str, default=None, help="Export per-object CSV")
    ap.add_argument("--json", type=str, default=None, help="Export summary JSON")
    ap.add_argument("--grid-res", type=float, default=None, help="Occupancy grid resolution (m/pixel)")
    ap.add_argument("--grid-png", type=str, default=None, help="Output occupancy PNG path")
    args = ap.parse_args()

    summary = collect_geometry(args.usd, mesh_only=args.mesh_only, min_size_m=args.min_size)

    # Console report
    print(f"[USD] {summary['usd']}")
    print(f"[Units] metersPerUnit = {summary['meters_per_unit']:.6g}")
    if summary["scene_bounds_m"]:
        mn = summary["scene_bounds_m"]["min_m"]; mx = summary["scene_bounds_m"]["max_m"]
        print(f"[Scene XY bounds] X: [{mn['x']:.3f}, {mx['x']:.3f}] m, Y: [{mn['y']:.3f}, {mx['y']:.3f}] m")
    print(f"[Scene area (outer XY rect)] {summary['scene_area_xy_m2']:.3f} m^2")
    print(f"[Objects] {summary['object_count']}")

    # Save JSON/CSV
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(f"[Write] {args.json}")

    if args.csv:
        save_csv(summary["objects"], args.csv)
        print(f"[Write] {args.csv}")

    if args.grid_res and args.grid_png:
        occ = rasterize_occupancy(summary, grid_res_m=args.grid_res)
        maybe_save_png(occ, args.grid_png)
        print(f"[Write] {args.grid_png}  (res={args.grid_res} m/px, shape={occ.shape[::-1]} px)")

if __name__ == "__main__":
    sys.exit(main())
