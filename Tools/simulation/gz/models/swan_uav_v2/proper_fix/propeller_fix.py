#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Create propeller-only rotor STL files for swan_uav_v2.

Problem this solves:
  base_link.STL already contains the fixed motor / motor base.
  rot*_Link.STL also contains a motor shell plus blades.
  If the whole rot*_Link.STL spins, the duplicated motor shell spins with the propeller,
  and any small offset makes it look like the rotor is wobbling /甩动.

This script:
  1) reads rot1_Link.STL ... rot4_Link.STL
  2) recenters the mesh in XY
  3) removes the central motor-shell region by radius
  4) writes rot1_Link_prop_only.STL ... rot4_Link_prop_only.STL

Recommended first run:
  cd /home/renwang/data_storage/VisionFlow-PX4/Tools/simulation/gz/models/swan_uav_v2
  python3 make_rotor_propeller_only_meshes.py --mesh-dir meshes --center-method bbox --cut-radius 0.065

If blade roots are cut too much:
  python3 make_rotor_propeller_only_meshes.py --mesh-dir meshes --center-method bbox --cut-radius 0.050

If motor shell remains:
  python3 make_rotor_propeller_only_meshes.py --mesh-dir meshes --center-method bbox --cut-radius 0.080

Then use the SDF that references:
  meshes/rot1_Link_prop_only.STL ... meshes/rot4_Link_prop_only.STL
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path
from typing import Tuple

import numpy as np


ROTOR_FILES = [
    "rot1_Link.STL",
    "rot2_Link.STL",
    "rot3_Link.STL",
    "rot4_Link.STL",
]


def is_binary_stl(path: Path) -> bool:
    data = path.read_bytes()
    if len(data) < 84:
        return False

    tri_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + tri_count * 50
    if expected_size == len(data):
        return True

    head = data[:512].lower()
    if b"facet normal" in head and b"outer loop" in data[:4096].lower():
        return False

    return True


def read_binary_stl_triangles(path: Path):
    data = path.read_bytes()
    header = data[:80]
    tri_count = struct.unpack("<I", data[80:84])[0]

    normals = []
    tris = []
    attrs = []

    offset = 84
    for _ in range(tri_count):
        normals.append(struct.unpack("<fff", data[offset:offset + 12]))
        offset += 12

        tri = []
        for _j in range(3):
            tri.append(struct.unpack("<fff", data[offset:offset + 12]))
            offset += 12

        attrs.append(data[offset:offset + 2])
        offset += 2
        tris.append(tri)

    return header, np.asarray(normals, dtype=np.float32), np.asarray(tris, dtype=np.float64), attrs


def write_binary_stl(path: Path, header: bytes, normals: np.ndarray, tris: np.ndarray, attrs) -> None:
    out = bytearray()
    if len(header) != 80:
        header = (header[:80]).ljust(80, b" ")
    out += header
    out += struct.pack("<I", len(tris))

    for n, tri, attr in zip(normals, tris, attrs):
        out += struct.pack("<fff", float(n[0]), float(n[1]), float(n[2]))
        for v in tri:
            out += struct.pack("<fff", float(v[0]), float(v[1]), float(v[2]))
        out += attr if isinstance(attr, (bytes, bytearray)) and len(attr) == 2 else b"\x00\x00"

    path.write_bytes(out)


def read_ascii_stl_triangles(path: Path):
    tris = []
    current = []
    for line in path.read_text(errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            current.append([float(parts[1]), float(parts[2]), float(parts[3])])
            if len(current) == 3:
                tris.append(current)
                current = []
    if not tris:
        raise ValueError(f"No ASCII STL triangles found in {path}")
    normals = np.zeros((len(tris), 3), dtype=np.float32)
    return np.asarray(normals, dtype=np.float32), np.asarray(tris, dtype=np.float64)


def write_ascii_stl(path: Path, name: str, tris: np.ndarray) -> None:
    lines = [f"solid {name}"]
    for tri in tris:
        # Let Gazebo / renderer handle normals. Use zero normals for simplicity.
        lines.append("  facet normal 0 0 0")
        lines.append("    outer loop")
        for v in tri:
            lines.append(f"      vertex {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    path.write_text("\n".join(lines) + "\n")


def bbox_center_xy(vertices: np.ndarray) -> np.ndarray:
    xy = vertices[:, :2]
    return 0.5 * (xy.min(axis=0) + xy.max(axis=0))


def hub_center_xy(vertices: np.ndarray, percentile: float = 28.0, iterations: int = 8) -> np.ndarray:
    xy = vertices[:, :2]
    center = bbox_center_xy(vertices)
    for _ in range(iterations):
        r = np.linalg.norm(xy - center[None, :], axis=1)
        radius_limit = np.percentile(r, percentile)
        core = xy[r <= radius_limit]
        if len(core) >= 10:
            center = np.median(core, axis=0)
    return center


def triangle_normals(tris: np.ndarray) -> np.ndarray:
    a = tris[:, 1, :] - tris[:, 0, :]
    b = tris[:, 2, :] - tris[:, 0, :]
    n = np.cross(a, b)
    length = np.linalg.norm(n, axis=1)
    valid = length > 1e-12
    n[valid] = n[valid] / length[valid, None]
    n[~valid] = 0
    return n.astype(np.float32)


def process_one(src: Path, cut_radius: float, center_method: str, suffix: str, keep_if_any_vertex_outside: bool):
    if not src.exists():
        raise FileNotFoundError(src)

    binary = is_binary_stl(src)

    if binary:
        header, normals, tris, attrs = read_binary_stl_triangles(src)
    else:
        normals, tris = read_ascii_stl_triangles(src)
        header = f"prop-only {src.name}".encode("ascii", errors="ignore").ljust(80, b" ")
        attrs = [b"\x00\x00"] * len(tris)

    vertices = tris.reshape(-1, 3)

    if center_method == "bbox":
        center_xy = bbox_center_xy(vertices)
    elif center_method == "hub":
        center_xy = hub_center_xy(vertices)
    else:
        raise ValueError(center_method)

    # Recenter all vertices so estimated propeller axis is at local link origin.
    shifted = tris.copy()
    shifted[:, :, 0] -= center_xy[0]
    shifted[:, :, 1] -= center_xy[1]

    r_vertices = np.linalg.norm(shifted[:, :, :2], axis=2)
    r_centroid = np.linalg.norm(shifted[:, :, :2].mean(axis=1), axis=1)

    if keep_if_any_vertex_outside:
        # Keeps triangles that touch the outer blade region.
        keep = np.any(r_vertices >= cut_radius, axis=1)
    else:
        # Cleaner motor removal, but can cut blade roots more aggressively.
        keep = r_centroid >= cut_radius

    filtered = shifted[keep]
    normals_out = triangle_normals(filtered)
    attrs_out = [a for a, k in zip(attrs, keep) if k]

    dst = src.with_name(src.stem + suffix + src.suffix)

    if binary:
        write_binary_stl(dst, header, normals_out, filtered, attrs_out)
    else:
        write_ascii_stl(dst, dst.stem, filtered)

    total = len(tris)
    kept = len(filtered)
    removed = total - kept

    print(
        f"[OK] {src.name:14s} -> {dst.name:24s} "
        f"center=({center_xy[0]: .8g}, {center_xy[1]: .8g}) "
        f"cut_radius={cut_radius:.4f} "
        f"triangles kept={kept}/{total}, removed={removed}"
    )


def default_mesh_dir() -> Path:
    script_dir = Path(__file__).resolve().parent
    candidate = script_dir / "meshes"
    if candidate.exists():
        return candidate
    return Path.cwd() / "meshes"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh-dir", type=Path, default=default_mesh_dir())
    parser.add_argument("--cut-radius", type=float, default=0.065,
                        help="Central motor-shell radius to remove, in meters. Try 0.05, 0.065, 0.08.")
    parser.add_argument("--center-method", choices=["bbox", "hub"], default="bbox",
                        help="bbox usually centers the propeller sweep; hub emphasizes central geometry.")
    parser.add_argument("--suffix", default="_prop_only")
    parser.add_argument("--keep-if-any-vertex-outside", action="store_true",
                        help="Less aggressive cutting. Useful if blade roots disappear.")
    args = parser.parse_args()

    mesh_dir = args.mesh_dir.resolve()
    print(f"[INFO] mesh_dir      = {mesh_dir}")
    print(f"[INFO] center_method = {args.center_method}")
    print(f"[INFO] cut_radius    = {args.cut_radius}")

    for name in ROTOR_FILES:
        process_one(
            mesh_dir / name,
            cut_radius=args.cut_radius,
            center_method=args.center_method,
            suffix=args.suffix,
            keep_if_any_vertex_outside=args.keep_if_any_vertex_outside,
        )

    print("\n[DONE] Generated *_prop_only.STL files.")
    print("       Now point the SDF rotor mesh URIs to rot*_Link_prop_only.STL.")
    print("       Recommended SDF: swan_uav_v2_use_prop_only_rotors.sdf")


if __name__ == "__main__":
    main()
