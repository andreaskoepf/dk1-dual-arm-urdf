#!/usr/bin/env python3
"""
Create parametric mesh primitives for the DK1 mounting structure.

Generates STL (collision) and GLB (visual) meshes for:
  - Front crossbar: HFS8-4080-700 aluminum extrusion (700×40×80 mm)
  - Camera mast: HFS6-3030-400 vertical extrusion (30×30×400 mm)

Meshes are centered at origin; the URDF joint origins handle placement.
"""

import struct
import os
import numpy as np

MESHES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")


def box_triangles(sx, sy, sz):
    """Generate triangles for an axis-aligned box centered at origin.
    Returns list of (normal, v1, v2, v3) tuples. Dimensions in meters.
    """
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    # 8 corners
    corners = [
        (-hx, -hy, -hz), (+hx, -hy, -hz), (+hx, +hy, -hz), (-hx, +hy, -hz),
        (-hx, -hy, +hz), (+hx, -hy, +hz), (+hx, +hy, +hz), (-hx, +hy, +hz),
    ]
    # 6 faces, each as 2 triangles (CCW winding for outward normal)
    faces = [
        # -Z face (bottom)
        ((0, 0, -1), [0, 2, 1]), ((0, 0, -1), [0, 3, 2]),
        # +Z face (top)
        ((0, 0, +1), [4, 5, 6]), ((0, 0, +1), [4, 6, 7]),
        # -X face
        ((-1, 0, 0), [0, 4, 7]), ((-1, 0, 0), [0, 7, 3]),
        # +X face
        ((+1, 0, 0), [1, 2, 6]), ((+1, 0, 0), [1, 6, 5]),
        # -Y face
        ((0, -1, 0), [0, 1, 5]), ((0, -1, 0), [0, 5, 4]),
        # +Y face
        ((0, +1, 0), [2, 3, 7]), ((0, +1, 0), [2, 7, 6]),
    ]
    triangles = []
    for normal, indices in faces:
        v1 = corners[indices[0]]
        v2 = corners[indices[1]]
        v3 = corners[indices[2]]
        triangles.append((normal, v1, v2, v3))
    return triangles


def write_binary_stl(filepath, triangles):
    """Write triangles to a binary STL file."""
    with open(filepath, "wb") as f:
        # 80-byte header
        f.write(b"\x00" * 80)
        # Number of triangles
        f.write(struct.pack("<I", len(triangles)))
        for normal, v1, v2, v3 in triangles:
            f.write(struct.pack("<fff", *normal))
            f.write(struct.pack("<fff", *v1))
            f.write(struct.pack("<fff", *v2))
            f.write(struct.pack("<fff", *v3))
            f.write(struct.pack("<H", 0))  # attribute byte count
    print(f"  STL: {filepath} ({len(triangles)} triangles)")


def write_glb(filepath, triangles, color_rgba=(0.75, 0.75, 0.75, 1.0)):
    """Write triangles to a GLB file using trimesh."""
    try:
        import trimesh
        vertices = []
        faces = []
        for i, (normal, v1, v2, v3) in enumerate(triangles):
            base = i * 3
            vertices.extend([v1, v2, v3])
            faces.append([base, base + 1, base + 2])

        mesh = trimesh.Trimesh(
            vertices=np.array(vertices, dtype=np.float64),
            faces=np.array(faces, dtype=np.int64),
        )
        # Merge duplicate vertices
        mesh.merge_vertices()
        mesh.fix_normals()

        # Set color
        mesh.visual = trimesh.visual.ColorVisuals(
            mesh=mesh,
            face_colors=np.tile(
                np.array([int(c * 255) for c in color_rgba], dtype=np.uint8),
                (len(mesh.faces), 1),
            ),
        )

        mesh.export(filepath, file_type="glb")
        print(f"  GLB: {filepath} ({len(mesh.faces)} triangles)")
    except ImportError:
        print(f"  GLB: skipped (trimesh not available)")
    except Exception as e:
        print(f"  GLB: failed ({e})")


def main():
    os.makedirs(MESHES_DIR, exist_ok=True)

    aluminum_color = (0.75, 0.75, 0.75, 1.0)

    # Front crossbar: HFS8-4080-700
    # 700mm along X, 40mm along Y, 80mm along Z
    print("Front crossbar (HFS8-4080-700):")
    crossbar_tris = box_triangles(0.700, 0.040, 0.080)
    write_binary_stl(os.path.join(MESHES_DIR, "front_crossbar.stl"), crossbar_tris)
    write_glb(os.path.join(MESHES_DIR, "front_crossbar.glb"), crossbar_tris, aluminum_color)

    # Camera mast: HFS6-3030-400
    # 30mm along X, 30mm along Y, 400mm along Z
    print("Camera mast (HFS6-3030-400):")
    mast_tris = box_triangles(0.030, 0.030, 0.400)
    write_binary_stl(os.path.join(MESHES_DIR, "camera_mast.stl"), mast_tris)
    write_glb(os.path.join(MESHES_DIR, "camera_mast.glb"), mast_tris, aluminum_color)

    print("Done!")


if __name__ == "__main__":
    main()
