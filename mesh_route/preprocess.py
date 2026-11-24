"""
preprocess.py — canonical actuator preprocessor.

Reads a Gmsh `.msh` with named physical groups and writes a single
`actuator.npz` containing everything the SOFA scene needs. Multi-chamber
aware from day one.

Required physical groups in the .msh:
  Volume (dim 3):
    'silicone'         soft body material           (required)
    'strain_limiter'   stiff backing layer          (optional)
  Surface (dim 2):
    'inner_cavity'                  single chamber
    OR
    'inner_cavity_1', 'inner_cavity_2', ...   multi-chamber

Cavity surface normals are oriented to point INTO silicone via the
parent-tet 4th-vertex method (so SOFA's SurfacePressureConstraint can
inflate the cavity with a single consistent sign convention).

Output schema (actuator.npz):
  points                       (N, 3)  float64  body mesh node positions
  tet_nodes                    (T, 4)  int64    full tet connectivity (file order)
  silicone_tet_indices         (S,)    int32    which tets are silicone
  limiter_tet_indices          (L,)    int32    which tets are limiter (may be empty)
  cavity_names                 (C,)    str      chamber names in saving order
  cavity_<i>_triangles         (M_i,3) int64    triangles for chamber i (global indices, oriented)
  cavity_<i>_node_indices      (K_i,)  int64    sorted unique global indices used by chamber i
  outer_triangles              (B, 3)  int64    outer-skin triangles (boundary faces minus
                                                cavity faces; normals oriented OUT of the body).
                                                Used for collision models (e.g. bladder/braid
                                                contact in the two-body McKibben).
  outer_node_indices           (Q,)    int64    sorted unique global indices of the outer skin

Optional (present when a label_gui.py actuators.json sidecar exists):
  actuator_spec_json           ()      str      full actuator spec (types + params) as JSON
  region_names                 (R,)    str      non-pneumatic actuator region group names
  region_types                 (R,)    str      actuator type key per region (cable_*, motor, ...)
  region_<i>_triangles         (M_i,3) int64    triangles of region i (global indices)
  region_<i>_node_indices      (K_i,)  int64    sorted unique global indices of region i

Usage:
    python preprocess.py <input.msh> [<output.npz>]

If the input is Gmsh v4, first convert it (converter not included) to produce
v2 ASCII (SOFA's MeshGmshLoader needs v2; meshio reads either, but
keeping both formats aligned avoids confusion).
"""

import sys
import os
import json
import numpy as np
import meshio


def collect_tets(mesh):
    """Walk mesh.cells in file order, accumulate tet connectivity + tags."""
    tet_blocks = []
    tag_blocks = []
    for cell_block, tag_array in zip(mesh.cells, mesh.cell_data["gmsh:physical"]):
        if cell_block.type == "tetra":
            tet_blocks.append(cell_block.data[:, :4])
            tag_blocks.append(tag_array)
        elif cell_block.type == "tetra10":
            tet_blocks.append(cell_block.data[:, :4])  # corner nodes only
            tag_blocks.append(tag_array)
    if not tet_blocks:
        raise RuntimeError("No tetrahedra found in mesh.")
    return (np.vstack(tet_blocks).astype(np.int64),
            np.concatenate(tag_blocks).astype(np.int64))


def collect_surface_triangles(mesh, surface_tag):
    """All triangles in mesh with the given physical tag."""
    tri_blocks = []
    for cell_block, tag_array in zip(mesh.cells, mesh.cell_data["gmsh:physical"]):
        if cell_block.type == "triangle":
            mask = tag_array == surface_tag
            if mask.any():
                tri_blocks.append(cell_block.data[mask])
        elif cell_block.type == "triangle6":
            mask = tag_array == surface_tag
            if mask.any():
                tri_blocks.append(cell_block.data[mask, :3])
    if not tri_blocks:
        return None
    return np.vstack(tri_blocks).astype(np.int64)


def orient_outward_from_silicone(triangles, points, all_tets):
    """Flip each triangle so its normal points AWAY from the 4th vertex
    of the tet that owns the face — i.e. away from the silicone, into
    the cavity interior. SOFA's SurfacePressureConstraint expects this
    convention: with normals pointing into the cavity, a POSITIVE value
    on the constraint inflates the cavity (force on the surface points
    outward from cavity → into silicone → pushes silicone away → cavity
    expands).

    Triangles whose face isn't found in any tet are left alone and
    counted as orphans."""
    face_to_fourth = {}
    for tet in all_tets:
        for missing in range(4):
            face = tuple(sorted(int(tet[i]) for i in range(4) if i != missing))
            face_to_fourth[face] = int(tet[missing])

    out = triangles.copy()
    n_flipped = 0
    n_orphan = 0
    for i in range(len(triangles)):
        tri = triangles[i]
        face = tuple(sorted(int(v) for v in tri))
        fourth = face_to_fourth.get(face)
        if fourth is None:
            n_orphan += 1
            continue
        v0, v1, v2 = points[tri[0]], points[tri[1]], points[tri[2]]
        v3 = points[fourth]
        n = np.cross(v1 - v0, v2 - v0)
        d = v3 - (v0 + v1 + v2) / 3.0
        # Flip when the current normal points TOWARD the silicone 4th
        # vertex; we want it pointing AWAY (into the cavity).
        if np.dot(n, d) > 0:
            out[i] = tri[::-1]
            n_flipped += 1
    return out, n_flipped, n_orphan


def extract_outer_surface(tet_nodes, cavity_triangles_list):
    """Outer skin = tet boundary faces (owned by exactly one tet) minus all
    cavity faces. Returned with the raw winding from the tet face — caller
    should orient via `orient_outward_from_silicone` (which, for boundary
    faces, points normals OUT of the body — what collision models expect)."""
    faces = np.concatenate([
        tet_nodes[:, [1, 2, 3]],
        tet_nodes[:, [0, 3, 2]],
        tet_nodes[:, [0, 1, 3]],
        tet_nodes[:, [0, 2, 1]],
    ])
    faces_sorted = np.sort(faces, axis=1)
    uniq, first_idx, counts = np.unique(
        faces_sorted, axis=0, return_index=True, return_counts=True
    )
    boundary = faces[first_idx[counts == 1]]

    cavity_keys = set()
    for cav_tris in cavity_triangles_list:
        for tri in cav_tris:
            cavity_keys.add(tuple(sorted(int(v) for v in tri)))
    keep = np.array(
        [tuple(sorted(int(v) for v in tri)) not in cavity_keys
         for tri in boundary],
        dtype=bool,
    )
    return boundary[keep].astype(np.int64)


def preprocess(msh_path, out_npz=None):
    if out_npz is None:
        out_npz = os.path.join(os.path.dirname(msh_path) or '.', 'actuator.npz')

    print(f"Reading {msh_path}")
    mesh = meshio.read(msh_path)
    print(f"  points: {len(mesh.points)}")
    print("Physical groups:")
    for name, (tag, dim) in mesh.field_data.items():
        print(f"  {name}: tag={tag}, dim={dim}")

    # --- Tets + material partition --------------------------------------
    tet_nodes, tet_tags = collect_tets(mesh)
    print(f"  total tets: {len(tet_nodes)}")

    silicone_tag = mesh.field_data.get("silicone", (None,))[0]
    limiter_tag = mesh.field_data.get("strain_limiter", (None,))[0]

    if silicone_tag is None:
        print("  WARNING: no 'silicone' group; treating all tets as silicone.")
        silicone_tet_indices = np.arange(len(tet_nodes), dtype=np.int32)
        limiter_tet_indices = np.array([], dtype=np.int32)
    else:
        silicone_tet_indices = np.where(tet_tags == silicone_tag)[0].astype(np.int32)
        limiter_tet_indices = (np.where(tet_tags == limiter_tag)[0].astype(np.int32)
                               if limiter_tag is not None
                               else np.array([], dtype=np.int32))
    print(f"  silicone tets: {len(silicone_tet_indices)}")
    print(f"  limiter tets:  {len(limiter_tet_indices)}")

    # --- Cavities --------------------------------------------------------
    cavity_names = sorted(
        [n for n in mesh.field_data
         if n == "inner_cavity" or n.startswith("inner_cavity_")]
    )
    if not cavity_names:
        # Cavity-less meshes are legitimate for non-pneumatic actuators
        # (e.g. cable/tendon fingers from make_cable_finger.py). The scene
        # gates all pressure machinery on the chamber count, so an empty
        # list is safe — but shout, in case this is a mislabeled pneumatic
        # CAD (the usual cause: forgot the inner_cavity physical group).
        print("  WARNING: no 'inner_cavity' physical groups found — "
              "proceeding with ZERO pressure chambers (fine for cable/"
              "tendon CADs, a labeling bug for pneumatic ones).")

    save_dict = {
        'points': mesh.points.astype(np.float64),
        'tet_nodes': tet_nodes,
        'silicone_tet_indices': silicone_tet_indices,
        'limiter_tet_indices': limiter_tet_indices,
        'cavity_names': np.array(cavity_names),
    }
    for i, cav_name in enumerate(cavity_names):
        cav_tag = mesh.field_data[cav_name][0]
        triangles = collect_surface_triangles(mesh, cav_tag)
        if triangles is None:
            print(f"  WARNING: no triangles for {cav_name}")
            continue
        triangles, n_flipped, n_orphan = orient_outward_from_silicone(
            triangles, mesh.points, tet_nodes
        )
        node_indices = np.unique(triangles).astype(np.int64)
        print(f"  {cav_name}: {len(triangles)} triangles, {len(node_indices)} nodes "
              f"(normal fix: {n_flipped} flipped, {n_orphan} orphan)")
        save_dict[f'cavity_{i}_triangles'] = triangles
        save_dict[f'cavity_{i}_node_indices'] = node_indices

    # --- Outer skin (for collision models, e.g. McKibben bladder/braid) --
    cavity_tris_all = [save_dict[k] for k in save_dict
                       if k.startswith('cavity_') and k.endswith('_triangles')]
    outer_tris = extract_outer_surface(tet_nodes, cavity_tris_all)
    outer_tris, n_flipped, n_orphan = orient_outward_from_silicone(
        outer_tris, mesh.points, tet_nodes
    )
    outer_nodes = np.unique(outer_tris).astype(np.int64)
    print(f"  outer skin: {len(outer_tris)} triangles, {len(outer_nodes)} nodes "
          f"(normal fix: {n_flipped} flipped, {n_orphan} orphan)")
    save_dict['outer_triangles'] = outer_tris
    save_dict['outer_node_indices'] = outer_nodes

    # Optional sidecar: actuator spec from label_gui.py. Pneumatic
    # chambers are already covered by the cavity_* fields above; here we
    # embed the full spec (types + params) and carry NON-pneumatic
    # actuator regions (cables, motors, ...) through generically as
    # region_<i>_* arrays so a future scene can wire them up without a
    # preprocess change.
    stem = os.path.splitext(msh_path)[0]
    spec_candidates = [stem + "_actuators.json",
                       os.path.join(os.path.dirname(msh_path) or '.',
                                    'actuators.json')]
    spec_path = next((p for p in spec_candidates if os.path.exists(p)), None)
    if spec_path is not None:
        with open(spec_path, "r", encoding="utf-8") as f:
            spec = json.load(f)
        save_dict['actuator_spec_json'] = np.array(json.dumps(spec))
        region_names, region_types = [], []
        for entry in spec.get("actuators", []):
            if entry["type"] == "pneumatic":
                continue  # already saved as cavity_* above
            gname = entry["physical_group"]
            if gname not in mesh.field_data:
                print(f"  WARNING: spec region '{gname}' not in mesh; skipped")
                continue
            triangles = collect_surface_triangles(
                mesh, mesh.field_data[gname][0])
            if triangles is None:
                print(f"  WARNING: no triangles for region '{gname}'")
                continue
            i = len(region_names)
            save_dict[f'region_{i}_triangles'] = triangles
            save_dict[f'region_{i}_node_indices'] = \
                np.unique(triangles).astype(np.int64)
            region_names.append(gname)
            region_types.append(entry["type"])
            print(f"  {gname} [{entry['type']}]: {len(triangles)} triangles")
        if region_names:
            save_dict['region_names'] = np.array(region_names)
            save_dict['region_types'] = np.array(region_types)
        print(f"  actuator spec: {spec_path} "
              f"({len(spec.get('actuators', []))} actuators)")

    # Optional sidecar: helical-cable topology (written by
    # make_singlebody_mckibben.py). Pneunet meshes have no sibling
    # cables.npz; the merge is a no-op and actuator.npz omits the
    # cable_* fields, so parametricGripper.py's cable wiring is skipped.
    cables_sidecar = os.path.join(
        os.path.dirname(msh_path) or '.', 'cables.npz'
    )
    if os.path.exists(cables_sidecar):
        cables = np.load(cables_sidecar)
        merged_keys = []
        for k in cables.files:
            save_dict[k] = cables[k]
            merged_keys.append(k)
        n_edges = (int(cables['cable_edges'].shape[0])
                   if 'cable_edges' in cables.files else 0)
        print(f"  cables sidecar: {cables_sidecar} "
              f"({n_edges} edges; merged keys: {merged_keys})")

    np.savez(out_npz, **save_dict)
    print(f"\nWrote {out_npz}")
    return out_npz


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        sys.exit(1)
    preprocess(sys.argv[1], sys.argv[2] if len(sys.argv) == 3 else None)
