"""
make_pneunet.py — parametric pneunet CAD generator with multi-material support.

Generates a segmented pneumatic-network actuator:
  - Outer silicone body containing (n_spines + 1) tall cavity chambers
    arranged along the long axis, connected by a thin channel at the
    bottom of the cavity (the classic pneunet topology).
  - Each chamber sits inside a bellow column of silicone. Adjacent
    bellows are separated by external grooves cut from `sil_ymax` down
    by `groove_depth` (default = `spine_cut_fraction * (height -
    2*wall_thickness)` — i.e. `spine_cut_fraction` is the fraction of
    the cavity's Y extent that the groove eats into the silicone).
    Grooves run the full Z width, and are inset from the cavity edges
    by `wall_thickness` on either X side (so the chamber side walls
    remain at `wall_thickness`). Set `groove_depth=0` to suppress
    grooves (spines internal only).
  - The cavity (chambers + connecting channel) is auto-sized so the
    chamber bottom sits `groove_floor_thickness` below the groove
    floor at the spine X positions, leaving that thickness of silicone
    as a load-bearing slab above the channel. Default
    `groove_floor_thickness = 2 * wall_thickness`. The connecting
    channel takes whatever Y space remains; if the geometry can't fit
    a positive channel height, the call raises with a hint.
  - The Z-side silicone wall around the cavity uses a separate
    parameter `side_wall_thickness` (default `2 * wall_thickness`)
    because pneunets typically have thicker lateral walls than top/
    bottom walls. The chambers are slimmer in Z; the outer width is
    unchanged.
  - Optional strain-limiter slab below the silicone, with a conforming
    mesh interface produced by Gmsh OCC `fragment`.

Output is a Gmsh v2 ASCII .msh with physical groups already set:
  - Volume:  silicone  (always),  strain_limiter  (if limiter_thickness > 0)
  - Surface: inner_cavity  (all cavity triangles, fused into one group)

That means the output goes DIRECTLY into preprocess.py — no
`mesh_tagger.py` pass is needed. `mesh_tagger.py --auto` still works
on these meshes (it will just re-derive what's already labelled), so
existing pipelines are not broken.

Usage:
    python make_pneunet.py --out cads/pneunet/source/mesh.msh

For a plain rectangular cavity (original behaviour), pass --n_spines 0
and --limiter_thickness 0:
    python make_pneunet.py --out cads/pneunet/wall_0.0010m/mesh.msh \\
        --n_spines 0 --limiter_thickness 0 --wall_thickness 0.001

Geometry convention (matches parametricGripper.py):
  X axis = finger long axis, X=0 = inlet end
  Y axis = bending direction; +Y is the bulge side (cavity side),
           -Y is the limiter side (curl is toward -Y when inflated)
  Z axis = lateral
"""

from __future__ import annotations

import argparse
import os
import sys


def make_pneunet(
    out_path: str,
    length: float = 0.073,
    width: float = 0.023,
    height: float = 0.013,
    wall_thickness: float = 0.0015,
    mesh_size: float = 0.0015,
    n_spines: int = 5,
    spine_thickness: float = 0.0045,
    groove_x_width: float | None = None,
    spine_cut_fraction: float = 0.7,
    groove_depth: float | None = None,
    groove_floor_thickness: float | None = None,
    limiter_thickness: float = 0.0015,
    side_wall_thickness: float | None = None,
    cavity_inset_x: float | None = None,
    floor_thickness: float = 0.0,
    verbose: bool = True,
) -> dict:
    """Build the geometry, mesh, write a Gmsh v2 ASCII .msh, return stats.

    Coordinate layout (X = length, Y = bending direction, Z = width):
        Silicone outer box: (0, -H/2, -W/2) to (L, H/2, W/2).
        Cavity is inset by `wall_thickness` on all sides, with optional
        Y shift via `cavity_offset_y` (>0 makes the -Y wall thicker,
        stacking with the strain limiter).
        If `n_spines > 0`, the cavity is decomposed into N+1 tall chambers
        (one per gap between spines, plus end caps) at the top, sharing
        a single thin connecting channel at the bottom — the standard
        pneunet topology.
        If `limiter_thickness > 0`, a slab is added below the silicone
        (Y from -H/2 - limiter_thickness to -H/2). The silicone and
        limiter share a conforming interface via OCC fragment.

    Setting `n_spines=0` reproduces the original single-rectangle cavity.
    Setting `limiter_thickness=0` skips the strain limiter (single-material).
    """
    import gmsh

    if cavity_inset_x is None:
        cavity_inset_x = wall_thickness
    if side_wall_thickness is None:
        # Pneunets typically have thicker silicone walls in Z (lateral) to
        # resist bulging sideways under pressure. Default is 2x the
        # primary wall_thickness; override to match wall_thickness if you
        # want a uniform shell.
        side_wall_thickness = 2.0 * wall_thickness
    # `groove_x_width` is the outer-fixed parameter: it's the visible
    # gap between two adjacent bellows. When specified, it pins the
    # external X layout regardless of wall_thickness — spine_thickness
    # is derived so the wall_thickness silicone walls on either side of
    # the groove stay at exactly wall_thickness. This is the right knob
    # for wall_thickness studies where the outer pneunet shape must be
    # identical across variants and only the cavity shrinks.
    if groove_x_width is not None:
        if groove_x_width <= 0:
            raise ValueError(
                f"groove_x_width={groove_x_width} must be > 0"
            )
        spine_thickness = groove_x_width + 2.0 * wall_thickness

    L, W, H, t = length, width, height, wall_thickness
    sw = side_wall_thickness

    if t * 2.0 >= H:
        raise ValueError(
            f"wall_thickness={t} too large for height={H}; "
            f"need 2*wall_thickness < height"
        )
    if sw * 2.0 >= W:
        raise ValueError(
            f"side_wall_thickness={sw} too large for width={W}; "
            f"need 2*side_wall_thickness < width"
        )
    if floor_thickness < 0:
        raise ValueError(
            f"floor_thickness={floor_thickness} must be >= 0"
        )
    if floor_thickness + wall_thickness >= H:
        raise ValueError(
            f"floor_thickness ({floor_thickness}) + wall_thickness "
            f"({wall_thickness}) >= height ({H}) — no room for the cavity"
        )
    if floor_thickness == 0.0 and limiter_thickness == 0.0:
        raise ValueError(
            "floor_thickness=0 means the cavity extends to the silicone "
            "bottom and relies on the strain-limiter slab to seal it. "
            "With limiter_thickness=0 the cavity has no floor at all "
            "(open to outside air). Pass either floor_thickness > 0 "
            "(silicone floor) or limiter_thickness > 0 (limiter floor)."
        )
    if n_spines < 0:
        raise ValueError(f"n_spines={n_spines} must be >= 0")
    if not (0.0 < spine_cut_fraction < 1.0):
        raise ValueError(
            f"spine_cut_fraction={spine_cut_fraction} must be in (0, 1)"
        )
    if spine_thickness <= 0:
        raise ValueError(f"spine_thickness={spine_thickness} must be > 0")
    if groove_depth is not None and groove_depth < 0:
        raise ValueError(f"groove_depth={groove_depth} must be >= 0")
    if groove_floor_thickness is not None and groove_floor_thickness < 0:
        raise ValueError(
            f"groove_floor_thickness={groove_floor_thickness} must be >= 0"
        )
    if limiter_thickness < 0:
        raise ValueError(f"limiter_thickness={limiter_thickness} must be >= 0")

    # Silicone outer bbox
    sil_xmin, sil_ymin, sil_zmin = 0.0, -H / 2.0, -W / 2.0
    sil_xmax, sil_ymax, sil_zmax = L, H / 2.0, W / 2.0

    # Cavity bbox. In X and Z the cavity is inset by wall_thickness /
    # side_wall_thickness as before. In Y the cavity extends all the way
    # down to the silicone bottom (= limiter top) by default — the
    # strain-limiter slab seals the cavity from below, so there's no
    # silicone "floor". Set `floor_thickness` > 0 to add a silicone slab
    # between the cavity and the limiter.
    cav_xmin = cavity_inset_x
    cav_xmax = L - t
    cav_ymin = sil_ymin + floor_thickness
    cav_ymax = sil_ymax - t
    cav_zmin = sil_zmin + sw
    cav_zmax = sil_zmax - sw
    cav_length_x = cav_xmax - cav_xmin
    cav_height_y = cav_ymax - cav_ymin
    cav_width_z = cav_zmax - cav_zmin
    if cav_length_x <= 0 or cav_height_y <= 0 or cav_width_z <= 0:
        raise ValueError(
            f"cavity bbox degenerate: dx={cav_length_x}, dy={cav_height_y}, "
            f"dz={cav_width_z}"
        )

    # Derive groove_depth from spine_cut_fraction by default. Reference
    # height is the full silicone height H (= the "total height" the
    # groove visibly cuts into), so spine_cut_fraction=0.7 means "the
    # groove cuts 70 % of the silicone height". Users can override
    # groove_depth directly.
    if groove_depth is None:
        groove_depth_eff = spine_cut_fraction * H
    else:
        groove_depth_eff = groove_depth
    # The silicone slab between the groove floor and the channel top at
    # the spine X positions — the "wall in the thin part of the pneunet"
    # the user references. Defaults to wall_thickness so the spine wall
    # in this region matches the chamber roof / X-side walls.
    if groove_floor_thickness is None:
        groove_floor_thickness_eff = wall_thickness
    else:
        groove_floor_thickness_eff = groove_floor_thickness

    channel_height = 0.0
    chamber_height = 0.0
    chamber_length = 0.0
    n_chambers = 1
    if n_spines > 0:
        # Vertical layout from sil_ymax downward at the *spine* X
        # positions (i.e. inside the silicone column between two
        # chambers):
        #   [sil_ymax - groove_depth, sil_ymax]                 : groove (air)
        #   [channel_y_top, sil_ymax - groove_depth]            : silicone slab
        #     (thickness = groove_floor_thickness)
        #   [cav_ymin, channel_y_top]                           : channel cavity
        #   [sil_ymin, cav_ymin]                                : silicone floor (if any)
        #   [sil_ymin - limiter_thickness, sil_ymin]            : limiter
        # At the *chamber* X positions the chamber cavity extends from
        # channel_y_top up to cav_ymax (= sil_ymax - wall_thickness);
        # above the chamber sits the wall_thickness silicone roof.
        chamber_height = (
            groove_depth_eff + groove_floor_thickness_eff - t
        )
        if chamber_height <= 0:
            raise ValueError(
                f"computed chamber_height={chamber_height:.6f} m <= 0; "
                f"increase groove_depth or groove_floor_thickness"
            )
        channel_height = cav_height_y - chamber_height
        if channel_height <= 0:
            # User's "cavity height accommodates the groove + floor" check.
            # cav_height_y = H - wall_thickness - floor_thickness.
            # channel_height > 0 iff
            # groove_depth + groove_floor_thickness + floor_thickness < H - wall_thickness
            # which with the defaults (floor=0, floor_thickness_grove = wall_thickness)
            # reduces to wall_thickness < H - groove_depth — the user's
            # "wall_thickness < (1 - spine_cut_fraction) * H" constraint.
            raise ValueError(
                f"channel_height = cav_height_y - chamber_height = "
                f"{channel_height:.6f} m <= 0. With the current defaults "
                f"this means wall_thickness ({t} m) > "
                f"H - groove_depth ({H - groove_depth_eff:.4f} m). "
                f"Reduce spine_cut_fraction ({spine_cut_fraction}) or "
                f"wall_thickness, or increase height ({H} m)."
            )
        n_chambers = n_spines + 1
        chamber_length = (
            cav_length_x - n_spines * spine_thickness
        ) / n_chambers
        if chamber_length <= 0:
            raise ValueError(
                f"computed chamber_length={chamber_length:.6f} m <= 0; "
                f"reduce n_spines ({n_spines}) or spine_thickness "
                f"({spine_thickness}), or increase length / cavity_inset_x"
            )

    gmsh.initialize()
    if not verbose:
        gmsh.option.setNumber("General.Terminal", 0)

    # Force v2 ASCII so SOFA's MeshGmshLoader can read it directly.
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
    gmsh.option.setNumber("Mesh.Binary", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", mesh_size)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", mesh_size * 0.5)

    gmsh.model.add("pneunet")

    outer = gmsh.model.occ.addBox(sil_xmin, sil_ymin, sil_zmin, L, H, W)

    # Build the cavity volume(s) to subtract
    if n_spines <= 0:
        cavity_tag = gmsh.model.occ.addBox(
            cav_xmin, cav_ymin, cav_zmin,
            cav_length_x, cav_height_y, cav_width_z,
        )
        cavity_dimtags = [(3, cavity_tag)]
    else:
        # Thin channel at the bottom of the cavity, full X length
        channel_y_top = cav_ymin + channel_height
        channel_tag = gmsh.model.occ.addBox(
            cav_xmin, cav_ymin, cav_zmin,
            cav_length_x, channel_height, cav_width_z,
        )
        # Tall chambers above the channel
        chamber_tags = []
        for i in range(n_chambers):
            x0 = cav_xmin + i * (chamber_length + spine_thickness)
            chamber_tags.append(
                gmsh.model.occ.addBox(
                    x0, channel_y_top, cav_zmin,
                    chamber_length, chamber_height, cav_width_z,
                )
            )
        # Fuse everything into one cavity volume so the silicone sees a
        # single subtractor (cleaner topology after the cut)
        cavity_dimtags, _ = gmsh.model.occ.fuse(
            [(3, channel_tag)],
            [(3, ct) for ct in chamber_tags],
        )

    # Carve cavity from silicone
    silicone_dimtags, _ = gmsh.model.occ.cut([(3, outer)], cavity_dimtags)

    # External grooves between adjacent bellows. Each groove sits in the
    # silicone gap between two chambers, inset by `wall_thickness` from
    # each chamber's outer X-wall, cuts down from sil_ymax by
    # `groove_depth_eff`, and runs the full Z width. Result: the silicone
    # is a row of chamber-containing bellows separated by daylight, with
    # the chamber roof (above the cavity) intact at wall_thickness.
    # `groove_floor_thickness_eff` silicone separates the groove floor
    # from the connecting channel below — that's the load-bearing slab.
    if n_spines > 0 and groove_depth_eff > 0:
        # Groove width along X = the inter-chamber gap minus a
        # wall_thickness silicone wall on either side of the gap.
        groove_x_width = spine_thickness - 2.0 * t
        if groove_x_width <= 0:
            raise ValueError(
                f"spine_thickness={spine_thickness} is too small to host "
                f"a groove with wall_thickness={t} on each side "
                f"(needs spine_thickness > 2*wall_thickness). Either "
                f"increase spine_thickness, decrease wall_thickness, or "
                f"pass --groove_depth 0 to suppress grooves."
            )
        groove_y_bot = sil_ymax - groove_depth_eff
        # Slight overshoot in Y / Z so the cut surfaces meet the silicone
        # outer surfaces exactly (avoid sliver faces from numerical
        # coincidence).
        eps_cut = max(mesh_size * 0.1, 1e-5)
        groove_tags = []
        for i in range(n_spines):
            # The i-th inter-chamber gap starts where chamber i ends.
            chamber_i_end_x = cav_xmin + (i + 1) * chamber_length + i * spine_thickness
            groove_x0 = chamber_i_end_x + t
            groove_tags.append((
                3,
                gmsh.model.occ.addBox(
                    groove_x0, groove_y_bot, sil_zmin - eps_cut,
                    groove_x_width, groove_depth_eff + eps_cut,
                    W + 2 * eps_cut,
                ),
            ))
        silicone_dimtags, _ = gmsh.model.occ.cut(silicone_dimtags, groove_tags)

    # Optional strain-limiter slab below silicone, conforming interface
    limiter_dimtags = []
    if limiter_thickness > 0:
        limiter_tag = gmsh.model.occ.addBox(
            sil_xmin, sil_ymin - limiter_thickness, sil_zmin,
            L, limiter_thickness, W,
        )
        # fragment makes silicone + limiter share their interface face so
        # the meshes connect node-for-node along Y = sil_ymin.
        frag_dimtags, _ = gmsh.model.occ.fragment(
            silicone_dimtags, [(3, limiter_tag)]
        )
        # Re-classify the resulting volumes by Y centre.
        silicone_dimtags = []
        for d, tg in frag_dimtags:
            if d != 3:
                continue
            xmin2, ymin2, zmin2, xmax2, ymax2, zmax2 = (
                gmsh.model.occ.getBoundingBox(d, tg)
            )
            yc = 0.5 * (ymin2 + ymax2)
            if yc < sil_ymin - 1e-9:
                limiter_dimtags.append((d, tg))
            else:
                silicone_dimtags.append((d, tg))

    gmsh.model.occ.synchronize()

    if not silicone_dimtags:
        gmsh.finalize()
        raise RuntimeError("no silicone volume left after boolean operations")

    # --- Physical groups ------------------------------------------------
    sil_tags = [tg for _, tg in silicone_dimtags]
    sil_phys = gmsh.model.addPhysicalGroup(3, sil_tags)
    gmsh.model.setPhysicalName(3, sil_phys, "silicone")

    if limiter_dimtags:
        lim_tags = [tg for _, tg in limiter_dimtags]
        lim_phys = gmsh.model.addPhysicalGroup(3, lim_tags)
        gmsh.model.setPhysicalName(3, lim_phys, "strain_limiter")

    # Inner cavity surface: every face on the boundary of the combined
    # silicone + limiter solid whose bbox is entirely inside the cavity
    # bbox. Using both volumes (combined=True default) means:
    #  - the silicone–limiter shared interface (internal to the union)
    #    is correctly excluded;
    #  - the limiter top piece exposed inside the cavity X-Z region is
    #    included — it becomes the cavity floor when the silicone floor
    #    is removed (floor_thickness=0).
    # Gmsh's getBoundingBox includes a small OCC-tolerance inflation
    # (~1e-7 m in practice). Use a generous geometric tolerance for the
    # bbox-containment check — still far below the smallest mesh feature.
    eps_geom = 1e-5
    inner_cavity_surface_tags: list[int] = []
    boundary_input = list(silicone_dimtags) + list(limiter_dimtags)
    cav_surfaces = gmsh.model.getBoundary(
        boundary_input, oriented=False
    )
    for d, tg in cav_surfaces:
        if d != 2:
            continue
        xmin2, ymin2, zmin2, xmax2, ymax2, zmax2 = (
            gmsh.model.getBoundingBox(d, tg)
        )
        in_cavity = (
            xmin2 >= cav_xmin - eps_geom
            and xmax2 <= cav_xmax + eps_geom
            and ymin2 >= cav_ymin - eps_geom
            and ymax2 <= cav_ymax + eps_geom
            and zmin2 >= cav_zmin - eps_geom
            and zmax2 <= cav_zmax + eps_geom
        )
        if in_cavity:
            inner_cavity_surface_tags.append(tg)

    if not inner_cavity_surface_tags:
        gmsh.finalize()
        raise RuntimeError(
            "no inner_cavity surfaces identified; geometry mismatch"
        )
    cav_phys = gmsh.model.addPhysicalGroup(2, inner_cavity_surface_tags)
    gmsh.model.setPhysicalName(2, cav_phys, "inner_cavity")

    # --- Mesh -----------------------------------------------------------
    if verbose:
        print(f"Generating 3D mesh (target size = {mesh_size} m) ...")
    gmsh.model.mesh.generate(3)

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    gmsh.write(out_path)

    # Stats
    node_tags, _, _ = gmsh.model.mesh.getNodes()
    elem_types, elem_tags, _ = gmsh.model.mesh.getElements()
    counts = {f"type_{int(et)}": int(len(elem_tags[i]))
              for i, et in enumerate(elem_types)}
    stats = {
        "n_nodes": int(len(node_tags)),
        "n_elements_by_type": counts,
        "n_chambers": n_chambers,
        "n_spines": n_spines,
        "has_limiter": bool(limiter_dimtags),
    }

    gmsh.finalize()

    if verbose:
        print(f"Wrote {out_path}")
        print(f"  nodes: {stats['n_nodes']}")
        print(f"  elements: {stats['n_elements_by_type']}")
        print(f"  silicone bbox: X=[{sil_xmin:.4f}, {sil_xmax:.4f}], "
              f"Y=[{sil_ymin:+.4f}, {sil_ymax:+.4f}], "
              f"Z=[{sil_zmin:+.4f}, {sil_zmax:+.4f}]")
        if limiter_dimtags:
            lim_y_min = sil_ymin - limiter_thickness
            print(f"  limiter bbox:  X=[{sil_xmin:.4f}, {sil_xmax:.4f}], "
                  f"Y=[{lim_y_min:+.4f}, {sil_ymin:+.4f}], "
                  f"Z=[{sil_zmin:+.4f}, {sil_zmax:+.4f}]")
            print(f"  combined Y span (silicone + limiter): "
                  f"[{lim_y_min:+.4f}, {sil_ymax:+.4f}] m")
            print(f"  NB: parametricGripper.py INLET_CLAMP_BOX Y_min should "
                  f"be <= {lim_y_min:+.4f} so the limiter is clamped at the "
                  f"inlet, not just the silicone.")
        print(f"  cavity bbox:   X=[{cav_xmin:.4f}, {cav_xmax:.4f}], "
              f"Y=[{cav_ymin:+.4f}, {cav_ymax:+.4f}], "
              f"Z=[{cav_zmin:+.4f}, {cav_zmax:+.4f}]")
        if n_spines > 0:
            print(f"  cavity layout: {n_chambers} chamber(s) "
                  f"× {chamber_length * 1000:.2f} mm long, "
                  f"separated by {n_spines} spine(s) "
                  f"× {spine_thickness * 1000:.2f} mm thick")
            print(f"  chamber depth = {chamber_height * 1000:.2f} mm; "
                  f"connecting channel = {channel_height * 1000:.2f} mm "
                  f"(cavity height total = {cav_height_y * 1000:.2f} mm)")
            if groove_depth_eff > 0:
                groove_x_width = spine_thickness - 2.0 * wall_thickness
                print(f"  external grooves between bellows: "
                      f"{groove_x_width * 1000:.2f} mm wide × "
                      f"{groove_depth_eff * 1000:.2f} mm deep, full Z, "
                      f"with {wall_thickness * 1000:.2f} mm silicone wall "
                      f"on each X side")
                print(f"  silicone slab between groove floor and channel "
                      f"top: {groove_floor_thickness_eff * 1000:.2f} mm "
                      f"(default = wall_thickness)")
                if floor_thickness > 0:
                    print(f"  silicone floor below cavity: "
                          f"{floor_thickness * 1000:.2f} mm")
                else:
                    print(f"  silicone floor: none — cavity bottom sits "
                          f"on the strain-limiter top (limiter seals "
                          f"the cavity from below)")
            else:
                print(f"  external grooves: disabled "
                      f"(spines visible only from inside the cavity)")
        print(f"  Z silicone wall: {sw * 1000:.2f} mm "
              f"(chamber Z width = {(W - 2 * sw) * 1000:.2f} mm of "
              f"{W * 1000:.2f} mm total)")
        groups = ["silicone (vol)"]
        if limiter_dimtags:
            groups.append("strain_limiter (vol)")
        groups.append("inner_cavity (surf)")
        print(f"  physical groups: {', '.join(groups)}")
        print(f"  next: python preprocess.py {out_path} <out.npz>")

    return stats


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--out", required=True,
                        help="output .msh path (parent dir auto-created)")
    parser.add_argument("--length", type=float, default=0.073,
                        help="overall length along X (m, default 0.073)")
    parser.add_argument("--width", type=float, default=0.023,
                        help="overall width along Z (m, default 0.023)")
    parser.add_argument("--height", type=float, default=0.013,
                        help="silicone height along Y (m, default 0.013); "
                             "the limiter sits below this Y range when "
                             "limiter_thickness > 0.")
    parser.add_argument("--wall_thickness", type=float, default=0.0015,
                        help="primary silicone wall thickness (m, default "
                             "0.0015); applies to the chamber roof + floor "
                             "in Y and the chamber side walls in X. The "
                             "Z-side wall uses `side_wall_thickness`.")
    parser.add_argument("--side_wall_thickness", type=float, default=None,
                        help="Z-side (lateral) silicone wall thickness in "
                             "meters; defaults to 2 * wall_thickness. "
                             "Thicker Z walls resist lateral bulging "
                             "under pressure; the chambers become slimmer "
                             "in Z but the outer width is unchanged.")
    parser.add_argument("--mesh_size", type=float, default=0.0015,
                        help="target mesh characteristic length "
                             "(m, default 0.0015; keep this <= "
                             "spine_thickness so spines aren't undermeshed)")
    parser.add_argument("--n_spines", type=int, default=5,
                        help="number of internal spine walls dividing the "
                             "cavity into chambers (default 5; pass 0 for a "
                             "single rectangular cavity)")
    parser.add_argument("--spine_thickness", type=float, default=0.0045,
                        help="thickness of each spine wall in X "
                             "(m, default 0.0045); ignored if "
                             "--groove_x_width is given.")
    parser.add_argument("--groove_x_width", type=float, default=None,
                        help="outer (visible) X width of each groove "
                             "between bellows (m). When set, "
                             "spine_thickness is derived as "
                             "groove_x_width + 2*wall_thickness so the "
                             "outer X layout is independent of "
                             "wall_thickness. Use this for wall-thickness "
                             "studies where the outer pneunet shape must "
                             "stay constant across variants.")
    parser.add_argument("--spine_cut_fraction", type=float, default=0.7,
                        help="fraction of cav_height_y (= height - "
                             "2*wall_thickness) the external groove cuts "
                             "down from sil_ymax (0..1, default 0.7). "
                             "Drives the default groove_depth.")
    parser.add_argument("--groove_depth", type=float, default=None,
                        help="depth of the external grooves cut into the "
                             "silicone (m). Defaults to spine_cut_fraction "
                             "* (height - 2*wall_thickness). Pass 0 to "
                             "suppress grooves (spines internal only).")
    parser.add_argument("--groove_floor_thickness", type=float, default=None,
                        help="silicone slab thickness between the groove "
                             "floor and the connecting channel top (m, "
                             "default = 2*wall_thickness). This is the "
                             "load-bearing layer that ties adjacent "
                             "bellows together above the channel.")
    parser.add_argument("--limiter_thickness", type=float, default=0.0015,
                        help="thickness of the strain-limiter slab below "
                             "the silicone (m, default 0.0015; pass 0 to "
                             "skip the limiter and produce a single-"
                             "material mesh)")
    parser.add_argument("--cavity_inset_x", type=float, default=None,
                        help="cavity start offset from inlet end "
                             "(m, default = wall_thickness)")
    parser.add_argument("--floor_thickness", type=float, default=0.0,
                        help="silicone floor thickness below the cavity "
                             "(m, default 0). When 0 the cavity bottom "
                             "sits directly on the strain-limiter top "
                             "(the limiter acts as the cavity floor and "
                             "seals it). Set to >0 to keep a silicone "
                             "slab between the cavity and the limiter.")
    parser.add_argument("--quiet", action="store_true",
                        help="suppress Gmsh's stdout noise")
    args = parser.parse_args()

    try:
        make_pneunet(
            out_path=args.out,
            length=args.length,
            width=args.width,
            height=args.height,
            wall_thickness=args.wall_thickness,
            mesh_size=args.mesh_size,
            n_spines=args.n_spines,
            spine_thickness=args.spine_thickness,
            groove_x_width=args.groove_x_width,
            spine_cut_fraction=args.spine_cut_fraction,
            groove_depth=args.groove_depth,
            groove_floor_thickness=args.groove_floor_thickness,
            limiter_thickness=args.limiter_thickness,
            side_wall_thickness=args.side_wall_thickness,
            cavity_inset_x=args.cavity_inset_x,
            floor_thickness=args.floor_thickness,
            verbose=not args.quiet,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
