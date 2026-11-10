"""The records' controllability Gramians side by side, in one figure.

    python tools/figure_gramians.py
        -> outputs/record_gramians.png, outputs/record_gramians.svg,
           outputs/record_gramians.json, outputs/record_gramians.md

Run with a Python that has numpy and matplotlib (the analysis venv).

What is drawn. The Gramian is the one the M-11 pipeline builds
(Gramain_extraction, gramian_map/jacobian.py):

    W_c = sum_k |du_k| J_k J_k^T,   J_k = dy/du at input level k,

with J_k by forward difference at the first level, backward at the last and
central elsewhere, and du_k the matching level spacing. Every record here has
one input, so a wide W_c means the direction of motion turns along the
stroke, not that two directions can be chosen.

Two groups, in separate panels, because their units differ:

- Simulated records carry a stored Gramian (the record's `shape` block: top
  eigenvalues and modes of W_c over 10 spine points). Each cell draws the
  rest spine with the first two modes on it, the way jacobian.py draws them,
  and the tip block of W_c as an ellipse at the tip. Their pressures are far
  below physical ones, so magnitudes are comparative, not predictive.
- Published records carry no stored Gramian. Where a record holds a free tip
  curve, the same formula is applied to that curve, giving the tip block
  only. For every simulated record the same calculation on its free curve
  reproduces the tip block of its stored Gramian (checked here and in
  tests/test_figure_gramians.py), which is why the two are drawn as the same
  quantity.

Records that hold neither a stored Gramian nor a free tip curve are named on
the figure and in the table, with the reason made from which blocks they
hold. Nothing is dropped silently.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RECORDS = os.path.join(ROOT, "data", "records")
OUT = os.path.join(ROOT, "outputs")
STEM = "record_gramians"

# Order of the simulated cells: by family, then by the record id.
FAMILIES = ["wall_", "cable_", "bellows", "mckibben"]
MODE_SCALE = 0.25          # mode drawn at this fraction of the spine span (as jacobian.py)
MIN_LEVELS = 3             # a free curve needs this many levels to give J


# ---------------------------------------------------------------- analysis

def record_ids(records_dir=RECORDS):
    return sorted(f[:-5] for f in os.listdir(records_dir)
                  if f.endswith(".json") and f != "MANIFEST.json")


def load(rid, records_dir=RECORDS):
    with open(os.path.join(records_dir, rid + ".json"), encoding="utf-8") as f:
        return json.load(f)


def manifest_quality(records_dir=RECORDS):
    path = os.path.join(records_dir, "MANIFEST.json")
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        m = json.load(f)
    return {k: v.get("quality") or {} for k, v in m.get("records", {}).items()}


def jacobians(u, y):
    """jacobian.py's central_diff_jacobians: forward, central, backward."""
    J = np.zeros_like(y)
    J[0] = (y[1] - y[0]) / (u[1] - u[0])
    J[-1] = (y[-1] - y[-2]) / (u[-1] - u[-2])
    J[1:-1] = (y[2:] - y[:-2]) / (u[2:] - u[:-2])[:, None]
    return J


def level_weights(u):
    """jacobian.py's |dp|: one-sided at the ends, half the span elsewhere."""
    du = np.zeros_like(u)
    du[0] = u[1] - u[0]
    du[-1] = u[-1] - u[-2]
    du[1:-1] = 0.5 * (u[2:] - u[:-2])
    return np.abs(du)


def tip_gramian_from_free(free):
    u = np.asarray(free["u"], float)[:, 0]
    y = np.asarray(free["tip_displacement_m"], float)
    J = jacobians(u, y)
    return np.einsum("k,ki,kj->ij", level_weights(u), J, J)


def stored_blocks(shape):
    """Spine rest (P,3), modes (P,3,k), eigenvalues (k,), and the tip block
    of the stored W_c rebuilt from its top-k modes."""
    S = np.asarray(shape["spine_rest_m"], float)
    lam = np.asarray(shape["mode_eigenvalues"], float)
    V = np.asarray(shape["mode_vectors"], float).reshape(len(S), 3, -1)
    Vt = V[-1]                                   # last spine point = free end
    return S, V, lam, Vt @ np.diag(lam) @ Vt.T


def held_blocks(r):
    return [b for b in ("free", "blocked", "stiffness", "shape")
            if r.get(b)]


def has_free_curve(r):
    fr = r.get("free")
    return bool(fr) and len(fr.get("u") or []) >= MIN_LEVELS


def leave_out_reason(r):
    held = held_blocks(r)
    if not r.get("shape") and not has_free_curve(r):
        what = ", ".join(f"{b} curve" for b in held) if held else "headline numbers only"
        return f"no stored Gramian and no free tip curve (holds: {what})"
    if r.get("shape") and not has_free_curve(r):
        return "stored Gramian but no free tip curve to place the tip block"
    return None


def channel(r):
    ch = r["input"]["channels"][0]
    return ch.get("unit") or "", ch.get("range")


def gramian_unit(in_unit):
    """Units of W_c and of sqrt(trace W_c) for a tip displacement in m."""
    if in_unit == "m":
        return "m", "m^1/2"
    return f"m^2/{in_unit}", f"m/{in_unit}^1/2"


def ellipse_xy(W, n=181):
    """Points of {W_xy^(1/2) v : |v| = 1}, the tip block projected on XY."""
    Wxy = W[:2, :2]
    ev, evec = np.linalg.eigh(Wxy)
    root = evec @ np.diag(np.sqrt(np.clip(ev, 0, None))) @ evec.T
    t = np.linspace(0, 2 * np.pi, n)
    return (root @ np.vstack([np.cos(t), np.sin(t)])).T


def describe(rid, r, quality):
    """Everything drawn for one record, or None with a reason."""
    reason = leave_out_reason(r)
    if reason:
        return None, reason
    unit, rng = channel(r)
    free = r["free"]
    u = np.asarray(free["u"], float)[:, 0]
    W = tip_gramian_from_free(free)
    ev = np.sort(np.linalg.eigvalsh(W))[::-1]
    w_unit, a_unit = gramian_unit(unit)
    tip_end = np.asarray(free["tip_displacement_m"], float)[-1]
    d = {
        "id": rid,
        "name": r["name"],
        "group": "simulated" if r["source"]["kind"] == "simulation" else "published",
        "source_kind": r["source"]["kind"],
        "reference": r["source"].get("reference"),
        "input_unit": unit,
        "input_range": rng,
        "free_levels": int(len(u)),
        "free_u_span": [float(u.min()), float(u.max())],
        "tip_W_c": W.tolist(),
        "tip_W_c_unit": w_unit,
        "tip_eigenvalues": ev.tolist(),
        "tip_ratio_2_to_1": float(ev[1] / ev[0]) if ev[0] > 0 else None,
        "tip_sqrt_trace": float(np.sqrt(np.trace(W))),
        "sqrt_trace_unit": a_unit,
        "tip_out_of_plane_share": float(W[2, 2] / np.trace(W)) if np.trace(W) > 0 else None,
        "free_end_displacement_m": tip_end.tolist(),
        "quality_flags": quality or {},
    }
    if r.get("shape"):
        S, V, lam, Wt = stored_blocks(r["shape"])
        d.update({
            "spine_points": int(len(S)),
            "stored_eigenvalues": lam.tolist(),
            "stored_ratio_2_to_1": float(lam[1] / lam[0]),
            "stored_authority": float(r["shape"]["authority"]),
            "stored_tip_block": Wt.tolist(),
            "tip_block_rel_diff": float(np.linalg.norm(W - Wt) / np.linalg.norm(Wt)),
            "mode1_out_of_plane_share": float(np.sum(V[:, 2, 0] ** 2)),
        })
    return d, None


def analyse(records_dir=RECORDS):
    quality = manifest_quality(records_dir)
    drawn, left_out = [], []
    for rid in record_ids(records_dir):
        d, reason = describe(rid, load(rid, records_dir), quality.get(rid))
        if d:
            drawn.append(d)
        else:
            left_out.append({"id": rid, "reason": reason})
    return drawn, left_out


def family_key(d):
    for i, f in enumerate(FAMILIES):
        if d["id"].startswith(f):
            return (i, d["id"])
    return (len(FAMILIES), d["id"])


# ---------------------------------------------------------------- drawing

def sci(x):
    """1.2e-02 -> '1.2e-2'."""
    m, e = f"{x:.1e}".split("e")
    return f"{m}e{int(e)}"


def span_text(d):
    lo, hi = d["free_u_span"]
    unit = d["input_unit"]
    if unit == "Pa" and max(abs(lo), abs(hi)) >= 1000:
        return f"{lo / 1000:g} to {hi / 1000:g} kPa"
    if unit == "m":
        return f"{lo * 1000:g} to {hi * 1000:g} mm pull"
    return f"{lo:.3g} to {hi:.3g} {unit}"


def sign_fix(mode_xy, d):
    """Modes are sign-blind: turn each so its tip moves the way the free tip
    ends up moving."""
    ref = np.asarray(d["free_end_displacement_m"][:2])
    return -mode_xy if mode_xy[-1] @ ref < 0 else mode_xy


def draw_sim_cell(ax, d, r, colours):
    S, V, lam, _ = stored_blocks(r["shape"])
    xy = S[:, :2] * 1000.0
    span = float(np.linalg.norm(xy.max(0) - xy.min(0)))
    ax.plot(xy[:, 0], xy[:, 1], "-", color="0.55", lw=2.2, solid_capstyle="round", zorder=2)
    for mi, (c, ls) in enumerate(zip(colours, ("-", "--"))):
        m = V[:, :2, mi]
        n = float(np.linalg.norm(m))
        if n < 1e-12:
            continue
        m = sign_fix(m * (MODE_SCALE * span / n), d)
        ax.plot(xy[:, 0] + m[:, 0], xy[:, 1] + m[:, 1], ls, color=c, lw=1.6, zorder=3)
    E = ellipse_xy(np.asarray(d["tip_W_c"]))
    emax = float(np.abs(E).max())
    if emax > 0:
        E = E * (MODE_SCALE * span / emax)
        ax.fill(xy[-1, 0] + E[:, 0], xy[-1, 1] + E[:, 1], color="#3b6fb6", alpha=0.25, lw=0, zorder=4)
        ax.plot(xy[-1, 0] + E[:, 0], xy[-1, 1] + E[:, 1], color="#3b6fb6", lw=1.6, zorder=4)
    ax.plot(xy[0, 0], xy[0, 1], "s", color="0.3", ms=4, zorder=5)
    pad = 0.42 * span
    ax.set_xlim(xy[:, 0].min() - 0.12 * span, xy[:, 0].max() + pad)
    ax.set_ylim(-pad - 0.05 * span, pad + 0.05 * span)


def draw_pub_cell(ax, d):
    W = np.asarray(d["tip_W_c"])
    E = ellipse_xy(W)
    emax = float(np.abs(E).max())
    L = 1.0
    ax.plot([-L, 0], [0, 0], "-", color="0.55", lw=2.2, solid_capstyle="round")
    ax.plot(-L, 0, "s", color="0.3", ms=4)
    if emax > 0:
        E = E * (MODE_SCALE * 1.6 * L / emax)
        ax.fill(E[:, 0], E[:, 1], color="#3b6fb6", alpha=0.25, lw=0, zorder=4)
        ax.plot(E[:, 0], E[:, 1], color="#3b6fb6", lw=1.6, zorder=4)
    ax.set_xlim(-1.12 * L, 0.55 * L)
    ax.set_ylim(-0.55 * L, 0.55 * L)


def cell_text(ax, title, lines, flag=None):
    ax.set_title(title, fontsize=10.5, pad=3)
    ax.text(0.0, -0.03, "\n".join(lines), transform=ax.transAxes, fontsize=9.4,
            va="top", ha="left", color="0.15", linespacing=1.15)
    if flag:
        ax.text(0.98, 0.97, flag, transform=ax.transAxes, fontsize=8.6,
                va="top", ha="right", color="#b03a2e", fontweight="bold")


def tidy(ax):
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("0.8")


def short_name(d):
    n = d["name"]
    for a, b in (("Pneunet bender, wall ", "Pneu-net wall "),
                 ("Tendon-driven silicone finger (tendon 1)", "Tendon finger"),
                 ("Bellows + slack spine tendon", "Bellows + tendon"),
                 ("Two-body McKibben, braid ", "McKibben "),
                 ("Abboodi 2026 OSSA (OSSA 5) ", "Abboodi 2026 OSSA, "),
                 ("OSSA, axial extend", "OSSA, extend"), ("OSSA, vacuum contract", "OSSA, contract"),
                 (" (fast pneu-net)", ""), (" (slow pneu-net)", "")):
        n = n.replace(a, b)
    if n.startswith("Pneu-net wall ") and n.endswith(" m"):
        n = f"Pneu-net wall {float(n.split()[2]) * 1000:g} mm"
    return n


def draw(drawn, left_out, records_dir, png, svg):
    import matplotlib
    matplotlib.use("Agg")
    import logging
    logging.getLogger("matplotlib.axes._base").setLevel(logging.ERROR)
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    sim = sorted([d for d in drawn if d["group"] == "simulated"], key=family_key)
    pub = sorted([d for d in drawn if d["group"] == "published"], key=lambda d: d["id"])
    colours = ("#c0392b", "#d68910")

    ncol = 6
    nsim_rows = -(-len(sim) // ncol) if sim else 0
    npub_rows = -(-len(pub) // 2) if pub else 0
    fig = plt.figure(figsize=(20, 11.25))
    outer = GridSpec(1, 2, width_ratios=[6, 2.35], wspace=0.06,
                     left=0.012, right=0.992, top=0.875, bottom=0.175, figure=fig)
    gs_sim = outer[0].subgridspec(max(nsim_rows, 1), ncol, hspace=0.62, wspace=0.08)
    right = outer[1].subgridspec(2, 1, height_ratios=[max(npub_rows, 1) * 1.0, 1.05], hspace=0.38)
    gs_pub = right[0].subgridspec(max(npub_rows, 1), 2, hspace=0.62, wspace=0.08)

    fig.text(0.012, 0.965, "Simulated records: the stored Gramian (FEM sweeps, comparative units)",
             fontsize=15, fontweight="bold", va="top")
    fig.text(0.012, 0.932, "Grey: rest spine, base at the square.  Red: Gramian mode 1 on the spine.  "
             "Orange dashed: mode 2.  Blue: tip block of the Gramian, as an ellipse at the tip.  "
             "Each cell has its own scale.",
             fontsize=10.5, va="top", color="0.25")
    xr = outer[1].get_position(fig).x0
    fig.text(xr, 0.965, "Published records (physical units)", fontsize=15, fontweight="bold", va="top")
    fig.text(xr, 0.932, "No stored Gramian. Tip block only, computed here\nfrom the digitised free tip curve.",
             fontsize=10.5, va="top", color="0.25")

    by_id = {d["id"]: load(d["id"], records_dir) for d in sim}
    for i, d in enumerate(sim):
        ax = fig.add_subplot(gs_sim[i // ncol, i % ncol])
        draw_sim_cell(ax, d, by_id[d["id"]], colours)
        tidy(ax)
        flag = None
        if d["quality_flags"]:
            flag = "at noise floor"
        elif d["mode1_out_of_plane_share"] >= 0.2:
            flag = f"mode 1 {d['mode1_out_of_plane_share']:.0%} out of view"
        cell_text(ax, short_name(d), [
            f"mode 2/mode 1 = {sci(d['stored_ratio_2_to_1'])}",
            f"tip size {sci(d['tip_sqrt_trace'])} {d['sqrt_trace_unit']}",
            f"over {span_text(d)}"], flag)

    for i, d in enumerate(pub):
        ax = fig.add_subplot(gs_pub[i // 2, i % 2])
        draw_pub_cell(ax, d)
        tidy(ax)
        flag = None
        if d["tip_ratio_2_to_1"] is not None and d["tip_ratio_2_to_1"] < 1e-12:
            flag = "axial curve only"
        cell_text(ax, short_name(d), [
            f"tip 2/1 = {sci(d['tip_ratio_2_to_1']) if d['tip_ratio_2_to_1'] else '0'}",
            f"tip size {sci(d['tip_sqrt_trace'])} {d['sqrt_trace_unit']}",
            f"{d['free_levels']} levels, {span_text(d)}"], flag)

    ax = fig.add_subplot(right[1])
    ax.axis("off")
    ax.text(0.0, 1.0, f"Not drawn ({len(left_out)} of {len(drawn) + len(left_out)} records)",
            fontsize=12.5, fontweight="bold", va="top", transform=ax.transAxes)
    lines = []
    heads = {lo["id"]: lo["reason"].partition(" (holds: ") for lo in left_out}
    for head in sorted({h for h, _, _ in heads.values()}):
        lines.append(head[0].upper() + head[1:] + ":")
        lines += [f"    {lo['id']}  (holds: {heads[lo['id']][2]}" for lo in left_out if heads[lo["id"]][0] == head]
    ax.text(0.0, 0.86, "\n".join(lines), fontsize=9.2, va="top", transform=ax.transAxes,
            family="DejaVu Sans", linespacing=1.2)

    pa = [d["free_u_span"][1] for d in sim if d["input_unit"] == "Pa"]
    top_pa = f"no more than {max(pa):.0f} Pa" if pa else "low pressures"
    worst = max((d["tip_block_rel_diff"] for d in sim), default=0.0)
    caption = (
        "The Gramian is M-11's controllability Gramian, W_c = sum over input levels of |du| J J^T, with J = d(position)/du "
        "(Gramain_extraction, gramian_map/jacobian.py). Every record has one input, so a wide ellipse means the direction of tip motion "
        "turns along the stroke;\nit does not mean two directions can be chosen. "
        f"Simulated pressure-driven records run at {top_pa}, far below physical pressures, so their magnitudes are comparative, "
        "not predictive, and cannot be set against the published panel.\n"
        "Modes are sign-blind; each is drawn turned the way the free tip ends up moving. Views are in the record's XY plane; "
        "where a cell gives a share 'out of view', that share of mode 1 lies out of that plane. 'Tip size' is the square root of the trace of the tip block.\n"
        f"For all {len(sim)} simulated records, the tip block computed from the free tip curve matches the tip block of the stored Gramian "
        f"(largest relative difference {sci(worst)}); the published cells apply that same calculation to digitised curves. "
        "Numbers: outputs/record_gramians.md.")
    fig.text(0.012, 0.012, caption, fontsize=9.6, va="bottom", ha="left", color="0.2", wrap=False)

    fig.savefig(png, dpi=120)
    fig.savefig(svg)
    plt.close(fig)


# ---------------------------------------------------------------- table

def table_md(drawn, left_out):
    rows = [
        "# The records' Gramians: numbers drawn",
        "",
        "Made by `tools/figure_gramians.py` from `data/records/`. Figure: `outputs/record_gramians.png` (and `.svg`).",
        "Simulated magnitudes are comparative, not predictive. Units of the tip block follow each record's input unit.",
        "",
        "## Simulated records (stored Gramian)",
        "",
        "| record | input span | levels | stored mode 2/mode 1 | stored sqrt(trace), whole spine | tip size | tip 2/1 | tip out of XY | mode 1 out of XY | tip block vs stored, rel. diff | flags |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for d in sorted([d for d in drawn if d["group"] == "simulated"], key=family_key):
        rows.append(
            f"| {d['id']} | {span_text(d)} | {d['free_levels']} | {sci(d['stored_ratio_2_to_1'])} | "
            f"{sci(d['stored_authority'])} {d['sqrt_trace_unit']} | {sci(d['tip_sqrt_trace'])} {d['sqrt_trace_unit']} | "
            f"{sci(d['tip_ratio_2_to_1'])} | {d['tip_out_of_plane_share']:.2f} | {d['mode1_out_of_plane_share']:.2f} | "
            f"{sci(d['tip_block_rel_diff'])} | {', '.join(f'{k}: {v}' for k, v in d['quality_flags'].items()) or '-'} |")
    rows += [
        "",
        "## Published records (tip block only, computed here from the free tip curve)",
        "",
        "| record | source | input span | levels | tip size | tip 2/1 | tip out of XY |",
        "|---|---|---|---|---|---|---|",
    ]
    for d in sorted([d for d in drawn if d["group"] == "published"], key=lambda d: d["id"]):
        ratio = sci(d["tip_ratio_2_to_1"]) if d["tip_ratio_2_to_1"] else "0"
        rows.append(
            f"| {d['id']} | {d['reference'] or '-'} | {span_text(d)} | {d['free_levels']} | "
            f"{sci(d['tip_sqrt_trace'])} {d['sqrt_trace_unit']} | {ratio} | {d['tip_out_of_plane_share']:.2f} |")
    rows += ["", "## Not drawn", "", "| record | reason |", "|---|---|"]
    for lo in left_out:
        rows.append(f"| {lo['id']} | {lo['reason']} |")
    rows.append("")
    return "\r\n".join(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--records", default=RECORDS)
    ap.add_argument("--out-dir", default=OUT)
    a = ap.parse_args(argv)
    drawn, left_out = analyse(a.records)
    os.makedirs(a.out_dir, exist_ok=True)
    base = os.path.join(a.out_dir, STEM)
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump({"made_by": "tools/figure_gramians.py", "drawn": drawn, "left_out": left_out}, f, indent=1)
    with open(base + ".md", "w", encoding="utf-8", newline="") as f:
        f.write(table_md(drawn, left_out))
    draw(drawn, left_out, a.records, base + ".png", base + ".svg")
    print(f"drawn {len(drawn)}, left out {len(left_out)}; wrote {base}.png/.svg/.json/.md")
    for lo in left_out:
        print(f"  not drawn: {lo['id']}: {lo['reason']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
