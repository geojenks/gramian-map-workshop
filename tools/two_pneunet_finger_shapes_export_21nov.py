"""Export the settled free shapes of the two-pneu-net finger (member
twist_90deg) for drawing and for fitting, from the 20 Nov shape model.
READS saved SOFA runs through that model; runs no simulation; writes one new
file here and nothing in the other repository.

The 20 Nov model (gramian_map/experiment_log/scripts/
2026-11-20_tandem_knuckle_analysis/tandem_knuckle_model.py, read only) gives
every mesh node's settled place for any pair of the two inputs. This tool
tabulates, on the same 25 x 25 grid of input pairs as the reach check's
refined tip table (0 to 120 scene units, 5-unit steps):

  - the outer skin of the mesh (nodes and triangles, triangles wound so the
    normal points out of the material; cavity walls dropped);
  - the tip face: skin triangles all of whose nodes are in the last 1/12 of
    the finger's length (the 20 Nov analysis's "tip" nodes) and whose rest
    normal is within PAD_CONE_DEG of the direction the tip moves when the
    second input alone is raised (rest to full). Its area-weighted centre and
    outward normal are tabulated per input pair. This is the patch that
    faces the object when the finger curls towards it;
  - N_BODY skin nodes spread evenly over the skin (farthest-point sampling
    at rest, starting from the node nearest the tip face's centre).

PAD_CONE_DEG and N_BODY are ASSUMED and were fixed before the first run.

Needs scipy (the model uses it): run with the Gramain_extraction environment,
with PYTHONDONTWRITEBYTECODE=1 so nothing is written beside the model:
    python tools/two_pneunet_finger_shapes_export_21nov.py --gramian-map <path to gramian_map>
Writes outputs/two_pneunet_fingers_human_like_mounting_21nov/finger_shapes.npz
and finger_shapes_report.txt.
"""
import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_human_like_mounting_21nov")
LEVELS = np.linspace(0.0, 120.0, 25)
PAD_CONE_DEG = 30.0       # ASSUMED
N_BODY = 32               # ASSUMED


def skin_of(tets, rest):
    """Boundary faces wound outwards, split into connected pieces; the piece with the largest box is the skin."""
    faces, opp = [], []
    for a, b, c, d in ((0, 1, 2, 3), (0, 1, 3, 2), (0, 2, 3, 1), (1, 2, 3, 0)):
        faces.append(tets[:, [a, b, c]])
        opp.append(tets[:, d])
    faces, opp = np.concatenate(faces), np.concatenate(opp)
    key = np.sort(faces, 1)
    _, first, count = np.unique(key, axis=0, return_index=True, return_counts=True)
    keep = first[count == 1]
    F, O = faces[keep], opp[keep]
    n = np.cross(rest[F[:, 1]] - rest[F[:, 0]], rest[F[:, 2]] - rest[F[:, 0]])
    flip = (n * (rest[O] - rest[F[:, 0]])).sum(1) > 0          # normal must point away from the tet's fourth node
    F[flip] = F[flip][:, [0, 2, 1]]
    # connected pieces by shared nodes
    lab = np.arange(len(rest))

    def find(i):
        while lab[i] != i:
            lab[i] = lab[lab[i]]
            i = lab[i]
        return i
    for a, b, c in F:
        ra, rb, rc = find(a), find(b), find(c)
        lab[rb] = ra
        lab[find(rc)] = ra
    piece = np.array([find(i) for i in F[:, 0]])
    best, vol = None, -1
    sizes = []
    for p in np.unique(piece):
        nodes = np.unique(F[piece == p])
        v = float(np.prod(np.ptp(rest[nodes], axis=0)))
        sizes.append((int((piece == p).sum()), v))
        if v > vol:
            best, vol = p, v
    return F[piece == best], sizes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gramian-map", required=True)
    args = ap.parse_args()
    sys.dont_write_bytecode = True
    sys.path.insert(0, os.path.join(args.gramian_map, "experiment_log", "scripts", "2026-11-20_tandem_knuckle_analysis"))
    import tandem_knuckle_model as tk
    fg = tk.Finger("twist_90deg")
    rest = np.asarray(fg.shape_corrected(0.0, 0.0), float)
    tets = np.asarray(fg.tets)
    F, sizes = skin_of(tets, rest)
    nodes = np.unique(F)
    remap = -np.ones(len(rest), int)
    remap[nodes] = np.arange(len(nodes))
    tri = remap[F]
    n1 = len(LEVELS)
    V = np.zeros((n1, n1, len(nodes), 3), np.float32)
    tipc = np.zeros((n1, n1, 3))
    for i, a in enumerate(LEVELS):
        for j, b in enumerate(LEVELS):
            X = np.asarray(fg.shape_corrected(float(a), float(b)), float)
            V[i, j] = X[nodes]
            tipc[i, j] = X[fg.tip].mean(0)
    r = V[0, 0].astype(float)
    curl = tipc[0, -1] - tipc[0, 0]
    curl /= np.linalg.norm(curl)
    wag = tipc[-1, 0] - tipc[0, 0]
    wag /= np.linalg.norm(wag)
    istip = np.zeros(len(rest), bool)
    istip[fg.tip] = True
    nr = np.cross(r[tri[:, 1]] - r[tri[:, 0]], r[tri[:, 2]] - r[tri[:, 0]])
    nr /= np.linalg.norm(nr, axis=1, keepdims=True)
    pad = np.where(istip[nodes][tri].all(1) & (nr @ curl > np.cos(np.radians(PAD_CONE_DEG))))[0]
    assert len(pad) > 0, "no tip face found"
    P = np.zeros((n1, n1, 3), np.float32)
    N = np.zeros((n1, n1, 3), np.float32)
    for i in range(n1):
        for j in range(n1):
            v = V[i, j].astype(float)
            t = tri[pad]
            cr = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
            ar = np.linalg.norm(cr, axis=1)
            P[i, j] = (v[t].mean(1) * ar[:, None]).sum(0) / ar.sum()
            N[i, j] = cr.sum(0) / np.linalg.norm(cr.sum(0))
    # body nodes: farthest-point sampling over the rest skin
    pick = [int(np.argmin(np.linalg.norm(r - P[0, 0], axis=1)))]
    d = np.linalg.norm(r - r[pick[0]], axis=1)
    while len(pick) < N_BODY:
        k = int(np.argmax(d))
        pick.append(k)
        d = np.minimum(d, np.linalg.norm(r - r[k], axis=1))
    os.makedirs(OUT, exist_ok=True)
    f = os.path.join(OUT, "finger_shapes.npz")
    np.savez_compressed(f, levels=LEVELS, skin=V, tri=tri.astype(np.int32), pad_tri=pad.astype(np.int32), pad=P, pad_normal=N,
                        tip_centre=tipc.astype(np.float32), body_nodes=np.array(pick, np.int32),
                        curl_dir=curl, wag_dir=wag)
    ptp = np.ptp(r, axis=0) * 1e3
    area = 0.5 * np.linalg.norm(np.cross(r[tri[pad][:, 1]] - r[tri[pad][:, 0]], r[tri[pad][:, 2]] - r[tri[pad][:, 0]]), axis=1).sum()
    rep = ["Settled free shapes of the two-pneu-net finger (twist_90deg), exported from the 20 Nov shape model",
           "No simulation run here. Comparative, not predictive.", "",
           f"Mesh: {len(rest)} nodes, {len(tets)} tetrahedra. Boundary pieces (triangles, box volume m^3): {sizes}.",
           f"Outer skin kept: {len(nodes)} nodes, {len(tri)} triangles. Rest box {ptp[0]:.1f} x {ptp[1]:.1f} x {ptp[2]:.1f} mm; "
           f"rest x from {r[:, 0].min() * 1e3:.2f} to {r[:, 0].max() * 1e3:.2f} mm (clamped end at the low end).",
           f"Input pairs: {n1} x {n1}, each input {LEVELS[0]:g} to {LEVELS[-1]:g} in steps of {LEVELS[1] - LEVELS[0]:g}.",
           f"Tip centre at rest {np.round(tipc[0, 0] * 1e3, 2).tolist()} mm; first input alone moves it "
           f"{np.round((tipc[-1, 0] - tipc[0, 0]) * 1e3, 2).tolist()} mm, second input alone "
           f"{np.round((tipc[0, -1] - tipc[0, 0]) * 1e3, 2).tolist()} mm.",
           f"Direction of the second input's tip travel (the curl), unit: {np.round(curl, 3).tolist()}; of the first input's "
           f"(the wag): {np.round(wag, 3).tolist()}.",
           f"Tip face: {len(pad)} triangles, area {area * 1e6:.1f} mm^2, within {PAD_CONE_DEG:g} deg of the curl direction (ASSUMED); "
           f"centre at rest {np.round(P[0, 0] * 1e3, 2).tolist()} mm, outward normal {np.round(N[0, 0], 3).tolist()}; "
           f"it sits {np.linalg.norm(P[0, 0] - tipc[0, 0]) * 1e3:.2f} mm from the tip centre.",
           f"Body nodes: {N_BODY} skin nodes, farthest-point sampling (ASSUMED number).",
           f"File: {os.path.relpath(f, ROOT)} ({os.path.getsize(f) / 1e6:.1f} MB)."]
    with open(os.path.join(OUT, "finger_shapes_report.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    print("\n".join(rep))


if __name__ == "__main__":
    main()
