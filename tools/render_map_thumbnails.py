"""Draw the small picture of each actuator shown on the map page.

Reads data/map_thumbnails.json and writes one PNG per entry under site/.

Two kinds of picture:

* "mesh": the rest shape of a simulated actuator, drawn from the mesh the
  simulation ran on (actuator.npz in a Gramain_extraction checkout, plus the
  braid or tendon file where the entry lists one). Every file is pinned in
  the sidecar by commit, path and sha256; a file whose sha256 differs is
  refused. Nothing is written to that checkout.
* "schematic": a plain diagram of the type of actuator, drawn here from the
  description in the entry's "basis". It is not the geometry of the published
  device and no figure from a paper is copied.

    python tools/render_map_thumbnails.py            # draw every picture
    python tools/render_map_thumbnails.py --pin      # fill in the pins first
    python tools/render_map_thumbnails.py --only bellows1 mosadegh2014_fPN

The mesh checkout is GRAMAIN_EXTRACTION if set, else ../ARIA/Gramain_extraction
beside this repository. --pin reads each file from the checkout's HEAD commit
and refuses a file that is not tracked or differs from it. All pictures are
of the rest shape, each scaled to fill its own frame: they are not to a
common scale. The script ends by printing which records got real geometry
and which a schematic.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIDECAR = os.path.join(ROOT, "data", "map_thumbnails.json")
SITE = os.path.join(ROOT, "site")
MESH_REPO_NAME = "Gramain_extraction"
SIZE = 192                      # pixels, each side of the written picture
# one fixed view for every mesh: from beyond the tip, to one side and above,
# so the tip is the near end and the base the far one
VIEW_FROM = (0.62, 0.60, 0.50)
VIEW_UP = (0.0, 0.0, 1.0)
BODY = "#c3c8cf"                # silicone
LIMITER = "#7f8894"             # the stiffer second material, where present
LINE = "#30353c"                # braid, tendon, schematic ink
FILL = "#c3c8cf"
FILL_DARK = "#7f8894"


def mesh_repo():
    """The Gramain_extraction checkout, or None."""
    for p in (os.environ.get("GRAMAIN_EXTRACTION"),
              os.path.join(os.path.dirname(ROOT), "ARIA", MESH_REPO_NAME)):
        if p and os.path.isdir(os.path.join(p, ".git")) or (
                p and os.path.isfile(os.path.join(p, ".git"))):
            return p
    return None


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def sha256_file(path):
    with open(path, "rb") as fo:
        return sha256_bytes(fo.read())


def _git(repo, *args):
    return subprocess.run(["git", "-C", repo, *args], capture_output=True,
                          check=True).stdout


def pin_sources(doc, repo):
    """Fill commit and sha256 into every mesh source of the sidecar from the
    checkout's HEAD. Raises ValueError for a file HEAD does not hold or that
    differs on disk from HEAD."""
    head = _git(repo, "rev-parse", "HEAD").decode().strip()
    for rid, e in doc["records"].items():
        for src in e.get("sources") or []:
            try:
                blob = _git(repo, "show", f"{head}:{src['path']}")
            except subprocess.CalledProcessError:
                raise ValueError(f"{rid}: {src['path']} is not in commit "
                                 f"{head} of {MESH_REPO_NAME}")
            disk = sha256_file(os.path.join(repo, src["path"]))
            if disk != sha256_bytes(blob):
                raise ValueError(f"{rid}: {src['path']} on disk differs from "
                                 f"commit {head}")
            src["repo"], src["commit"], src["sha256"] = (MESH_REPO_NAME, head,
                                                         disk)
    return doc


def boundary_faces(pts, tets, groups):
    """The triangles used by exactly one tetrahedron, wound so each faces
    out of the tetrahedron that owns it, with that tetrahedron's group
    number. groups is one number per tet."""
    idx = np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]])
    faces = tets[:, idx].reshape(-1, 3)
    fourth = tets[:, [3, 2, 1, 0]].reshape(-1)
    owner = np.repeat(np.asarray(groups), 4)
    key = np.sort(faces, axis=1)
    _, first, count = np.unique(key, axis=0, return_index=True,
                                return_counts=True)
    keep = first[count == 1]
    faces, fourth, owner = faces[keep], fourth[keep], owner[keep]
    a, b, c = (pts[faces[:, k]] for k in range(3))
    inward = np.einsum("ij,ij->i", np.cross(b - a, c - a),
                       pts[fourth] - a) > 0
    faces[inward] = faces[inward][:, ::-1]
    return faces, owner


def _lines(nodes, edges):
    import pyvista as pv
    cells = np.hstack([np.full((len(edges), 1), 2), edges]).ravel()
    return pv.PolyData(np.asarray(nodes, float), lines=cells)


def _square(png_path, size=SIZE, pad=0.06):
    """Crop the picture to what is drawn, centre it in a square with a
    margin and scale it to size x size."""
    from PIL import Image
    im = Image.open(png_path).convert("RGBA")
    box = im.getchannel("A").getbbox()
    if box is None:
        raise ValueError(f"{png_path}: nothing was drawn")
    im = im.crop(box)
    side = int(round(max(im.size) * (1 + 2 * pad)))
    out = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    out.paste(im, ((side - im.width) // 2, (side - im.height) // 2))
    out.resize((size, size), Image.LANCZOS).save(png_path)


def draw_mesh(entry, repo, out_path):
    """The rest shape from the entry's pinned files."""
    import pyvista as pv
    by_part = {}
    for src in entry["sources"]:
        path = os.path.join(repo, src["path"])
        got = sha256_file(path)
        if got != src.get("sha256"):
            raise ValueError(f"{src['path']} has sha256 {got}, not the "
                             f"pinned {src.get('sha256')}")
        by_part[src["draws"]] = np.load(path, allow_pickle=True)
    body = by_part["body"]
    pts = np.asarray(body["points"], float)
    tets = np.asarray(body["tet_nodes"], int)
    group = np.zeros(len(tets), int)
    if "limiter_tet_indices" in body.files:
        group[np.asarray(body["limiter_tet_indices"], int)] = 1
    faces, owner = boundary_faces(pts, tets, group)
    pl = pv.Plotter(off_screen=True, window_size=(640, 640))
    for g, colour in ((0, BODY), (1, LIMITER)):
        f = faces[owner == g]
        if not len(f):
            continue
        surf = pv.PolyData(pts, np.hstack([np.full((len(f), 1), 3),
                                           f]).ravel())
        # a body with cables inside is drawn part see-through, so they show
        pl.add_mesh(surf, color=colour, smooth_shading=False,
                    ambient=0.28, diffuse=0.75, specular=0.05,
                    opacity=0.45 if "cables" in by_part else 1.0)
    span = float(np.ptp(pts, axis=0).max())
    if "braid" in by_part:
        b = by_part["braid"]
        pl.add_mesh(_lines(b["braid_nodes"], b["braid_edges"]).tube(
            radius=0.0045 * span), color=LINE)
    if "tendon" in by_part:
        t = by_part["tendon"]
        pl.add_mesh(_lines(t["tendon_nodes"], t["tendon_edges"]).tube(
            radius=0.008 * span), color=LINE)
    if "cables" in by_part:
        c = by_part["cables"]
        for k in range(len(c["cable_names"])):
            p = np.asarray(c[f"cable_{k}_points"], float)
            e = np.column_stack([np.arange(len(p) - 1),
                                 np.arange(1, len(p))])
            pl.add_mesh(_lines(p, e).tube(radius=0.008 * span), color=LINE)
            pl.add_mesh(pv.PolyData(p).glyph(
                geom=pv.Sphere(radius=0.016 * span), scale=False,
                orient=False), color=LINE)
    centre = (pts.min(0) + pts.max(0)) / 2
    pl.camera.position = tuple(centre + 10 * span * np.array(VIEW_FROM))
    pl.camera.focal_point = tuple(centre)
    pl.camera.up = VIEW_UP
    pl.enable_parallel_projection()
    pl.reset_camera()
    pl.screenshot(out_path, transparent_background=True)
    pl.close()
    _square(out_path)


# ---- schematics: plain diagrams of a type, in a unit square ----------------

def _s_sleeve_bellows(ax):
    """A sleeve whose wall is folded like a bellows, seen from the side."""
    from matplotlib.patches import Polygon
    n = 7
    x = np.linspace(0.12, 0.88, 2 * n + 1)
    amp = np.where(np.arange(2 * n + 1) % 2 == 0, 0.20, 0.30)
    for sign in (1, -1):
        outer = np.column_stack([x, 0.5 + sign * amp])
        inner = np.column_stack([x[::-1], np.full(len(x), 0.5 + sign * 0.10)])
        ax.add_patch(Polygon(np.vstack([outer, inner]), closed=True,
                             fc=FILL, ec=LINE, lw=1.4, joinstyle="round"))


def _s_pneunet(ax):
    """A row of chambers standing on a flat strip."""
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((0.08, 0.30), 0.84, 0.09, fc=FILL_DARK, ec=LINE,
                           lw=1.4))
    n, gap = 6, 0.035
    w = (0.84 - (n - 1) * gap) / n
    for k in range(n):
        ax.add_patch(Rectangle((0.08 + k * (w + gap), 0.39), w, 0.31,
                               fc=FILL, ec=LINE, lw=1.4))


def _s_parallel_helix(ax):
    """A rod with three chambers running along it, round a core."""
    from matplotlib.patches import Ellipse, Rectangle
    ax.add_patch(Rectangle((0.10, 0.34), 0.70, 0.32, fc=FILL, ec=LINE,
                           lw=1.4))
    ax.add_patch(Ellipse((0.10, 0.5), 0.14, 0.32, fc=FILL, ec=LINE, lw=1.4))
    ax.add_patch(Ellipse((0.80, 0.5), 0.14, 0.32, fc="#e3e6ea", ec=LINE,
                         lw=1.4))
    for dx, dy in ((0.0, 0.085), (-0.032, -0.05), (0.032, -0.05)):
        ax.add_patch(Ellipse((0.80 + dx, 0.5 + dy), 0.04, 0.075, fc=FILL_DARK,
                             ec=LINE, lw=1.0))
    for y in (0.585, 0.45):
        ax.plot([0.12, 0.76], [y, y], color=LINE, lw=1.0, ls=(0, (4, 3)))


def _s_coiled_fibre(ax):
    """A fibre wound into a coil, like a spring."""
    t = np.linspace(0, 1, 600)
    turns = 9
    x = 0.12 + 0.76 * t + 0.035 * np.cos(2 * np.pi * turns * t)
    y = 0.5 + 0.20 * np.sin(2 * np.pi * turns * t)
    ax.plot(x, y, color=LINE, lw=2.6, solid_capstyle="round")
    ax.plot(x, y, color=FILL, lw=1.1, solid_capstyle="round")


def _vine_body(ax):
    from matplotlib.patches import FancyBboxPatch
    ax.add_patch(FancyBboxPatch((0.08, 0.30), 0.84, 0.22,
                                boxstyle="round,pad=0,rounding_size=0.11",
                                fc="#e3e6ea", ec=LINE, lw=1.4))


def _s_vine_pouch(ax):
    """An inflated tube with a line of flat rectangular pouches along it."""
    from matplotlib.patches import Rectangle
    _vine_body(ax)
    for k in range(5):
        ax.add_patch(Rectangle((0.14 + k * 0.148, 0.52), 0.128, 0.10,
                               fc=FILL_DARK, ec=LINE, lw=1.4))


def _s_vine_cpam(ax):
    """An inflated tube with a line of short round muscles along it."""
    from matplotlib.patches import Ellipse
    _vine_body(ax)
    for k in range(5):
        ax.add_patch(Ellipse((0.204 + k * 0.148, 0.585), 0.132, 0.15,
                             fc=FILL_DARK, ec=LINE, lw=1.4))


def _s_vine_fpam(ax):
    """An inflated tube with one long thin tube along it."""
    from matplotlib.patches import FancyBboxPatch
    _vine_body(ax)
    ax.add_patch(FancyBboxPatch((0.14, 0.52), 0.72, 0.09,
                                boxstyle="round,pad=0,rounding_size=0.045",
                                fc=FILL_DARK, ec=LINE, lw=1.4))


def _s_buckling_block(ax):
    """A block with rows of cavities separated by thin beams."""
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((0.14, 0.24), 0.72, 0.52, fc=FILL, ec=LINE,
                           lw=1.4))
    for r in range(3):
        n = 4 if r % 2 == 0 else 3
        x0 = 0.19 if r % 2 == 0 else 0.275
        for k in range(n):
            ax.add_patch(Rectangle((x0 + k * 0.17, 0.30 + r * 0.15), 0.11,
                                   0.10, fc="#ffffff", ec=LINE, lw=1.0))


SCHEMATICS = {
    "sleeve_bellows": _s_sleeve_bellows,
    "pneunet": _s_pneunet,
    "parallel_helix": _s_parallel_helix,
    "coiled_fibre": _s_coiled_fibre,
    "vine_pouch": _s_vine_pouch,
    "vine_cpam": _s_vine_cpam,
    "vine_fpam": _s_vine_fpam,
    "buckling_block": _s_buckling_block,
}


def draw_schematic(entry, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(SIZE / 96, SIZE / 96), dpi=96)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect("equal")
    ax.axis("off")
    SCHEMATICS[entry["schematic"]](ax)
    fig.savefig(out_path, transparent=True, dpi=96)
    plt.close(fig)


def load(path=SIDECAR):
    with open(path, encoding="utf-8") as fo:
        return json.load(fo)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pin", action="store_true",
                    help="fill commit and sha256 into the sidecar from the "
                         "mesh checkout's HEAD, then draw")
    ap.add_argument("--only", nargs="*", help="record ids to draw")
    args = ap.parse_args(argv)
    doc = load()
    repo = mesh_repo()
    if args.pin:
        if repo is None:
            sys.exit("no Gramain_extraction checkout (set GRAMAIN_EXTRACTION)")
        doc = pin_sources(doc, repo)
        with open(SIDECAR, "w", encoding="utf-8", newline="\n") as fo:
            fo.write(json.dumps(doc, indent=1, ensure_ascii=False) + "\n")
    done, real, plain = set(), [], []
    for rid, e in doc["records"].items():
        (real if e["kind"] == "mesh" else plain).append(rid)
        if args.only and rid not in args.only:
            continue
        out = os.path.join(SITE, *e["image"].split("/"))
        if out in done:
            continue
        os.makedirs(os.path.dirname(out), exist_ok=True)
        if e["kind"] == "mesh":
            if repo is None:
                sys.exit("no Gramain_extraction checkout "
                         "(set GRAMAIN_EXTRACTION)")
            draw_mesh(e, repo, out)
        else:
            draw_schematic(e, out)
        done.add(out)
        print("wrote", out)
    print(f"{len(done)} pictures written.")
    print(f"real geometry ({len(real)}): " + ", ".join(real))
    print(f"schematic of the type ({len(plain)}): " + ", ".join(
        f"{r} [{doc['records'][r]['schematic']}]" for r in plain))


if __name__ == "__main__":
    main()
