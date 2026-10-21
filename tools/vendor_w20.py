"""Vendor the W20 readout's numpy-only core, verbatim, from Wave_manip.

    python tools/vendor_w20.py --wave-manip ../wave_manip

Copies named top-level functions and constants BY SOURCE TEXT (ast source
segments, never retyped) from
  unified_v6/bridge/stability.py       -> gramian_workshop/vendor/stability_core.py
  a3h_sandbox/w20_capability.py        -> gramian_workshop/vendor/capability_core.py
and stamps each file with the source commit and sha256. The MuJoCo-bound
parts of w20_capability.py (HandConfig, point_jacobian, friction_use) are
left behind: the workshop builds its Jacobians from records instead.
tests/test_vendor.py re-extracts and checks the copies are identical.
"""
import argparse
import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _provenance import git_commit_for, sha256  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.join(os.path.dirname(HERE), "gramian_workshop", "vendor")

SOURCES = {
    "stability_core.py": {
        "src": "unified_v6/bridge/stability.py",
        "names": ["nnls", "_tangent_basis", "cone_generators"],
        "imports": "import math\n\nimport numpy as np\n",
    },
    "capability_core.py": {
        "src": "a3h_sandbox/w20_capability.py",
        "names": ["CONE_EDGES", "OUTER_FACTOR", "KAPPA_OLD", "FEAS_REL",
                  "RANK_REL", "BISECT_TOL", "SCALE_STEP", "SCALE_MAX",
                  "SCALE_GRID", "NNLS_ITER_MULT", "NNLS_RECHECK_MULT",
                  "REASONS", "R_NOCON", "nnls_iter", "rank", "scale_wrench",
                  "direction_basis", "generators", "torque_matrix", "solve",
                  "direction_test", "friction_test", "torque_system",
                  "torque_test", "scale_margin", "skew", "input_count"],
        "imports": ("import math\n\nimport numpy as np\n\n"
                    "from . import stability_core as stability\n"),
    },
}


def node_names(node):
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
        return [node.name]
    if isinstance(node, ast.Assign):
        out = []
        for t in node.targets:
            if isinstance(t, ast.Name):
                out.append(t.id)
            elif isinstance(t, ast.Tuple):
                out += [e.id for e in t.elts if isinstance(e, ast.Name)]
        return out
    return []


def extract(path, names):
    """Source segments of the top-level nodes defining `names`, in source
    order (a tuple assignment counts once)."""
    with open(path) as f:
        text = f.read()
    tree = ast.parse(text)
    want = set(names)
    segs, got = [], set()
    for node in tree.body:
        hit = set(node_names(node)) & want
        if hit:
            segs.append(ast.get_source_segment(text, node))
            got |= set(node_names(node))
    missing = want - got
    if missing:
        raise SystemExit(f"{path}: not found {sorted(missing)}")
    return segs


def render(spec, wm):
    src = os.path.join(wm, spec["src"])
    segs = extract(src, spec["names"])
    head = (
        '"""VENDORED VERBATIM -- do not edit by hand; regenerate with\n'
        "tools/vendor_w20.py.\n\n"
        f"Source: geojenks/Wave_manip {spec['src']}\n"
        f"Source commit (last to touch the file): "
        f"{git_commit_for(wm, spec['src'])}\n"
        f"Source sha256: {sha256(src)}\n\n"
        "Only the named top-level definitions are copied, by source text.\n"
        '"""\n')
    return head + spec["imports"] + "\n\n" + "\n\n\n".join(segs) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    a = ap.parse_args()
    wm = os.path.abspath(a.wave_manip)
    os.makedirs(VENDOR, exist_ok=True)
    for out, spec in SOURCES.items():
        p = os.path.join(VENDOR, out)
        with open(p, "w") as f:
            f.write(render(spec, wm))
        print(f"wrote {p}")
    init = os.path.join(VENDOR, "__init__.py")
    if not os.path.exists(init):
        with open(init, "w") as f:
            f.write('"""Verbatim copies of the W20 readout core '
                    '(see tools/vendor_w20.py)."""\n')


if __name__ == "__main__":
    main()
