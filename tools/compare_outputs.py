"""Compare a JSON output you just made with the one stored in git.

    python tools/compare_outputs.py outputs/cup_hold.json [more.json ...]

Numbers must agree to within --tol (absolute, default 1e-9); round-off from
a different machine's maths library is far below that. Everything else must
match exactly. Run-time fields (elapsed_s) are ignored. Exit code 0 means
every file reproduces the stored one.
"""
import argparse
import json
import subprocess
import sys

IGNORE = {"elapsed_s"}


def diff(a, b, path, tol, out):
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        if a != b:
            out.append((path, a, b))
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if abs(a - b) > tol:
            out.append((path, a, b))
    elif isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k in IGNORE:
                continue
            if k not in a or k not in b:
                out.append((f"{path}/{k}", a.get(k, "<missing>"), b.get(k, "<missing>")))
            else:
                diff(a[k], b[k], f"{path}/{k}", tol, out)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append((f"{path} (length)", len(a), len(b)))
        for i, (x, y) in enumerate(zip(a, b)):
            diff(x, y, f"{path}[{i}]", tol, out)
    elif a != b:
        out.append((path, a, b))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--tol", type=float, default=1e-9)
    ap.add_argument("--ref", default="HEAD", help="git revision holding the stored files")
    a = ap.parse_args()
    bad = 0
    for f in a.files:
        stored = json.loads(subprocess.check_output(
            ["git", "show", f"{a.ref}:{f.replace(chr(92), '/')}"]))
        with open(f, encoding="utf-8") as fh:
            new = json.load(fh)
        out = []
        diff(stored, new, "", a.tol, out)
        if out:
            bad += 1
            print(f"{f}: {len(out)} differences from {a.ref}")
            for p, x, y in out[:20]:
                print(f"  {p or '/'}: stored {x!r}, now {y!r}")
        else:
            print(f"{f}: reproduces {a.ref} (numbers within {a.tol:g})")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
