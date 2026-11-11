"""Which fingers and joints fail, and when, for stored follow results.

    python tools/report_failures.py outputs/follow_<hand>__<leader>.json [...]
        [--out-dir DIR] [--tag TAG] [--print-only]

For each result it prints a few lines (the fingers that are amber or red and
over which frames, the joints posed past a limit) and writes two files next
to the result, or into --out-dir:

    <result name>__failures[__TAG].md     the tables as text
    <result name>__failures[__TAG].html   one page: a finger-by-frame grid and
                                          a joint-by-frame grid, hover for the
                                          numbers

It reads the result file only: nothing is re-checked, and it needs neither
mujoco nor the hand models. An existing file is never overwritten (give a
--tag). --print-only writes nothing.

What it can and cannot say is in gramian_workshop/failures.py. For a result
that carries the keys of issue 29 it also lists each finger's own tests
missed and, joint by joint, what a torque test left unbalanced, naming the
joint that carries the largest residual. For a result without them the torque left
unbalanced is per finger and the joint is not named.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import failures  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("results", nargs="+")
    ap.add_argument("--out-dir")
    ap.add_argument("--tag")
    ap.add_argument("--print-only", action="store_true")
    a = ap.parse_args(argv)
    for p in a.results:
        with open(p, encoding="utf-8") as f:
            r = json.load(f)
        if a.print_only:
            print(failures.brief(failures.summarise(r)))
            continue
        stem = os.path.splitext(os.path.basename(p))[0]
        s, paths = failures.write(r, stem, a.out_dir or os.path.dirname(p) or ".",
                                  tag=a.tag)
        print(failures.brief(s))
        for q in paths:
            print("  wrote", q)


if __name__ == "__main__":
    main()
