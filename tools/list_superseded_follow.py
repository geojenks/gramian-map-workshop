"""List the stored follow results (outputs/follow_*.json) that the plan's
section "Stored results that predate the current check"
(docs/MINIMAL_PRODUCT_PLAN.md) supersedes, and so need a fresh placement
search before they can be quoted, with a count.

    python tools/list_superseded_follow.py [--out list.md]

The rules, as the plan states them:
  - first committed before 6922372 (26 Oct), the check that a finger must
    not sink into the object;
  - on a stored w10_cup_physical_* leader that is not a __contactfix copy
    (issue 22);
  - a __provisional file (written before equilibrium posing, 89a6b86).
A result may meet more than one rule. Results with a __margins suffix are
re-runs, not stored verdicts, and are left out. Results that meet no rule
are listed apart: they are not superseded by these rules.
"""
import argparse
import glob
import os
import re
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECK_COMMIT = "6922372"


def _git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True,
                          check=True).stdout.strip()


def first_commit_time(rel):
    """Unix time of the first commit that added `rel`, or None if untracked."""
    out = _git("log", "--diff-filter=A", "--format=%ct", "--", rel)
    return int(out.splitlines()[-1]) if out else None


def rules(rel, t, t_check):
    name = os.path.basename(rel)
    why = []
    if t is not None and t < t_check:
        why.append(f"first committed before {CHECK_COMMIT}")
    if re.search(r"__w10_cup_physical_", name) and "__contactfix" not in name:
        why.append("stored w10 cup leader, not a __contactfix copy")
    if "__provisional" in name:
        why.append("provisional (before equilibrium posing)")
    return why


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="also write the list as Markdown here")
    a = ap.parse_args()
    t_check = int(_git("log", "-1", "--format=%ct", CHECK_COMMIT))
    sup, other = [], []
    for p in sorted(glob.glob(os.path.join(ROOT, "outputs", "follow_*.json"))):
        rel = os.path.relpath(p, ROOT).replace(os.sep, "/")
        if "__margins" in rel:
            continue
        why = rules(rel, first_commit_time(rel), t_check)
        (sup if why else other).append((rel, why))
    lines = [f"Superseded, need a fresh placement search: {len(sup)}"]
    lines += [f"- `{rel}`: {'; '.join(why)}" for rel, why in sup]
    lines += ["", f"Not superseded by these rules: {len(other)}"]
    lines += [f"- `{rel}`" for rel, _ in other]
    text = "\n".join(lines)
    print(text)
    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\r\n") as f:
            f.write("# Stored follow results superseded by the current check\n\n"
                    "Written by `tools/list_superseded_follow.py`; the rules are in its "
                    "docstring.\n\n" + text + "\n")


if __name__ == "__main__":
    main()
