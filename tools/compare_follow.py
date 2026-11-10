"""Compare a re-run follow result with the stored one it repeats, frame by
frame: the frames checked, each frame's verdict and reason, each finger's
state, reason and station, the placement and assignment, the settings, and
the scale margin where both carry it.

    python tools/compare_follow.py NEW.json STORED.json [NEW STORED ...] [--out table.md]

Outcome differences are in the frames, verdicts, finger states, placement,
assignment or scale margin. A setting that differs is listed apart: the
check may have changed since the stored run without changing its outcome.
Settings that only say which margins were asked for, or keys added to the
result after the stored run, are listed as notes. Prints one line per pair
and, with --out, writes the same as a Markdown table. Exits 1 if any pair
differs in outcome.
"""
import argparse
import json
import os
import sys

# settings that say what extra readings were asked for, not how the
# verdict was reached
MARGIN_SETTINGS = ("scale_margin", "disturbance_margin")


def _close(a, b, rel=1e-9):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= rel * max(1.0, abs(a), abs(b))
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_close(x, y, rel) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_close(a[k], b[k], rel) for k in a)
    return a == b


def compare(new, old):
    """(differences, settings, notes): lists of plain-language strings.
    Differences are in the outcome and must not occur; settings are the
    settings that differ; notes are what may change without touching the
    verdict (new settings keys, margin settings)."""
    diff, sets, notes = [], [], []
    for key in ("hand", "leader", "assignment"):
        if new.get(key) != old.get(key):
            diff.append(f"{key}: {old.get(key)} -> {new.get(key)}")
    if not _close(new.get("placement"), old.get("placement")):
        diff.append("placement differs")
    if not _close(new.get("respace_deg"), old.get("respace_deg")):
        diff.append("respace_deg differs")
    sn, so = new.get("settings", {}), old.get("settings", {})
    for k in sorted(set(sn) | set(so)):
        if k in MARGIN_SETTINGS:
            if sn.get(k) != so.get(k):
                notes.append(f"setting {k}: {so.get(k)} -> {sn.get(k)}")
        elif k == "reused_placement":
            if sn.get(k) != so.get(k):
                notes.append(f"setting {k}: {so.get(k)} -> {sn.get(k)}")
        elif k not in so:
            notes.append(f"setting {k} new since the stored run")
        elif k not in sn:
            sets.append(f"setting {k} gone")
        elif not _close(sn[k], so[k]):
            sets.append(f"setting {k}: {so[k]!r} -> {sn[k]!r}")
    fn, fo = new["frames"], old["frames"]
    if [f["k"] for f in fn] != [f["k"] for f in fo]:
        diff.append(f"frames checked: {[f['k'] for f in fo]} -> {[f['k'] for f in fn]}")
        return diff, sets, notes
    for a, b in zip(fn, fo):
        k = a["k"]
        if (a["verdict"], a["reason"]) != (b["verdict"], b["reason"]):
            diff.append(f"frame {k}: {b['verdict']} ({b['reason']}) -> {a['verdict']} ({a['reason']})")
        xa = {x["finger"]: x for x in a["fingers"]}
        xb = {x["finger"]: x for x in b["fingers"]}
        if xa.keys() != xb.keys():
            diff.append(f"frame {k}: fingers {sorted(xb)} -> {sorted(xa)}")
            continue
        for name in xb:
            for field in ("state", "reason", "station"):
                if xa[name].get(field) != xb[name].get(field):
                    diff.append(f"frame {k}, {name} {field}: {xb[name].get(field)} -> {xa[name].get(field)}")
        if isinstance(a.get("scale_margin"), dict) and isinstance(b.get("scale_margin"), dict):
            if not _close(a["scale_margin"], b["scale_margin"]):
                diff.append(f"frame {k}: scale margin {b['scale_margin']} -> {a['scale_margin']}")
        elif ("scale_margin" in a) and ("scale_margin" in b) and a["scale_margin"] != b["scale_margin"]:
            diff.append(f"frame {k}: scale margin {b['scale_margin']} -> {a['scale_margin']}")
    return diff, sets, notes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+", help="pairs: NEW STORED")
    ap.add_argument("--out", default=None, help="also write a Markdown table here")
    a = ap.parse_args()
    if len(a.paths) % 2:
        ap.error("give the results in pairs: NEW STORED")
    rows, bad = [], 0
    for pn, po in zip(a.paths[::2], a.paths[1::2]):
        with open(pn, encoding="utf-8") as f:
            new = json.load(f)
        with open(po, encoding="utf-8") as f:
            old = json.load(f)
        diff, sets, notes = compare(new, old)
        bad += bool(diff)
        n_fing = sum(len(fr["fingers"]) for fr in old["frames"])
        rows.append((os.path.basename(pn), os.path.basename(po), len(old["frames"]), n_fing,
                     "; ".join(diff) or "same", "; ".join(sets) or "none", "; ".join(notes)))
        print(f"{os.path.basename(pn)} vs {os.path.basename(po)}: {len(old['frames'])} frames, "
              f"{n_fing} finger states: " + ("; ".join(diff) if diff else "same")
              + ("; settings that differ: " + "; ".join(sets) if sets else ""), flush=True)
    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\r\n") as f:
            f.write("# Re-run against stored result\n\n"
                    "Written by `tools/compare_follow.py`. \"same\" means the frames checked, "
                    "each frame's verdict and reason, each finger's state, reason and station, "
                    "the placement, the assignment and, where both carry it, the scale margin "
                    "are equal. Settings that differ are listed apart. Notes list settings that "
                    "only say which margins were asked for, and keys added since the stored "
                    "run.\n\n"
                    "| Re-run | Stored | Frames | Finger states | Outcome | Settings that differ | Notes |\n"
                    "|---|---|---|---|---|---|---|\n")
            for r in rows:
                f.write("| " + " | ".join(f"`{c}`" if i < 2 else str(c).replace("|", "/")
                                          for i, c in enumerate(r)) + " |\n")
        print("wrote", a.out)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
