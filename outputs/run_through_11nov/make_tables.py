"""The two tables of docs/RUN_THROUGH.md, from the files stored beside this
script. Prints markdown; writes nothing.

    python outputs/run_through_11nov/make_tables.py > tables.md

Steps: every step of the four logs (logs/*_log.txt), with its command, exit
code and wall time, as the logging wrapper wrote them ("=== name",
"$ command", "--- name: exit N in T s").
Verdicts: one row per follow result in results/, read from the result file.
"""
import glob
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = ["run0", "run", "run3", "run2"]   # the order the page tells them in
PY = {"<hands env>/Scripts/python.exe": "python (hands)",
      "<analysis env>/Scripts/python.exe": "python (analysis)"}


def steps(name):
    out, cmd = [], None
    with open(os.path.join(HERE, "logs", name + "_log.txt"), encoding="utf-8") as f:
        for ln in f:
            ln = ln.rstrip("\r\n")
            if ln.startswith("$ "):
                cmd = ln[2:]
                for k, v in PY.items():
                    cmd = cmd.replace(k, v)
            m = re.match(r"--- (.+): exit (\d+) in (\d+) s$", ln)
            if m:
                out.append((m.group(1), cmd, int(m.group(2)), int(m.group(3))))
                cmd = None
    return out


def smallest(frames, key):
    got = [(fr[key]["margin"], fr["k"], fr[key].get("direction"), bool(fr[key].get("capped")))
           for fr in frames if isinstance(fr.get(key), dict) and fr[key].get("margin") is not None]
    return min(got) if got else None


def past_limits(frames):
    by = {}
    for fr in frames:
        for x in fr["fingers"]:
            for b in x.get("beyond_limit") or []:
                by.setdefault((b["joint"], b["side"]), []).append(b["by"])
    return by


def main():
    print("## Steps\n")
    print("| Log | Step | Command | Exit | Seconds |")
    print("|---|---|---|---|---|")
    total = 0
    for name in LOGS:
        for step, cmd, rc, sec in steps(name):
            total += sec
            print(f"| `logs/{name}_log.txt` | {step} | `{cmd}` | {rc} | {sec} |")
    print(f"\nAll steps together: {total} s.\n")

    print("## Verdicts\n")
    print("| Result | Verdicts | Scale margin | Disturbance margin | Joints posed past a limit "
          "| Follow time (s) | Units caveat in the result |")
    print("|---|---|---|---|---|---|---|")
    for p in sorted(glob.glob(os.path.join(HERE, "results", "follow_*.json"))):
        with open(p, encoding="utf-8") as f:
            r = json.load(f)
        fr = r["frames"]
        counts = {}
        for x in fr:
            counts[x["verdict"]] = counts.get(x["verdict"], 0) + 1
        verdicts = ", ".join(f"{v} {n}/{len(fr)}" for v, n in sorted(counts.items()))
        reasons = sorted({x["reason"] for x in fr if x.get("reason")})
        if reasons:
            verdicts += " (" + ", ".join(reasons) + ")"
        sm, dm = smallest(fr, "scale_margin"), smallest(fr, "disturbance_margin")
        s_txt = f"{sm[0]:.3g}" if sm else "not computed"
        d_txt = f"{dm[0]:.3g} ({dm[2]})" if dm else "not computed"
        pl = past_limits(fr)
        p_txt = "; ".join(f"{j} {'upper' if s == 'hi' else 'lower'} by {min(v):.3f} to {max(v):.3f} "
                          f"in {len(v)} finger-frames" for (j, s), v in sorted(pl.items())) or "none"
        cav = r.get("caveat") or ""
        c_txt = "yes" if "PROVISIONAL" in cav else "no"
        print(f"| `results/{os.path.basename(p)}` | {verdicts} | {s_txt} | {d_txt} | {p_txt} "
              f"| {r['elapsed_s']:.1f} | {c_txt} |")


if __name__ == "__main__":
    main()
