"""Counts of verdicts in position_turning_<hand>.json files, as the film's counts shot colours them.

    python tools/count_position_turning.py <checks dir> [hands ...]

Position: the position-only follow.py verdict per frame. Turning: green if the touching fingers can hold
the ball ("ok") with torque to spare about the turn above zero, amber if they can hold it with none to
spare, red if they cannot (the film's turn_colour). Also counts unserved frames and touching fingers per
frame. Prints only; writes nothing.
"""
import collections, json, os, sys


def turn_colour(fr):
    if fr["on_axis"] == "ok":
        return "green" if (fr["spare_on_axis_mNm"] or 0) > 0 else "amber"
    return "red" if fr["on_axis"] == "cannot" else str(fr["on_axis"])


def main():
    d = sys.argv[1]
    hands = sys.argv[2:] or ["allegro", "leap", "shadow"]
    for h in hands:
        r = json.load(open(os.path.join(d, f"position_turning_{h}.json"), encoding="utf-8"))
        F = r["frames"]
        pos = collections.Counter(fr["position"] for fr in F)
        turn = collections.Counter(turn_colour(fr) for fr in F)
        touch = collections.Counter(len(fr["touching"]) for fr in F)
        uns = sum(bool(fr["unserved"]) for fr in F)
        print(f"{h}: {len(F)} frames; position {dict(pos)}; turning {dict(turn)}; unserved frames {uns}; "
              f"touching fingers per frame {dict(sorted(touch.items()))}; leader {os.path.basename(str(r['leader']))}")


if __name__ == "__main__":
    main()
