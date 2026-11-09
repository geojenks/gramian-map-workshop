"""follow.py on a human take with each finger paired to the person's finger of the same name.

follow.py pairs fingers to contacts freely: any finger may take any contact, whichever pairing reaches
best. On a five-contact human take a four-fingered hand can then serve the contacts out of order, for
example its index finger taking the person's little-finger contact, so its fingers cross. Here the
pairing is fixed by name instead (thumb to thumb, index to index, and so on), for the placement search
and the per-frame check alike. follow.py itself is unchanged: its pairing function is replaced, in this
process only, by one that returns the pairing by name.

A contact whose finger the hand does not have (the little finger, on a four-fingered hand) is dropped
from a copy of the take before the check: its "contact" is set false in every frame, so follow.py does
not count it. The copy gets its own id and is written next to the result.

    python tools/run_follow_named_pairing.py --hand allegro --leader <take .json> --frames 120 --out-dir <dir>
        [--placement-from <result .json>]    re-score that result's placement with the pairing by name

Needs mujoco and the hand models (tools/fetch_hands.py).
"""
import argparse, collections, copy, json, os, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import follow as F  # noqa: E402
from gramian_workshop.hands import HANDS  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", required=True); ap.add_argument("--leader", required=True)
    ap.add_argument("--frames", type=int, default=120); ap.add_argument("--out-dir", required=True)
    ap.add_argument("--placement-from", default=None)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    L = json.load(open(a.leader, encoding="utf-8"))
    names = list(L["stations"]); fingers = list(HANDS[a.hand]["fingers"])
    perm = [names.index(f) for f in fingers]                 # finger order as follow.py's Hand builds it
    drop = [i for i, s in enumerate(names) if s not in fingers]
    if drop:
        L = copy.deepcopy(L)
        for fr in L["frames"]:
            for i in drop:
                fr["contact"][i] = False
        L["id"] = L["id"] + "__no_" + "_".join(names[i] for i in drop)
        L["named_pairing_note"] = (f"Made by tools/run_follow_named_pairing.py from {os.path.basename(a.leader)}: the "
                                   f"{', '.join(names[i] for i in drop)} contact set false in every frame, as the hand "
                                   "has no finger of that name. Nothing else changed.")
        p = os.path.join(a.out_dir, L["id"] + ".json")
        if not os.path.exists(p):
            json.dump(L, open(p, "w", encoding="utf-8"), indent=1)
        print("take with", [names[i] for i in drop], "dropped:", p, flush=True)

    calls = collections.Counter()

    def by_name(C):
        calls["n"] += 1
        return list(perm), sum(C[r, perm[r]] for r in range(C.shape[0])) / C.shape[0]
    F.best_assignment = by_name                              # this process only; follow.py's file is unchanged

    placement = json.load(open(a.placement_from, encoding="utf-8")) if a.placement_from else None
    if placement is not None:
        placement = dict(placement); placement["assignment"] = {f: perm[r] for r, f in enumerate(fingers)}
    t0 = time.time()
    r = F.follow(a.hand, L, n_eval=a.frames, placement=placement)
    r["elapsed_s"] = round(time.time() - t0, 1)
    want = {f: perm[k] for k, f in enumerate(fingers)}
    assert r["assignment"] == want, (r["assignment"], want)
    r["pairing"] = ("fixed by finger name by tools/run_follow_named_pairing.py (follow.py's free pairing replaced in "
                    "that process); contacts the hand has no finger for are dropped from the take: "
                    + (", ".join(names[i] for i in drop) or "none"))
    if a.placement_from:
        r["placement_reused_from"] = os.path.abspath(a.placement_from)
    out = os.path.join(a.out_dir, f"follow_{a.hand}__{L['id']}__named_pairing" + ("__earlier_placement" if a.placement_from else "") + ".json")
    assert not os.path.exists(out), f"{out} exists: not overwriting"
    json.dump(r, open(out, "w", encoding="utf-8"), indent=1, default=float)
    v = collections.Counter(fr["verdict"] for fr in r["frames"])
    print(f"{a.hand}: {dict(v)}; unserved frames {sum(bool(fr['unserved']) for fr in r['frames'])}; assignment {r['assignment']}; "
          f"placement cost {r['placement_cost_mm']:.2f} mm; pairing calls {calls['n']}; {r['elapsed_s']} s", flush=True)
    print("wrote", out, flush=True)


if __name__ == "__main__":
    main()
