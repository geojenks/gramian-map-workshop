"""Gates V0-V5 (registration section 6). Run before reading any number:

    python -m gramian_workshop.gates [--wave-manip ../wave_manip]

Writes outputs/gates.json. V0's source comparison and V5's export check
need a Wave_manip checkout; without one they are reported as skipped, not
passed.
"""
import argparse
import hashlib
import importlib.util
import itertools
import json
import math
import os
import sys
import tempfile
import types

import numpy as np

from .arrange import Arrangement
from .finger import DATA, all_fingers, load_manifest
from .readout import ideal_hold, soft_hold, soft_hold_passive
from .task import load_task
from .vendor import capability_core as cap

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
N_RANDOM = 200
MOUNTS = [("aimed", 0.0), ("aimed", 90.0), ("palm", 0.0), ("palm", 90.0)]


def _load(name, path):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s)
    sys.modules[name] = m
    s.loader.exec_module(m)
    return m


def random_contacts(rng, n):
    p = rng.normal(size=(n, 3))
    p /= np.linalg.norm(p, axis=1, keepdims=True)
    return 0.03 * p, -p


def v0(wm):
    out = {"text": None, "behaviour": None}
    if wm is None:
        out["skipped"] = "no --wave-manip checkout given"
        return out
    vend = _load("_vendor_w20", os.path.join(ROOT, "tools", "vendor_w20.py"))
    text_ok = {}
    for fname, spec in vend.SOURCES.items():
        with open(os.path.join(HERE, "vendor", fname)) as f:
            text_ok[fname] = f.read() == vend.render(spec, wm)
    out["text"] = text_ok
    sys.modules.setdefault("mujoco", types.ModuleType("mujoco"))
    src = _load("_w20_capability_src",
                os.path.join(wm, "a3h_sandbox", "w20_capability.py"))
    rng = np.random.default_rng(20261021)
    mism = 0
    for _ in range(N_RANDOM):
        n = int(rng.integers(1, 6))
        P, Nn = random_contacts(rng, n)
        w = rng.normal(size=6)
        mu = float(rng.uniform(0.0, 1.5))
        L = 0.03
        for mod_a, mod_b in [(cap, src)]:
            ws_a, ws_b = mod_a.scale_wrench(w, L), mod_b.scale_wrench(w, L)
            Da = mod_a.direction_basis(P, Nn, np.zeros(3), L)
            Db = mod_b.direction_basis(P, Nn, np.zeros(3), L)
            ra = mod_a.direction_test(Da, ws_a)
            rb = mod_b.direction_test(Db, ws_b)
            Ga = mod_a.generators(P, Nn, np.zeros(3), mu, L)[0]
            Gb = mod_b.generators(P, Nn, np.zeros(3), mu, L)[0]
            fa = mod_a.friction_test(Ga, ws_a)
            fb = mod_b.friction_test(Gb, ws_b)
            A = rng.normal(size=(n, Ga.shape[1]))
            u0 = rng.normal(size=n) * 0.1
            ta = mod_a.torque_test(Ga, A, u0, ws_a)
            tb = mod_b.torque_test(Gb, A, u0, ws_b)
            J = rng.normal(size=(n, 3, n))
            ia = mod_a.input_count(P, np.zeros(3), J, L)
            ib = mod_b.input_count(P, np.zeros(3), J, L)
            same = (np.array_equal(Da, Db) and ra == rb
                    and np.array_equal(Ga, Gb) and fa[0] == fb[0]
                    and fa[1] == fb[1] and np.array_equal(fa[2], fb[2])
                    and ta[0] == tb[0] and ta[1] == tb[1] and ia == ib)
            mism += int(not same)
    out["behaviour"] = {"sets": N_RANDOM, "mismatches": mism}
    out["pass"] = all(text_ok.values()) and mism == 0
    return out


def brute_feasible(G, b):
    """Non-negative combination of G's columns equal to b: try every subset
    of at most six columns (Caratheodory), least squares, weights >= 0."""
    bar = cap.FEAS_REL * float(np.linalg.norm(b))
    m = G.shape[1]
    for k in range(1, min(6, m) + 1):
        for S in itertools.combinations(range(m), k):
            x, *_ = np.linalg.lstsq(G[:, S], b, rcond=None)
            if x.min() >= -1e-12 and np.linalg.norm(G[:, S] @ x - b) <= bar:
                return True
    return False


def v1():
    rng = np.random.default_rng(6)
    agree = 0
    for t in range(N_RANDOM):
        m = int(rng.integers(1, 9))
        G = rng.normal(size=(6, m))
        if t % 2 == 0:
            k = int(rng.integers(1, min(6, m) + 1))
            cols = rng.choice(m, size=k, replace=False)
            b = G[:, cols] @ rng.uniform(0.1, 2.0, size=k)
        else:
            b = rng.normal(size=6)
        ok = cap.friction_test(G, b, recheck=cap.NNLS_RECHECK_MULT)[0]
        agree += int(ok == brute_feasible(G, b))
    return {"sets": N_RANDOM, "agree": agree, "pass": agree == N_RANDOM}


def v2(task):
    worst = {"orth": 0.0, "det": 0.0, "aimed_b": 0.0, "palm_x": 0.0}
    for f in all_fingers():
        for n in range(1, 9):
            for style, yaw in MOUNTS:
                a = Arrangement(f, task, n, style, yaw)
                for i, R in enumerate(a.R):
                    worst["orth"] = max(worst["orth"], float(
                        np.abs(R.T @ R - np.eye(3)).max()))
                    worst["det"] = max(worst["det"],
                                       abs(float(np.linalg.det(R)) - 1.0))
                    if style == "aimed" and f.b is not None:
                        worst["aimed_b"] = max(worst["aimed_b"], float(
                            np.abs(R @ f.b - a.cup.normals[i]).max()))
                    if style == "palm":
                        worst["palm_x"] = max(worst["palm_x"], float(
                            np.abs(R[:, 0] - [0, 0, 1.0]).max()))
    return {"worst": worst, "bar": 1e-12,
            "pass": all(v <= 1e-12 for v in worst.values())}


def hold_sweep(task, phase0=0.0):
    res = {}
    for f in all_fingers():
        for style, yaw in MOUNTS:
            for n in range(1, 9):
                a = Arrangement(f, task, n, style, yaw, phase0)
                c = a.cup
                s = soft_hold(c.points, c.normals, c.com, c.weight_demand(),
                              c.mu, c.L, a.B, a.F, a.D)
                res[(f.id, style, yaw, n)] = s
    for n in range(1, 9):
        c = Arrangement(all_fingers()[0], task, n, "aimed", 0.0, phase0).cup
        res[("ideal", n)] = ideal_hold(c.points, c.normals, c.com,
                                       c.weight_demand(), c.mu, c.L)
    return res


def v3_v4(task):
    a = hold_sweep(task)
    worst = max([r["closure"] for r in a.values()
                 if r.get("closure") is not None] or [0.0])
    b = hold_sweep(task, phase0=0.37)
    diff = [k for k in a if a[k]["reason"] != b[k]["reason"]]
    return ({"worst_rel": worst, "bar": cap.FEAS_REL,
             "pass": worst <= cap.FEAS_REL},
            {"phase0_rad": 0.37, "verdicts": len(a), "changed": len(diff),
             "first_changed": [list(map(str, k)) for k in diff[:5]],
             "pass": not diff})


def _b_angle(arr, i):
    b, n = arr.B[i], arr.cup.normals[i]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(b, n))))))


def q1_g1(task):
    """delta_max = 0 reproduces the active-only verdicts, except in the band
    where the exact circle and the 8-edge pyramids disagree."""
    half = math.degrees(math.atan(load_task("cup_hold")["mu"]))
    diff, band = [], 0
    for f in all_fingers():
        for style, yaw in MOUNTS:
            for n in range(1, 9):
                a = Arrangement(f, task, n, style, yaw)
                c = a.cup
                w = c.weight_demand()
                r0 = soft_hold(c.points, c.normals, c.com, w, c.mu, c.L,
                               a.B, a.F)["reason"]
                r1 = soft_hold_passive(c.points, c.normals, c.com, w, c.mu,
                                       c.L, a.B, a.passive(0.0), a.F)["reason"]
                if r0 != r1:
                    ang = None if f.b is None else _b_angle(a, 0)
                    in_band = ang is not None and abs(ang - half) <= 2.3
                    band += int(in_band)
                    diff.append({"id": f.id, "mount": f"{style}_{int(yaw)}",
                                 "n": n, "active": r0, "passive0": r1,
                                 "b_to_normal_deg": ang, "in_band": in_band})
    return {"differences": len(diff), "in_band": band, "list": diff[:40],
            "pass": all(d["in_band"] for d in diff)}


def q1_g2_g3(task):
    """Rigid limit equals the ideal contacts; closure of every met verdict."""
    bad, worst = [], 0.0
    for f in all_fingers():
        for style, yaw in MOUNTS:
            for n in range(1, 9):
                a = Arrangement(f, task, n, style, yaw)
                c = a.cup
                w = c.weight_demand()
                Q = [np.eye(3) * 1e6 for _ in range(n)]
                r = soft_hold_passive(c.points, c.normals, c.com, w, c.mu,
                                      c.L, a.B, Q, [1e6] * n)
                ri = ideal_hold(c.points, c.normals, c.com, w, c.mu, c.L)
                if r["reason"] != ri["reason"]:
                    bad.append((f.id, style, yaw, n, r["reason"],
                                ri["reason"]))
                for dm in (0.001, 0.002, 0.004):
                    rp = soft_hold_passive(c.points, c.normals, c.com, w,
                                           c.mu, c.L, a.B, a.passive(dm), a.F)
                    if rp["closure"] is not None:
                        worst = max(worst, rp["closure"])
    return ({"mismatches": len(bad), "first": bad[:10], "pass": not bad},
            {"worst_rel": worst, "bar": cap.FEAS_REL,
             "pass": worst <= cap.FEAS_REL})


def v5(wm):
    man = load_manifest()
    bad = []
    for rid, e in man["records"].items():
        p = os.path.join(DATA, "records", e["file"])
        with open(p, "rb") as f:
            if hashlib.sha256(f.read()).hexdigest() != e["sha256"]:
                bad.append(rid)
    out = {"records": len(man["records"]), "sha_mismatch": bad}
    if wm is None:
        out["export"] = "skipped: no --wave-manip checkout given"
        out["pass"] = not bad
        return out
    exp = _load("_export_tasks", os.path.join(ROOT, "tools", "export_tasks.py"))
    with tempfile.TemporaryDirectory() as td:
        exp.OUT = td
        argv, sys.argv = sys.argv, ["export_tasks.py", "--wave-manip", wm]
        try:
            exp.main()
        finally:
            sys.argv = argv
        same = {}
        for tid in ("cup_hold", "cup_turn"):
            with open(os.path.join(td, f"{tid}.json")) as f1, \
                    open(os.path.join(DATA, "tasks", f"{tid}.json")) as f2:
                same[tid] = json.load(f1) == json.load(f2)
    out["export_identical"] = same
    out["pass"] = not bad and all(same.values())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", default=None)
    ap.add_argument("--out", default=os.path.join(ROOT, "outputs",
                                                  "gates.json"))
    a = ap.parse_args()
    wm = None if a.wave_manip is None else os.path.abspath(a.wave_manip)
    task = load_task("cup_hold")
    g = {"V0": v0(wm), "V1": v1(), "V2": v2(task)}
    g["V3"], g["V4"] = v3_v4(task)
    g["V5"] = v5(wm)
    g["Q1-G1"] = q1_g1(task)
    g["Q1-G2"], g["Q1-G3"] = q1_g2_g3(task)
    g["all_pass"] = all(v.get("pass", False) for v in g.values()
                        if isinstance(v, dict))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(g, f, indent=1, default=str)
    for k, v in g.items():
        print(k, json.dumps(v, default=str) if isinstance(v, dict) else v)
    return 0 if g["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
