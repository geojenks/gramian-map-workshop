"""Check a leader file against docs/TASK_FILE_FORMAT.md and show what the
follow check reads from it.

    python tools/check_leader.py data/leaders/my_task.json [more.json ...]
    python tools/check_leader.py --table data/leaders/*.json

A leader is one method (one contact plan) for doing a task: per frame, the
object's pose, which stations touch it, where, and the force each puts on
it. The follow check (gramian_workshop/follow.py) asks whether a hand can
supply those forces at those contacts.

ERROR  the follow check would stop on the file, or read part of it as
       something other than the format says. Fix these.
warn   the follow check runs, but reads part of the file in a way you may
       not expect; each message says what it does instead.

Then it prints what the follow check reads: object, stations, frames, time
step, contacts per frame, and the net contact force against the object's
weight. --table prints one Markdown row per file instead.

Reads the file only; numpy only.
"""
import json
import math
import os
import sys

import numpy as np

SCHEMA = "gramian-workshop/leader@1"
MU_DEFAULT = 1.0           # follow.py: mu = leader_json.get("mu", 1.0)
G = 9.81                   # as tools/import_demo_leader.py; used only to
                           # report the net contact force against the weight
SURFACE_TOL_M = 0.001      # proposed: a contact point this far off the
                           # sphere's surface is reported (the stored
                           # leaders sit within JSON rounding of it)
UNIT_TOL = 0.01            # proposed: a quaternion or normal whose length
                           # is off 1 by more than this is reported
NORMAL_TOL_DEG = 5.0       # proposed: a stated contact normal this far from
                           # the direction to the object's centre is reported

TOP_REQUIRED = ("schema", "id", "n_stations", "object", "frames")
FRAME_REQUIRED = ("t", "obj_pos", "contact", "contact_point", "force_on_object_N")
# what follow.py reads, so that everything else can be listed as not read
TOP_READ = ("id", "n_stations", "object", "frames", "mu")
FRAME_READ = ("t", "obj_pos", "contact", "contact_point", "station_pos",
              "force_on_object_N", "untracked")


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _vec(v, n=3):
    return isinstance(v, list) and len(v) == n and all(_num(x) for x in v)


class Report:
    """Messages grouped by text, with the frames (and stations) they occur in."""

    def __init__(self):
        self.errors, self.warnings = {}, {}

    def error(self, msg, k=None, i=None):
        self.errors.setdefault(msg, []).append((k, i))

    def warn(self, msg, k=None, i=None):
        self.warnings.setdefault(msg, []).append((k, i))

    @staticmethod
    def lines(d, frames):
        out = []
        for msg, hits in d.items():
            ks = [k for k, _ in hits if k is not None]
            if not ks:
                out.append(msg)
                continue
            st = sorted({i for _, i in hits if i is not None})
            f0 = frames[ks[0]] if 0 <= ks[0] < len(frames) else None
            t = f0.get("t") if isinstance(f0, dict) else None
            where = f"first frame {ks[0]}" + (f", t = {t:g} s" if _num(t) else "")
            what = (f"{len(hits)} times in {len(set(ks))} frames" if st else
                    f"{len(set(ks))} frame{'s' if len(set(ks)) > 1 else ''}")
            if st:
                what += ", station" + ("s " if len(st) > 1 else " ") + ", ".join(map(str, st))
            out.append(f"{msg} ({what}; {where})")
        return out

    @staticmethod
    def frames_hit(d):
        return len({k for hits in d.values() for k, _ in hits if k is not None})


def check(L):
    """Check a parsed leader. Returns (report, summary); summary is None
    when the errors stop the per-frame reading."""
    rep = Report()
    if not isinstance(L, dict):
        rep.error("the file is not a JSON object")
        return rep, None
    for k in TOP_REQUIRED:
        if k not in L:
            rep.error(f"missing top-level key '{k}'")
    if "schema" in L and L["schema"] != SCHEMA:
        rep.error(f"schema is {L['schema']!r}, not {SCHEMA!r}")
    if "id" in L and not (isinstance(L["id"], str) and L["id"]):
        rep.error("id must be a non-empty string (the follow check names its result by it)")
    n = L.get("n_stations")
    if "n_stations" in L and not (isinstance(n, int) and not isinstance(n, bool) and n >= 1):
        rep.error("n_stations must be a whole number, 1 or more")
        n = None
    ob = L.get("object")
    R = mass = None
    if "object" in L:
        if not isinstance(ob, dict):
            rep.error("object must be a JSON object")
        else:
            if ob.get("shape") != "sphere":
                rep.error(f"object.shape is {ob.get('shape')!r}: the follow check "
                          "handles spheres only")
            if _num(ob.get("radius_m")) and ob["radius_m"] > 0:
                R = float(ob["radius_m"])
            else:
                rep.error("object.radius_m must be a number above 0 (metres)")
            if _num(ob.get("mass_kg")) and ob["mass_kg"] > 0:
                mass = float(ob["mass_kg"])
            else:
                rep.error("object.mass_kg must be a number above 0 (kg); the follow "
                          "check reads it even though the forces already carry the load")
    if "mu" in L and not (_num(L["mu"]) and L["mu"] > 0):
        rep.error("mu, when given, must be a number above 0")
    fr = L.get("frame")
    if isinstance(fr, dict) and "units" in fr and fr["units"] != "m":
        rep.error(f"frame.units is {fr['units']!r}: every length must be in metres")
    st = L.get("stations")
    if st is not None and n is not None and not (
            isinstance(st, list) and len(st) == n and all(isinstance(s, str) for s in st)):
        rep.warn("stations, when given, should list one name per station "
                 "(the follow check does not read it)")
    F = L.get("frames")
    if "frames" in L and not (isinstance(F, list) and F):
        rep.error("frames must be a non-empty list")
        F = None
    if "n_frames" in L and isinstance(F, list) and L["n_frames"] != len(F):
        rep.warn(f"n_frames says {L['n_frames']} but the file holds {len(F)} frames "
                 "(the follow check counts the frames itself)")
    if rep.errors or F is None or n is None:
        return rep, None

    t_prev = None
    dts, touching, zero_force, nets, fallback_gap = [], [], 0, [], []
    extra_frame_keys = set()
    for k, f in enumerate(F):
        if not isinstance(f, dict):
            rep.error("a frame is not a JSON object", k)
            continue
        extra_frame_keys |= set(f) - set(FRAME_READ)
        miss = [x for x in FRAME_REQUIRED if x not in f]
        for x in miss:
            rep.error(f"frame lacks '{x}'", k)
        if miss:
            continue
        t = f["t"]
        if not _num(t):
            rep.error("t must be a number (seconds)", k)
        else:
            if t_prev is not None:
                if t <= t_prev:
                    rep.error("t does not increase from the previous frame", k)
                else:
                    dts.append(t - t_prev)
            t_prev = t
        if not _vec(f["obj_pos"]):
            rep.error("obj_pos must be three numbers (metres)", k)
            continue
        c = np.array(f["obj_pos"], float)
        q = f.get("obj_quat")
        if q is not None:
            if not _vec(q, 4):
                rep.error("obj_quat, when given, must be four numbers (w, x, y, z) or null", k)
            elif abs(np.linalg.norm(q) - 1.0) > UNIT_TOL:
                rep.warn("obj_quat is not of unit length", k)
        lists = {}
        for x in ("contact", "contact_point", "force_on_object_N", "station_pos",
                  "contact_normal"):
            if x not in f or (x == "contact_normal" and f[x] is None):
                continue
            if not (isinstance(f[x], list) and len(f[x]) == n):
                rep.error(f"{x} must hold one entry per station ({n})", k)
            else:
                lists[x] = f[x]
        if not all(x in lists for x in ("contact", "contact_point", "force_on_object_N")):
            continue
        made, net = 0, np.zeros(3)
        for i in range(n):
            on = lists["contact"][i]
            if not isinstance(on, bool):
                rep.error("contact must hold true or false for each station", k, i)
                continue
            fv = lists["force_on_object_N"][i]
            cp = lists["contact_point"][i]
            sp = lists.get("station_pos", [None] * n)[i]
            for name, v in (("contact_point", cp), ("station_pos", sp)):
                if v is not None and not _vec(v):
                    rep.error(f"{name} entries must be three numbers (metres) or null", k, i)
            if fv is not None and not _vec(fv):
                rep.error("force_on_object_N entries must be three numbers (newtons)", k, i)
                continue
            if not on:
                if fv is None:
                    rep.warn("force_on_object_N is null where the station does not "
                             "touch; write [0, 0, 0]", k, i)
                elif any(fv):
                    rep.warn("a station marked not touching carries a force; the "
                             "follow check ignores that force", k, i)
                continue
            made += 1
            if fv is None:
                rep.error("a touching station's force_on_object_N is null", k, i)
                continue
            fv = np.array(fv, float)
            net += fv
            if not np.any(fv):
                zero_force += 1
            if cp is None:
                if sp is None:
                    rep.error("a touching station has neither contact_point nor "
                              "station_pos", k, i)
                    continue
                rep.warn("a touching station has no contact_point; the follow "
                         "check uses its station_pos instead (the station's centre, "
                         "not a point on the object)", k, i)
                p = np.array(sp, float)
                if R is not None:
                    fallback_gap.append(float(np.linalg.norm(p - c)) - R)
            else:
                p = np.array(cp, float)
                if R is not None and abs(np.linalg.norm(p - c) - R) > SURFACE_TOL_M:
                    rep.warn(f"a contact_point lies more than {SURFACE_TOL_M * 1e3:g} mm "
                             "off the sphere's surface; the follow check measures reach "
                             "to this point", k, i)
            d = np.linalg.norm(c - p)
            if d == 0:
                rep.error("a touching station's contact is at the object's centre", k, i)
                continue
            u = (c - p) / d
            if fv @ u < -1e-9:
                rep.warn("a touching station's force pulls on the object (points "
                         "away from its centre); no friction cone holds a pull, so "
                         "this fails the force check", k, i)
            nn = lists.get("contact_normal", [None] * n)[i]
            if nn is not None:
                if not _vec(nn):
                    rep.error("contact_normal entries must be three numbers or null", k, i)
                else:
                    nn = np.array(nn, float)
                    ln = np.linalg.norm(nn)
                    if abs(ln - 1.0) > UNIT_TOL:
                        rep.warn("a contact_normal is not of unit length", k, i)
                    if ln > 0 and math.degrees(math.acos(
                            max(-1.0, min(1.0, float(nn @ u) / ln)))) > NORMAL_TOL_DEG:
                        rep.warn("a contact_normal does not point from the contact to "
                                 "the object's centre; the follow check ignores it and "
                                 "uses that direction", k, i)
        touching.append(made)
        if made:
            nets.append(np.linalg.norm(net))
    if rep.errors:
        return rep, None
    dts = np.array(dts) if dts else np.zeros(1)
    summary = {
        "id": L["id"], "name": L.get("name"), "kind": L.get("kind"),
        "radius_m": R, "mass_kg": mass,
        "mu": float(L["mu"]) if "mu" in L else None,
        "n_stations": n, "n_frames": len(F),
        "t_first_s": float(F[0]["t"]), "t_last_s": float(F[-1]["t"]),
        "dt_median_s": float(np.median(dts)), "dt_max_s": float(np.max(dts)),
        "touching_min": int(min(touching)), "touching_max": int(max(touching)),
        "frames_without_contact": int(sum(1 for x in touching if x == 0)),
        "touching_with_zero_force": zero_force,
        "net_over_weight_median": (float(np.median(nets)) / (mass * G)) if nets else None,
        "station_pos_gap_median_m": float(np.median(fallback_gap)) if fallback_gap else None,
        "has_obj_quat": any(isinstance(f, dict) and f.get("obj_quat") is not None for f in F),
        "top_not_read": sorted(set(L) - set(TOP_READ)),
        "frame_not_read": sorted(extra_frame_keys),
    }
    return rep, summary


def load(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def check_file(path):
    """Print the report for one file; True if it has no errors."""
    print(f"\n== {path}")
    try:
        L = load(path)
    except (OSError, ValueError) as e:
        print(f"   ERROR  cannot read it as JSON: {e}")
        return False
    rep, s = check(L)
    F = L.get("frames") if isinstance(L, dict) and isinstance(L.get("frames"), list) else []
    for m in Report.lines(rep.errors, F):
        print(f"   ERROR  {m}")
    for m in Report.lines(rep.warnings, F):
        print(f"   warn   {m}")
    if rep.errors:
        print("   -> fix the errors above; the follow check will not read this file as meant")
        return False
    print("   what the follow check reads:")
    print(f"     id        {s['id']}")
    print(f"     object    sphere, radius {s['radius_m'] * 1e3:.4g} mm, mass "
          f"{s['mass_kg']:.4g} kg (read, but the forces already carry the load)")
    print(f"     friction  mu {s['mu'] if s['mu'] is not None else MU_DEFAULT:g}"
          + ("" if s["mu"] is not None else " (not in the file: the follow check's default)"))
    print(f"     stations  {s['n_stations']}; {s['touching_min']}-{s['touching_max']} "
          f"touching per frame; {s['frames_without_contact']} frames with none")
    print(f"     frames    {s['n_frames']}, t {s['t_first_s']:g} to {s['t_last_s']:g} s; "
          f"step median {s['dt_median_s']:.4g} s, largest {s['dt_max_s']:.4g} s")
    if s["net_over_weight_median"] is not None:
        print(f"     load      net contact force, median over frames with contact: "
              f"{s['net_over_weight_median']:.3f} x the object's weight")
    if s["station_pos_gap_median_m"] is not None:
        print(f"     fallback  where station_pos stands in for a contact point, it lies "
              f"{s['station_pos_gap_median_m'] * 1e3:.3g} mm outside the surface (median)")
    print(f"     zero-force contacts  {s['touching_with_zero_force']} (station touching, "
          "force [0, 0, 0]: still counted as a contact)")
    print(f"     not read  top level: {', '.join(s['top_not_read']) or '-'}")
    print(f"               per frame: {', '.join(s['frame_not_read']) or '-'}"
          + ("" if s["has_obj_quat"] else "  (obj_quat absent or null)"))
    return True


def table(paths):
    """Two Markdown tables: one row per file, then one row per warning."""
    print("| File | Frames | Stations | Touching per frame | Time span (s) | "
          "Step, median (s) | Largest step (s) | Radius (mm) | Mass (kg) | "
          "Net contact force / weight, median | Touching with zero force | "
          "station_pos gap where it stands in, median (mm) | Errors | "
          "Frames with a warning |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    ok, warned = True, []
    for p in paths:
        try:
            rep, s = check(load(p))
        except (OSError, ValueError):
            rep, s = Report(), None
            rep.error("unreadable")
        ne = sum(len(v) for v in rep.errors.values())
        nw = Report.frames_hit(rep.warnings)
        ok &= ne == 0
        name = os.path.basename(p)
        warned += [(name, msg, hits) for msg, hits in rep.warnings.items()]
        if s is None:
            print(f"| `{name}` | | | | | | | | | | | | {ne} | {nw} |")
            continue
        nw_ = "-" if s["net_over_weight_median"] is None else f"{s['net_over_weight_median']:.3f}"
        gap = ("-" if s["station_pos_gap_median_m"] is None
               else f"{s['station_pos_gap_median_m'] * 1e3:.3g}")
        print(f"| `{name}` | {s['n_frames']} | {s['n_stations']} | "
              f"{s['touching_min']}-{s['touching_max']} | "
              f"{s['t_first_s']:g}-{s['t_last_s']:g} | {s['dt_median_s']:.4g} | "
              f"{s['dt_max_s']:.4g} | {s['radius_m'] * 1e3:.4g} | {s['mass_kg']:.4g} | "
              f"{nw_} | {s['touching_with_zero_force']} | {gap} | {ne} | {nw} |")
    if warned:
        print()
        print("| File | Warning | Times | Frames |")
        print("|---|---|---|---|")
        for name, msg, hits in warned:
            ks = {k for k, _ in hits if k is not None}
            print(f"| `{name}` | {msg} | {len(hits)} | {len(ks) or '-'} |")
    return ok


def main(argv):
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 2
    if argv[0] == "--table":
        return 0 if table(argv[1:]) else 1
    ok = all([check_file(p) for p in argv])
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
