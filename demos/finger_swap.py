"""Before and after: swap the finger the backwards lookup names, same hand.

    python demos/finger_swap.py [--before mckibben_v2_40deg] [--after ID]
                                [--task cup_turn] [--n 4] [--mount aimed]
                                [--yaw 90] [--slide]

Two hands of N copies of one record on the cup ring, same task, mount, yaw
and wave duty (the task's banked duty). The "before" hand is the record
given; the "after" hand is the record given, or by default the top-ranked
record of the backwards lookup (gramian_workshop/lookup.py) that carries no
flags.

Each hand goes through the turn check demos/cup_turn.py runs
(gramian_workshop/turn.py run on an Arrangement). Two things come out:

  holds      on every tick of the wave, can the fingers in contact carry the
             ball's weight? (the reason ladder, one reason per tick)
  turner     does the finger's motion drive the turn? demos/cup_turn.py
             decides this inline, by comparing the azimuthal share and the
             off-axis fraction with TURNER_AZ and TURNER_OFF; there is no
             function to call, so the same comparison is repeated here with
             the thresholds imported from demos/cup_turn.py.

The turner verdict is one result for the whole hand; only the hold has a
result per tick. Where outputs/cup_turn.json holds a stored verdict for the
same setting, each hand is checked against it.

Writes outputs/finger_swap/finger_swap.{txt,json} (the printed readout), a
side-by-side still (finger_swap.png) and an animation of the wave
(finger_swap.mp4, through ffmpeg as tools/render_force_film.py does). Never
writes anywhere else.

With --slide it writes only finger_swap_slide.png and finger_swap_slide.mp4
in the same folder: the same picture without the per-tick weight-carried
strip, its legend entries and the line "One verdict for the whole hand, not
per tick". The other four files are not touched.

Records only: comparative, not predictive. Nothing is stepped.
"""
import argparse
import importlib.util
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from gramian_workshop import report  # noqa: E402
from gramian_workshop.arrange import Arrangement  # noqa: E402
from gramian_workshop.finger import Finger, load_record  # noqa: E402
from gramian_workshop.lookup import rank  # noqa: E402
from gramian_workshop.readout import HELD  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402
from gramian_workshop.turn import TICKS_PER_CYCLE, run  # noqa: E402


def _load_demo(name):
    """Import a sibling demo as a module (its main() is guarded)."""
    spec = importlib.util.spec_from_file_location(
        f"_demo_{name}", os.path.join(ROOT, "demos", f"{name}.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_CUP_TURN = _load_demo("cup_turn")
TURNER_AZ, TURNER_OFF = _CUP_TURN.TURNER_AZ, _CUP_TURN.TURNER_OFF
CAVEAT = _CUP_TURN.CAVEAT

DEFAULT_BEFORE = "mckibben_v2_40deg"
OUT_DIR = os.path.join(ROOT, "outputs", "finger_swap")
STORED = os.path.join(ROOT, "outputs", "cup_turn.json")
# the stored turner verdict was made by demos/cup_turn.py at this setting
STORED_STYLE, STORED_YAW, STORED_N = "aimed", 90.0, 4
STRIDE = 1           # ticks per animation frame
FPS = 30
W_PX, H_PX, DPI = 1920, 1080, 100


# ----------------------------------------------------------------------
# the check
# ----------------------------------------------------------------------
def pick_after(task, n, mount, yaw):
    """The top-ranked row of the backwards lookup with no flags, and the
    whole ranking (rows, left_out)."""
    rows, left_out = rank(task, n, mount, yaw)
    for r in rows:
        if not r["flags"]:
            return r, rows, left_out
    return None, rows, left_out


def hand_result(record_id, task, n=4, mount="aimed", yaw=90.0):
    """One hand of n copies of a record, through the turn check that
    demos/cup_turn.py runs, at the task's banked duty. Returns a plain dict
    (the run's per-tick arrays under "made" and "soft_reason")."""
    finger = Finger.from_id(record_id)
    arr = Arrangement(finger, task, n, mount, yaw)
    duty = task["wave"]["duty"]
    r = run(arr, duty)
    az = r["drive_azimuth"]
    off = r["drive_offaxis"]
    # the comparison demos/cup_turn.py makes inline for its turner list
    az_ok = (az or 0) > TURNER_AZ
    off_ok = off is not None and off < TURNER_OFF
    reasons = [str(x) for x in r["soft_reason"]]
    counts = {k: reasons.count(k) for k in report.REASON_STYLE if k in reasons}
    not_held = [k for k in reasons if k not in HELD]
    held_ticks = len(reasons) - len(not_held)
    return {
        "id": finger.id,
        "name": finger.name,
        "flags": finger.flags(),
        "stored_verdict_possible": finger.b is not None and not finger.quality,
        "n": n, "mount": mount, "yaw_deg": float(yaw), "duty": duty,
        "ticks": len(reasons),
        "held_ticks": held_ticks,
        "soft_held_all": r["soft_held_all"],
        "soft_held_frac": r["soft_held_frac"],
        "reason_counts": counts,
        "first_failing_reason": not_held[0] if not_held else None,
        "drive_azimuth": az,
        "drive_offaxis": off,
        "azimuth_passes": az_ok,
        "offaxis_passes": off_ok,
        "turner": az_ok and off_ok,
        "turn_per_stroke_deg": r["turn_per_stroke_deg"],
        "made_count": r["made_count"],
        "made": r["made"],
        "soft_reason": reasons,
        "_arr": arr,
    }


def reason_words(reason):
    return report.REASON_STYLE[reason][2]


def hold_line(h):
    if h["soft_held_all"]:
        return (f"carries the ball's weight on all {h['ticks']} ticks of the "
                "wave")
    return (f"carries the ball's weight on {h['held_ticks']} of {h['ticks']} "
            f"ticks; first reason it fails: {reason_words(h['first_failing_reason'])}")


def turn_line(h):
    nums = (f"azimuthal share {h['drive_azimuth']:.3f} (needs above "
            f"{TURNER_AZ:g}), off-axis {h['drive_offaxis']:.4f} (needs below "
            f"{TURNER_OFF:g})")
    if h["turner"]:
        return f"drives the turn: {nums}"
    why = []
    if not h["azimuth_passes"]:
        why.append(f"azimuthal share {h['drive_azimuth']:.3f} is not above "
                   f"{TURNER_AZ:g}")
    if not h["offaxis_passes"]:
        why.append(f"off-axis {h['drive_offaxis']:.4f} is not below "
                   f"{TURNER_OFF:g}")
    return f"does not drive the turn: {'; '.join(why)}"


def verdict(h):
    if h["soft_held_all"] and h["turner"]:
        return "holds the cup and turns it"
    if h["soft_held_all"]:
        return "holds the cup but cannot turn it"
    drops = ("drops the cup on every tick" if h["held_ticks"] == 0
             else "drops the cup on some ticks")
    if h["turner"]:
        return f"could drive the turn but {drops}"
    return f"{drops} and cannot turn it"


def stored_check(h, task_id, task):
    """Compare one hand with outputs/cup_turn.json. Returns a dict with
    "compared" False and "why" when there is nothing to compare with."""
    if (task_id, h["mount"], h["yaw_deg"], h["n"]) != (
            "cup_turn", STORED_STYLE, STORED_YAW, STORED_N):
        return {"compared": False, "why": (
            f"outputs/cup_turn.json holds a turner verdict for task cup_turn, "
            f"mount {STORED_STYLE}, yaw {STORED_YAW:g} deg, N={STORED_N} only; "
            "this run asks something else")}
    if not os.path.exists(STORED):
        return {"compared": False, "why": "outputs/cup_turn.json not found"}
    with open(STORED, encoding="utf-8") as fo:
        stored = json.load(fo)
    key = (f"{h['id']}|{h['mount']}_yaw{int(h['yaw_deg'])}|"
           f"{task['wave']['duty']}|{h['n']}")
    row = stored["summary"].get(key)
    if row is None:
        return {"compared": False, "why": f"no stored row {key}"}
    out = {"compared": True, "key": key,
           "stored_soft_held_all": row["soft_held_all"],
           "stored_drive_azimuth": row["drive_azimuth"],
           "stored_drive_offaxis": row["drive_offaxis"],
           "held_agrees": row["soft_held_all"] == h["soft_held_all"],
           "azimuth_difference": abs(row["drive_azimuth"] - h["drive_azimuth"]),
           "offaxis_difference": abs(row["drive_offaxis"] - h["drive_offaxis"])}
    if h["stored_verdict_possible"]:
        stored_turner = h["id"] in stored["prediction_detail"]["7"]["turners"]
        out["stored_turner"] = stored_turner
        out["turner_agrees"] = stored_turner == h["turner"]
    else:
        out["stored_turner"] = None
        out["turner_why_not"] = ("demos/cup_turn.py scores only records with "
                                 "force data and no quality flags; this one "
                                 "has no stored turner verdict")
    return out


def stored_lines(s):
    if not s["compared"]:
        return [f"  stored result: not compared; {s['why']}."]
    held = "yes" if s["stored_soft_held_all"] else "no"
    ln = [f"  stored result ({s['key']} in outputs/cup_turn.json): "
          f"holds on every tick {held} "
          f"({'agrees' if s['held_agrees'] else 'DISAGREES'}); azimuthal "
          f"share {s['stored_drive_azimuth']:.3f}, off-axis "
          f"{s['stored_drive_offaxis']:.4f} (differences "
          f"{s['azimuth_difference']:.1e}, {s['offaxis_difference']:.1e})"]
    if s["stored_turner"] is None:
        ln.append(f"  stored turner verdict: none; {s['turner_why_not']}.")
    else:
        ln.append(f"  stored turner verdict: "
                  f"{'turner' if s['stored_turner'] else 'not a turner'} "
                  f"({'agrees' if s['turner_agrees'] else 'DISAGREES'})")
    return ln


# ----------------------------------------------------------------------
# drawing
# ----------------------------------------------------------------------
def light_colours():
    """The light-theme colour tokens of gramian_workshop/report.py, read
    from its CSS rather than retyped."""
    block = report.CSS[report.CSS.index(":root {"):]
    block = block[:block.index("}")]
    return dict(re.findall(r"--([a-z0-9-]+):(#[0-9a-fA-F]{6})", block))


def still_tick(made):
    """The first tick on which a finger is lifted off (tick 0 if none)."""
    k = np.where(~made.all(axis=1))[0]
    return int(k[0]) if len(k) else 0


class Board:
    """Both hands side by side: each cup from above with its fingers, then a
    made/lifted-off row per finger and the per-tick hold strip under it."""

    def __init__(self, hands, roles, task, col, slide=False):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
        self.plt = plt
        plt.rcParams.update({"font.family": "DejaVu Sans"})
        self.col = col
        self.hands = hands
        ink, ink2 = col["ink"], col["ink2"]
        fig = plt.figure(figsize=(W_PX / DPI, H_PX / DPI), dpi=DPI)
        fig.patch.set_facecolor(col["surface"])
        self.fig = fig
        self.dots, self.arrows, self.cursors, self.tick_text = [], [], [], []
        R_mm = 1000.0 * float(task["object"]["radius_m"])
        reasons_seen = []
        for j, (h, role) in enumerate(zip(hands, roles)):
            x0 = 0.03 + 0.5 * j
            arr = h["_arr"]
            c = arr.cup
            fig.text(x0, 0.955, f"{role}: {h['name']}", fontsize=26,
                     fontweight="bold", color=ink, va="top")
            fig.text(x0, 0.905, f"record {h['id']}, {h['n']} fingers, "
                     f"mount {h['mount']}, yaw {h['yaw_deg']:g} deg",
                     fontsize=18, color=ink2, va="top")
            # the cup from above
            ax = fig.add_axes([x0, 0.47, 0.20, 0.40])
            ax.set_aspect("equal")
            ax.axis("off")
            ax.add_patch(Circle((0, 0), R_mm, fill=False, lw=2, ec=ink2))
            pts = 1000.0 * c.points[:, :2]
            ring = 1000.0 * c.R * math.cos(c.alt)
            ax.add_patch(Circle((0, 0), ring, fill=False, lw=1.2, ls=":",
                                ec=col["muted"]))
            L = 0.55 * R_mm
            dots, arrows = [], []
            for i in range(c.n):
                e = c.e_phi[i, :2]
                ax.annotate("", xy=pts[i] + 0.9 * L * e, xytext=pts[i],
                            arrowprops=dict(arrowstyle="-|>", ls="--", lw=1.5,
                                            color=col["muted"]))
                d = arr.D[i] if arr.D[i] is not None else np.zeros(3)
                a = ax.annotate("", xy=pts[i] + L * d[:2], xytext=pts[i],
                                arrowprops=dict(arrowstyle="-|>", lw=4,
                                                color=col["made"],
                                                mutation_scale=28))
                arrows.append(a)
                dots.append(ax.add_patch(Circle(pts[i], 0.14 * R_mm, lw=2,
                                                ec=ink, fc=col["made"],
                                                zorder=5)))
                ax.text(*(pts[i] * 1.62), f"{i}", ha="center", va="center",
                        fontsize=18, color=ink2)
            lim = 1.45 * R_mm
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            self.dots.append(dots)
            self.arrows.append(arrows)
            # the words beside the cup
            tx = x0 + 0.21
            fig.text(tx, 0.84, "Carries the ball's weight", fontsize=18,
                     color=ink2, va="top")
            fig.text(tx, 0.805, ("on every tick" if h["soft_held_all"] else
                                 f"on {h['held_ticks']} of {h['ticks']} ticks"),
                     fontsize=22, color=ink, va="top")
            fig.text(tx, 0.745, "Drives the turn", fontsize=18, color=ink2,
                     va="top")
            fig.text(tx, 0.71, "yes" if h["turner"] else "no", fontsize=22,
                     fontweight="bold",
                     color=col["st-good"] if h["turner"] else col["st-critical"],
                     va="top")
            fig.text(tx, 0.665,
                     f"azimuthal share {h['drive_azimuth']:.3f}\n"
                     f"   (needs above {TURNER_AZ:g})\n"
                     f"off-axis {h['drive_offaxis']:.4f}\n"
                     f"   (needs below {TURNER_OFF:g})",
                     fontsize=17, color=ink, va="top", linespacing=1.3)
            if not slide:
                fig.text(tx, 0.52,
                         "One verdict for the whole hand,\nnot per tick",
                         fontsize=18, color=col["muted"], va="top")
            fig.text(x0, 0.455, verdict(h).capitalize(), fontsize=26,
                     fontweight="bold", va="top",
                     color=col["st-good"] if (h["soft_held_all"] and h["turner"])
                     else col["st-critical"])
            # made rows and hold strip
            made = h["made"]
            T, n = made.shape
            sx = fig.add_axes([x0 + 0.07, 0.17, 0.37, 0.23])
            sx.set_xlim(0, T)
            sx.set_ylim(-(n + 1.6), 0.4)
            sx.axis("off")
            for i in range(n):
                k = 0
                while k < T:
                    v = made[k, i]
                    k2 = k
                    while k2 < T and made[k2, i] == v:
                        k2 += 1
                    sx.add_patch(Rectangle((k, -i - 0.4), k2 - k, 0.8,
                                           color=col["made"] if v
                                           else col["broken"]))
                    k = k2
                sx.text(-4, -i, f"finger {i}", ha="right", va="center",
                        fontsize=18, color=ink2)
            if not slide:
                rs = h["soft_reason"]
                y = -(n + 0.6)
                k = 0
                while k < T:
                    k2 = k
                    while k2 < T and rs[k2] == rs[k]:
                        k2 += 1
                    sx.add_patch(Rectangle(
                        (k, y - 0.5), k2 - k, 1.0,
                        color=col[report.REASON_STYLE[rs[k]][1][2:]]))
                    if rs[k] not in reasons_seen:
                        reasons_seen.append(rs[k])
                    k = k2
                sx.text(-4, y, "weight\ncarried", ha="right", va="center",
                        fontsize=18, color=ink2)
            sx.text(-4, -(n + 1.45), "wave tick", ha="right", va="top",
                    fontsize=18, color=ink2)
            self.cursors.append(sx.plot([0, 0], [0.4, -(n + 1.1)], color=ink,
                                        lw=2.5)[0])
            self.tick_text.append(sx.text(4, -(n + 1.45), "", ha="left",
                                          va="top", fontsize=18, color=ink))
        # legend
        items = [(col["made"], "finger in contact"),
                 (col["broken"], "finger lifted off")]
        items += [(col[report.REASON_STYLE[r][1][2:]],
                   "weight carried: " + reason_words(r)) for r in
                  [r for r in report.REASON_STYLE if r in reasons_seen]]
        rows = [items[:2], items[2:]]
        for ly, row in zip((0.125, 0.085), rows):
            lx = 0.03
            for colour, words in row:
                fig.patches.append(Rectangle((lx, ly - 0.013), 0.014, 0.026,
                                             transform=fig.transFigure,
                                             color=colour, figure=fig))
                t = fig.text(lx + 0.019, ly, words, fontsize=18, color=ink,
                             va="center")
                fig.canvas.draw()
                bb = t.get_window_extent().transformed(fig.transFigure.inverted())
                lx = bb.x1 + 0.025
        fig.text(0.03, 0.012,
                 "Arrows: the way each fingertip moves, seen from above "
                 "(dashed: the way the turn needs). Numbers: fingers.\n"
                 + CAVEAT, fontsize=16, color=col["muted"], va="bottom",
                 linespacing=1.4)

    def show(self, k):
        for h, dots, arrows, cur, tt in zip(self.hands, self.dots, self.arrows,
                                            self.cursors, self.tick_text):
            m = h["made"][k]
            for i, (dot, a) in enumerate(zip(dots, arrows)):
                dot.set_facecolor(self.col["made"] if m[i] else self.col["broken"])
                a.arrow_patch.set_alpha(1.0 if m[i] else 0.2)
            cur.set_xdata([k + 0.5, k + 0.5])
            tt.set_text(f"{k} of {h['made'].shape[0]}")

    def rgb(self):
        self.fig.canvas.draw()
        a = np.asarray(self.fig.canvas.buffer_rgba())
        return np.ascontiguousarray(a[:, :, :3])


def draw(hands, roles, task, png, mp4, slide=False):
    col = light_colours()
    b = Board(hands, roles, task, col, slide=slide)
    T = hands[0]["made"].shape[0]
    ks = still_tick(hands[0]["made"])
    b.show(ks)
    b.fig.savefig(png, dpi=DPI, facecolor=b.fig.get_facecolor())
    first = b.rgb()
    hh, ww = first.shape[:2]
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                          "-pix_fmt", "rgb24", "-s", f"{ww}x{hh}", "-r",
                          str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt",
                          "yuv420p", "-crf", "18", mp4], stdin=subprocess.PIPE)
    frames = 0
    for k in range(0, T, STRIDE):
        b.show(k)
        p.stdin.write(b.rgb().tobytes())
        frames += 1
    p.stdin.close()
    p.wait()
    b.plt.close(b.fig)
    return {"still_tick": ks,
            "still_tick_rule": "the first tick on which a finger is lifted off",
            "frames": frames, "stride_ticks": STRIDE, "fps": FPS,
            "size_px": [ww, hh], "ffmpeg_returncode": p.returncode}


# ----------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--before", default=DEFAULT_BEFORE)
    ap.add_argument("--after", default=None,
                    help="default: the top-ranked record of the backwards "
                         "lookup with no flags")
    ap.add_argument("--task", default="cup_turn")
    ap.add_argument("--n", type=int, default=4, help="number of fingers")
    ap.add_argument("--mount", default="aimed", choices=["aimed", "palm"])
    ap.add_argument("--yaw", type=float, default=90.0, help="mount yaw, deg")
    ap.add_argument("--slide", action="store_true",
                    help="write only finger_swap_slide.{png,mp4}: no "
                         "weight-carried strip; the other outputs are not "
                         "touched")
    a = ap.parse_args(argv)
    task = load_task(a.task)

    lines = [f"Finger swap - {a.task}, N={a.n}, mount {a.mount}, yaw "
             f"{a.yaw:g} deg, duty {task['wave']['duty']} (the task's banked "
             "duty)", CAVEAT, ""]
    pick = None
    if a.after is None:
        row, rows, _ = pick_after(task, a.n, a.mount, a.yaw)
        if row is None:
            print("The backwards lookup ranks no record without flags; give "
                  "--after.")
            return 1
        a.after = row["id"]
        pick = {"id": row["id"], "share_first_third": row["share_first_third"],
                "first_third_u": row["first_third_u"],
                "input_unit": row["input_unit"],
                "rank": rows.index(row) + 1}
        lines += [f"After: the backwards lookup's top-ranked record with no "
                  f"flags is {row['id']} (rank {pick['rank']}), first-third "
                  f"share {row['share_first_third']:.3f} over input "
                  f"{row['first_third_u'][0]:.4g} to "
                  f"{row['first_third_u'][1]:.4g} {row['input_unit']}.", ""]

    hands, js_hands = [], []
    for role, rid in (("Before", a.before), ("After", a.after)):
        h = hand_result(rid, task, a.n, a.mount, a.yaw)
        s = stored_check(h, a.task, task)
        hands.append(h)
        lines += [f"{role}: {h['name']} ({h['id']})"
                  + (f" [{' '.join(h['flags'])}]" if h["flags"] else ""),
                  f"  holds: {'yes' if h['soft_held_all'] else 'no'}; "
                  + hold_line(h),
                  "  ticks by reason: " + ", ".join(
                      f"{reason_words(k)} {v}" for k, v in
                      h["reason_counts"].items()),
                  f"  turn: {turn_line(h)}",
                  "  turn per stroke: " + (
                      "none (no stroke in the record)"
                      if h["turn_per_stroke_deg"] is None
                      else f"{h['turn_per_stroke_deg']:.1f} deg"),
                  f"  verdict: {verdict(h)}"]
        lines += stored_lines(s)
        lines.append("")
        js_hands.append({"role": role.lower(), **{
            k: v for k, v in h.items()
            if k not in ("made", "soft_reason", "_arr")},
            "hold_words": hold_line(h), "turn_words": turn_line(h),
            "verdict": verdict(h), "stored": s,
            "made": h["made"].astype(int).tolist(),
            "soft_reason": h["soft_reason"]})
    lines += ["Only the hold has a result per tick (the reason ladder). The "
              "turner verdict is one result for the whole hand: "
              "demos/cup_turn.py compares the azimuthal share and off-axis "
              "fraction with its thresholds inline, and the same comparison "
              "is repeated here with its thresholds imported."]

    os.makedirs(OUT_DIR, exist_ok=True)
    if a.slide:
        # slide mode writes the two pictures only; the readout files stay
        png = os.path.join(OUT_DIR, "finger_swap_slide.png")
        mp4 = os.path.join(OUT_DIR, "finger_swap_slide.mp4")
        pic = draw(hands, ["Before", "After"], task, png, mp4, slide=True)
        print(f"Slide pictures: "
              f"{os.path.relpath(png, ROOT).replace(os.sep, '/')} (tick "
              f"{pic['still_tick']}, {pic['still_tick_rule']}) and "
              f"{os.path.relpath(mp4, ROOT).replace(os.sep, '/')} "
              f"({pic['frames']} frames, ffmpeg return code "
              f"{pic['ffmpeg_returncode']}).")
        return 0 if pic["ffmpeg_returncode"] == 0 else 1
    png = os.path.join(OUT_DIR, "finger_swap.png")
    mp4 = os.path.join(OUT_DIR, "finger_swap.mp4")
    pic = draw(hands, ["Before", "After"], task, png, mp4)
    lines += ["", f"Pictures: {os.path.relpath(png, ROOT).replace(os.sep, '/')} (tick "
              f"{pic['still_tick']}, {pic['still_tick_rule']}) and "
              f"{os.path.relpath(mp4, ROOT).replace(os.sep, '/')} ({pic['frames']} frames, one "
              f"per {pic['stride_ticks']} tick of the wave, {pic['fps']} "
              "frames a second)."]
    txt = "\n".join(lines) + "\n"
    with open(os.path.join(OUT_DIR, "finger_swap.txt"), "w",
              encoding="utf-8") as fo:
        fo.write(txt)
    js = {"task": a.task, "caveat": CAVEAT, "made_by": "demos/finger_swap.py",
          "n": a.n, "mount": a.mount, "yaw_deg": a.yaw,
          "duty": task["wave"]["duty"], "ticks_per_cycle": TICKS_PER_CYCLE,
          "turner_thresholds": {"azimuthal_share_above": TURNER_AZ,
                                "offaxis_below": TURNER_OFF,
                                "from": "demos/cup_turn.py"},
          "after_pick": pick, "hands": js_hands, "pictures": pic}
    with open(os.path.join(OUT_DIR, "finger_swap.json"), "w",
              encoding="utf-8") as fo:
        json.dump(js, fo, indent=1)
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
