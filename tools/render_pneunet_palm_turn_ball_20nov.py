"""Clips and stills of the pneu-net hands turning a ball, from the mounts
found by tools/pneunet_palm_turn_ball_20nov.py (run that first).

    python tools/render_pneunet_palm_turn_ball_20nov.py
      -> outputs/pneunet_palm_turn_ball_20nov/
           turning_back_and_forth_<record>_x<N>.mp4 / .png
           failed_return_<record>_x<N>.mp4 / .png
           turn_against_push_out.png

Every number drawn is read from results.json or recomputed from the record
by the analysis script's own functions. SIMULATION ONLY, comparative.

View: along the turning axis. Bodies are the fitted stand-in in its free
shape (the record holds no body shape under input); in contact the tip is
drawn pushed out onto the ball, as the analysis assumes. Slip is ignored, so
the ball turns with the tips.
"""
import json
import os
import shutil
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import animation  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import pneunet_palm_turn_ball_20nov as A  # noqa: E402

OUT = A.OUT
FPS = 25
INK, SOFT, BLUE, RED, GREY = "#1b1b1f", "#6b6b76", "#2a6fdb", "#d1495b", "#c9c9d2"
ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
plt.rcParams["animation.ffmpeg_path"] = ff
plt.rcParams.update({"font.size": 12, "font.family": "DejaVu Sans"})


def setup(rid, n, res):
    fg = A.load_finger(rid)
    rr = res["records"][rid]
    ch = rr["hands"][str(n)]["chosen"]["record depth"]
    c = np.array(ch["axis_in_finger_frame_m"])
    i, j = ch["level_i"], ch["level_j"]
    P, B, u = fg["P"] - c, fg["B"] - c, fg["u"]
    gap = np.array(ch["gap_m"])
    th = np.unwrap(np.arctan2(P[:, 1], P[:, 0]))
    # ball angle as a function of the input level alone (slip ignored)
    psi = np.clip((th - th[i]) * ch["sense"], 0, None)
    psi[:i] = 0.0
    psi[j:] = psi[j]
    psi = psi * ch["sense"]
    return {"fg": fg, "rr": rr, "ch": ch, "P": P, "B": B, "u": u, "gap": gap,
            "psi": psi, "i": i, "j": j, "n": n, "rid": rid,
            "rho": rr["ring_radius_tip_centre_m"], "rc": rr["tip_radius_m"],
            "R": res["task"]["radius_m"]}


def level_at(s, x):
    """Fractional input level x -> interpolated tip, body, gap, ball angle."""
    k = int(np.clip(np.floor(x), 0, len(s["u"]) - 2))
    t = x - k
    f = lambda a: (1 - t) * a[k] + t * a[k + 1]  # noqa: E731
    return f(s["P"]), f(s["B"]), f(s["gap"]), f(s["psi"]), f(s["u"])


def draw_frame(s, x, trace, title, right, fig=None):
    fig = fig or plt.figure(figsize=(12.8, 7.2), dpi=100)
    fig.clf()
    ax = fig.add_axes([0.03, 0.05, 0.52, 0.86])
    tip, body, gap, psi, uu = level_at(s, x)
    R, rho, rc, n = s["R"], s["rho"], s["rc"], s["n"]
    lim = max(np.abs(s["B"]).max(), np.abs(s["P"]).max()) * 1e3 + 12
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(plt.Circle((0, 0), lim - 2, fc="#f4f4f7", ec=GREY, lw=1))
    ax.text(0, -lim + 8, "palm", ha="center", color=SOFT, fontsize=11)
    # ball with marks that turn with it
    ax.add_patch(plt.Circle((0, 0), R * 1e3, fc="#ffe9b8", ec="#b98900",
                            lw=1.5, alpha=0.9, zorder=2))
    for m in range(8):
        a = psi + m * np.pi / 4
        ax.plot([0.25 * R * 1e3 * np.cos(a), 0.93 * R * 1e3 * np.cos(a)],
                [0.25 * R * 1e3 * np.sin(a), 0.93 * R * 1e3 * np.sin(a)],
                color="#b98900", lw=3 if m == 0 else 1, zorder=3,
                solid_capstyle="round")
    ax.add_patch(plt.Circle((0, 0), rho * 1e3, fc="none", ec=SOFT, lw=0.8,
                            ls=(0, (4, 4)), zorder=3))
    lw = 2 * rc * 1e3 * (ax.get_window_extent().width / (2 * lim)) * 72 / fig.dpi
    touching = gap < 0
    for kf in range(n):
        Rk = A.rot2(2 * np.pi * kf / n)
        path = s["P"] @ Rk.T * 1e3
        ax.plot(path[:, 0], path[:, 1], color=GREY, lw=1.2, zorder=4)
        seg = path[s["i"]:s["j"] + 1]
        ax.plot(seg[:, 0], seg[:, 1], color=RED, lw=2.2, zorder=4)
        b = body.copy()
        tp = tip.copy()
        if touching:
            tp = tip / np.linalg.norm(tip) * rho
            b[-1] = tp
        b = b @ Rk.T * 1e3
        ax.plot(b[:, 0], b[:, 1], color=BLUE, lw=lw, alpha=0.45, zorder=5,
                solid_capstyle="round", solid_joinstyle="round")
        t2 = Rk @ tp * 1e3
        ax.add_patch(plt.Circle(t2, rc * 1e3, fc=RED if touching else "white",
                                ec=INK, lw=1.2, zorder=6))
    ax.text(-lim + 4, lim - 8, "red: the stretch of each tip's one path "
            "that lies in the ball", color=RED, fontsize=10.5)
    fig.text(0.03, 0.945, title, fontsize=14, color=INK, weight="bold")
    fig.text(0.03, 0.02, "Simulation only, comparative. Slip ignored. Body: "
             "fitted stand-in, free shape. Seen along the turning axis.",
             fontsize=9.5, color=SOFT)
    right(fig, s, x, trace, uu, gap, psi)
    return fig


def right_turn(fig, s, x, trace, uu, gap, psi):
    ax = fig.add_axes([0.63, 0.56, 0.34, 0.32])
    ax.plot(np.arange(len(trace)) / FPS, np.degrees(np.abs(trace)), color=INK, lw=2)
    ax.set_xlim(0, s["T"]); ax.set_ylim(-5, s["ch"]["turn_deg"] * 1.15)
    ax.set_xlabel("time (s, arbitrary)"); ax.set_ylabel("ball angle (deg)")
    ax.axhline(s["ch"]["turn_deg"], color=RED, lw=1, ls="--")
    ax.text(s["T"], s["ch"]["turn_deg"], f" {s['ch']['turn_deg']:.0f} deg ",
            color=RED, ha="right", va="bottom")
    ax.spines[["top", "right"]].set_visible(False)
    ax2 = fig.add_axes([0.63, 0.12, 0.34, 0.32])
    ax2.plot(s["u"], np.degrees(np.abs(s["psi"])), color=GREY, lw=4)
    ax2.plot([uu], [np.degrees(abs(psi))], "o", color=RED, ms=9)
    ax2.set_xlabel("input: chamber pressure (Pa, simulation units)")
    ax2.set_ylabel("ball angle (deg)")
    ax2.set_title("one input level, one ball angle", fontsize=11.5, color=SOFT)
    ax2.spines[["top", "right"]].set_visible(False)


def right_return(fig, s, x, trace, uu, gap, psi):
    ax = fig.add_axes([0.63, 0.50, 0.34, 0.38])
    g = s["gap"] * 1e3
    ax.axhspan(g.min() * 1.1, 0, color="#ffe9b8", alpha=0.6)
    ax.axhline(0, color="#b98900", lw=1.2)
    ax.plot(s["u"], g, color=GREY, lw=5)
    xs = np.array(s["xs_done"])
    if len(xs):
        k = xs.round().astype(int).clip(0, len(g) - 1)
        out = np.arange(len(xs)) <= s["turn_frame"]
        ax.plot(s["u"][k][out], g[k][out], color=BLUE, lw=2.2, label="going out")
        if (~out).any():
            ax.plot(s["u"][k][~out], g[k][~out], color=RED, lw=2.2,
                    ls=(0, (3, 2)), label="coming back")
    ax.plot([uu], [gap * 1e3], "o", color=INK, ms=8)
    ax.set_ylabel("gap, tip to ball (mm)")
    ax.set_xlabel("input: chamber pressure (Pa, simulation units)")
    ax.text(s["u"][(s["i"] + s["j"]) // 2], g.min() * 0.55, "pressed into\nthe ball",
            ha="center", color="#8a6500", fontsize=11)
    ax.legend(frameon=False, loc="upper left", fontsize=10.5)
    ax.set_title("the way back is the way out: no return clear of the ball",
                 fontsize=11.5, color=SOFT)
    ax.spines[["top", "right"]].set_visible(False)
    ax2 = fig.add_axes([0.63, 0.12, 0.34, 0.24])
    ax2.plot(np.arange(len(trace)) / FPS, np.degrees(np.abs(trace)), color=INK, lw=2)
    ax2.set_xlim(0, s["T"]); ax2.set_ylim(-5, s["ch"]["turn_deg"] * 1.15)
    ax2.set_xlabel("time (s, arbitrary)"); ax2.set_ylabel("ball angle (deg)")
    ax2.spines[["top", "right"]].set_visible(False)


def clip(s, xs, title, right, name, still_at):
    s["T"] = len(xs) / FPS
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    trace = []
    s["xs_done"] = []

    def frame(f):
        _, _, _, psi, _ = level_at(s, xs[f])
        del trace[f:]
        del s["xs_done"][f:]
        trace.append(psi)
        s["xs_done"].append(xs[f])
        draw_frame(s, xs[f], trace, title, right, fig)
        if f == still_at:
            fig.savefig(os.path.join(OUT, name + ".png"), dpi=150)
    an = animation.FuncAnimation(fig, frame, frames=len(xs), interval=1000 / FPS)
    an.save(os.path.join(OUT, name + ".mp4"),
            writer=animation.FFMpegWriter(fps=FPS, bitrate=3000,
                                          extra_args=["-pix_fmt", "yuv420p"]))
    plt.close(fig)
    print("wrote", os.path.join(OUT, name + ".mp4"))
    print("wrote", os.path.join(OUT, name + ".png"))


def ease(a, b, m):
    t = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, m))
    return a + (b - a) * t


def main():
    res = json.load(open(os.path.join(OUT, "results.json")))
    for rid, n in (("wall_0.0020m", 3), ("wall_0.0010m", 4)):
        s = setup(rid, n, res)
        ch = s["ch"]
        top = len(s["u"]) - 1
        wall = rid.split("_")[1].rstrip("m")
        label = f"{n} pneu-net fingers (wall {float(wall) * 1e3:g} mm) on a palm"
        # 1 back and forth, twice, inside the contact stretch
        one = np.concatenate([ease(0, s["j"], 60), ease(s["j"], 0, 60)])
        xs = np.concatenate([one, one])
        clip(s, xs, f"{label}: one stroke turns the ball "
             f"{ch['turn_deg']:.0f} deg, the return turns it back",
             right_turn, f"turning_back_and_forth_{rid}_x{n}", still_at=45)
        # 2 trying for a second stroke: go as far as the input allows, come back
        s = setup(rid, n, res)
        out = ease(0, top, 90)
        s["turn_frame"] = len(out) - 1
        xs = np.concatenate([out, np.full(10, top), ease(top, 0, 90)])
        clip(s, xs, f"{label}: coming back for a second stroke undoes the first",
             right_return, f"failed_return_{rid}_x{n}", still_at=len(out) + 55)

    # turn against push-out, all records, 3 fingers
    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    for rid, col in zip(A.RECORDS, ("#2a6fdb", "#1f9d8a", "#e0912f", "#d1495b")):
        h = res["records"][rid]["hands"]["3"]
        cv = h["curve"]
        ax.plot([c_["depth_cap_mm"] for c_ in cv], [c_["turn_deg"] for c_ in cv],
                color=col, lw=2, label=f"wall {float(rid.split('_')[1].rstrip('m')) * 1e3:g} mm")
        ax.axvline(h["hold_depth_m"] * 1e3, color=col, lw=1, ls=":")
    ax.set_xlabel("largest push-out of the tip allowed (mm)")
    ax.set_ylabel("ball angle turned in one stroke (deg)")
    ax.set_title("Three pneu-net fingers on a palm: one stroke, slip ignored\n"
                 "dotted: push-out at which three fingers carry the ball's weight",
                 fontsize=11)
    ax.legend(frameon=False)
    ax.spines[["top", "right"]].set_visible(False)
    fig.text(0.01, 0.01, "Simulation only, comparative.", fontsize=8, color=SOFT)
    fig.tight_layout()
    p = os.path.join(OUT, "turn_against_push_out.png")
    fig.savefig(p)
    print("wrote", p)


if __name__ == "__main__":
    main()
