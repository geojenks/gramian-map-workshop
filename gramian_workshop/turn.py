"""cup_turn: the cup turned about the vertical by a make/break wave
(registration section 4).

DECLARATION (Wave_manip FRAMEWORK.md section 0, rule 2(b)). This is a
kinematic duty schedule for a capability readout, not a controller and not
unified_v6: nothing is stepped and no contact force is made. It does without
every canonical feature: the PI hold on the ring anchor, implicit friction,
the real-contact gate, forceBand, the grip-support solve, the stability
margin, the supervisor, the reflex, the capability weights and feedback@1.
It asks only which made-sets of record-built fingers could carry the cup's
demand, tick by tick.
"""
import numpy as np

from .readout import HELD, ideal_hold, soft_hold

TICKS_PER_CYCLE = 360


def schedule(n, duty, ticks=TICKS_PER_CYCLE):
    """made[k, i]: finger i in contact at tick k (frac(k/K - i/N) < duty)."""
    k = np.arange(ticks)[:, None] / ticks
    i = np.arange(n)[None, :] / max(n, 1)
    return np.mod(k - i, 1.0) < duty


def run(arr, duty, ticks=TICKS_PER_CYCLE):
    c = arr.cup
    made = schedule(c.n, duty, ticks)
    w = c.weight_demand()
    soft, ideal, ic = [], [], []
    cache = {}
    for k in range(ticks):
        key = made[k].tobytes()
        if key not in cache:
            P, Nn, B, F, D = arr.subset(made[k])
            s = soft_hold(P, Nn, c.com, w, c.mu, c.L, B, F, D)
            i = ideal_hold(P, Nn, c.com, w, c.mu, c.L)
            cache[key] = (s["reason"], i["reason"], s["input_count"])
        s, i, n_in = cache[key]
        soft.append(s)
        ideal.append(i)
        ic.append(n_in)
    drive = [arr.drive(i) for i in range(c.n)]
    soft = np.array(soft)
    ideal = np.array(ideal)
    return {
        "made": made,
        "soft_reason": soft,
        "ideal_reason": ideal,
        "input_count": ic,
        "soft_held_frac": float(np.isin(soft, HELD).mean()),
        "ideal_held_frac": float((ideal == "met").mean()),
        "soft_held_all": bool(np.isin(soft, HELD).all()),
        "ideal_held_all": bool((ideal == "met").all()),
        "made_count": sorted({int(x) for x in made.sum(1)}),
        "drive_azimuth": drive[0][0] if drive else None,
        "drive_offaxis": max(d[1] for d in drive) if drive else None,
        "turn_per_stroke_deg": arr.turn_per_stroke_deg(0) if c.n else None,
    }
