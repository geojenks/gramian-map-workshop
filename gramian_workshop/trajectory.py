"""The whole motion at once: can one input history hold every checked frame?

follow(rate_limit=, lag_s=) links each checked frame to one solution of the
previous one (see follow._reach). That carries one arbitrary solution, not
the set that holds; a failed frame passes its least-bad attempt on; and its
scale margin grows only the frame's own load. Here the checked frames of a
run are solved together, as one linear programme (scipy's HiGHS), with the
rate and lag limits as rows between consecutive frames' net torques.

Per frame (the building block) the torque stack of follow._torque_stack,
unchanged, with its soft input-bound rows (x + slack = 1) and the right-hand
side [ws, jr, 1] (jr from follow._joint_rhs, zeros without inertia). The
residual Mx z - rhs = p - q, p, q >= 0, and sum(p + q) <= tol_k: an L1
budget on the whole residual of that frame (wrench, joint and input rows),
tol_k = max(tol_frac |ws|, cap.FEAS_REL) as follow._redistribute. The
existing test bounds the L2 norm of the same vector by the same tol, and L1
>= L2, so this test is never looser than it, and can be stricter by up to
sqrt(rows). Where the budget is cap.FEAS_REL (a zero-wrench frame) the test
is exact to HiGHS's own feasibility tolerance (about 1e-7, coarser than
FEAS_REL), as the existing test is exact to its solver's.

Scale s, as held(s) in follow._scale_margin: s <= 1, wrench rows s ws and
budget max(tol_frac s |ws|, FEAS_REL); s > 1, each frame's own contact
torque block Mx[6:6+nt, :ng] times s, wrench rows ws, budget max(tol_frac
|ws|, FEAS_REL). x is the true input use in both, so the link rows need no
rescaling.

Link rows, frame k after frame k-1 of the same run (dt between them): each
actuator's net torque u = sum of v_j x_j over its input columns
(follow._input_senses: the passive stiffness columns are not inputs) obeys
|u_k - u_{k-1}| <= r max(hi, -lo) dt (rate r) and (1 - e) lo <= u_k - e
u_{k-1} <= (1 - e) hi, e = exp(-dt / T) (lag T); both given, both. A finger
in frame k but not in k-1's check starts from u = 0 (the chain's rule); one
in k-1 but not k has no row. follow._reach also intersects with [lo, hi] and
clips u_{k-1} into it; here u is held near [lo, hi] only by the soft input
rows, and no hard bound is added. The first frame of a run has no link.

Frames failing the direction or friction test (at s = 1; neither depends on
scale), with no contact, or failing on their own at s = 1, break the motion
into runs; nothing passes across a break. A frame with contacts but no
torque unknowns holds and adds no variables (fingers after it start from 0).

A solve that HiGHS ends with no verdict is tried again without presolve,
then by interior point; one still without a verdict counts as failing. The
readout counts both (lp.retried, lp.other_status).

Simulation only; comparative, not predictive."""
import math
import time

import numpy as np

from . import follow as fo
from .vendor import capability_core as cap

NORM = "L1 of the whole stacked residual per frame"


def frame_record(k, t, pts, nrms, Js, c, W, mu, L, taus, fingers_on, lims,
                 resid_tol=None):
    """What the check needs of one checked frame, as follow collects it:
    generators and torque stack built once, and the direction and friction
    tests at s = 1 (the tolerance of follow._redistribute). lims: every
    finger's torque_limits limits (None without them)."""
    tol_frac = fo.RESID_TOL if resid_tol is None else resid_tol
    ws = cap.scale_wrench(W, L)
    lims_on = [lims[a] for a in fingers_on]
    if not len(pts):
        return stack_record(k, t, ws, None, None, None, Js, L, taus, fingers_on, lims_on,
                            tol_frac, "no_contact")
    pts, nrms = np.array(pts), np.array(nrms)
    tol = max(tol_frac * float(np.linalg.norm(ws)), cap.FEAS_REL)
    D = cap.direction_basis(pts, nrms, c, L)
    x, *_ = np.linalg.lstsq(D, ws, rcond=None)
    Gs, cidx, Fc = cap.generators(pts, nrms, c, mu, L)
    reason = ("direction" if np.linalg.norm(D @ x - ws) > tol else
              "friction" if not fo._feasible(Gs, ws, tol)[0] else None)
    return stack_record(k, t, ws, Gs, cidx, Fc, Js, L, taus, fingers_on, lims_on,
                        tol_frac, reason)


def stack_record(k, t, ws, Gs, cidx, Fc, Js, L, taus, fingers_on, lims, tol_frac,
                 reason=None):
    """The record itself, from the scaled wrench ws, the generators (None
    with no contact) and the outcome of the direction and friction tests
    (reason None: both pass). Js, fingers_on and lims (each finger's limits)
    are parallel, one per contact."""
    r = {"k": int(k), "t": float(t), "L": float(L), "tol_frac": tol_frac,
         "fingers_on": list(fingers_on), "Js": list(Js), "taus": taus,
         "lims": list(lims), "ws": np.asarray(ws, float),
         "wn": float(np.linalg.norm(ws)), "ok": reason is None, "reason": reason,
         "torque": False}
    if Gs is not None and any(c_ is not None for c_ in Js):
        Mx, nt, nx = fo._torque_stack(Gs, cidx, Fc, Js, L)
        jr = np.zeros(nt) if taus is None else fo._joint_rhs(Js, taus, L)
        r.update(torque=True, Mx=Mx, nt=nt, nx=nx, ng=Gs.shape[1], jr=jr)
    return r


def _columns(r):
    """[(finger, lim, offset of its x in the frame's x block, ncols)] for
    each finger of the frame with torque columns."""
    out, c0 = [], 0
    for cols, a, lim in zip(r["Js"], r["fingers_on"], r["lims"]):
        if cols is None:
            continue
        n = cols[1].shape[1]
        out.append((a, lim, c0, n))
        c0 += n
    return out


class _Stats:
    def __init__(self):
        self.n, self.wall, self.other, self.retried = 0, 0.0, {}, 0


def _solve(recs, scales, rate=None, lag=None, bounds=None, budget=True,
           stats=None):
    """One LP over the frames recs (consecutive, the first unlinked), frame
    i at scale scales[i]. bounds (one per rec, or None): each finger's input
    box as follow._stack_rhs (x = low + y; the chain), no link rows then.
    The objective is the total L1 residual, so the solution returned is the
    least-residual one; budget=False drops the budget rows (the least-bad
    attempt of a failed frame). Returns (holds, {rec index: {finger: x}})."""
    from scipy import sparse
    from scipy.optimize import linprog
    Ei, Ej, Ev, beq = [], [], [], []
    Ui, Uj, Uv, bub = [], [], [], []
    cost = []
    nv = ne = nu = 0
    xvar = {}              # rec index: (first x variable, low, columns)
    prev = None            # (t, {finger: [(var, v, actuator)]}) of the frame before
    for i, r in enumerate(recs):
        terms = {}
        if r["torque"]:
            s, ng, nt, nx = scales[i], r["ng"], r["nt"], r["nx"]
            M = r["Mx"]
            if s > 1.0:
                M = M.copy()
                M[6:6 + nt, :ng] *= s
            if bounds is None or bounds[i] is None:
                jr, span, low = r["jr"], np.ones(nx), np.zeros(nx)
            else:
                jr, span, low = fo._stack_rhs(r["Js"], r["taus"], bounds[i], r["L"])
            rhs = np.concatenate([(s if s <= 1.0 else 1.0) * r["ws"], jr, span])
            tol = max(r["tol_frac"] * (s if s <= 1.0 else 1.0) * r["wn"], cap.FEAS_REL)
            m_, nz = M.shape
            ii, jj = np.nonzero(M)
            Ei += [ne + ii, ne + np.arange(m_), ne + np.arange(m_)]
            Ej += [nv + jj, nv + nz + np.arange(m_), nv + nz + m_ + np.arange(m_)]
            Ev += [M[ii, jj], -np.ones(m_), np.ones(m_)]
            beq.append(rhs)
            cost += [np.zeros(nz), np.ones(2 * m_)]
            if budget:
                Ui.append(np.full(2 * m_, nu))
                Uj.append(nv + nz + np.arange(2 * m_))
                Uv.append(np.ones(2 * m_))
                bub.append([tol])
                nu += 1
            cols = _columns(r)
            xvar[i] = (nv + ng, low, cols)
            for a, lim, c0, n in cols:
                terms[a] = [(nv + ng + c0 + j, v, act)
                            for j, (act, v) in enumerate(fo._input_senses(lim))]
            ne, nv = ne + m_, nv + nz + 2 * m_
        if prev is not None and bounds is None and (rate is not None or lag is not None):
            dt = r["t"] - prev[0]
            for a, tm in terms.items():
                lim = np.asarray(recs[i]["lims"][recs[i]["fingers_on"].index(a)], float)
                old = prev[1].get(a, [])          # absent: u_{k-1} = 0
                for act, (lo, hi) in enumerate(lim):
                    now = [(var, v) for var, v, b in tm if b == act]
                    was = [(var, v) for var, v, b in old if b == act]
                    if not now:
                        continue
                    rows = []
                    if rate is not None:
                        R = rate * max(hi, -lo) * dt
                        rows += [(1.0, 1.0, R), (-1.0, -1.0, R)]
                    if lag is not None:
                        e = math.exp(-dt / lag)
                        rows += [(1.0, e, (1 - e) * hi), (-1.0, -e, -(1 - e) * lo)]
                    # each row: sg (u_k) - w (u_{k-1}) <= bound
                    for sg, w, bnd in rows:
                        Ui.append(np.full(len(now) + len(was), nu))
                        Uj.append(np.array([var for var, _ in now] + [var for var, _ in was], int))
                        Uv.append(np.array([sg * v for _, v in now] + [-w * v for _, v in was]))
                        bub.append([bnd])
                        nu += 1
        prev = (r["t"], terms)
    if nv == 0:
        return True, {}
    cat = lambda p: np.concatenate(p) if p else np.zeros(0)  # noqa: E731
    Aeq = sparse.csr_matrix((cat(Ev), (cat(Ei), cat(Ej))), shape=(ne, nv))
    Aub = sparse.csr_matrix((cat(Uv), (cat(Ui), cat(Uj))), shape=(nu, nv)) if nu else None
    t0 = time.perf_counter()
    lp = dict(A_ub=Aub, b_ub=cat(bub) if nu else None, A_eq=Aeq, b_eq=cat(beq),
              bounds=(0, None))
    res = linprog(cat(cost), method="highs", **lp)
    retried = res.status not in (0, 2)
    # HiGHS sometimes ends with no verdict (model status unknown); try
    # again without presolve, then by interior point. Still none: fails.
    for method, opts in (("highs", {"presolve": False}), ("highs-ipm", {})):
        if res.status in (0, 2):
            break
        res = linprog(cat(cost), method=method, options=opts, **lp)
    if stats is not None:
        stats.n += 1
        stats.retried += retried
        stats.wall += time.perf_counter() - t0
        if res.status not in (0, 2):
            stats.other[res.message] = stats.other.get(res.message, 0) + 1
    if res.status != 0:
        return False, {}
    xs = {}
    for i, (x0, low, cols) in xvar.items():
        xs[i] = {a: low[c0:c0 + n] + res.x[x0 + c0:x0 + c0 + n] for a, _, c0, n in cols}
    return True, xs


def _search(held, proven, bisect=False):
    """The search of follow._scale_margin on held(s): every point of
    cap.SCALE_GRID, bisection to cap.BISECT_TOL above the largest feasible
    one, and past the grid doubling to fo.MARGIN_MAX and bisection to
    fo.MARGIN_REL of the margin. When proven (the feasible s run from 0, see
    check), the largest feasible grid point is found by bisection over the
    grid instead, which gives the same point; bisect does the same without
    proof (contiguous is then not known: None). Returns {margin, capped,
    contiguous, search}."""
    grid = cap.SCALE_GRID
    memo = {}

    def h(s):
        s = float(s)
        if s not in memo:
            memo[s] = bool(held(s))
        return memo[s]

    search = "bisection" if (proven or bisect) else "grid"
    none = {"margin": 0.0, "capped": False, "contiguous": True if proven else None,
            "search": search}
    if search == "grid":
        flags = np.array([h(s) for s in grid])
        idx = np.where(flags)[0]
        if len(idx) == 0:
            return {**none, "contiguous": True}
        contiguous = bool(idx[-1] - idx[0] + 1 == len(idx))
        top = int(idx[-1])
    else:
        if not h(grid[0]):
            return none
        lo, hi = 0, len(grid)        # grid[lo] holds; grid[hi] fails (or is past the end)
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h(grid[mid]):
                lo = mid
            else:
                hi = mid
        top, contiguous = lo, (True if proven else None)
    out = {"margin": 0.0, "capped": False, "contiguous": contiguous, "search": search}
    if top == len(grid) - 1:
        a = float(grid[top])
        while a < fo.MARGIN_MAX and h(2 * a):
            a *= 2
        if a >= fo.MARGIN_MAX:
            out.update(margin=a, capped=True)
            return out
        b, step = 2 * a, fo.MARGIN_REL * a
    else:
        a, b = float(grid[top]), float(grid[top + 1])
        step = cap.BISECT_TOL
    while b - a > step:
        m = 0.5 * (a + b)
        if h(m):
            a = m
        else:
            b = m
    out["margin"] = a
    return out


def _proven(recs, linked):
    """Do the feasible s of these frames, solved together at one s, run
    from 0? Yes when no frame needs joint torque of its own (jr = 0) and,
    with links, every actuator range holds 0: a solution at s scales to any
    s' < s (above 1, x times s'/s with the cone weights kept; at or below
    1, both times s'/s; the slacks take up the rest), every residual
    shrinking at least as fast as its budget and every link row still met.
    Not with an input box (the chain), whose lower bounds do not scale."""
    for r in recs:
        if not r["torque"]:
            continue
        if np.any(r["jr"] != 0):
            return False
        if linked and any(lim is not None and (lim[:, 0].max() > 0 or lim[:, 1].min() < 0)
                          for lim in r["lims"]):
            return False
    return True


def check(recs, rate=None, lag=None, per_frame=False, full_grid=False):
    """The trajectory readout over the checked frames recs (frame_record's,
    in time order). rate, lag: as follow(rate_limit=, lag_s=); with neither
    there are no link rows. per_frame: also each frame's largest scale with
    the rest of its run at s = 1 and every link in place (one search per
    frame over the whole run's LP, at every grid point; slow). full_grid:
    evaluate every grid point even where the feasible s are proven to run
    from 0.

    Returns {settings, frames, runs, pairs, lp}: per frame, whether it
    holds alone at s = 1 (unlinked_holds; reason when not), its scale
    margin alone (unlinked_margin), and the LP chain's (chain_lp_holds,
    chain_lp_margin: one carried solution, as follow's link, see _chain);
    per run (consecutive holding frames), whether it holds as a whole at s
    = 1, else the first frame at which it stops holding (the shortest
    failing prefix), and one scale margin for all its frames; per pair of
    consecutive frames in a run, the two-frame margin (an optimistic bound
    on the run's). Margins are None where no torque is checked."""
    st = _Stats()
    t_start = time.perf_counter()
    linked = rate is not None or lag is not None
    tol_frac = recs[0]["tol_frac"] if recs else fo.RESID_TOL

    def held(sub, scales):
        return _solve(sub, scales, rate, lag, stats=st)[0]

    def margin(sub):
        if not any(r["torque"] for r in sub):
            return None
        pv = _proven(sub, linked and len(sub) > 1) and not full_grid
        return _search(lambda s: held(sub, [s] * len(sub)), pv)

    frames = []
    for r in recs:
        fr = {"k": r["k"], "t": r["t"], "run": None, "unlinked_holds": False,
              "reason": r.get("reason"), "unlinked_margin": None}
        if r["ok"]:
            if r["torque"]:
                fr["unlinked_margin"] = margin([r])
                fr["unlinked_holds"] = held([r], [1.0])
                if not fr["unlinked_holds"]:
                    fr["reason"] = "torque"
            else:
                fr["unlinked_holds"] = True
        elif r["torque"]:
            # the direction or friction test fails at every scale
            fr["unlinked_margin"] = {"margin": 0.0, "capped": False, "contiguous": True,
                                     "search": None}
        frames.append(fr)

    # runs of consecutive holding frames
    runs, cur = [], []
    for i, fr in enumerate(frames):
        if fr["unlinked_holds"]:
            cur.append(i)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)

    out_runs, pairs = [], []
    for ri, idx in enumerate(runs):
        sub = [recs[i] for i in idx]
        for i in idx:
            frames[i]["run"] = ri
        ok = len(sub) == 1 or held(sub, [1.0] * len(sub))
        run = {"k_first": sub[0]["k"], "k_last": sub[-1]["k"], "n_frames": len(sub),
               "holds": bool(ok), "first_break_k": None, "first_break_t": None}
        if not ok:
            # prefixes: one frame holds, the whole run fails; more frames
            # only add rows, so the shortest failing prefix is bisected
            lo, hi = 1, len(sub)
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if held(sub[:mid], [1.0] * mid):
                    lo = mid
                else:
                    hi = mid
            run.update(first_break_k=sub[hi - 1]["k"], first_break_t=sub[hi - 1]["t"])
        run["margin"] = margin(sub)
        if per_frame:
            for j, i in enumerate(idx):
                if not sub[j]["torque"]:
                    frames[i]["trajectory_margin"] = None
                    continue
                # every grid point: the feasible s need not run from 0
                # (at s = 0 the frame needs no torque, and its neighbours'
                # link rows may not let it drop there and back)
                frames[i]["trajectory_margin"] = _search(
                    lambda s, j=j: held(sub, [s if jj == j else 1.0 for jj in range(len(sub))]),
                    False)
        out_runs.append(run)
        for a, b in zip(idx[:-1], idx[1:]):
            pairs.append({"k_a": recs[a]["k"], "k_b": recs[b]["k"], "run": ri,
                          "margin": margin([recs[a], recs[b]])})

    if linked:
        _chain(recs, frames, rate, lag, st, full_grid)
    return {"settings": {"rate_limit_per_s": rate, "lag_s": lag, "tol_frac": tol_frac,
                         "norm": NORM, "per_frame": per_frame, "full_grid": full_grid},
            "frames": frames, "runs": out_runs, "pairs": pairs,
            "lp": {"n_solves": st.n, "solve_s": st.wall,
                   "wall_s": time.perf_counter() - t_start, "retried": st.retried,
                   "other_status": st.other}}


def _chain(recs, frames, rate, lag, st, full_grid=False):
    """The LP chain, follow's link with this module's test: frame k's inputs
    boxed by follow._reach / _input_box from the net torque of the solution
    carried from the checked frame before (a finger not there starts from 0;
    the first checked frame is unboxed). The solution carried is the least
    total L1 residual one: within the budget when the frame holds, else
    with no budget (the least-bad attempt, as the NNLS chain carries its
    failed solve's). A frame failing the direction or friction test still
    carries its stack's solution; one with no contact or no torque columns
    carries nothing. Its margin keeps the box fixed as s varies (as
    follow._scale_margin with bounds). Writes chain_lp_holds and
    chain_lp_margin into frames.

    The box rows sit inside the residual, as in follow's chain, so a solve
    can leave its box and pay from the budget, while a run's link rows are
    hard. The chain can therefore take a step the run cannot, by up to the
    residual budget (tools/trajectory_box_check.py measures it)."""
    u_state, t_prev = {}, None
    for r, fr in zip(recs, frames):
        fr["chain_lp_holds"], fr["chain_lp_margin"] = False, None
        if not r["torque"]:
            fr["chain_lp_holds"] = bool(r["ok"])
            u_state, t_prev = {}, r["t"]
            continue
        bnd = None
        if t_prev is not None:
            dt = r["t"] - t_prev
            bnd = []
            for cols, a, lim in zip(r["Js"], r["fingers_on"], r["lims"]):
                if cols is None:
                    bnd.append(None)
                    continue
                u_lo, u_hi = fo._reach(lim, u_state.get(a), dt, rate, lag)
                bnd.append(fo._input_box(lim, cols[1].shape[1], u_lo, u_hi))
        bb = None if bnd is None else [bnd]
        ok, xs = _solve([r], [1.0], bounds=bb, stats=st)
        if not ok:
            xs = _solve([r], [1.0], bounds=bb, budget=False, stats=st)[1]
        lims = dict(zip(r["fingers_on"], r["lims"]))
        u_state = {a: fo._net_torque(lims[a], x) for a, x in xs.get(0, {}).items()}
        t_prev = r["t"]
        if r["ok"]:
            fr["chain_lp_holds"] = bool(ok)
            pv = bnd is None and _proven([r], False) and not full_grid
            fr["chain_lp_margin"] = _search(
                lambda s: _solve([r], [s], bounds=bb, stats=st)[0], pv)
        else:
            fr["chain_lp_margin"] = {"margin": 0.0, "capped": False, "contiguous": True,
                                     "search": None}
