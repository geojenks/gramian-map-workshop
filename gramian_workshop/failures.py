"""Which fingers and joints fail, and when, read from a stored follow result.

    from gramian_workshop.failures import summarise, brief, to_text, to_html
    s = summarise(result)          # result: the dict tools/run_follow.py saves

Nothing is re-checked and no threshold is applied here: every line restates
what the result file says (follow.py wrote it). Standard library only, so it
runs without mujoco or the hand models.

What a result can say about a finger at a frame:
  off     the leader has no contact for it there
  green   in place and its joints balance the contact force
  amber   close, and the hand as a whole still holds
  red     with its own reason ("reach", "nail side", "penetrates": the pad
          misses the leader's contact), or with none of its own, in which
          case the pad is in place and the frame's reason says what the hand
          as a whole lacked ("direction", "friction", "torque", "no_contact")
and about a joint:
  beyond_limit  posed past its range (side, and by how much: rad for a
                hinge, m for a slide)
  stops         resting on an end stop whose reaction is credited
The torque left unbalanced is stated per finger (unbalanced_Nm, and
unbalanced_N for slides), not per joint: a result does not say which joint
of a finger falls short.
"""
import collections
import os

from . import report

# cause code -> (letter, css colour role, what it means)
CAUSES = collections.OrderedDict([
    ("green", ("✓", "--st-good", "in place, joints balance the force")),
    ("amber", ("~", "--st-warn", "close; the hand as a whole still holds")),
    ("reach", ("r", "--st-critical", "red: the pad cannot reach its contact")),
    ("nail side", ("n", "--st-critical", "red: the pad faces away (nail side)")),
    ("penetrates", ("p", "--st-critical", "red: the finger sinks too far into the object")),
    ("facing", ("a", "--st-critical", "red: the pad cannot face its contact (older results)")),
    ("direction", ("d", "--st-critical", "red: pad in place; the hand as a whole cannot push that way")),
    ("friction", ("f", "--st-critical", "red: pad in place; the hand as a whole would slip")),
    ("torque", ("t", "--st-critical", "red: pad in place; the hand as a whole is too weak at the joints")),
    ("no_contact", ("–", "--st-critical", "red: no finger in place")),
    ("red", ("x", "--st-critical", "red: no reason stated in the result")),
    ("off", ("·", "--st-none", "not in contact at this frame")),
])
JOINT = collections.OrderedDict([
    ("past limit", ("L", "--st-serious", "posed past its range")),
    ("on stop", ("s", "--st-dir", "resting on an end stop, reaction credited")),
])
TINY = 1e-9   # display only: an unbalanced torque below this is not printed
OWN = "the finger's own test"
WHOLE = "the whole-hand force test"


def finger_cause(frame, x):
    """(cause code, where the result states it) for finger record x of
    frame. A red finger with no reason of its own takes the frame's."""
    st = x.get("state", "off")
    if st != "red":
        return st, None
    if x.get("reason"):
        return str(x["reason"]), OWN
    if frame.get("reason"):
        return str(frame["reason"]), WHOLE
    return "red", None


def _runs(keys):
    """[(i0, i1, key)] for runs of equal consecutive keys (i1 inclusive)."""
    out = []
    for i, k in enumerate(keys):
        if out and out[-1][2] == k:
            out[-1][1] = i
        else:
            out.append([i, i, k])
    return [tuple(r) for r in out]


def _rng(vals):
    vals = [v for v in vals if v is not None]
    return (min(vals), max(vals)) if vals else None


def summarise(result):
    """The result's failures as spans of consecutive frames:
      frames   [{i0, k0, k1, n, verdict, reason, unserved,
                 palm_penetration_mm, finger_finger_mm, finger_palm_mm}]
      fingers  {finger: [{i0, k0, k1, n, state, cause, stated_by, station,
                 reach_mm, facing_deg, side_deg, penetration_mm,
                 unbalanced_Nm, unbalanced_N}]}   ranges are (min, max)
      joints   {joint: [{i0, k0, k1, n, finger, kind, side, by | reaction,
                 decided_by}]}
      grid     per frame, the cause code of each finger and the joint
               events, for drawing
    plus hand, leader, ks, by_position, verdicts, finger_frames. Frames are
    named by the leader's frame index k, or by their position in the result
    where that index repeats (by_position); i0 is the span's first position
    in the result."""
    frames = result.get("frames") or []
    ks = [f.get("k", i) for i, f in enumerate(frames)]
    # a run with more frames than its leader repeats the leader's index;
    # frames are then named by their position in the result instead
    by_position = any(b <= a for a, b in zip(ks, ks[1:]))
    if by_position:
        ks = list(range(len(frames)))
    names = []
    for f in frames:
        for x in f.get("fingers", []):
            if x.get("finger") not in names:
                names.append(x.get("finger"))
    per = {n: [None] * len(frames) for n in names}
    for i, f in enumerate(frames):
        for x in f.get("fingers", []):
            per[x.get("finger")][i] = x

    def span(i0, i1):
        return {"i0": i0, "k0": ks[i0], "k1": ks[i1], "n": i1 - i0 + 1}

    out_frames = []
    fkey = [(f.get("verdict"), f.get("reason"), tuple(f.get("unserved") or ()))
            for f in frames]
    for i0, i1, (v, why, uns) in _runs(fkey):
        fs = frames[i0:i1 + 1]
        so = [f.get("self_overlap") or {} for f in fs]
        out_frames.append(dict(
            span(i0, i1), verdict=v, reason=why, unserved=list(uns),
            palm_penetration_mm=_rng([f.get("palm_penetration_mm") for f in fs]),
            finger_finger_mm=_rng([s.get("finger_finger_mm") for s in so]),
            finger_palm_mm=_rng([s.get("finger_palm_mm") for s in so])))

    out_fingers, grid_f = collections.OrderedDict(), collections.OrderedDict()
    for n in names:
        keys = []
        for i, f in enumerate(frames):
            x = per[n][i]
            keys.append(finger_cause(f, x) if x is not None else ("off", None))
        grid_f[n] = [k[0] for k in keys]
        spans = []
        for i0, i1, (cause, by) in _runs(keys):
            xs = [x for x in per[n][i0:i1 + 1] if x is not None]
            state = xs[0].get("state", "off") if xs else "off"
            s = dict(span(i0, i1), state=state, cause=cause, stated_by=by,
                     station=sorted({x.get("station") for x in xs
                                     if x.get("station") is not None}))
            for key in ("reach_mm", "facing_deg", "side_deg", "penetration_mm",
                        "unbalanced_Nm", "unbalanced_N"):
                s[key] = _rng([x.get(key) for x in xs])
            spans.append(s)
        out_fingers[n] = spans

    # joints: one track per (joint, kind, side, finger)
    tracks = collections.OrderedDict()
    for n in names:
        for i in range(len(frames)):
            x = per[n][i] or {}
            for kind, key, val in (("past limit", "beyond_limit", "by"),
                                   ("on stop", "stops", "reaction")):
                for e in x.get(key) or []:
                    t = tracks.setdefault((e.get("joint"), kind, e.get("side"), n),
                                          [None] * len(frames))
                    t[i] = (e.get(val), e.get("decided_by"))
    out_joints = collections.OrderedDict()
    grid_j = collections.OrderedDict()
    for (joint, kind, side, n), t in tracks.items():
        row = grid_j.setdefault(joint, [None] * len(frames))
        for i0, i1, on in _runs([v is not None for v in t]):
            if not on:
                continue
            vals = t[i0:i1 + 1]
            s = dict(span(i0, i1), finger=n, kind=kind, side=side)
            s["by" if kind == "past limit" else "reaction"] = _rng([v[0] for v in vals])
            if kind == "on stop":
                s["decided_by"] = sorted({str(v[1]) for v in vals if v[1] is not None})
            out_joints.setdefault(joint, []).append(s)
            for i in range(i0, i1 + 1):
                # past limit outranks on stop where a joint has both
                if row[i] is None or kind == "past limit":
                    row[i] = (kind, side, t[i][0])

    counts = collections.Counter(c for n in names for c in grid_f[n])
    states = collections.Counter(x["state"] for n in names for x in out_fingers[n]
                                 for _ in range(x["n"]))
    return {"hand": result.get("hand"), "hand_name": result.get("hand_name"),
            "leader": result.get("leader"), "resid_tol": result.get("resid_tol"),
            "ks": ks, "by_position": by_position, "frames": out_frames, "fingers": out_fingers,
            "joints": out_joints, "grid": {"fingers": grid_f, "joints": grid_j},
            "verdicts": dict(collections.Counter(f.get("verdict") for f in frames)),
            "finger_frames": dict(counts), "finger_states": dict(states)}


def _when(s):
    if s["n"] == 1:
        return f"frame {s['k0']}"
    return f"frames {s['k0']} to {s['k1']} ({s['n']})"


def _r(v, unit=""):
    if v is None:
        return ""
    a, b = f"{v[0]:.3g}", f"{v[1]:.3g}"
    return (a if a == b else f"{a} to {b}") + unit


def _side(side):
    return {"hi": "upper", "lo": "lower"}.get(side, str(side))


def _finger_detail(s):
    bits = []
    for key, lab, unit in (("reach_mm", "off its contact by", " mm"),
                           ("facing_deg", "facing", " deg off"),
                           ("penetration_mm", "into the object", " mm"),
                           ("unbalanced_Nm", "unbalanced", " N.m"),
                           ("unbalanced_N", "unbalanced (slides)", " N")):
        v = s.get(key)
        if v is None or (key.startswith("unbalanced") and max(v) < TINY):
            continue
        bits.append(f"{lab} {_r(v, unit)}")
    return "; ".join(bits)


def brief(s):
    """A few lines for a log: the fingers that are amber or red and when,
    and the joints posed past a limit."""
    v = s["verdicts"]
    lines = [f"{s.get('hand')} on {s.get('leader')}: {len(s['ks'])} frames, "
             + ", ".join(f"{v[c]} {c}" for c in ("green", "amber", "red") if v.get(c))
             + (" (frames numbered by position)" if s.get("by_position") else "")]
    for name, spans in s["fingers"].items():
        bits = []
        for x in spans:
            if x["state"] != "red":
                continue
            if x["stated_by"] == WHOLE:
                # the pad is in place: the reason is the whole hand's, and
                # this finger's own unbalanced torque says how much is its
                u = x.get("unbalanced_Nm")
                own = (f"; this finger unbalanced {_r(u, ' N.m')}" if u and max(u) >= TINY
                       else "; this finger balanced" if u else "")
                bits.append(f"red (whole hand: {x['cause']}{own}) {_when(x)}")
            else:
                bits.append(f"red ({x['cause']}) {_when(x)}")
        amber = sum(x["n"] for x in spans if x["state"] == "amber")
        if amber:
            bits.append(f"amber in {amber} of {len(s['ks'])} frames")
        if bits:
            lines.append(f"  {name}: " + "; ".join(bits))
    for joint, spans in s["joints"].items():
        for x in spans:
            if x["kind"] == "past limit":
                lines.append(f"  {joint}: past its {_side(x['side'])} limit by "
                             f"{_r(x['by'])}, {_when(x)}")
    if len(lines) == 1:
        lines.append("  no finger is amber or red, no joint is past a limit")
    return "\n".join(lines)


def _naming(s):
    if s.get("by_position"):
        return ("Frames are numbered by their position in the result, from 0: "
                "the leader's frame index repeats in it.")
    return "Frames are the leader's frame index."


def to_text(s):
    """The summary as Markdown tables."""
    L = [f"# Which fingers and joints fail, and when: {s.get('hand_name') or s.get('hand')}",
         "", f"Leader `{s.get('leader')}`, {len(s['ks'])} frames. Read from the stored "
         "result; nothing is re-checked. " + _naming(s), "",
         "## The hand as a whole", "",
         "| when | verdict | reason | stations with no finger | palm into object (mm) | "
         "finger in finger beyond rest (mm) | finger in palm beyond rest (mm) |",
         "|---|---|---|---|---|---|---|"]
    for f in s["frames"]:
        L.append(f"| {_when(f)} | {f['verdict']} | {f['reason'] or ''} | "
                 f"{', '.join(map(str, f['unserved']))} | {_r(f['palm_penetration_mm'])} | "
                 f"{_r(f['finger_finger_mm'])} | {_r(f['finger_palm_mm'])} |")
    L += ["", "## Fingers", "",
          "Green and off spans are left out. \"Stated by\" says whether a red is the "
          "finger's own (its pad misses the contact) or the whole hand's (the pad is in "
          "place and the force test fails).", "",
          "| finger | when | state | cause | stated by | measured |", "|---|---|---|---|---|---|"]
    rows = [f"| {name} | {_when(x)} | {x['state']} | "
            f"{x['cause'] if x['state'] == 'red' else ''} | {x['stated_by'] or ''} | "
            f"{_finger_detail(x)} |"
            for name, spans in s["fingers"].items() for x in spans
            if x["state"] in ("amber", "red")]
    L += rows or ["| (none) | | | | | |"]
    L += ["", "## Joints", "",
          "Past a limit: rad for a hinge, m for a slide. On a stop: the reaction the check "
          "credited (N.m, or N for a slide), which helps the hold and is not a failure.", "",
          "| joint | finger | when | what | side | by | reaction | decided by |",
          "|---|---|---|---|---|---|---|---|"]
    rows = [f"| {joint} | {x['finger']} | {_when(x)} | {x['kind']} | {_side(x['side'])} | "
            f"{_r(x.get('by'))} | {_r(x.get('reaction'))} | "
            f"{', '.join(x.get('decided_by', []))} |"
            for joint, spans in s["joints"].items() for x in spans]
    L += rows or ["| (none) | | | | | | | |"]
    L += ["", "The torque left unbalanced is stated per finger. Which joint of a finger "
          "falls short is not in a result file.", ""]
    return "\n".join(L)


def _legend(table):
    items = [f'<span><span class="sw" style="background:var({role});color:var({role}-ink)">'
             f'{report.esc(letter)}</span>{report.esc(text)}</span>'
             for letter, role, text in table.values()]
    return '<div class="legend">' + "".join(items) + "</div>"


def to_html(s):
    """One self-contained page: a finger-by-frame grid, a joint-by-frame
    grid, and the same content as text."""
    ks = s["ks"]
    cw = 34 if len(ks) <= 14 else 18
    cols = [k if (cw == 34 or j % 5 == 0) else "" for j, k in enumerate(ks)]
    fv = [f for f in s["frames"] for _ in range(f["n"])]
    names = list(s["fingers"])
    at = {n: [x for x in s["fingers"][n] for _ in range(x["n"])] for n in names}

    def cell(i, j):
        if i == 0:
            f = fv[j]
            code = f["verdict"] if f["verdict"] != "red" else (f["reason"] or "red")
            tip = f"frame {ks[j]}: {f['verdict']}" + (f", {f['reason']}" if f["reason"] else "")
        else:
            n = names[i - 1]
            code = s["grid"]["fingers"][n][j]
            x = at[n][j]
            tip = f"{n}, frame {ks[j]}: {x['state']}"
            if x["state"] == "red":
                tip += f", {code}" + (f" ({x['stated_by']})" if x["stated_by"] else "")
            d = _finger_detail(x)
            if d:
                tip += f". Over {_when(x)}: {d}"
        letter, role, _t = CAUSES.get(code, (str(code)[:1], "--st-critical", ""))
        return f"var({role})", letter, tip, f"var({role}-ink)"

    body = ['<div class="card">',
            report.grid_svg(["whole hand"] + names, cols, cell, label_w=120, cw=cw),
            _legend(CAUSES), "</div>",
            "<h2>Joints past a limit or on a stop</h2>"]
    joints = list(s["grid"]["joints"])
    if joints:
        def jcell(i, j):
            e = s["grid"]["joints"][joints[i]][j]
            if e is None:
                return ("var(--st-none)", "", f"{joints[i]}, frame {ks[j]}: nothing stated",
                        "var(--st-none-ink)")
            kind, side, val = e
            letter, role, _t = JOINT[kind]
            what = (f"past its {_side(side)} limit by {val:.3g}" if kind == "past limit"
                    else f"on its {_side(side)} stop, reaction {val:.3g}")
            return (f"var({role})", letter, f"{joints[i]}, frame {ks[j]}: {what}",
                    f"var({role}-ink)")
        body += ['<div class="card">',
                 report.grid_svg(joints, cols, jcell, label_w=120, cw=cw),
                 _legend(JOINT), "</div>"]
    else:
        body.append('<p class="note">No joint is posed past its range or credited with a '
                    'stop reaction in this result.</p>')
    body += ["<h2>The same, as text</h2>",
             f'<div class="card"><pre>{report.esc(to_text(s))}</pre></div>']
    title = f"Which fingers and joints fail, and when: {s.get('hand_name') or s.get('hand')}"
    caveat = (f"Leader {s.get('leader')}, {len(ks)} frames. Read from the stored follow "
              "result; nothing is re-checked. " + _naming(s) + " Hover a cell for its numbers. The torque left "
              "unbalanced is per finger: a result does not say which joint of a finger "
              "falls short.")
    return report.page(title, caveat, "".join(body))


def write(result, stem, out_dir, tag=None):
    """Write <stem>__failures[__tag].md and .html into out_dir, never over
    an existing file. Returns (summary, [paths])."""
    s = summarise(result)
    name = f"{stem}__failures" + (f"__{tag}" if tag else "")
    paths = [os.path.join(out_dir, f"{name}.{ext}") for ext in ("md", "html")]
    for p in paths:
        if os.path.exists(p):
            raise FileExistsError(f"{p} exists; stored outputs are not overwritten "
                                  "(give another --tag or --out-dir)")
    os.makedirs(out_dir, exist_ok=True)
    for p, text in zip(paths, (to_text(s), to_html(s))):
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    return s, paths
