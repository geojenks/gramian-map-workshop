"""A film part as layers for slides: the pictures as video, and the panel's words and snapshot tiles as a timeline.

    python tools/render_film_layers.py <layers folder> <render_force_film.py arguments ...>

Runs tools/render_force_film.py unchanged, except that each frame's panel is not painted in words.
The video it writes (the --out path) keeps only what is drawn: the scene, the inset, the friction discs
with their live readouts, the count bars, and the running clock in the corner. Everything else the
panel would have said is written to <layers folder>/<out name>_timeline.json, one entry per run of
frames that say the same thing, with each item's place in the 1600 x 900 frame. Snapshot tiles are
saved as PNGs in the same folder. tools/layers_to_deck.py turns these into slides.

Writes into a new folder; never overwrites.
"""
import json, os, sys, textwrap
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
import render_force_film as R

OUT_DIR = sys.argv[1]; sys.argv = [sys.argv[0]] + sys.argv[2:]
STEM = os.path.splitext(os.path.basename(sys.argv[sys.argv.index("--out") + 1]))[0]
os.makedirs(OUT_DIR, exist_ok=False)
FPS = int(sys.argv[sys.argv.index("--fps") + 1]) if "--fps" in sys.argv else 30
hexc = lambda c: "%02X%02X%02X" % tuple(int(v) for v in c[:3])

state = dict(n=0, runs=[], tiles=[])


def text_item(x, y, w, size, bold, color, paras):
    return dict(x=x, y=y, w=w, size_px=size, bold=bold, color=hexc(color), paras=paras)


def frame(c, pic, head, lines, fingers, mu, plane, tag, note=None, active=None, footer="Simulated: comparative, not predictive.", inset=None, borders=None, strips=None, marks=None, bars=None):
    """Panel.frame with the words lifted out: same layout, same drawn marks."""
    W, H, P = R.W, R.H, R.P
    im = Image.new("RGB", (W, H), R.BG); im.paste(Image.fromarray(pic), (0, 0)); dr = ImageDraw.Draw(im)
    if inset is not None: im.paste(inset, (12, P - inset.height - 12))
    x, y = P + 20, 30; items = []; tw_ = W - 20 - x
    hl = textwrap.wrap(head, 36); items.append(text_item(x, y, tw_, 36, True, R.INK, [[(None, head)]])); y += 44 * len(hl) + 6
    paras = []; y0 = y
    for ln in lines:
        col = None
        if isinstance(ln, tuple): col, ln = ln
        paras.append([(hexc(col), "■ ")] + [(None, ln)] if col else [(None, ln)])
        y += 26 * len(textwrap.wrap(ln, 62))
    if paras: items.append(text_item(x, y0, tw_, 19, False, R.INK, paras))
    if strips:
        y += 18
        for name, res in strips:
            cnt = {v: sum(f["verdict"] == v for f in res) for v in R.RGB}
            items.append(text_item(x, y, tw_, 19, False, R.INK, [[(None, f"{name}: {cnt['green']} green, {cnt['amber']} amber, {cnt['red']} red of {len(res)}")]])); y += 26
            tw = max(2, min(13, (W - x - 30) // len(res) - 1))
            for i, f in enumerate(res): dr.rectangle([x + i * (tw + 1), y, x + i * (tw + 1) + tw - 1, y + 24], fill=R.RGB[f["verdict"]])
            y += 42
    elif bars:
        y += 14
        for name, rows_ in bars:
            items.append(text_item(x, y, tw_, 24, True, R.INK, [[(None, name)]])); y += 30
            for what_, res in rows_:
                cnt = {v: res.count(v) for v in R.RGB}
                items.append(text_item(x, y, tw_, 19, False, R.INK, [[(None, f"{what_}: {cnt['green']} green, {cnt['amber']} amber, {cnt['red']} red of {len(res)}")]])); y += 24
                tw = max(2, min(13, (W - x - 30) // len(res) - 1))
                for i, v_ in enumerate(res): dr.rectangle([x + i * (tw + 1), y, x + i * (tw + 1) + tw - 1, y + 16], fill=R.RGB[v_])
                y += 24
            y += 8
    elif fingers is not None:   # the discs and their readouts change every frame: they stay in the picture
        n = len(c.names); rows = (n + 1) // 2; top = y + 6; h = (660 - top) // rows
        for i, (nm, f) in enumerate(zip(c.names, fingers)): c.disc(dr, x + (i % 2) * 335, top + (i // 2) * h, h, nm, f, mu, plane)
    if note:
        ls_ = textwrap.wrap(note, 52); items.append(text_item(x, 664 - 29 * (len(ls_) - 1), tw_, 24, True, R.INK, [[(None, note)]]))
    items.append(text_item(x, H - 46, tw_, 19, False, R.SOFT, [[(None, footer)]]))
    if tag:
        if " t = " in tag:     # a running clock: drawn, as in the film
            b = dr.textbbox((20, 18), tag, font=R.MID); dr.rectangle([b[0] - 8, b[1] - 6, b[2] + 8, b[3] + 6], fill=R.BG); dr.text((20, 18), tag, fill=R.INK, font=R.MID)
        else: items.append(text_item(20, 18, 600, 25, False, R.INK, [[(None, tag)]]))
    while len(state["tiles"]) < len(c.tiles):    # a new snapshot tile: saved once, at the frame it first shows
        i = len(state["tiles"]); p = f"{STEM}_tile{i + 1}.png"; c.tiles[i].save(os.path.join(OUT_DIR, p))
        state["tiles"].append(dict(file=p, from_frame=state["n"], x=x + i * 126, y=702, size=118))
    key = dict(items=items, active=active, borders={str(k): hexc(v) for k, v in (borders or {}).items()},
               marks={str(k): [hexc(q) for q in v] for k, v in (marks or {}).items()})
    s = json.dumps(key, sort_keys=True)
    if state["runs"] and state["runs"][-1]["key"] == s: state["runs"][-1]["to_frame"] = state["n"] + 1
    else: state["runs"].append(dict(key=s, from_frame=state["n"], to_frame=state["n"] + 1))
    state["n"] += 1
    return np.asarray(im, np.uint8)


R.Panel.frame = frame
_close = R.Video.close


def close(v):
    _close(v)
    runs = [dict(from_s=r["from_frame"] / FPS, to_s=r["to_frame"] / FPS, **json.loads(r["key"])) for r in state["runs"]]
    for t in state["tiles"]: t["from_s"] = t["from_frame"] / FPS
    json.dump(dict(fps=FPS, frames=state["n"], frame_px=[R.W, R.H], video=os.path.abspath(sys.argv[sys.argv.index("--out") + 1]),
                   args=sys.argv[1:], runs=runs, tiles=state["tiles"]),
              open(os.path.join(OUT_DIR, f"{STEM}_timeline.json"), "w", encoding="utf-8"), indent=1)
    print("timeline", len(runs), "runs,", len(state["tiles"]), "tiles", flush=True)


R.Video.close = close
R.main()
