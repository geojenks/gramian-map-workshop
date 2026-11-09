"""Slides that play like the film: the pictures as video, the words as text boxes, the snapshots as pictures,
each timed to appear and go with the video.

    python tools/layers_to_deck.py <spec .json>              build the deck
    python tools/layers_to_deck.py <spec .json> --qa <dir>   also export each slide as it looks at set times

Input is tools/render_film_layers.py's output: a video of each film part without the panel's words, and
a timeline of those words and the snapshot tiles. Each shot becomes one slide: its stretch of the video
plays by itself when the slide opens, and every text box, tile, highlight and verdict square has an
Appear entrance and exit at the time it shows and goes in the film. Shape names in the Selection pane
give those times. Speaker notes carry the shot's caption and source from the captions file. Nothing on
a slide is typed here.

Spec:
    {"captions": "<captions .md>", "out_dir": "<new folder>", "deck": "<name>.pptx", "title": "...",
     "parts": {"<key>": "<timeline .json>"},
     "shots": [{"part": "<key>", "head": "<panel heading that marks the shot>", "row": 1}],
     "qa_times": [0.5, ...]}    (seconds into each shot, for --qa)

Needs PowerPoint (driven through COM, so animations are written by PowerPoint itself), pywin32 and ffmpeg.
Writes into a new folder; never overwrites.
"""
import json, os, subprocess, sys

import win32com.client

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from film_to_deck import read_captions

PT = 960 / 1600     # points per film pixel: 1600 x 900 frame on a 13.333 x 7.5 in slide
APPEAR, MEDIA_PLAY, WITH_PREV = 1, 83, 2
DARK, LIGHT, INK, MUTED = "1F242B", "FFFFFF", "1E1E1E", "5B636E"


def bgr(h): return int(h[4:6], 16) * 65536 + int(h[2:4], 16) * 256 + int(h[0:2], 16)


def intervals(runs, key_of, t0, t1):
    """(key, start, end) for each stretch over which key_of(run) holds an item, within [t0, t1], relative to t0."""
    out, open_ = [], {}
    for r in runs:
        if r["to_s"] <= t0 + 1e-6 or r["from_s"] >= t1 - 1e-6: continue
        keys = {json.dumps(k, sort_keys=True): k for k in key_of(r)}
        for s in list(open_):
            if s not in keys: out.append((open_[s][0], open_[s][1], max(t0, r["from_s"]) - t0)); del open_[s]
        for s, k in keys.items():
            if s not in open_: open_[s] = (k, max(t0, r["from_s"]) - t0)
    for s, (k, a) in open_.items(): out.append((k, a, t1 - t0))
    return out


def timed(slide, shp, a, b, end, name):
    shp.Name = f"{name}, {a:.2f} to {b:.2f} s"
    seq = slide.TimeLine.MainSequence
    if a > 1e-3:
        e = seq.AddEffect(shp, APPEAR, 0, WITH_PREV); e.Timing.TriggerDelayTime = a
    if b < end - 1e-3:
        e = seq.AddEffect(shp, APPEAR, 0, WITH_PREV); e.Exit = True; e.Timing.TriggerDelayTime = b


def textbox(slide, x, y, w, h, paras, size, color, bold=False, font="Arial", space=1.15):
    """paras: list of paragraphs, each a list of (colour hex or None, text) runs."""
    tb = slide.Shapes.AddTextbox(1, x, y, w, h); tf = tb.TextFrame
    tf.MarginLeft = tf.MarginRight = tf.MarginTop = tf.MarginBottom = 0; tf.WordWrap = True; tf.AutoSize = 1
    tf.TextRange.Text = "\r".join("".join(t for _, t in p) for p in paras)
    tr = tf.TextRange; tr.Font.Name = font; tr.Font.Size = size; tr.Font.Bold = bold; tr.Font.Color.RGB = bgr(color)
    tr.ParagraphFormat.SpaceWithin = space; tr.ParagraphFormat.SpaceAfter = 0
    pos = 1
    for p in paras:
        for col, t in p:
            if col: tr.Characters(pos, len(t)).Font.Color.RGB = bgr(col)
            pos += len(t)
        pos += 1
    return tb


def shot_span(T, head):
    sel = [r for r in T["runs"] if r["items"][0]["paras"][0][0][1] == head]
    assert sel, f"no shot headed {head!r}"
    a, b = sel[0]["from_s"], sel[-1]["to_s"]
    assert all(r["items"][0]["paras"][0][0][1] == head for r in T["runs"] if a <= r["from_s"] < b), f"shot {head!r} is not one stretch"
    return a, b


def main():
    S = json.load(open(sys.argv[1], encoding="utf-8")); qa = sys.argv[sys.argv.index("--qa") + 1] if "--qa" in sys.argv else None
    out = os.path.abspath(S["out_dir"]); assert not os.path.exists(out), f"{out} exists: not overwriting"
    os.makedirs(os.path.join(out, "clips"))
    intro, rows, rest = read_captions(S["captions"])
    parts = {}
    for k, p in S["parts"].items():
        T = json.load(open(p, encoding="utf-8")); T["dir"] = os.path.dirname(os.path.abspath(p))
        dur = float(subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", T["video"]], capture_output=True, text=True, check=True).stdout)
        if T["runs"][-1]["to_s"] < dur - 0.5 / T["fps"]:    # the film's last frame is held by repeating it, not redrawn: the last run lasts to the video's end
            print(f"{k}: last run held from {T['runs'][-1]['to_s']:.2f} to the video's end at {dur:.2f} s", flush=True); T["runs"][-1]["to_s"] = dur
        parts[k] = T
    app = win32com.client.Dispatch("PowerPoint.Application")
    pres = app.Presentations.Add(True); pres.PageSetup.SlideWidth, pres.PageSetup.SlideHeight = 960, 540

    def dark_slide(title, paras, size):
        s = pres.Slides.Add(pres.Slides.Count + 1, 12)
        s.FollowMasterBackground = False; s.Background.Fill.Solid(); s.Background.Fill.ForeColor.RGB = bgr(DARK)
        textbox(s, 58, 60, 840, 60, [[(None, title)]], 30, LIGHT, True, "Calibri")
        textbox(s, 58, 150, 800, 360, [[(None, p)] for p in paras], size, LIGHT, False, "Calibri", 1.0)
        return s

    dark_slide(S["title"], intro + rest.get("", []), 16)
    log = []
    for n, sh in enumerate(S["shots"], 1):
        T = parts[sh["part"]]; r = rows[sh["row"] - 1]; t0, t1 = shot_span(T, sh["head"]); end = t1 - t0
        clip = os.path.join(out, "clips", f"shot{n:02d}.mp4")
        subprocess.run(["ffmpeg", "-v", "error", "-i", T["video"], "-ss", f"{t0:.4f}", "-to", f"{t1:.4f}", "-an",
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", clip], check=True)
        s = pres.Slides.Add(pres.Slides.Count + 1, 12)
        v = s.Shapes.AddMediaObject2(clip, False, True, 0, 0, 960, 540); v.Name = f"Video, 0.00 to {end:.2f} s"
        s.TimeLine.MainSequence.AddEffect(v, MEDIA_PLAY, 0, WITH_PREV)
        k = 0
        for it, a, b in intervals(T["runs"], lambda r_: r_["items"], t0, t1):
            tb = textbox(s, it["x"] * PT, it["y"] * PT, it["w"] * PT, 30, [[(c, x) for c, x in p] for p in it["paras"]], round(it["size_px"] * PT, 1), it["color"], it["bold"], space=1.15 if it["size_px"] < 22 else 1.0)
            k += 1; timed(s, tb, a, b, end, f"Text {k}")
        for i, tl in enumerate(T["tiles"]):
            a = max(0.0, tl["from_s"] - t0)
            if a >= end: continue
            p = s.Shapes.AddPicture(os.path.join(T["dir"], tl["file"]), False, True, tl["x"] * PT, tl["y"] * PT, tl["size"] * PT, tl["size"] * PT)
            timed(s, p, a, end, end, f"Snapshot {i + 1}")
        sz = T["tiles"][0]["size"] if T["tiles"] else 118

        def frame_box(i, col, name, a, b):
            tl = T["tiles"][i]; q = s.Shapes.AddShape(1, (tl["x"] - 3) * PT, (tl["y"] - 3) * PT, (sz + 5) * PT, (sz + 5) * PT)
            q.Fill.Visible = False; q.Line.ForeColor.RGB = bgr(col); q.Line.Weight = 4 * PT; timed(s, q, a, b, end, name)
        for i, a, b in intervals(T["runs"], lambda r_: [] if r_["active"] is None or str(r_["active"]) in r_["borders"] else [r_["active"]], t0, t1):
            frame_box(i, INK, f"Current snapshot {i + 1}", a, b)
        for (i, col), a, b in intervals(T["runs"], lambda r_: [(int(i_), c_) for i_, c_ in r_["borders"].items()], t0, t1):
            frame_box(i, col, f"Snapshot {i + 1} verdict", a, b)
        for (i, j, col), a, b in intervals(T["runs"], lambda r_: [(int(i_), j_, c_) for i_, cs in r_["marks"].items() for j_, c_ in enumerate(cs)], t0, t1):
            tl = T["tiles"][i]; q = s.Shapes.AddShape(1, (tl["x"] + j * (sz // 2 + 1)) * PT, (tl["y"] + sz + 5) * PT, (sz // 2) * PT, 16 * PT)
            q.Fill.ForeColor.RGB = bgr(col); q.Line.Visible = False
            timed(s, q, a, b, end, f"Snapshot {i + 1} {'position' if j == 0 else 'turning'} check")
        s.NotesPage.Shapes.Placeholders(2).TextFrame.TextRange.Text = f"{r['shot']}\r\r{r['caption']}\r\rSource: {r['source']}\rClip: clips/shot{n:02d}.mp4, {t0:.2f} to {t1:.2f} s of {os.path.basename(T['video'])}"
        log.append(dict(slide=pres.Slides.Count, shot=r["shot"], part=sh["part"], from_s=round(t0, 3), to_s=round(t1, 3), effects=s.TimeLine.MainSequence.Count))
        print(f"slide {pres.Slides.Count}: {r['shot']}: {t0:.2f} to {t1:.2f} s of {os.path.basename(T['video'])}, {s.TimeLine.MainSequence.Count} timed effects", flush=True)

    for sec, paras in rest.items():
        if not sec or sec.lower().startswith("rebuild"): continue
        dark_slide(sec, [p.lstrip("- ") for p in paras], 16)
    deck = os.path.join(out, S["deck"]); pres.SaveAs(deck)
    json.dump(dict(spec=S, slides=log), open(os.path.join(out, "built.json"), "w", encoding="utf-8"), indent=1)
    print("wrote", deck, flush=True)

    if qa:     # each shot slide as it looks at the set times: shapes shown by their timing, video at that moment
        os.makedirs(qa)
        for sl in log:
            s = pres.Slides(sl["slide"])
            for tq in S.get("qa_times", [0.5]):
                if tq > sl["to_s"] - sl["from_s"]: continue
                for shp in s.Shapes:
                    nm = shp.Name
                    if nm.startswith("Video"):
                        try: shp.MediaFormat.SetDisplayPicture(int(tq * 1000))
                        except Exception as e: print("poster frame not set:", e)
                        continue
                    a, b = [float(x) for x in nm.rsplit(", ", 1)[1].replace(" s", "").split(" to ")]
                    shp.Visible = a - 1e-3 <= tq < b + 1e-3
                s.Export(os.path.join(qa, f"slide{sl['slide']}_t{tq:g}.png"), "PNG", 1600, 900)
        for i in range(1, pres.Slides.Count + 1):
            if not any(sl["slide"] == i for sl in log): pres.Slides(i).Export(os.path.join(qa, f"slide{i}.png"), "PNG", 1600, 900)
        print("qa pictures in", qa, flush=True)
    pres.Close()


if __name__ == "__main__":
    main()
