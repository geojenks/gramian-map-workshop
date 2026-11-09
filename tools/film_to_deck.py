"""A film cut as a slide deck: one slide per shot, with the shot's clip, stills, caption and source.

The captions come from the cut's captions file (its markdown table: time, shot, caption with "Source:"),
so the slides say exactly what the film's caption file says; nothing on a slide is typed here. Shot
boundaries are found in each part's video by ffmpeg's scene-change filter, and the shot list says which
detected cuts bound each shot. The opening slide carries the captions file's introduction, the closing
slide its other sections (limits, rebuilding notes are left out).

    python tools/film_to_deck.py <shot list .json>

Shot list:
    {"captions": "<captions .md>", "out_dir": "<new folder>", "deck": "<name>.pptx", "title": "...",
     "parts": {"<key>": {"file": "<part .mp4>", "scene": 0.03}},
     "shots": [{"row": 1, "part": "<key>", "from": 0, "to": 1, "stills": 5}]}
"row" is the captions table row (1 = first). "from"/"to" index the part's boundaries: 0 is its start,
k its k-th detected cut, -1 its end. Stills sit at the centres of equal slices of the shot (a shot of
five equal snapshot holds gets one still per snapshot).

Needs python-pptx (the Python 3.10 install has it) and ffmpeg on PATH. Writes into a new folder; never
overwrites.
"""
import json, os, re, subprocess, sys

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.util import Inches, Pt

INK, MUTED, DARK, LIGHT = RGBColor(0x1F, 0x24, 0x2B), RGBColor(0x5B, 0x63, 0x6E), RGBColor(0x1F, 0x24, 0x2B), RGBColor(0xFF, 0xFF, 0xFF)
FONT = "Calibri"


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True, check=True)


def duration(path):
    return float(run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path]).stdout)


def cuts(path, scene):
    out = subprocess.run(["ffmpeg", "-hide_banner", "-i", path, "-vf", f"select='gt(scene,{scene})',showinfo", "-an", "-f", "null", "-"],
                         capture_output=True, text=True).stderr
    return [float(x) for x in re.findall(r"pts_time:([0-9.]+)", out)]


def read_captions(path):
    """Intro paragraphs, table rows (time, shot, caption, source), and the other sections."""
    text = open(path, encoding="utf-8").read().replace("`", "")
    lines = text.splitlines()
    rows, intro, rest, seen_table = [], [], {}, False
    sec = None
    for ln in lines[1:]:
        if ln.startswith("|"):
            seen_table = True
            cells = [c.strip() for c in ln.strip("|").split("|")]
            if cells[0] in ("Time",) or set(cells[0]) <= set("-: "):
                continue
            cap, _, src = cells[2].partition("Source:")
            rows.append({"time": cells[0], "shot": cells[1], "caption": cap.strip(), "source": src.strip()})
        elif ln.startswith("## "):
            sec = ln[3:].strip(); rest[sec] = []
        elif ln.strip():
            if sec:
                rest[sec].append(ln.strip())
            elif not seen_table:
                intro.append(ln.strip())
            else:
                rest.setdefault("", []).append(ln.strip())
    return intro, rows, rest


def text(slide, x, y, w, h, paras, size, color, bold=False, bullets=False):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)); tf = tb.text_frame; tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    for i, p_ in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        r = p.add_run(); r.text = ("• " if bullets else "") + p_
        r.font.size, r.font.bold, r.font.name, r.font.color.rgb = Pt(size), bold, FONT, color
        p.space_after = Pt(size * 0.5)
    return tb


def main():
    spec_path = sys.argv[1]
    S = json.load(open(spec_path, encoding="utf-8"))
    out = S["out_dir"]
    assert not os.path.exists(out), f"{out} exists: not overwriting"
    os.makedirs(os.path.join(out, "clips")); os.makedirs(os.path.join(out, "stills"))
    intro, rows, rest = read_captions(S["captions"])
    bounds = {}
    for k, p in S["parts"].items():
        c = cuts(p["file"], p["scene"]); bounds[k] = [0.0] + c + [duration(p["file"])]
        print(f"{k}: cuts at {[round(x, 2) for x in c]}; ends {bounds[k][-1]:.2f} s", flush=True)

    prs = Presentation(); prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
    blank = prs.slide_layouts[6]

    def dark_slide():
        s = prs.slides.add_slide(blank); s.background.fill.solid(); s.background.fill.fore_color.rgb = DARK
        return s

    s = dark_slide()
    text(s, 0.8, 1.2, 11.7, 1.2, [S["title"]], 36, LIGHT, bold=True)
    text(s, 0.8, 2.8, 10.5, 3.8, intro + rest.get("", []), 18, LIGHT)

    for n, sh in enumerate(S["shots"], 1):
        r = rows[sh["row"] - 1]; b = bounds[sh["part"]]
        t0, t1 = b[sh["from"]], b[sh["to"]]
        stem = f"shot{n:02d}"
        clip = os.path.join(out, "clips", stem + ".mp4")
        run(["ffmpeg", "-v", "error", "-ss", f"{t0:.3f}", "-to", f"{t1:.3f}", "-i", sh_file(S, sh), "-an",
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", clip])
        ns = sh.get("stills", 5); stills = []
        for j in range(ns):
            ts = t0 + (j + 0.5) * (t1 - t0) / ns
            p = os.path.join(out, "stills", f"{stem}_{j + 1}.png")
            run(["ffmpeg", "-v", "error", "-ss", f"{ts:.3f}", "-i", sh_file(S, sh), "-frames:v", "1", p]); stills.append(p)
        print(f"slide {n + 1}: {r['shot']}: {t0:.2f} to {t1:.2f} s of {os.path.basename(sh_file(S, sh))}, {ns} stills", flush=True)

        s = prs.slides.add_slide(blank)
        text(s, 0.5, 0.35, 12.3, 0.85, [r["shot"]], 24, INK, bold=True)
        s.shapes.add_movie(clip, Inches(0.5), Inches(1.35), Inches(8.0), Inches(4.5), poster_frame_image=stills[0], mime_type="video/mp4")
        if ns > 1:
            w = (8.0 - 0.12 * (ns - 1)) / ns
            for j, p in enumerate(stills):
                s.shapes.add_picture(p, Inches(0.5 + j * (w + 0.12)), Inches(6.0), Inches(w), Inches(w * 9 / 16))
        text(s, 8.85, 1.35, 4.0, 4.6, [r["caption"]], 12, INK)
        where = f"Film: {r['time']} of the cut" if r["time"][:1].isdigit() else r["time"]   # an alternative shot is not in the film
        text(s, 8.85, 6.0, 4.0, 1.0, ["Source: " + r["source"], where], 10, MUTED)
        s.notes_slide.notes_text_frame.text = f"{r['caption']}\n\nSource: {r['source']}\nClip: clips/{stem}.mp4 ({t0:.2f} to {t1:.2f} s of {os.path.basename(sh_file(S, sh))})"

    for sec, paras in rest.items():
        if not sec or sec.lower().startswith("rebuild"):
            continue
        s = dark_slide()
        text(s, 0.8, 0.8, 11.7, 0.9, [sec], 32, LIGHT, bold=True)
        text(s, 0.8, 2.0, 11.0, 5.0, [p.lstrip("- ") for p in paras], 18, LIGHT, bullets=True)

    deck = os.path.join(out, S["deck"]); prs.save(deck)
    json.dump(S, open(os.path.join(out, "shot_list_used.json"), "w", encoding="utf-8"), indent=1)
    print("wrote", os.path.abspath(deck), flush=True)


def sh_file(S, sh):
    return S["parts"][sh["part"]]["file"]


if __name__ == "__main__":
    main()
