"""One page of what a participant sees on the day, screen by screen: the
browser pages as pictures, the terminal as the text it prints, and the
files they open.

    python tools/make_day_view.py --out outputs/day_view_12nov/day_view.html
    python tools/make_day_view.py --inline --out ONE_FILE.html
    python tools/make_day_view.py --capture

The page runs nothing. Terminal text is read from logs: the stored run of
11 Nov (outputs/run_through_11nov/logs) for the steps up to the task, and
the day folder's own logs (outputs/day_view_12nov/logs) for the verdict
steps, which were run again on 12 Nov so that the pages are the ones the
tool writes now. The pictures are of the real pages, taken by --capture
with a headless Edge or Chrome into the day folder's shots/; --capture
never writes over a picture. The 12 Nov steps wrote to a scratch folder;
their files were then copied into the day folder and the logs' paths
rewritten to match (logs/verdicts.sh is the script that ran). The prose is in SCREENS below and carries no
numbers of its own.

To change the page, edit SCREENS. --inline puts the pictures inside the
page, so that it is one file to send; without it the page names them as
shots/NAME.png beside it. The page is a fragment (no <html> or <body>),
which is what the page host wants; a browser opens it as it is. --out is
never written over.

Standard library only.
"""
import argparse
import base64
import html
import io
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from make_walkthrough import parse_log, prose, read  # noqa: E402

E = html.escape
RUN = os.path.join(REPO, "outputs", "run_through_11nov")
DAY = os.path.join(REPO, "outputs", "day_view_12nov")
R = "results/follow_%s__hold_%s__self__selfoverlapfix__fixed__failures.html"
BROWSERS = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Google\Chrome\Application\chrome.exe"]
LONG = 16     # a terminal output longer than this many lines is cut
TAIL = 6      # ... to its last lines, with the count of those left out

# ---------------------------------------------------------------- the screens
# ("head", title, text)                         a part of the day
# ("page", shot, source, (w, h), title, note)   a browser page: shots/<shot>.png
#                                               is a picture of <source>, a path
#                                               under the day folder or "repo:..."
# ("picture", file, title, note)                a picture file the tool wrote,
#                                               shown as it is (under the day folder)
# ("term", [(log, step), ...], title, note)     a terminal: the commands and what
#                                               they print, from the logs
# ("table", file, title, note)                  a Markdown table file the tool
#                                               wrote (under the day folder)

SCREENS = [
 ("head", "The quick start",
  "After the install, the README's first three commands run the demos on "
  "the stored records. Each prints a few lines and writes a page."),
 ("page", "cup_hold", "repo:outputs/cup_hold.html", (1280, 900),
  "The page a demo writes",
  "One row per stored actuator, one column per number of fingers. Hovering "
  "a cell gives its numbers. This is the page stored in the repository; "
  "the demos were not run again for this view."),

 ("head", "Bring an actuator",
  "Three ways in: type datasheet numbers into a form, give test curves, or "
  "give a table of joints."),
 ("page", "record_form", "repo:forms/record_form.html", (1280, 1240),
  "The record form, as it opens",
  "The values shown are the form's own starting values. The button at the "
  "bottom downloads a record file. The picture shows the top of the form."),
 ("term", [("run0", "curves_to_record"), ("run0", "check_curves_record")],
  "Test curves to a record, and what the tool reads from it",
  "No page or picture is written at this step. The check says in words "
  "that this record has motion and no force."),
 ("term", [("run0", "table_to_record"), ("run0", "check_table_record")],
  "A joint table to a record", ""),

 ("head", "Look at the records' Gramians",
  "One command draws every stored record's Gramian in one figure."),
 ("term", [("run0", "gramians_mujoco_python")],
  "In the environment the install gives",
  "It stops for want of matplotlib, which no install extra provides "
  "(issue 36)."),
 ("term", [("gramians", "gramians")], "In an environment that has matplotlib", ""),
 ("picture", "gramians/record_gramians_b.png", "The figure it writes", ""),

 ("head", "Make a hand",
  "Either fingers built from a record, placed on a ring by a hand file the "
  "participant writes, or a hand they bring as a model file."),
 ("term", [("run0", "build_curves_record_by_id"), ("run0", "build_curves_record_by_path")],
  "Building a hand from the test-curve record: the first two tries",
  "Both stop. The first needs the record named by its path, the second "
  "needs a contact radius in the hand file. Neither is in the docs "
  "(issue 32)."),
 ("term", [("run3", "build")], "The build once the hand file is right",
  "The tool draws no picture of the hand it built. What the participant "
  "has is this text, a model file and a fit report."),
 ("term", [("run0", "build_table_record_by_path")],
  "Building a hand from the joint-table record",
  "It crashes, and this way in ends here (issue 33)."),
 ("term", [("verdicts", "add_allegro")], "A hand brought as a model file",
  "The tool repeats back what it read from the model, then gives the next "
  "two commands."),

 ("head", "Give it a task",
  "The simplest task is a hold made from the hand's own kinematics."),
 ("term", [("run", "hold_allegro"), ("run", "hold_mosadegh")],
  "Make a hold for each hand",
  "One line each. The second hand's `facing` is the only sign, at this "
  "step, that its fingers curl away from the ball."),

 ("head", "The verdict",
  "The check prints its verdict in the terminal. Two more commands turn the "
  "result into pages. These steps were run again on 12 Nov with the code "
  "as it is now."),
 ("term", [("verdicts", "follow_allegro_left")], "The check on the model-file hand", ""),
 ("term", [("verdicts", "follow_mosadegh_x3_ring")],
  "The check on the test-curve hand: red",
  "Every finger is red for reach. The fix is to reverse each finger's "
  "`bend_axis` in the hand file; the tool does not say so."),
 ("term", [("verdicts", "follow_mosadegh_x3_ring_flipped")],
  "The same hand with the bend axis reversed: green",
  "This green is not a strength result: the record has no force data "
  "(issue 35). It also stands with a base hinge posed past its limit "
  "(issue 34)."),
 ("term", [("verdicts", "failures_mosadegh_x3_ring_flipped")],
  "Which fingers and joints fail", "It prints this and writes the page below."),
 ("page", "failures_allegro", R % ("allegro_left", "allegro_left"), (1280, 1500),
  "The failures page: the model-file hand", "Top of the page."),
 ("page", "failures_red", R % ("mosadegh_x3_ring", "mosadegh_x3_ring"), (1280, 1100),
  "The failures page: the red hand", "Top of the page."),
 ("page", "failures_green", R % ("mosadegh_x3_ring_flipped", "mosadegh_x3_ring_flipped"),
  (1280, 1500), "The failures page: the hand that closes", "Top of the page."),
 ("page", "follow", "results/follow_today.html", (1280, 1500),
  "The page for all three hands", "Top of the page."),
 ("table", "results/follow_today.md", "The margins table", ""),
]


def load(day, run):
    logs = {}
    for d in (os.path.join(run, "logs"), os.path.join(day, "logs")):
        for f in sorted(os.listdir(d)):
            if f.endswith("_log.txt"):
                assert f[:-8] not in logs, f
                logs[f[:-8]] = parse_log(os.path.join(d, f))
    return {"day": day, "run": run, "logs": logs,
            "day_name": os.path.relpath(day, REPO).replace(os.sep, "/"),
            "run_name": os.path.relpath(run, REPO).replace(os.sep, "/")}


def shown_cmd(cmd):
    """The command as the docs give it: python, not the environment's path."""
    return re.sub(r"<(hands|analysis) env>/Scripts/python\.exe", "python", cmd)


def image(ctx, rel, alt):
    if ctx["inline"]:
        with open(os.path.join(ctx["day"], rel), "rb") as f:
            src = "data:image/png;base64," + base64.b64encode(f.read()).decode("ascii")
    else:
        assert os.path.isfile(os.path.join(ctx["day"], rel)), rel
        src = rel
    return '<img src="%s" alt="%s" loading="lazy">' % (src, E(alt, quote=True))


def source(ctx, src):
    if src.startswith("repo:"):
        return os.path.join(REPO, src[5:]), src[5:]
    return os.path.join(ctx["day"], src), ctx["day_name"] + "/" + src


def frame(kind, label, title, note, body, src):
    return ('<article class="screen %s"><header><h3>%s</h3>'
            '<span class="where">%s</span></header>%s%s'
            '<p class="src">%s</p></article>'
            % (kind, prose(title), E(label), '<p>%s</p>' % prose(note) if note else "",
               body, src))


def r_page(ctx, shot, src, size, title, note):
    full, shown = source(ctx, src)
    assert os.path.isfile(full), full
    body = ('<div class="win"><div class="bar"><i></i><i></i><i></i>'
            '<span>%s</span></div>%s</div>'
            % (E(os.path.basename(shown)), image(ctx, "shots/%s.png" % shot, title)))
    return frame("page", "browser page", title, note, body,
                 "Picture of %s. Hovering does nothing in a picture." % E(shown))


def r_picture(ctx, rel, title, note):
    return frame("page", "picture file", title, note,
                 '<div class="win">%s</div>' % image(ctx, rel, title),
                 E(ctx["day_name"] + "/" + rel))


def r_term(ctx, steps, title, note):
    out, srcs = [], []
    for log, name in steps:
        s = ctx["logs"][log][name]
        out.append('<span class="p">$ %s</span>' % E(shown_cmd(s["cmd"])))
        rows = s["out"].strip("\n").split("\n") if s["out"].strip() else []
        if len(rows) > LONG:
            out.append('<span class="cut">[%d earlier lines not shown]</span>'
                       % (len(rows) - TAIL))
            rows = rows[-TAIL:]
        out.append('<span class="%s">%s</span>' % ("o" if s["exit"] == 0 else "o bad",
                                                   E("\n".join(rows))))
        srcs.append("%s, step %s (exit %d, %d s)"
                    % (log + "_log.txt", name, s["exit"], s["secs"]))
    return frame("term", "terminal", title, note,
                 '<div class="win"><pre>%s</pre></div>' % "\n".join(out),
                 E("; ".join(srcs)))


def r_table(ctx, rel, title, note):
    rows = [l for l in read(os.path.join(ctx["day"], rel)).split("\n") if l.startswith("|")]
    cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
    assert len(cells) > 2 and set("".join(cells[1])) <= set("-: "), rel
    t = "<tr>%s</tr>" % "".join("<th>%s</th>" % prose(c) for c in cells[0])
    t += "".join("<tr>%s</tr>" % "".join("<td>%s</td>" % prose(c) for c in r)
                 for r in cells[2:])
    return frame("file", "text file", title, note,
                 '<div class="win scroll"><table>%s</table></div>' % t,
                 E(ctx["day_name"] + "/" + rel))


RENDER = {"page": r_page, "picture": r_picture, "term": r_term, "table": r_table}

CSS = """
/* layout: one column of framed screens in the order of the day, each
   labelled by where the participant sees it */
:root{--bg:#f6f5f1;--panel:#fffefb;--ink:#1d1f1c;--muted:#666a62;--line:#d9d6cc;
--accent:#2f5d50;--bad:#a23b2a;--term:#1b1e1c;--termink:#dfe3da;--termdim:#8d9389;
--termbad:#f0a090;--prompt:#9fd3bf}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;
--accent:#8fc7b3;--bad:#ef9a8a;--term:#0e100f;--termink:#dfe3da;--termdim:#8d9389;
--termbad:#f0a090;--prompt:#9fd3bf}}
:root[data-theme="dark"]{color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;
--accent:#8fc7b3;--bad:#ef9a8a;--term:#0e100f;--termink:#dfe3da;--termdim:#8d9389;
--termbad:#f0a090;--prompt:#9fd3bf}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
font:16px/1.55 "Source Sans 3",system-ui,sans-serif}
.wrap{max-width:980px;margin:0 auto;padding-block:32px 64px;
padding-left:max(16px,env(safe-area-inset-left,0px));
padding-right:max(16px,env(safe-area-inset-right,0px));
display:flex;flex-direction:column;gap:18px}
h1{font:600 30px/1.2 "Source Serif 4",Georgia,serif;margin:0;text-wrap:balance}
h2{font:600 23px/1.25 "Source Serif 4",Georgia,serif;margin:22px 0 0;text-wrap:balance}
h3{font-size:16px;font-weight:600;margin:0}
p{margin:0;max-width:70ch}a{color:var(--accent)}
.lede,.part{color:var(--muted)}
code{font:13px "JetBrains Mono",ui-monospace,Consolas,monospace;overflow-wrap:anywhere}
.count{display:flex;flex-wrap:wrap;gap:8px 22px;padding:12px 0;border-block:1px solid var(--line);
font-variant-numeric:tabular-nums}
.count b{font-weight:600}
.screen{display:flex;flex-direction:column;gap:8px;min-width:0}
.screen header{display:flex;justify-content:space-between;align-items:baseline;gap:12px;flex-wrap:wrap}
.where{font-size:12.5px;font-weight:600;letter-spacing:.06em;text-transform:uppercase;color:var(--muted)}
.term .where{color:var(--accent)}
.win{border:1px solid var(--line);border-radius:8px;overflow:hidden;background:var(--panel);min-width:0}
.win img{display:block;width:100%;height:auto}
.bar{display:flex;align-items:center;gap:6px;padding:7px 10px;border-bottom:1px solid var(--line);
font-size:12.5px;color:var(--muted)}
.bar i{width:9px;height:9px;border-radius:50%;background:var(--line);flex:none}
.bar span{margin-left:8px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;min-width:0}
.term .win{background:var(--term);border-color:var(--term)}
.term pre{margin:0;padding:12px 14px;color:var(--termink);white-space:pre-wrap;overflow-wrap:anywhere;
font:13px/1.5 "JetBrains Mono",ui-monospace,Consolas,monospace}
.term .p{color:var(--prompt)}.term .cut{color:var(--termdim)}.term .bad{color:var(--termbad)}
.scroll{overflow-x:auto}
table{border-collapse:collapse;font-size:14px;font-variant-numeric:tabular-nums;width:100%}
th,td{text-align:left;padding:7px 10px;border-bottom:1px solid var(--line);vertical-align:top}
th{color:var(--muted);font-weight:600}
.src{font-size:12.5px;color:var(--muted);overflow-wrap:anywhere}
footer{margin-top:28px;padding-top:14px;border-top:1px solid var(--line);color:var(--muted);font-size:14px}
a:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
"""


def build(day=DAY, run=RUN, inline=False):
    ctx = load(os.path.abspath(day), os.path.abspath(run))
    ctx["inline"] = inline
    body = []
    for s in SCREENS:
        if s[0] == "head":
            body.append('<h2>%s</h2><p class="part">%s</p>' % (E(s[1]), prose(s[2])))
        else:
            body.append(RENDER[s[0]](ctx, *s[1:]))
    n = {k: sum(1 for s in SCREENS if s[0] == k) for k in ("page", "picture", "term", "table")}
    count = ('<div class="count"><span><b>%d</b> terminal screens</span>'
             '<span><b>%d</b> browser pages</span><span><b>%d</b> picture file</span>'
             '<span><b>%d</b> text table</span></div>'
             % (n["term"], n["page"], n["picture"], n["table"]))
    head = (
     '<title>Workshop day view</title>'
     '<meta name="claude:viewport-fit-sides" content="cover">'
     '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono&amp;'
     'family=Source+Sans+3:wght@400;600&amp;family=Source+Serif+4:wght@600&amp;display=swap">'
     '<style>%s</style>' % CSS)
    top = (
     '<h1>What a participant sees on the day</h1>'
     '<p class="lede">The screens of the workshop tool as it is now, in the order a '
     'participant meets them. Each is labelled by where it appears. Most of the day '
     'is a terminal; the pages come at the start and at the verdict.</p>'
     '<p class="lede">The screens under The verdict, the Gramian figure and the '
     'model-file hand were run on 12 Nov. The other terminal screens are the stored '
     'run of 11 Nov; the code behind those steps has not changed since.</p>' + count)
    foot = ('<footer><p>Made by <code>tools/make_day_view.py</code> from %s and %s. '
            'The commands behind every screen, with the files they read and wrote, '
            'are on the walkthrough page of the same run.</p></footer>'
            % (E(ctx["run_name"]), E(ctx["day_name"])))
    return '%s<div class="wrap">%s%s%s</div>\n' % (head, top, "".join(body), foot)


def capture(day):
    exe = next((b for b in BROWSERS if os.path.isfile(b)), None)
    if exe is None:
        sys.exit("no Edge or Chrome found in " + "; ".join(BROWSERS))
    ctx = {"day": day, "day_name": os.path.relpath(day, REPO).replace(os.sep, "/")}
    os.makedirs(os.path.join(day, "shots"), exist_ok=True)
    profile = os.path.join(os.environ.get("TEMP", day), "day_view_browser_profile")
    for s in SCREENS:
        if s[0] != "page":
            continue
        shot, src, (w, h) = s[1], s[2], s[3]
        out = os.path.join(day, "shots", shot + ".png")
        if os.path.exists(out):
            print("kept  ", out)
            continue
        full = source(ctx, src)[0]
        subprocess.run([exe, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                        "--user-data-dir=" + profile, "--window-size=%d,%d" % (w, h),
                        "--screenshot=" + out,
                        "file:///" + os.path.abspath(full).replace(os.sep, "/")],
                       check=True, capture_output=True)
        print("wrote ", out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--day", default=DAY)
    ap.add_argument("--run", default=RUN)
    ap.add_argument("--capture", action="store_true")
    ap.add_argument("--inline", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    if a.capture:
        capture(os.path.abspath(a.day))
    if a.out:
        page = build(a.day, a.run, a.inline)
        with io.open(a.out, "x", encoding="utf-8", newline="\n") as f:
            f.write(page)
        print("wrote", a.out, "(%d bytes)" % len(page.encode("utf-8")))
    elif not a.capture:
        ap.error("give --out or --capture")


if __name__ == "__main__":
    main()
