"""One page that walks through a stored run of the whole chain: actuator in,
record, hand, task, verdict.

    python tools/make_walkthrough.py --out outputs/walkthrough_11nov_b.html
    python tools/make_walkthrough.py --run outputs/run_through_11nov --out PAGE.html

The page runs nothing. Every command, exit code, time and line of output on
it is read from the run folder's logs (logs/*_log.txt); the verdicts are read
from its tables.md; the input and hand files are shown as stored. The prose
between them is in WAYS below and carries no numbers of its own.

To change the page, edit WAYS: each way in is a list of stages, each stage a
list of items, and each kind of item has one function in RENDER. To show a
new run, store it in a folder of the same shape and pass --run. The page is
a fragment (no <html> or <body>), which is what the page host wants; a
browser opens it as it is. --out is never written over.

The page for the run of 11 Nov is published at
https://claude.ai/artifact/NErwUAEUktWKWvSaKcL9aB (private); a later build
for the same reader should update that page, not start another. A rebuild
needs a new file name.

Standard library only.
"""
import argparse
import html
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
ISSUES = "https://github.com/geojenks/gramian-map-workshop/issues/"


def read(path):
    with io.open(path, encoding="utf-8") as f:
        return f.read().replace("\r\n", "\n")


def parse_log(path):
    """{step name: {cmd, out, exit, secs}} from one log."""
    steps, name, cmd, out = {}, None, None, []
    for line in read(path).split("\n"):
        m = re.match(r"^=== (\S+)$", line)
        if m:
            assert name is None, (path, line)
            name, cmd, out = m.group(1), None, []
            continue
        if name is None:
            continue
        m = re.match(r"^--- (\S+): exit (\d+) in (\d+) s$", line)
        if m and m.group(1) == name:
            assert cmd is not None, (path, name)
            steps[name] = {"cmd": cmd, "out": "\n".join(out),
                           "exit": int(m.group(2)), "secs": int(m.group(3))}
            name = None
        elif cmd is None and line.startswith("$ "):
            cmd = line[2:]
        else:
            out.append(line)
    assert name is None, (path, "log ends inside step " + str(name))
    return steps


def parse_verdicts(path):
    """Rows of the Verdicts table of tables.md, as {column: cell}."""
    body = read(path).split("## Verdicts\n", 1)[1].strip().split("\n")
    cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in body]
    head, rows = cells[0], cells[2:]
    assert head[0] == "Result" and rows, head
    return [dict(zip(head, r)) for r in rows]


E = html.escape


def issue(n):
    return '<a href="%s%d">issue %d</a>' % (ISSUES, n, n)


def prose(text):
    """Escape, then `code` and (issue N)."""
    t = E(text)
    t = re.sub(r"`([^`]+)`", r"<code>\1</code>", t)
    return re.sub(r"issue (\d+)", lambda m: issue(int(m.group(1))), t)


# ---------------------------------------------------------------- the items
# ("step", log, name, title, note)      one logged command
# ("file", path, title, note, lines)    a stored file, first `lines` lines
#                                       (path under the run folder, or
#                                       "repo:..." under the repository)
# ("sh", script, prefix, title, note)   the one line of logs/<script> that
#                                       starts with prefix: a step the log
#                                       does not hold
# ("note", title, text)                 prose only
# ("verdict", hand key, note)           one row of the Verdicts table

def r_step(ctx, log, name, title, note):
    s = ctx["logs"][log][name]
    ok = s["exit"] == 0
    n = s["out"].count("\n") + 1 if s["out"] else 0
    out = ""
    if n:
        out = ('<details%s><summary>Output, %d line%s</summary>'
               '<div class="scroll"><pre>%s</pre></div></details>'
               % (" open" if n <= 14 else "", n, "" if n == 1 else "s",
                  E(s["out"])))
    return ('<article class="item step %s"><header><h4>%s</h4>'
            '<span class="badge %s">exit %d &middot; %d s</span></header>'
            '<div class="cmd"><div class="scroll"><pre>%s</pre></div>'
            '<button type="button" class="copy" data-copy="%s">Copy</button></div>'
            '%s%s<p class="src">logs/%s_log.txt, step %s</p></article>'
            % ("ok" if ok else "bad", prose(title), "ok" if ok else "bad",
               s["exit"], s["secs"], E(s["cmd"]), E(s["cmd"], quote=True),
               '<p>%s</p>' % prose(note) if note else "", out, E(log), E(name)))


def r_file(ctx, path, title, note, lines):
    if path.startswith("repo:"):
        full, shown = os.path.join(REPO, path[5:]), path[5:]
    else:
        full = os.path.join(ctx["run"], path)
        shown = ctx["run_name"] + "/" + path
    rows = read(full).rstrip("\n").split("\n")
    more = len(rows) - lines
    text = "\n".join(rows[:lines])
    tail = ('<p class="src">%s, first %d of %d lines</p>' % (E(shown), lines, len(rows))
            if more > 0 else '<p class="src">%s</p>' % E(shown))
    return ('<article class="item file"><header><h4>%s</h4>'
            '<span class="badge">file</span></header>%s'
            '<details open><summary>Contents</summary><div class="scroll">'
            '<pre>%s</pre></div></details>%s</article>'
            % (prose(title), '<p>%s</p>' % prose(note) if note else "",
               E(text), tail))


def r_sh(ctx, script, prefix, title, note):
    hits = [l for l in read(os.path.join(ctx["run"], "logs", script)).split("\n")
            if l.startswith(prefix)]
    assert len(hits) == 1, (script, prefix, hits)
    return ('<article class="item hand"><header><h4>%s</h4>'
            '<span class="badge">not in a log</span></header>'
            '<div class="cmd"><div class="scroll"><pre>%s</pre></div></div>'
            '<p>%s</p><p class="src">logs/%s</p></article>'
            % (prose(title), E(hits[0]), prose(note), E(script)))


def r_note(ctx, title, text):
    return ('<article class="item note"><header><h4>%s</h4></header>'
            '<p>%s</p></article>' % (prose(title), prose(text)))


def r_verdict(ctx, hand, note):
    rows = [v for v in ctx["verdicts"]
            if ("follow_%s__hold" % hand) in v["Result"]]
    assert len(rows) == 1, (hand, rows)
    v = rows[0]
    res = v["Result"].strip("`")
    assert os.path.isfile(os.path.join(ctx["run"], res)), res
    colour = v["Verdicts"].split()[0]
    assert colour in ("green", "amber", "red"), v["Verdicts"]
    keys = ["Scale margin", "Disturbance margin", "Joints posed past a limit",
            "Follow time (s)", "Units caveat in the result"]
    dl = "".join("<dt>%s</dt><dd>%s</dd>" % (E(k), E(v[k])) for k in keys)
    return ('<article class="item verdict %s"><header><h4>Verdict</h4>'
            '<span class="badge %s">%s</span></header><dl>%s</dl><p>%s</p>'
            '<p class="src">%s/tables.md, from %s</p></article>'
            % (colour, colour, E(v["Verdicts"]), dl, prose(note),
               E(ctx["run_name"]), E(res)))


RENDER = {"step": r_step, "file": r_file, "sh": r_sh, "note": r_note,
          "verdict": r_verdict}

FLIP = "hands/mosadegh2014_fPN_from_curves_x3_ring_flipped/"

WAYS = [
 {"key": "model", "tab": "A hand as a model file",
  "what": "The left Allegro hand from MuJoCo Menagerie, added with a short "
          "description file. This repeats `docs/BRING_YOUR_HAND.md`.",
  "ends": "Reaches a verdict, with both margins and a failures report.",
  "stages": [
   ("Hand", [
    ("file", "hand_descriptions/allegro_left.json", "The description file",
     "It names the hand, says which body is each fingertip, and gives the "
     "datasheet torque with its source.", 40),
    ("step", "run", "add_allegro", "Add the hand", ""),
    ("sh", "run.sh", "export GMW_ADDED_HANDS", "Where added hands were kept",
     "Set before the logged steps, to keep the run's added hands apart "
     "from the repository's."),
   ]),
   ("Task", [
    ("step", "run", "hold_allegro", "Make a hold from the hand's own kinematics",
     "This is the known answer for the check. We passed `--out-dir` to keep "
     "the run's files apart."),
    ("note", "Copy the task file",
     "`tools/run_follow.py --leader-id` reads only from `data/leaders/`, "
     "where the tool writes by default, so the task file was then copied "
     "there. That copy is not in a log."),
   ]),
   ("Verdict", [
    ("step", "run2", "follow_allegro_left", "Run the check, with margins", ""),
    ("step", "run2", "failures_allegro_left", "Report which fingers and joints fail", ""),
    ("verdict", "allegro_left",
     "The hand holding the hold made from its own kinematics. No hand made "
     "here was run on a task from elsewhere."),
   ]),
  ]},
 {"key": "curves", "tab": "An actuator as test curves",
  "what": "Digitised curves of one pneumatic bender "
          "(`data/test_curves/mosadegh2014_fPN.csv`), made into a record, "
          "then three stand-ins of it on a ring.",
  "ends": "Reaches a verdict, after two choices in the hand file that the "
          "docs do not give (issue 32). The result carries no units caveat "
          "(issue 35).",
  "stages": [
   ("Actuator in", [
    ("file", "repo:data/test_curves/mosadegh2014_fPN.csv", "The test curves",
     "Format: `docs/TEST_CURVE_FORMAT.md`.", 26),
    ("step", "run0", "curves_to_record", "Make the record", ""),
    ("step", "run0", "check_curves_record", "Check what the tool reads from it",
     "The record has motion but no force or stiffness data."),
   ]),
   ("Hand", [
    ("step", "run0", "build_curves_record_by_id", "Try 1: name the record by its id",
     "Fails: only records in `data/records/MANIFEST.json` are known by id. "
     "A file path works (issue 32)."),
    ("step", "run0", "build_curves_record_by_path", "Try 2: name the record by its path",
     "Fails: the test-curve format has no place for the actuator's size, so "
     "the hand file must give `contact_radius_m` (issue 32)."),
    ("note", "Try 3: give a contact radius",
     "With `contact_radius_m` set in the hand file the build passes its fit "
     "check. That first build was run by hand and is not in a log; its "
     "files are in `hands/mosadegh2014_fPN_from_curves_x3_ring/`. That "
     "hand file names the record as `rt_out/mosadegh_from_curves.json`, "
     "not the `rt_out/r0/` path the record was written to; the change of "
     "path is not in a log either. The "
     "radius is a choice made for this run, not data."),
    ("step", "run", "add_mosadegh", "Add the built hand",
     "A record-built hand that is not in `gramian_workshop/hands.py` has no "
     "other route into the check."),
    ("step", "run", "hold_mosadegh", "Make its hold",
     "Look at `facing`: with the ring layout copied from the stored example "
     "hand (`data/hands/wall_0.0010m_x3_ring/hand.json`) the fingers curl "
     "away from the ring's centre."),
    ("step", "run2", "follow_mosadegh_x3_ring", "Run the check: red", ""),
    ("step", "run2", "failures_mosadegh_x3_ring", "Report the failures", ""),
    ("verdict", "mosadegh_x3_ring",
     "Red in every frame: no finger reaches its contact. The record moves "
     "the other way from the example the layout was copied from."),
    ("file", FLIP + "hand.json", "Reverse each finger's bend axis",
     "The hand file that closes on the ball. `bend_axis` of each finger is "
     "reversed; the docs do not say which way it should point (issue 32).", 40),
    ("step", "run3", "build", "Build the hand", ""),
    ("step", "run3", "add", "Add it", ""),
   ]),
   ("Task", [
    ("step", "run3", "hold", "Make its hold", "Now `facing` is small: the fingers close on the ball."),
    ("sh", "run3.sh", "cp rt_out/leaders", "Copy the task file",
     "`tools/run_follow.py --leader-id` reads only from `data/leaders/`."),
   ]),
   ("Verdict", [
    ("step", "run3", "follow", "Run the check, with margins", ""),
    ("step", "run3", "failures", "Report which fingers and joints fail", ""),
    ("verdict", "mosadegh_x3_ring_flipped",
     "This green is not a strength result. The record has no force data, "
     "and the motor's force in the built model is the record's unitless "
     "drive level, not newtons; the registered record-built hand carries a "
     "PROVISIONAL caveat for exactly this, and a hand added with "
     "`tools/add_hand.py` does not (issue 35). The green also stands with "
     "the base hinge of each finger posed past its upper limit in every "
     "frame (issue 34). The contact radius, the palm and the ring are "
     "choices made for this run, not data."),
   ]),
  ]},
 {"key": "table", "tab": "An actuator as a joint table",
  "what": "One jointed finger written as a per-joint table "
          "(`data/joint_tables/shadow_hand_finger.csv`), made into a record.",
  "ends": "Stops at the hand: the hand build crashes on the record (issue 33).",
  "stages": [
   ("Actuator in", [
    ("file", "repo:data/joint_tables/shadow_hand_finger.csv", "The joint table",
     "Format: `docs/JOINT_TABLE_FORMAT.md`.", 17),
    ("step", "run0", "table_to_record", "Make the record", ""),
    ("step", "run0", "check_table_record", "Check what the tool reads from it", ""),
   ]),
   ("Hand", [
    ("step", "run0", "build_table_record_by_path", "Build a hand from the record",
     "Crashes in `gramian_workshop/record_fit.py`. We have not found the "
     "cause (issue 33). This way in goes no further."),
   ]),
  ]},
 {"key": "stored", "tab": "Stored records and tasks",
  "what": "Two side steps on what the repository already holds: reading "
          "stored task files, and drawing the stored records' Gramians. "
          "The mesh route to a record was not run; it is hours of SOFA "
          "compute in the M-11 project, and its records are in `data/records/`.",
  "ends": "Both task files read without complaint. The figure needs "
          "matplotlib, which no install extra provides (issue 36).",
  "stages": [
   ("Task files", [
    ("step", "run", "check_leader_sim", "Read one simulated task", ""),
    ("step", "run", "check_leader_human", "Read one human demonstration", ""),
   ]),
   ("Gramians of the stored records", [
    ("step", "run0", "gramians_mujoco_python", "In the hands environment", "Fails for want of matplotlib (issue 36)."),
    ("step", "run0", "gramians_analysis_python", "In the analysis environment", ""),
   ]),
  ]},
]

CSS = """
:root{--bg:#f6f5f1;--panel:#fffefb;--ink:#1d1f1c;--muted:#666a62;--line:#d9d6cc;
--code:#efede6;--accent:#2f5d50;--ok:#2c6e49;--okbg:#e2efe6;--bad:#a23b2a;
--badbg:#f6e3de;--amber:#8a6400;--amberbg:#f5ecd0}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;
--code:#262a26;--accent:#8fc7b3;--ok:#8ed0a4;--okbg:#1f3a2a;--bad:#ef9a8a;
--badbg:#44231d;--amber:#e3c16a;--amberbg:#3d3418}}
:root[data-theme="dark"]{color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;
--code:#262a26;--accent:#8fc7b3;--ok:#8ed0a4;--okbg:#1f3a2a;--bad:#ef9a8a;
--badbg:#44231d;--amber:#e3c16a;--amberbg:#3d3418}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
font:16px/1.55 "Source Sans 3",system-ui,sans-serif}
.wrap{max-width:900px;margin:0 auto;padding:32px max(16px,env(safe-area-inset-right,0px)) 64px max(16px,env(safe-area-inset-left,0px))}
h1{font:600 30px/1.2 "Source Serif 4",Georgia,serif;margin:0 0 8px;text-wrap:balance}
h2{font:600 22px/1.25 "Source Serif 4",Georgia,serif;margin:0 0 6px}
h3{font-size:13px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);
margin:32px 0 10px;display:flex;align-items:center;gap:10px}
h3 .n{display:inline-grid;place-items:center;width:24px;height:24px;border-radius:50%;
background:var(--accent);color:var(--bg);font-size:13px;letter-spacing:0}
h4{font-size:16px;margin:0;font-weight:600}
p{margin:8px 0}a{color:var(--accent)}
.lede{color:var(--muted);max-width:68ch}
.say{border-left:3px solid var(--accent);padding:2px 0 2px 14px;margin:18px 0;max-width:68ch}
code,pre{font-family:"JetBrains Mono",ui-monospace,Consolas,monospace;font-size:13px}
code{background:var(--code);padding:1px 5px;border-radius:4px;overflow-wrap:anywhere}
pre{margin:0;padding:10px 12px;white-space:pre}
.scroll{overflow-x:auto;background:var(--code);border-radius:6px;max-height:420px;overflow-y:auto}
.tabs{display:flex;flex-wrap:wrap;gap:8px;margin:26px 0 0}
.tabs button{font:inherit;font-weight:600;padding:9px 14px;border-radius:8px;cursor:pointer;
border:1px solid var(--line);background:var(--panel);color:var(--ink)}
.tabs button[aria-selected="true"]{background:var(--accent);border-color:var(--accent);color:var(--bg)}
.way{display:none;margin-top:22px}.way.on{display:block}
.ends{font-weight:600}
.item{background:var(--panel);border:1px solid var(--line);border-radius:10px;
padding:14px 16px;margin:10px 0}
.item header{display:flex;justify-content:space-between;align-items:baseline;gap:12px;
flex-wrap:wrap;margin-bottom:8px}
.item.bad{border-left:4px solid var(--bad)}.item.ok{border-left:4px solid var(--ok)}
.item.hand,.item.note{border-style:dashed}
.badge{font-size:12.5px;font-weight:600;padding:2px 9px;border-radius:999px;
background:var(--code);color:var(--muted);white-space:nowrap;font-variant-numeric:tabular-nums}
.badge.ok,.badge.green{background:var(--okbg);color:var(--ok)}
.badge.bad,.badge.red{background:var(--badbg);color:var(--bad)}
.badge.amber{background:var(--amberbg);color:var(--amber)}
.cmd{display:flex;gap:8px;align-items:flex-start}.cmd .scroll{flex:1;min-width:0}
.copy{font:inherit;font-size:13px;padding:7px 10px;border-radius:6px;cursor:pointer;
border:1px solid var(--line);background:var(--panel);color:var(--ink);flex:none}
details{margin-top:8px}summary{cursor:pointer;color:var(--muted);font-size:14px;margin-bottom:6px}
.src{font-size:12.5px;color:var(--muted);margin:8px 0 0;overflow-wrap:anywhere}
.verdict.green{background:var(--okbg)}.verdict.red{background:var(--badbg)}
.verdict.amber{background:var(--amberbg)}
.verdict .badge{font-size:15px;padding:4px 12px;background:var(--panel)}
dl{display:grid;grid-template-columns:minmax(120px,max-content) 1fr;gap:4px 16px;margin:8px 0}
dt{color:var(--muted)}dd{margin:0;overflow-wrap:anywhere;font-variant-numeric:tabular-nums}
@media (max-width:520px){dl{grid-template-columns:1fr}dd{margin-bottom:6px}
.cmd{flex-direction:column}.copy{align-self:flex-end}}
footer{margin-top:48px;padding-top:16px;border-top:1px solid var(--line);color:var(--muted);font-size:14px}
button:focus-visible,summary:focus-visible,a:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
"""

JS = """
(function(){
 var tabs=[].slice.call(document.querySelectorAll('.tabs button')),
     ways=[].slice.call(document.querySelectorAll('.way'));
 function show(k){tabs.forEach(function(b){b.setAttribute('aria-selected',b.dataset.way===k?'true':'false')});
  ways.forEach(function(w){w.classList.toggle('on',w.id==='way-'+k)});
  try{localStorage.setItem('walkthrough-way',k)}catch(e){}}
 tabs.forEach(function(b){b.addEventListener('click',function(){show(b.dataset.way)})});
 var k=null;try{k=localStorage.getItem('walkthrough-way')}catch(e){}
 if(!tabs.some(function(b){return b.dataset.way===k}))k=tabs[0].dataset.way;show(k);
 document.addEventListener('click',function(ev){var b=ev.target.closest('.copy');if(!b)return;
  var t=b.dataset.copy;function done(){b.textContent='Copied';setTimeout(function(){b.textContent='Copy'},1200)}
  function old(){var a=document.createElement('textarea');a.value=t;document.body.appendChild(a);a.select();
   try{document.execCommand('copy');done()}catch(e){}document.body.removeChild(a)}
  if(navigator.clipboard&&navigator.clipboard.writeText)navigator.clipboard.writeText(t).then(done,old);else old()});
})();
"""


def build(run):
    run = os.path.abspath(run)
    ctx = {"run": run, "run_name": os.path.relpath(run, REPO).replace(os.sep, "/"),
           "verdicts": parse_verdicts(os.path.join(run, "tables.md")), "logs": {}}
    for f in sorted(os.listdir(os.path.join(run, "logs"))):
        if f.endswith("_log.txt"):
            ctx["logs"][f[:-len("_log.txt")]] = parse_log(os.path.join(run, "logs", f))
    tabs, ways = [], []
    for w in WAYS:
        tabs.append('<button type="button" role="tab" data-way="%s">%s</button>'
                    % (w["key"], E(w["tab"])))
        body = ['<h2>%s</h2><p>%s</p><p class="ends">%s</p>'
                % (E(w["tab"]), prose(w["what"]), prose(w["ends"]))]
        for n, (stage, items) in enumerate(w["stages"], 1):
            body.append('<h3><span class="n">%d</span>%s</h3>' % (n, E(stage)))
            body.extend(RENDER[i[0]](ctx, *i[1:]) for i in items)
        ways.append('<section class="way" id="way-%s" role="tabpanel">%s</section>'
                    % (w["key"], "".join(body)))
    used = sum(1 for w in WAYS for _, items in w["stages"] for i in items if i[0] == "step")
    total = sum(len(v) for v in ctx["logs"].values())
    head = (
     '<title>Workshop walkthrough</title>'
     '<meta name="claude:viewport-fit-sides" content="cover">'
     '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono&amp;'
     'family=Source+Sans+3:wght@400;600&amp;family=Source+Serif+4:wght@600&amp;display=swap">'
     '<style>%s</style>' % CSS)
    intro = (
     '<h1>From an actuator or a hand to a verdict</h1>'
     '<p class="lede">A walk through one stored run of the whole chain: '
     'actuator in, record, hand, task, verdict. Pick a way in and read down.</p>'
     '<div class="say"><p><strong>This page runs nothing.</strong> Each command '
     'is shown as it ran, with its exit code, wall time and output read from '
     'the logs in <code>%s/logs/</code>. Verdicts are read from '
     '<code>tables.md</code> in that folder.</p>'
     '<p>In the commands, <code>&lt;hands env&gt;</code> is the Python '
     'environment with the hands extra (MuJoCo), <code>&lt;analysis env&gt;</code> '
     'is one that has matplotlib, <code>&lt;clone&gt;</code> is the fresh clone, '
     'and <code>rt_out/</code> is the scratch folder the run wrote to. Run from '
     'the clone\'s top folder.</p>'
     '<p>The run\'s files are stored under other names than the commands '
     'use: <code>rt_out/r0/*.json</code> records are in <code>records/</code>, '
     '<code>rt_out/*.json</code> description files in '
     '<code>hand_descriptions/</code>, <code>data/hands/&lt;name&gt;/</code> in '
     '<code>hands/&lt;name&gt;/</code>, and <code>rt_out/leaders</code> and '
     '<code>rt_out/results</code> in <code>leaders/</code> and '
     '<code>results/</code>.</p>'
     '<p>The laptop was shared with other jobs, so the '
     'times are a guide only. Simulated magnitudes are comparative, not '
     'predictive.</p></div>' % E(ctx["run_name"]))
    foot = ('<footer>Built by <code>tools/make_walkthrough.py</code> from '
            '<code>%s</code>. It shows %d of the %d logged steps. The written '
            'account of the same run is <code>docs/RUN_THROUGH.md</code>.</footer>'
            % (E(ctx["run_name"]), used, total))
    return (head + '<div class="wrap">' + intro
            + '<div class="tabs" role="tablist">' + "".join(tabs) + '</div>'
            + "".join(ways) + foot + '</div><script>' + JS + '</script>\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", default=os.path.join(REPO, "outputs", "run_through_11nov"),
                    help="the stored run folder (logs/, tables.md and the run's files)")
    ap.add_argument("--out", required=True, help="the page to write; never written over")
    a = ap.parse_args()
    page = build(a.run)
    with io.open(a.out, "x", encoding="utf-8", newline="\n") as f:
        f.write(page)
    print("wrote %s (%d bytes)" % (a.out, len(page.encode("utf-8"))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
