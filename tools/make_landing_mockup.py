"""A mock-up of a landing page for the workshop tool: scroll through the
hands on the left and the tasks on the right, choose one of each, and read
what the stored check says for the pair. Below that, the way to make a hand
of your own from characterised actuators, and the steps of the check.

    python tools/make_landing_mockup.py --out outputs/NEW_FOLDER/landing_mockup.html
    python tools/make_landing_mockup.py --hand shadow --task own_hold --out NEW_FILE.html

It is a mock-up, to judge the layout by eye. The page runs nothing: it
shows results that are already stored, and where there is none it shows the
command that would make one. Everything on it is read from the repository:

  hands     the HANDS table of gramian_workshop/hands.py (read as text, so
            MuJoCo is not needed)
  tasks     the files of data/leaders/ that no other file supersedes (each
            file's own `supersedes` field). Files whose ids share a start
            listed in TASK_NAMES go on one card, as its variants. A hand's
            own hold is one task, which stands for the chosen hand's hold
  results   the stored results that outputs/follow__superseded.md lists as
            not superseded (tools/list_superseded_follow.py writes that
            list), matched to a hand and a task by the result's own `hand`
            and `leader` fields. The counts are counts of frames[].verdict;
            the headroom is the smallest frames[].scale_margin.margin
  drawings  each task card's drawing is made here from the task file: the
            contact points round the object at one frame, and when each
            touches over the task's time
  pictures  the hand pictures and the picture of the actuator atlas are
            read from --pictures, a folder that
            tools/render_landing_pictures.py wrote (that tool needs MuJoCo;
            this one does not). Where a picture is missing the card says so
  renders   for a stored result of a hand on a task, the picture of the
            target beside the attempt and the link to the page that steps
            through every checked moment are read from --renders, a folder
            that tools/render_landing_tasks.py wrote (it needs MuJoCo too).
            Where there is none the answer says so

Hands and results whose key or name holds a word of LEFT_OUT are left off
the page. --hand and --task set the pair the page opens on (a task id, or
own_hold), so that a picture can be taken of any state. The page is a
fragment (no <html> or <body>), which is what the page host wants; a
browser opens it as it is. --out is never written over.

Standard library only.
"""
import argparse
import ast
import base64
import collections
import glob
import html
import io
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
E = html.escape
OWN = "own_hold"
LEFT_OUT = ("george",)
CURRENT = os.path.join("outputs", "follow__superseded.md")
PICTURES = os.path.join("outputs", "landing_pictures_12nov")
RENDERS = os.path.join("outputs", "landing_renders_12nov")

# where a hand's model comes from (the `base` field of the HANDS table)
SOURCES = {"menagerie": "fetched by the install step", "linker": "fetched by the install step",
           "local": "kept in the repository", "wave_manip": "needs a model from another project"}
# plain names for the task cards: (how the ids start, the name on the card)
TASK_NAMES = [
    ("w10_cup_physical_", "Cup wave"),
    ("w12c_cup_analytic_lock", "Cup wave, analytic lock"),
    ("tennis_new_cup_", "Tennis-ball demonstration"),
]
# the steps of the check, as the docstring of gramian_workshop/follow.py gives them
STEPS = [
    ("Place the hand", "A search over position and turn finds where to put the task's "
     "object and contact points relative to the hand, so that the fingers can best reach "
     "the points they serve. For a hand's own hold the place is already known."),
    ("Give each contact a finger", "Each finger serves at most one contact point. Every "
     "pairing is tried and the one that suits the hand best is kept. A point left without "
     "a finger must have its load carried by the others."),
    ("Reach and facing", "Each finger is posed to touch its point with the pad or a side "
     "of its last or middle segment, and the surface there must face the object. A finger "
     "that cannot get there is red, with the reason <code>no_contact</code>."),
    ("Friction and direction", "The force the task asks for at the point must lie inside "
     "the friction cone of the surface that touches. Red reasons: <code>direction</code>, "
     "<code>friction</code>."),
    ("Strength", "The joint torques that balance the contact force must be made by the "
     "finger's inputs, each within its own range. Red reason: <code>torque</code>."),
    ("The verdict for the frame", "Green: reaches, faces, and gives the task's own force. "
     "Amber: close, or a point has no finger, and the push on the object can still be made "
     "with different forces. Red: the first of these tests that failed. A frame takes its "
     "worst finger's verdict."),
    ("Headroom", "How many times larger the task's load could get and still be held. "
     "Stored only for runs made with <code>--margins</code>."),
]


def left_out(*texts):
    return any(w in (t or "").lower() for t in texts for w in LEFT_OUT)


def read_json(path):
    with io.open(path, encoding="utf-8") as f:
        return json.load(f)


def hands(repo):
    """The HANDS table, read from the source text: [{key, name, base, fingers}]."""
    with io.open(os.path.join(repo, "gramian_workshop", "hands.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "HANDS":
            table = ast.literal_eval(node.value)
            return [{"key": k, "name": v["name"], "base": v["base"], "fingers": len(v["fingers"])}
                    for k, v in table.items() if not left_out(k, v["name"])]
    raise SystemExit("no HANDS table in gramian_workshop/hands.py")


def object_text(leader):
    o = leader.get("object") or {}
    if o.get("shape") != "sphere":
        return o.get("shape", "object not given")
    return "ball of radius %.1f mm, %.0f g" % (o["radius_m"] * 1e3, o["mass_kg"] * 1e3)


def leaders(repo):
    out = {}
    for p in sorted(glob.glob(os.path.join(repo, "data", "leaders", "*.json"))):
        d = read_json(p)
        out[d["id"]] = d
    return out


def tasks(all_leaders, hand_keys):
    """(tasks, holds): the task files nothing supersedes. tasks is the list
    for the page; holds maps a hand to its own hold's {id, object}."""
    old = set(d.get("supersedes") for d in all_leaders.values())
    out, holds = [], {}
    for lid, d in all_leaders.items():
        if lid in old or left_out(lid, d.get("name")):
            continue
        m = re.match(r"^hold_(.+?)__self(__|$)", lid)
        if m:
            if m.group(1) in hand_keys:
                holds[m.group(1)] = {"id": lid, "object": object_text(d), "caveat": d.get("caveat")}
            continue
        out.append({"id": lid, "kind": d["kind"], "name": d["name"], "object": object_text(d),
                    "contacts": d.get("n_stations"), "frames": len(d["frames"]),
                    "seconds": round(d["frames"][-1]["t"] - d["frames"][0]["t"], 1),
                    "caveat": d.get("caveat")})
    return out, holds


def cards(ts):
    """The task cards: [{key, name, variants: [{id, label}]}]. Tasks whose
    ids start the same way (TASK_NAMES) share a card; a variant's label is
    the rest of its id, without the `__...fix` ending."""
    out = collections.OrderedDict()
    for t in ts:
        start, name = next(((s, n) for s, n in TASK_NAMES if t["id"].startswith(s)),
                           (t["id"], t["id"]))
        label = re.sub(r"__\w*fix$", "", t["id"][len(start):]).strip("_")
        out.setdefault(start, {"key": start, "name": name, "variants": []})
        out[start]["variants"].append({"id": t["id"], "label": label})
    return list(out.values())


def headroom(frames):
    """The smallest stored scale margin over the frames, or None if the
    result stores none."""
    ms = [f.get("scale_margin") for f in frames]
    if not ms or any(m is None for m in ms):
        return None
    return {"least": min(m["margin"] for m in ms), "capped": any(m["capped"] for m in ms)}


def results(repo, hand_keys, task_ids, holds):
    """{"hand|task": [result]} for the stored results the repository's own
    list counts as not superseded."""
    with io.open(os.path.join(repo, CURRENT), encoding="utf-8") as f:
        text = f.read()
    current = re.findall(r"`(outputs/follow_[^`]+\.json)`", text.split("\nNot superseded", 1)[1])
    out = {}
    for rel in current:
        d = read_json(os.path.join(repo, rel))
        hand, leader = d["hand"], d["leader"]
        if hand not in hand_keys:
            continue
        if holds.get(hand, {}).get("id") == leader:
            task = OWN
        elif leader in task_ids:
            task = leader
        else:
            continue
        frames = d["frames"]
        why = collections.Counter(f["reason"] for f in frames
                                  if f["verdict"] != "green" and f.get("reason"))
        failures = rel[:-5] + "__failures.html"
        out.setdefault("%s|%s" % (hand, task), []).append({
            "file": rel, "frames": [f["verdict"] for f in frames],
            "why": sorted(why.items()), "caveat": d.get("caveat"),
            "headroom": headroom(frames), "seconds": round(d["elapsed_s"]),
            "failures": failures if os.path.isfile(os.path.join(repo, failures)) else None})
    return out


def data(repo=REPO):
    hs = hands(repo)
    keys = set(h["key"] for h in hs)
    ts, holds = tasks(leaders(repo), keys)
    return {"hands": hs, "tasks": ts, "cards": cards(ts), "holds": holds, "own": OWN,
            "records": len(read_json(os.path.join(repo, "data", "records", "MANIFEST.json"))["records"]),
            "results": results(repo, keys, set(t["id"] for t in ts), holds)}


def drawing(leader):
    """A task file as a small drawing: above, the object and the contact
    points seen from above (x to the right, y up) at the first of the frames
    with the most points touching; below, one row per contact point, filled
    where it touches, over the task's time."""
    frames = leader["frames"]
    n = leader.get("n_stations") or len(frames[0]["contact"])
    k = max(range(len(frames)), key=lambda i: (sum(bool(c) for c in frames[i]["contact"]), -i))
    f = frames[k]
    r = (leader.get("object") or {}).get("radius_m") or 0.0
    c = f.get("obj_pos") or [0.0, 0.0, 0.0]
    pts = [(p, bool(on)) for p, on in zip(f["station_pos"], f["contact"]) if p]
    xs = [c[0] - r, c[0] + r] + [p[0] for p, _ in pts]
    ys = [c[1] - r, c[1] + r] + [p[1] for p, _ in pts]
    span = max(max(xs) - min(xs), max(ys) - min(ys)) or 1.0
    s = 84.0 / span
    mx, my = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2

    def at(p):
        return 80 + (p[0] - mx) * s, 50 - (p[1] - my) * s
    o = ['<svg viewBox="0 0 160 %d" role="img" aria-label="the contact points round the '
         'object at frame %d, and when each touches over the task">' % (106 + 5 * n, k)]
    if r:
        o.append('<circle class="ball" cx="%.1f" cy="%.1f" r="%.1f"/>' % (at(c) + (r * s,)))
    for p, on in pts:
        o.append('<circle class="%s" cx="%.1f" cy="%.1f" r="4"/>' % (("on" if on else "off",) + at(p)))
    t0, t1 = frames[0]["t"], frames[-1]["t"]
    for j in range(n):
        y = 104 + 5 * j
        o.append('<rect class="lane" x="8" y="%d" width="144" height="3"/>' % y)
        start = None
        for i in range(len(frames) + 1):
            on = i < len(frames) and bool(frames[i]["contact"][j])
            if on and start is None:
                start = i
            if not on and start is not None:
                a = frames[start]["t"]
                b = frames[min(i, len(frames) - 1)]["t"]
                x = 8 + 144 * (a - t0) / ((t1 - t0) or 1.0)
                w = max(144 * (b - a) / ((t1 - t0) or 1.0), 0.6)
                o.append('<rect class="on" x="%.1f" y="%d" width="%.1f" height="3"/>' % (x, y, w))
                start = None
    o.append("</svg>")
    return "".join(o)


def pictures(folder):
    """What tools/render_landing_pictures.py wrote in `folder`, as data
    URIs: ({hand: uri}, (uri, note) for the atlas or None)."""
    listed = os.path.join(folder, "pictures.json")
    if not os.path.isfile(listed):
        return {}, None

    def uri(name):
        with io.open(os.path.join(folder, name), "rb") as f:
            return "data:image/png;base64," + base64.b64encode(f.read()).decode("ascii")
    d = read_json(listed)
    atlas = d.get("atlas")
    return (dict((k, uri(v)) for k, v in d["hands"].items()),
            (uri(atlas["file"]), atlas.get("note", "")) if atlas else None)


CSS = """
/* layout: two panels side by side, hands and tasks, each scrolling on its
   own; the answer for the chosen pair below them; then the hand builder and
   the steps of the check. At phone width the panels stack and their cards
   run sideways. */
:root{--bg:#f6f5f1;--panel:#fffefb;--ink:#1d1f1c;--muted:#666a62;--line:#d9d6cc;--well:#eceae2;
--accent:#2f5d50;--pick:#e3efe9;--green:#3f8f5f;--amber:#c98a1b;--red:#b3402e}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;--well:#555b54;
--accent:#8fc7b3;--pick:#24352f;--green:#5fb583;--amber:#e0a63c;--red:#e5705c}}
:root[data-theme="dark"]{color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;--well:#555b54;
--accent:#8fc7b3;--pick:#24352f;--green:#5fb583;--amber:#e0a63c;--red:#e5705c}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
font:16px/1.5 "Source Sans 3",system-ui,sans-serif}
.wrap{max-width:1180px;margin:0 auto;padding-block:28px 56px;
padding-left:max(16px,env(safe-area-inset-left,0px));
padding-right:max(16px,env(safe-area-inset-right,0px));
display:flex;flex-direction:column;gap:22px}
h1{font:600 30px/1.2 "Source Serif 4",Georgia,serif;margin:0;text-wrap:balance}
h2{font:600 21px/1.25 "Source Serif 4",Georgia,serif;margin:0;text-wrap:balance}
h3{font-size:16px;font-weight:600;margin:0}
p{margin:0;max-width:70ch}
a{color:var(--accent)}
code{font:13px "JetBrains Mono",ui-monospace,Consolas,monospace;overflow-wrap:anywhere}
.top,.sec{display:flex;flex-direction:column;gap:10px;min-width:0}
.tag{align-self:flex-start;display:inline-block;font-size:12px;font-weight:600;letter-spacing:.06em;
text-transform:uppercase;padding:1px 8px;border-radius:3px;border:1px solid var(--line);
color:var(--muted);white-space:nowrap}
.tag.mock{color:var(--bg);background:var(--accent);border-color:var(--accent)}
.tag.works{color:var(--green);border-color:var(--green)}
.tag.prov,.tag.beta{color:var(--amber);border-color:var(--amber)}
.lede,.note,.small{color:var(--muted)}.small,.note{font-size:14px}
.cols{display:grid;grid-template-columns:1fr 1fr;gap:20px;align-items:start}
.panel{display:flex;flex-direction:column;gap:10px;min-width:0;border:1px solid var(--line);
border-radius:10px;padding:14px;background:var(--panel)}
.scroll{display:grid;gap:8px;max-height:min(66vh,620px);overflow-y:auto;padding:2px 4px 2px 2px;
align-content:start}
.hands .scroll{grid-template-columns:repeat(auto-fill,minmax(min(150px,100%),1fr))}
.card{display:flex;flex-direction:column;gap:6px;border:1px solid var(--line);border-radius:8px;
background:var(--bg);min-width:0;overflow-wrap:anywhere}
.card.is{border-color:var(--accent);background:var(--pick);box-shadow:inset 3px 0 0 var(--accent)}
.card.add{border-style:dashed;background:transparent;padding:10px 12px}
.pickme{display:flex;flex-direction:column;gap:2px;text-align:left;font:inherit;color:inherit;
background:none;border:0;padding:10px 12px;cursor:pointer;border-radius:8px;min-width:0;flex:1}
.tasks .pickme{flex-direction:row;gap:12px;align-items:center}
.tasks .pickme>span:last-child{display:flex;flex-direction:column;gap:2px;min-width:0}
.pickme:hover b{color:var(--accent)}
.pickme:focus-visible,.chip:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.pic{display:flex;align-items:center;justify-content:center;height:112px;border-radius:6px;
background:var(--well);margin-bottom:6px;color:var(--muted);font-size:13px}
.pic img{max-width:100%;max-height:100%;object-fit:contain;padding:6px}
.draw{flex:0 0 112px;width:112px;border-radius:6px;background:var(--panel);border:1px solid var(--line)}
.draw svg{display:block;width:100%}
.ball{fill:none;stroke:var(--muted);stroke-width:1.2}
.on{fill:var(--accent)}.off{fill:var(--panel);stroke:var(--muted);stroke-width:1.2}
.lane{fill:var(--line)}
.chips{display:flex;flex-wrap:wrap;gap:4px;padding:0 12px 10px}
.chip{font:12px "JetBrains Mono",ui-monospace,Consolas,monospace;color:inherit;background:var(--panel);
border:1px solid var(--line);border-radius:999px;padding:1px 8px;cursor:pointer}
.chip[aria-pressed="true"]{border-color:var(--accent);background:var(--accent);color:var(--bg)}
.chip.has{border-color:var(--accent)}
.has{color:var(--accent)}
.answer{border:1px solid var(--accent);border-radius:10px;background:var(--panel);
padding:16px 18px;display:flex;flex-direction:column;gap:10px;min-width:0;overflow-wrap:anywhere}
.pair{display:flex;flex-wrap:wrap;gap:6px 10px;align-items:baseline}
.pair span{color:var(--muted)}
.counts{display:flex;flex-wrap:wrap;gap:6px 22px;font-size:20px;font-variant-numeric:tabular-nums}
.counts i{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:7px}
.strip{display:flex;gap:2px;height:22px}
.strip i{flex:1;min-width:0;border-radius:2px}
.green{background:var(--green)}.amber{background:var(--amber)}.red{background:var(--red)}
.head{font-size:18px}
.move{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.4fr);gap:14px;align-items:start}
.render{display:flex;flex-direction:column;gap:6px;min-width:0}.render img{display:block;width:100%;border:1px solid var(--line);border-radius:8px}
.slot{display:flex;align-items:center;justify-content:center;text-align:center;min-height:120px;
border:1px dashed var(--line);border-radius:8px;color:var(--muted);font-size:14px;padding:12px}
.rows{display:flex;flex-direction:column;gap:8px;min-width:0}
.rows>div{display:flex;flex-direction:column;gap:2px;min-width:0}
.cmd{margin:0;padding:10px 12px;border:1px solid var(--line);border-radius:6px;background:var(--bg);
white-space:pre-wrap;overflow-wrap:anywhere;font:13px/1.5 "JetBrains Mono",ui-monospace,Consolas,monospace}
.three{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
.box{display:flex;flex-direction:column;gap:6px;border:1px solid var(--line);border-radius:8px;
background:var(--panel);padding:12px 14px;min-width:0;overflow-wrap:anywhere}
.say{border-left:3px solid var(--accent);padding:6px 0 6px 14px;display:flex;flex-direction:column;gap:6px}
figure{margin:0;display:flex;flex-direction:column;gap:6px}
figure img{display:block;width:100%;border:1px solid var(--line);border-radius:8px}
figcaption{font-size:14px;color:var(--muted)}
.steps{display:flex;gap:10px;overflow-x:auto;padding:2px 2px 10px;margin:0;list-style:none;
counter-reset:step}
.steps li{flex:0 0 250px;counter-increment:step}
.steps li.box::before{content:counter(step);font:600 22px "Source Serif 4",Georgia,serif;color:var(--accent)}
.steps li.notyet{border-style:dashed;background:transparent;counter-increment:none}
.steps li.notyet::before{content:none}
footer{padding-top:14px;border-top:1px solid var(--line);color:var(--muted);font-size:14px}
@media (max-width:760px){.cols,.three,.move{grid-template-columns:1fr}
.scroll,.hands .scroll{display:flex;overflow-x:auto;overflow-y:hidden;max-height:none;padding-bottom:8px}
.scroll>.card{flex:0 0 78%}.hands .scroll>.card{flex:0 0 58%}
.tasks .pickme{flex-direction:column;align-items:stretch}.draw{flex:none;width:140px}}
"""

# the page's own script: it only shows and hides what is in DATA
JS = r"""
(function(){
var RENDERS=JSON.parse(document.getElementById('renders').textContent);
var D=JSON.parse(document.getElementById('data').textContent),root=document.getElementById('chooser');
var pick={hand:root.dataset.hand||null,task:root.dataset.task||null};
function esc(s){return String(s).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function find(list,key,v){for(var i=0;i<list.length;i++)if(list[i][key]===v)return list[i];return null}
function count(a,v){return a.filter(function(x){return x===v}).length}
function cardOf(id){for(var i=0;i<D.cards.length;i++)if(find(D.cards[i].variants,'id',id))return D.cards[i];return null}
function has(h,id){return (D.results[h+'|'+id]||[]).length>0}
function commands(h,t){
  if(t!==D.own)return['python tools/run_follow.py --hands '+h.key+' --leader-id '+t+' --margins'];
  var id=(D.holds[h.key]||{}).id||('hold_'+h.key+'__self__selfoverlapfix');
  var c=[];if(!D.holds[h.key])c.push('python tools/make_hold_leader.py '+h.key);
  c.push('python tools/run_follow.py --hands '+h.key+' --leader-id '+id+' --placement-from outputs/hold_placement_'+h.key+'__selfoverlapfix.json --tag fixed --margins');
  return c}
function times(x){return x>=10?String(Math.round(x)):x.toFixed(2)}
function room(r){
  var m=r.headroom;
  if(!m)return '<p class="head"><b>Headroom:</b> none stored with this result.</p>';
  var o='<p class="head"><b>Headroom:</b> '+(m.least>0?'the task\'s load could be '+times(m.least)+' times larger'+(m.capped?' or more':'')+
    ' and still be held (the smallest over the frames).':'none. The check fails at the task\'s own load.')+'</p>';
  return o+'<p class="note">Read at the check\'s default tolerance, where headroom reads higher than the hand answers (<code>docs/MARGINS_WORKED_CHECK.md</code>).</p>'}
function move(r){
  var v=RENDERS[r.file],slot='<div class="slot">No stored render of this pair yet.</div>';
  if(v)slot='<div class="render">'+(v.still?'<a href="'+esc(v.view)+'"><img src="'+v.still+'" alt="The target beside the attempt, at one checked moment"></a>':'')+
    '<span class="small">The target beside the attempt'+(v.still?', at moment '+v.shown+' of '+v.moments:'')+'. <a href="'+esc(v.view)+'">Step through all '+v.moments+
    ' checked moments</a>, with the force at each contact and their sum on the object.</span></div>';
  return '<h3>See it move</h3><div class="move">'+slot+'<div class="rows">'+
   '<div><span><b>MuJoCo</b> <span class="tag works">works by command</span></span><span class="small">On your own machine: <code>python tools/render3d.py '+esc(r.file)+'</code></span></div>'+
   '<div><span><b>SOFA</b> <span class="tag">not built</span></span><span class="small">SOFA is how the mesh records are made, in the actuator-map project. It is hours of compute.</span></div>'+
   '<div><span><b>Another solver</b> <span class="tag">not built</span></span><span class="small">We do not drive it. Its output comes in as a record or a test-curve file.</span></div></div></div>'}
function one(r){
  var f=r.frames,o='<div class="counts">';
  ['green','amber','red'].forEach(function(v){o+='<span><i class="'+v+'"></i>'+count(f,v)+' '+v+'</span>'});
  o+='<span class="small">of '+f.length+' frames checked</span></div><div class="strip" role="img" aria-label="one cell per frame, in order">';
  f.forEach(function(v,k){o+='<i class="'+v+'" title="frame '+k+': '+v+'"></i>'});o+='</div>';
  if(r.why.length)o+='<p>Why not green: '+r.why.map(function(w){return esc(w[0])+' ('+w[1]+' frames)'}).join(', ')+'.</p>';
  o+=room(r);
  if(r.caveat)o+='<p class="note">Caution carried by the result: '+esc(r.caveat)+'</p>';
  o+='<p class="small">Stored result <code>'+esc(r.file)+'</code>; the run took '+r.seconds+' s.'+
     (r.failures?' Its failure report is <code>'+esc(r.failures)+'</code>.':'')+'</p>'+move(r);
  return o}
function answer(){
  var el=document.getElementById('answer'),h=find(D.hands,'key',pick.hand),
      t=pick.task===D.own?{id:D.own}:find(D.tasks,'id',pick.task);
  if(!h||!t){el.innerHTML='<h2>What the stored check says</h2><p class="lede">'+
    (h?'Now choose a task.':t?'Now choose a hand.':'Choose a hand and a task.')+'</p>';return}
  var own=t.id===D.own,hold=D.holds[h.key],c=own?null:cardOf(t.id),o='<h2>What the stored check says</h2><div class="pair"><b>'+esc(h.name)+
    '</b><span>on</span><b>'+(own?'its own hold'+(hold?' ('+esc(hold.object)+')':''):esc(c.name))+'</b>'+
    (own?'':'<code>'+esc(t.id)+'</code>')+'</div>';
  var rs=D.results[h.key+'|'+t.id]||[];
  if(rs.length){o+=rs.map(one).join('');
    var cav=own?(hold||{}).caveat:t.caveat;if(cav)o+='<p class="note">Caution carried by the task: '+esc(cav)+'</p>'}
  else{o+='<p>No stored result for this pair. This page runs nothing; the command'+(commands(h,t.id).length>1?'s':'')+
    ' that would make one:</p><pre class="cmd">'+commands(h,t.id).map(esc).join('\n')+'</pre>'+
    (own?'':'<p class="note">For a task that is not the hand\'s own hold, the run first searches for where to place the hand, which is a long run.</p>')}
  el.innerHTML=o}
function show(){
  Array.prototype.forEach.call(root.querySelectorAll('.card[data-kind]'),function(c){
    var k=c.dataset.kind,on=k==='hand'?pick.hand===c.dataset.key:
      (c.dataset.key===D.own?pick.task===D.own:!!pick.task&&(cardOf(pick.task)||{}).key===c.dataset.key);
    c.classList.toggle('is',on);c.querySelector('.pickme').setAttribute('aria-pressed',String(on))});
  Array.prototype.forEach.call(root.querySelectorAll('.chip'),function(b){
    b.setAttribute('aria-pressed',String(pick.task===b.dataset.task));
    b.classList.toggle('has',!!pick.hand&&has(pick.hand,b.dataset.task))});
  answer()}
root.addEventListener('click',function(e){
  var chip=e.target.closest('.chip');
  if(chip){pick.task=chip.dataset.task;show();return}
  var b=e.target.closest('.pickme');if(!b)return;
  var c=b.closest('.card'),k=c.dataset.kind,key=c.dataset.key;
  if(k==='hand')pick.hand=pick.hand===key?null:key;
  else if(key===D.own)pick.task=pick.task===D.own?null:D.own;
  else{var card=find(D.cards,'key',key),mine=pick.task&&find(card.variants,'id',pick.task);
    if(mine)pick.task=null;
    else{var v=card.variants.filter(function(x){return pick.hand&&has(pick.hand,x.id)})[0]||card.variants[0];pick.task=v.id}}
  show()});
show()})();
"""


def renders(folder, link):
    """What tools/render_landing_tasks.py wrote in `folder`, by result
    file: the still as a data URI (None if there is none), the link to the
    view (`link` is the way from the page to the folder), how many moments
    the view has and which one the still shows."""
    listed = os.path.join(folder, "renders.json")
    if not os.path.isfile(listed):
        return {}
    out = {}
    for v in read_json(listed)["views"]:
        still = None
        if v.get("still"):
            with io.open(os.path.join(folder, v["still"]), "rb") as f:
                still = "data:image/png;base64," + base64.b64encode(f.read()).decode("ascii")
        out[v["result"]] = {"still": still, "view": link + v["view"], "moments": v["moments"],
                            "shown": v["shown"]}
    return out


def stored(d, keys, part):
    """The card line that counts the stored results for a hand (part 0 of
    the pair) or for a card's tasks (part 1)."""
    n = sum(len(v) for k, v in d["results"].items() if k.split("|")[part] in keys)
    return ('<span class="small has">%d stored result%s</span>' % (n, "" if n == 1 else "s")) if n         else '<span class="small">no stored result</span>'


def hand_card(d, h, pics):
    pic = ('<span class="pic"><img alt="%s, drawn from its model" src="%s"></span>'
           % (E(h["name"]), pics[h["key"]])) if h["key"] in pics         else '<span class="pic">no picture yet</span>'
    return ('<div class="card" data-kind="hand" data-key="%s"><button type="button" class="pickme" '
            'aria-pressed="false">%s<b>%s</b><span class="small">%d fingers, %s</span>'
            '<span class="small"><code>%s</code></span>%s</button></div>'
            % (E(h["key"]), pic, E(h["name"]), h["fingers"], E(SOURCES.get(h["base"], h["base"])),
               E(h["key"]), stored(d, (h["key"],), 0)))


def task_card(key, name, draw, lines, chips=""):
    return ('<div class="card" data-kind="task" data-key="%s"><button type="button" class="pickme" '
            'aria-pressed="false"><span class="draw">%s</span><span><b>%s</b>%s</span></button>%s</div>'
            % (E(key), draw, E(name), "".join(lines), chips))


def build(repo=REPO, hand=None, task=None, pictures_folder=None, renders_folder=None, renders_link=None):
    d = data(repo)
    renders_folder = renders_folder or os.path.join(repo, RENDERS)
    rendered = renders(renders_folder, renders_link if renders_link is not None
                    else "../%s/" % os.path.basename(os.path.normpath(renders_folder)))
    if hand and hand not in [h["key"] for h in d["hands"]]:
        raise SystemExit("no hand %r on the page" % hand)
    if task and task != OWN and task not in [t["id"] for t in d["tasks"]]:
        raise SystemExit("no task %r on the page" % task)
    pics, atlas = pictures(pictures_folder or os.path.join(repo, PICTURES))
    all_leaders = leaders(repo)
    by_id = dict((t["id"], t) for t in d["tasks"])

    left = [hand_card(d, h, pics) for h in d["hands"]]
    left.append(
        '<div class="card add" id="add-hand"><b>+ Add a hand</b>'
        '<span class="small">Load a model file (MuJoCo XML or URDF). '
        '<span class="tag works">works by command</span> See <code>docs/BRING_YOUR_HAND.md</code></span>'
        '<span class="small">Or <a href="#builder">make one in the hand builder</a>, from '
        'characterised actuators.</span></div>')

    shown = next((h for h in d["hands"] if h["key"] in d["holds"]), None)
    right = [task_card(
        OWN, "The hand's own hold",
        drawing(all_leaders[d["holds"][shown["key"]]["id"]]) if shown else "",
        ['<span class="small">A static grip: the fingers curl and a ball is fitted to rest against '
         'the last link of each. The hold is built so that the hand should pass, so a failure '
         'points at the hand\'s model.</span>',
         '<span class="small">%d of the %d hands here have one stored%s</span>'
         % (len(d["holds"]), len(d["hands"]),
            "; drawn: %s" % E(shown["name"]) if shown else "")])]
    for c in d["cards"]:
        ts = [by_id[v["id"]] for v in c["variants"]]
        t = ts[0]
        same = lambda k: len(set(str(x[k]) for x in ts)) == 1   # noqa: E731
        lines = ['<span class="small">%s</span>' % E(t["kind"]),
                 '<span class="small">%s%s%s</span>' % (
                     E(t["object"]) if same("object") else "the object differs by variant",
                     ", %s contact points" % t["contacts"] if same("contacts") else "",
                     ", %s s" % t["seconds"] if same("seconds") else "")]
        if len(ts) == 1:
            lines.append('<span class="small"><code>%s</code></span>' % E(t["id"]))
        else:
            lines.append('<span class="small">%d variants; drawn: <code>%s</code></span>'
                         % (len(ts), E(c["variants"][0]["label"])))
        lines.append(stored(d, set(x["id"] for x in ts), 1))
        chips = ('<div class="chips">%s</div>' % "".join(
            '<button type="button" class="chip" data-task="%s" aria-pressed="false">%s</button>'
            % (E(v["id"]), E(v["label"])) for v in c["variants"])) if len(ts) > 1 else ""
        right.append(task_card(c["key"], c["name"], drawing(all_leaders[t["id"]]), lines, chips))
    right.append(
        '<div class="card add" id="add-task"><b>+ Add a task</b>'
        '<span class="small">Bring a task as a file: contact points and forces over time. '
        '<span class="tag works">works by command</span> See <code>docs/TASK_FILE_FORMAT.md</code></span>'
        '<span class="small">From a human demonstration. <span class="tag beta">beta</span> Today a '
        'demonstration comes in from three-camera keypoints (<code>tools/import_demo_leader.py</code>). '
        'From a single video: <span class="tag">not built</span></span></div>')

    figure = ('<figure><img alt="the actuator atlas: one point per characterised actuator, '
              'placed by how alike their capabilities are" src="%s"><figcaption>The actuator atlas '
              'of the actuator-map project: one point per characterised actuator. A picture of %s. '
              'It is not live on this page.</figcaption></figure>' % (atlas[0], E(atlas[1]))) if atlas         else '<div class="slot">No picture of the actuator atlas yet.</div>'
    builder = (
        '<section class="sec" id="builder"><h2>Make your manipulator</h2>'
        '<p class="lede">Put characterised actuators onto a hand arrangement: start from an empty '
        'palm, or take a hand above and swap its fingers. Then check it against the tasks like '
        'any other hand.</p>'
        '<div class="say"><p><b>Each finger is modelled on its own, from its record.</b> Coupling '
        'between fingers through the palm or a shared supply is not modelled.</p>'
        '<p class="note">A soft finger enters as a stand-in fitted to its record: hinges, springs '
        'and one drive. It is a first approximation, as good as the record.</p></div>'
        '<div class="three">'
        '<div class="box"><h3>Arrange the palm</h3><p class="small">A hand file says where each '
        'finger mounts and which way it bends (<code>examples/test_curve_hand/hand.json</code>). '
        '<code>tools/build_record_hand.py</code> builds the hand from it.</p>'
        '<span class="tag works">works by command</span>'
        '<p class="small">Arranging it on this page, and swapping one finger of a hand above:</p>'
        '<span class="tag">not built</span></div>'
        '<div class="box"><h3>Choose each finger\'s actuator</h3><p class="small">There are %d '
        'records in <code>data/records/</code>. The hand file names one per finger.</p>'
        '<span class="tag works">works by command</span>'
        '<p class="small">Choosing it from the actuator map, by what the finger has to do:</p>'
        '<span class="tag">not built</span></div>'
        '<div class="box"><h3>Add your own actuator</h3><p class="small">A new or unusual actuator '
        'comes in as a record: from test curves (<code>tools/curves_to_record.py</code>), a table '
        'of joints (<code>tools/joint_table_to_record.py</code>) or the form '
        '(<code>forms/record_form.html</code>).</p><span class="tag works">works by command</span>'
        '<p class="small">Uploading it so that it takes its place on the actuator map:</p>'
        '<span class="tag">not built</span></div></div>%s'
        '<p class="note"><span class="tag prov">provisional</span> Strength verdicts and headroom for '
        'a hand built from records wait on a ruling about force units: the motor works in the '
        'record\'s drive level, not newtons.</p></section>' % (d["records"], figure))
    how = (
        '<section class="sec" id="how"><h2>How this works under the hood</h2>'
        '<p class="lede">What decides whether a hand can do a task, in the order the check takes '
        'it. Scroll sideways.</p><ol class="steps">%s'
        '<li class="box notyet"><h3>Not checked yet</h3><p class="small">Each frame is checked on '
        'its own. Whether the poses join up from one frame to the next, and whether the hand is '
        'fast enough and can carry its own inertia, are not checked yet.</p></li></ol></section>'
        % "".join('<li class="box"><h3>%s</h3><p class="small">%s</p></li>' % (E(t), p)
                  for t, p in STEPS))
    n_res = sum(len(v) for v in d["results"].values())
    head = (
     '<title>Hand and task chooser</title>'
     '<meta name="claude:viewport-fit-sides" content="cover">'
     '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono&amp;'
     'family=Source+Sans+3:wght@400;600&amp;family=Source+Serif+4:wght@600&amp;display=swap">'
     '<style>%s</style>' % CSS)
    top = ('<div class="top"><span class="tag mock">Mock-up</span>'
           '<h1>Can this hand do this task?</h1>'
           '<p class="lede">Choose a hand and a task. The page shows what the stored check '
           'says for the pair: a moving task, checked frame by frame, and how much headroom is '
           'left. It runs nothing itself. Simulated results are comparative, not predictive.</p>'
           '</div>')
    foot = ('<footer><p>A mock-up made by <code>tools/make_landing_mockup.py</code> from the '
            'hands table, the task files of <code>data/leaders/</code>, the %d stored results for '
            'these hands that <code>%s</code> lists as not superseded, and the pictures that '
            '<code>tools/render_landing_pictures.py</code> wrote. Each task drawing shows the '
            'contact points round the object, seen from above, at one frame (filled: touching), '
            'and below it one row per contact point, filled while it touches.</p></footer>'
            % (n_res, E(CURRENT.replace(os.sep, "/"))))
    return ('%s<div class="wrap" id="chooser" data-hand="%s" data-task="%s">%s'
            '<div class="cols"><section class="panel hands"><h2>Hands</h2>'
            '<p class="small">%d hands. Scroll, and choose one.</p><div class="scroll">%s</div></section>'
            '<section class="panel tasks"><h2>Tasks</h2>'
            '<p class="small">%d tasks. Scroll, and choose one.</p><div class="scroll">%s</div></section></div>'
            '<section class="answer" id="answer" aria-live="polite"></section>%s%s%s</div>'
            '<script type="application/json" id="data">%s</script>'
            '<script type="application/json" id="renders">%s</script><script>%s</script>\n'
            % (head, E(hand or ""), E(task or ""), top, len(d["hands"]), "".join(left),
               len(d["cards"]) + 1, "".join(right), builder, how, foot,
               json.dumps(d, sort_keys=True).replace("</", "<\\/"),
               json.dumps(rendered, sort_keys=True).replace("</", "<\\/"), JS))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--hand", help="the hand the page opens on (a key of the hands table)")
    ap.add_argument("--task", help="the task the page opens on (a task id, or %s)" % OWN)
    ap.add_argument("--pictures", help="the folder tools/render_landing_pictures.py wrote "
                    "(default: %s)" % PICTURES.replace(os.sep, "/"))
    ap.add_argument("--renders", help="the folder tools/render_landing_tasks.py wrote "
                    "(default: %s)" % RENDERS.replace(os.sep, "/"))
    ap.add_argument("--renders-link", help="the way from the page to that folder, as the page's links "
                    "should say it (default: ../<the folder's name>/, which is right for a page "
                    "written to a folder beside it)")
    ap.add_argument("--out", required=True, help="the page to write (never written over)")
    a = ap.parse_args(argv)
    page = build(REPO, a.hand, a.task, a.pictures, a.renders, a.renders_link)
    with io.open(a.out, "x", encoding="utf-8", newline="\n") as f:
        f.write(page)
    print("wrote", a.out, "(%d bytes)" % len(page.encode("utf-8")))


if __name__ == "__main__":
    main()
