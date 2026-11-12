"""A mock-up of a landing page for the workshop tool: choose a hand on the
left, choose a task on the right, and read what the stored check says for
the pair.

    python tools/make_landing_mockup.py --out outputs/landing_mockup_12nov/landing_mockup.html
    python tools/make_landing_mockup.py --hand shadow --task own_hold --out NEW_FILE.html

It is a mock-up, to judge the layout by eye. The page runs nothing: it
shows results that are already stored, and where there is none it shows the
command that would make one. Everything on it is read from the repository:

  hands     the HANDS table of gramian_workshop/hands.py (read as text, so
            MuJoCo is not needed), grouped by where each model comes from
  tasks     the files of data/leaders/ that no other file supersedes (each
            file's own `supersedes` field). A hand's own hold is one task,
            which stands for the chosen hand's hold file
  results   the stored results that outputs/follow__superseded.md lists as
            not superseded (tools/list_superseded_follow.py writes that
            list), matched to a hand and a task by the result's own `hand`
            and `leader` fields. The counts are counts of frames[].verdict

Hands and results whose key or name holds a word of LEFT_OUT are left off
the page. --hand and --task set the pair the page opens on (a task id, or
own_hold), so that a picture can be taken of any state. The page is a
fragment (no <html> or <body>), which is what the page host wants; a
browser opens it as it is. --out is never written over.

Standard library only.
"""
import argparse
import ast
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

# where a hand's model comes from (the `base` field of the HANDS table)
GROUPS = [
    ("fetched", ("menagerie", "linker"), "Fetched by the install step",
     "<code>tools/fetch_hands.py</code> puts these models in <code>external/</code>."),
    ("local", ("local",), "Kept in the repository",
     "Models under <code>data/hands/</code>."),
    ("elsewhere", ("wave_manip",), "Needs a model from another project",
     "<code>tools/fetch_hands.py</code> fetches this one only with "
     "<code>--wave-manip</code> and a checkout of that project."),
]
# the cards that are not a choice: ways to bring something of your own
BRING_HAND = [
    ("Bring a hand as a model file", "A MuJoCo XML or URDF file.",
     "docs/BRING_YOUR_HAND.md"),
    ("Build a hand from your actuator", "From test curves or datasheet numbers, "
     "through a record, to a hand.", "docs/PARTICIPANT_SCRIPT.md"),
]
BRING_TASK = [
    ("Bring a task as a file", "Contact points and forces over time, in the "
     "task file format.", "docs/TASK_FILE_FORMAT.md"),
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


def tasks(repo, hand_keys):
    """(tasks, holds): the task files nothing supersedes. tasks is the list
    for the page; holds maps a hand to its own hold's {id, object}."""
    leaders = {}
    for p in sorted(glob.glob(os.path.join(repo, "data", "leaders", "*.json"))):
        d = read_json(p)
        leaders[d["id"]] = d
    old = set(d.get("supersedes") for d in leaders.values())
    out, holds = [], {}
    for lid, d in leaders.items():
        if lid in old or left_out(lid, d.get("name")):
            continue
        m = re.match(r"^hold_(.+?)__self(__|$)", lid)
        if m:
            if m.group(1) in hand_keys:
                holds[m.group(1)] = {"id": lid, "object": object_text(d), "caveat": d.get("caveat")}
            continue
        out.append({"id": lid, "kind": d["kind"], "name": d["name"], "object": object_text(d),
                    "contacts": d.get("n_stations"), "frames": len(d["frames"]),
                    "caveat": d.get("caveat")})
    return out, holds


def results(repo, hand_keys, task_ids, holds):
    """{"hand|task": result} for the stored results the repository's own
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
            "seconds": round(d["elapsed_s"]),
            "failures": failures if os.path.isfile(os.path.join(repo, failures)) else None})
    return out


def data(repo=REPO):
    hs = hands(repo)
    keys = set(h["key"] for h in hs)
    ts, holds = tasks(repo, keys)
    return {"hands": hs, "tasks": ts, "holds": holds, "own": OWN,
            "results": results(repo, keys, set(t["id"] for t in ts), holds)}


CSS = """
/* layout: hands on the left, tasks on the right, the answer for the chosen
   pair in a band that stays at the foot of the window; one column at phone
   width */
:root{--bg:#f6f5f1;--panel:#fffefb;--ink:#1d1f1c;--muted:#666a62;--line:#d9d6cc;
--accent:#2f5d50;--pick:#e3efe9;--green:#3f8f5f;--amber:#c98a1b;--red:#b3402e;--none:#c9c6bb}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;
--accent:#8fc7b3;--pick:#24352f;--green:#5fb583;--amber:#e0a63c;--red:#e5705c;--none:#4a4f48}}
:root[data-theme="dark"]{color-scheme:dark;
--bg:#161816;--panel:#1e211e;--ink:#e6e6df;--muted:#9a9e94;--line:#353a34;
--accent:#8fc7b3;--pick:#24352f;--green:#5fb583;--amber:#e0a63c;--red:#e5705c;--none:#4a4f48}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
font:16px/1.5 "Source Sans 3",system-ui,sans-serif}
.wrap{max-width:1120px;margin:0 auto;padding-block:28px 56px;
padding-left:max(16px,env(safe-area-inset-left,0px));
padding-right:max(16px,env(safe-area-inset-right,0px));
display:flex;flex-direction:column;gap:20px}
h1{font:600 30px/1.2 "Source Serif 4",Georgia,serif;margin:0;text-wrap:balance}
h2{font:600 21px/1.25 "Source Serif 4",Georgia,serif;margin:0}
h3{font-size:12.5px;font-weight:600;letter-spacing:.06em;text-transform:uppercase;
color:var(--muted);margin:0}
p{margin:0;max-width:70ch}
code{font:13px "JetBrains Mono",ui-monospace,Consolas,monospace;overflow-wrap:anywhere}
.top{display:flex;flex-direction:column;gap:8px}
.mock{align-self:flex-start;font-size:12.5px;font-weight:600;letter-spacing:.06em;
text-transform:uppercase;color:var(--bg);background:var(--accent);padding:2px 9px;border-radius:3px}
.lede,.note,.small{color:var(--muted)}.small,.note{font-size:14px}
.cols{display:grid;grid-template-columns:1fr 1fr;gap:28px;align-items:start}
.col,.group{display:flex;flex-direction:column;gap:10px;min-width:0}
.col{gap:18px}
.cards{display:grid;grid-template-columns:repeat(auto-fill,minmax(min(210px,100%),1fr));gap:8px}
.card,.answer{overflow-wrap:anywhere}
.tasks .cards{grid-template-columns:1fr}
.card{display:flex;flex-direction:column;gap:2px;text-align:left;font:inherit;color:inherit;
background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px 12px;min-width:0}
button.card{cursor:pointer}
button.card:hover{border-color:var(--accent)}
button.card[aria-pressed="true"]{border-color:var(--accent);background:var(--pick);
box-shadow:inset 3px 0 0 var(--accent)}
button.card:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.card b{font-weight:600}
.card.bring{border-style:dashed;background:transparent}
.answer{border:1px solid var(--accent);border-radius:8px;background:var(--panel);
padding:14px 18px;display:flex;flex-direction:column;gap:9px;min-width:0;
position:sticky;bottom:12px;max-height:46vh;overflow-y:auto;box-shadow:0 6px 24px rgba(0,0,0,.18)}
.has{color:var(--accent)}
.pair{display:flex;flex-wrap:wrap;gap:6px 10px;align-items:baseline}
.pair span{color:var(--muted)}
.counts{display:flex;flex-wrap:wrap;gap:6px 22px;font-size:20px;font-variant-numeric:tabular-nums}
.counts i{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:7px}
.strip{display:flex;gap:2px;height:22px}
.strip i{flex:1;min-width:0;border-radius:2px}
.green{background:var(--green)}.amber{background:var(--amber)}.red{background:var(--red)}
.cmd{margin:0;padding:10px 12px;border:1px solid var(--line);border-radius:6px;background:var(--bg);
white-space:pre-wrap;overflow-wrap:anywhere;font:13px/1.5 "JetBrains Mono",ui-monospace,Consolas,monospace}
footer{padding-top:14px;border-top:1px solid var(--line);color:var(--muted);font-size:14px}
@media (max-width:760px){.cols{grid-template-columns:1fr}}
"""

# the page's own script: it only shows and hides what is in DATA
JS = r"""
(function(){
var D=JSON.parse(document.getElementById('data').textContent),root=document.getElementById('chooser');
var pick={hand:root.dataset.hand||null,task:root.dataset.task||null};
function esc(s){return String(s).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function find(list,key,v){for(var i=0;i<list.length;i++)if(list[i][key]===v)return list[i];return null}
function count(a,v){return a.filter(function(x){return x===v}).length}
function commands(h,t){
  if(t!==D.own)return['python tools/run_follow.py --hands '+h.key+' --leader-id '+t+' --margins'];
  var id=(D.holds[h.key]||{}).id||('hold_'+h.key+'__self__selfoverlapfix');
  var c=[];if(!D.holds[h.key])c.push('python tools/make_hold_leader.py '+h.key);
  c.push('python tools/run_follow.py --hands '+h.key+' --leader-id '+id+' --placement-from outputs/hold_placement_'+h.key+'__selfoverlapfix.json --tag fixed --margins');
  return c}
function one(r){
  var f=r.frames,o='<div class="counts">';
  ['green','amber','red'].forEach(function(v){o+='<span><i class="'+v+'"></i>'+count(f,v)+' '+v+'</span>'});
  o+='<span class="small">of '+f.length+' frames checked</span></div><div class="strip" role="img" aria-label="one cell per frame, in order">';
  f.forEach(function(v,k){o+='<i class="'+v+'" title="frame '+k+': '+v+'"></i>'});o+='</div>';
  if(r.why.length)o+='<p>Why not green: '+r.why.map(function(w){return esc(w[0])+' ('+w[1]+' frames)'}).join(', ')+'.</p>';
  if(r.caveat)o+='<p class="note">Caution carried by the result: '+esc(r.caveat)+'</p>';
  o+='<p class="small">Stored result <code>'+esc(r.file)+'</code>; the run took '+r.seconds+' s.'+
     (r.failures?' Its failure report is <code>'+esc(r.failures)+'</code>.':'')+'</p>';
  return o}
function answer(){
  var el=document.getElementById('answer'),h=find(D.hands,'key',pick.hand),
      t=pick.task===D.own?{id:D.own}:find(D.tasks,'id',pick.task);
  if(!h||!t){el.innerHTML='<h2>What the stored check says</h2><p class="lede">'+
    (h?'Now choose a task.':t?'Now choose a hand.':'Choose a hand and a task.')+'</p>';return}
  var own=t.id===D.own,hold=D.holds[h.key],o='<h2>What the stored check says</h2><div class="pair"><b>'+esc(h.name)+
    '</b><span>on</span><b>'+(own?'its own hold'+(hold?' ('+esc(hold.object)+')':''):'<code>'+esc(t.id)+'</code>')+'</b></div>';
  var rs=D.results[h.key+'|'+t.id]||[];
  if(rs.length){o+=rs.map(one).join('');
    var cav=own?(hold||{}).caveat:t.caveat;if(cav)o+='<p class="note">Caution carried by the task: '+esc(cav)+'</p>'}
  else{o+='<p>No stored result for this pair. This page runs nothing; the command'+(commands(h,t.id).length>1?'s':'')+
    ' that would make one:</p><pre class="cmd">'+commands(h,t.id).map(esc).join('\n')+'</pre>'+
    (own?'':'<p class="note">For a task that is not the hand\'s own hold, the run first searches for where to place the hand, which is a long run.</p>')}
  el.innerHTML=o}
function show(){
  Array.prototype.forEach.call(root.querySelectorAll('button.card'),function(b){
    b.setAttribute('aria-pressed',String(pick[b.dataset.kind]===b.dataset.key))});
  answer()}
root.addEventListener('click',function(e){
  var b=e.target.closest('button.card');if(!b)return;
  pick[b.dataset.kind]=pick[b.dataset.kind]===b.dataset.key?null:b.dataset.key;show()});
show()})();
"""


def card(kind, key, title, lines):
    return ('<button type="button" class="card" data-kind="%s" data-key="%s" aria-pressed="false">'
            '<b>%s</b>%s</button>'
            % (kind, E(key), E(title),
               "".join('<span class="small">%s</span>' % l for l in lines if l)))


def stored(d, key, part):
    """The card line that counts the stored results for a hand (part 0 of
    the pair) or a task (part 1)."""
    n = sum(len(v) for k, v in d["results"].items() if k.split("|")[part] == key)
    return ('<span class="has">%d stored result%s</span>' % (n, "" if n == 1 else "s")) if n else         "no stored result"


def bring(title, text, doc):
    return ('<div class="card bring"><b>%s</b><span class="small">%s</span>'
            '<span class="small">See <code>%s</code></span></div>' % (E(title), E(text), E(doc)))


def group(title, note, cards):
    return ('<div class="group"><h3>%s</h3>%s<div class="cards">%s</div></div>'
            % (E(title), '<p class="note">%s</p>' % note if note else "", "".join(cards)))


def build(repo=REPO, hand=None, task=None):
    d = data(repo)
    if hand and hand not in [h["key"] for h in d["hands"]]:
        raise SystemExit("no hand %r on the page" % hand)
    if task and task != OWN and task not in [t["id"] for t in d["tasks"]]:
        raise SystemExit("no task %r on the page" % task)
    left = []
    for _, bases, title, note in GROUPS:
        hs = [h for h in d["hands"] if h["base"] in bases]
        if hs:
            left.append(group(title, note, [
                card("hand", h["key"], h["name"],
                     ["%d fingers" % h["fingers"], "<code>%s</code>" % E(h["key"]),
                      stored(d, h["key"], 0)]) for h in hs]))
    left.append(group("Your own", "", [bring(*b) for b in BRING_HAND]))
    right = [group("A hold the hand can do by construction",
                   "The fingers close on a ball until every fingertip touches. The answer "
                   "is known before the check runs, so it tests the hand's model, not the hand.",
                   [card("task", OWN, "The hand's own hold",
                         ["%d of the %d hands here have one stored"
                          % (len(d["holds"]), len(d["hands"]))])])]
    kinds = collections.OrderedDict()
    for t in d["tasks"]:
        kinds.setdefault(t["kind"], []).append(t)
    for kind, ts in kinds.items():
        right.append(group(kind, "", [
            card("task", t["id"], t["id"],
                 ["%s, %s contacts, %d frames" % (E(t["object"]), t["contacts"], t["frames"]),
                  E(t["name"]), stored(d, t["id"], 1)]) for t in ts]))
    right.append(group("Your own", "", [bring(*b) for b in BRING_TASK]))
    n_res = sum(len(v) for v in d["results"].values())
    head = (
     '<title>Hand and task chooser</title>'
     '<meta name="claude:viewport-fit-sides" content="cover">'
     '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono&amp;'
     'family=Source+Sans+3:wght@400;600&amp;family=Source+Serif+4:wght@600&amp;display=swap">'
     '<style>%s</style>' % CSS)
    top = ('<div class="top"><span class="mock">Mock-up</span>'
           '<h1>Can this hand do this task?</h1>'
           '<p class="lede">Choose a hand and a task. The page shows what the stored check '
           'says for the pair, frame by frame, or the command that would run it. It runs '
           'nothing itself. Simulated results are comparative, not predictive.</p></div>')
    foot = ('<footer><p>A mock-up made by <code>tools/make_landing_mockup.py</code> from the '
            'hands table, the task files of <code>data/leaders/</code> and the %d stored '
            'results for these hands that <code>%s</code> lists as not superseded. Task names '
            'are the ids and names the task files carry.</p></footer>'
            % (n_res, E(CURRENT.replace(os.sep, "/"))))
    return ('%s<div class="wrap" id="chooser" data-hand="%s" data-task="%s">%s'
            '<div class="cols"><section class="col hands"><h2>Choose a hand</h2>%s</section>'
            '<section class="col tasks"><h2>Choose a task</h2>%s</section></div>'
            '<section class="answer" id="answer" aria-live="polite"></section>%s</div>'
            '<script type="application/json" id="data">%s</script><script>%s</script>\n'
            % (head, E(hand or ""), E(task or ""), top, "".join(left), "".join(right), foot,
               json.dumps(d, sort_keys=True).replace("</", "<\\/"), JS))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--hand", help="the hand the page opens on (a key of the hands table)")
    ap.add_argument("--task", help="the task the page opens on (a task id, or %s)" % OWN)
    ap.add_argument("--out", required=True, help="the page to write (never written over)")
    a = ap.parse_args(argv)
    page = build(REPO, a.hand, a.task)
    with io.open(a.out, "x", encoding="utf-8", newline="\n") as f:
        f.write(page)
    print("wrote", a.out, "(%d bytes)" % len(page.encode("utf-8")))


if __name__ == "__main__":
    main()
