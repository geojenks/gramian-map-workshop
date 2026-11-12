"""The target beside the attempt, for the stored results the landing page
shows (tools/make_landing_mockup.py).

    python tools/render_landing_tasks.py --out outputs/NEW_FOLDER
    python tools/render_landing_tasks.py --out outputs/NEW_FOLDER --browser MSEDGE_OR_CHROME

A task is checked at a set of still moments. For each stored result of a
hand on a task (not a hand's own hold) this writes view_<hand>__<task>.html,
a page that shows every checked moment twice:

  target    the task's object, its contact points and the force the task
            asks for at each, with their sum
  attempt   the chosen hand in the pose the check solved, where each finger
            touches, how far that is from its contact point, the force the
            check found the finger supplies (where the result stores one)
            and the sum of those

Everything is read from the task file and the stored result; nothing is
solved again. The hand is drawn by tools/render3d.py's scene() from the
stored joint positions. Forces are turned into the hand's frame the way the
check does it (the stored placement, then any respacing), so the stored
`force_departure_N` of a finger is the length of the difference between the
two arrows drawn for it.

With --browser it also writes still_<hand>__<task>.png, a picture of that
page taken by the browser given (a Chromium browser, run headless), for the
landing page to show. renders.json lists what was written. Nothing is ever
written over: the folder must be new. Needs MuJoCo and numpy (the hands
extra).

    python tools/render_landing_tasks.py --tasks --out outputs/NEW_FOLDER

With --tasks it writes instead the view of each task on its own, with no
hand chosen: task_<card>.html, one page for each task card of the landing
page, with the card's variants inside. It shows the target only, at the
moments the check looks at by default (40 of them, spread evenly over the
task's frames), in the task's own frame: the object, the contact points
and the force asked for at each, with their sum. With --browser it also
writes still_task_<task>.png for every variant. tasks.json lists what was
written.
"""
import argparse
import io
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import make_landing_mockup as page  # noqa: E402
import render3d  # noqa: E402
from gramian_workshop.follow import Leader  # noqa: E402
from gramian_workshop.hands import Hand  # noqa: E402


MOMENTS = 40          # how many moments tools/run_follow.py checks by default (its --frames)


def r5(v):
    return [round(float(x), 5) for x in v]


def moments(n_frames, n=MOMENTS):
    """The frames of a task that the check looks at: `n` spread evenly."""
    return [int(k) for k in np.linspace(0, n_frames - 1, n).astype(int)]


def task_data(repo, task_id, label):
    """What the view of one task on its own shows, in the task's own frame
    (the object's centre at the first frame is the origin)."""
    L = page.read_json(os.path.join(repo, "data", "leaders", task_id + ".json"))
    leader = Leader(L)
    c0 = np.array(L["frames"][0]["obj_pos"], dtype=float)
    frames, largest = [], 0.0
    for k in moments(len(L["frames"])):
        lf = L["frames"][k]
        contacts, total = [], np.zeros(3)
        for i in range(leader.n):
            if not (lf["contact_point"][i] or lf["station_pos"][i]):
                continue
            p, _, _ = leader.station(k, i, np.eye(3), np.zeros(3))
            F = np.array(lf["force_on_object_N"][i], dtype=float)
            touch = bool(lf["contact"][i])
            if touch:
                total = total + F
            contacts.append({"i": i, "pos": r5(p), "touch": touch, "force": r5(F),
                             "force_N": round(float(np.linalg.norm(F)), 4)})
        largest = max([largest] + [c["force_N"] for c in contacts if c["touch"]])
        frames.append({"k": k, "t": round(float(lf["t"]), 3),
                       "ball": r5(np.array(lf["obj_pos"], dtype=float) - c0), "contacts": contacts,
                       "task_sum": {"force": r5(total), "force_N": round(float(np.linalg.norm(total)), 4),
                                    "of": sum(1 for c in contacts if c["touch"])}})
    return {"task": task_id, "label": label, "caveat": L.get("caveat"),
            "radius": float(L["object"]["radius_m"]), "mass_kg": L["object"].get("mass_kg"),
            "points": leader.n, "all_frames": len(L["frames"]),
            "largest_contact_N": round(largest, 4), "frames": frames}


def card_data(repo, card):
    """What the page for one task card of the landing page shows."""
    return {"card": card["key"], "name": card["name"],
            "variants": [task_data(repo, v["id"], v["label"]) for v in card["variants"]]}


def view_data(repo, rel, hand_name, task_name):
    """What one view shows, from the stored result `rel` and its task file."""
    src = page.read_json(os.path.join(repo, rel))
    L = page.read_json(os.path.join(repo, "data", "leaders", src["leader"] + ".json"))
    leader = Leader(L)
    leader.phi = np.radians(src.get("respace_deg") or np.zeros(leader.n))
    Rw = np.array(src["placement"]["rotation_matrix"])
    t = np.array(src["placement"]["translation_m"])
    hand = Hand(src["hand"])
    frames, largest = [], 0.0
    for fr in src["frames"]:
        k = fr["k"]
        lf = L["frames"][k]
        s = render3d.scene(hand, fr)
        asked = {}
        contacts = []
        for i in range(leader.n):
            if not (lf["contact_point"][i] or lf["station_pos"][i]):
                continue
            p, _, _ = leader.station(k, i, Rw, t)
            F = Rw @ leader._turn(i, False) @ np.array(lf["force_on_object_N"][i], dtype=float)
            asked[i] = F
            contacts.append({"i": i, "pos": r5(p), "touch": bool(lf["contact"][i]), "force": r5(F),
                             "force_N": round(float(np.linalg.norm(F)), 4)})
        touching = [c["i"] for c in contacts if c["touch"]]
        task_sum = sum((asked[i] for i in touching), np.zeros(3))
        fingers, given = [], []
        for x in fr["fingers"]:
            e = x.get("equilibrium") or {}
            F = e.get("force_N")
            row = {"finger": x["finger"], "state": x["state"], "station": x.get("station"),
                   "off_mm": round(x["reach_mm"], 1) if "reach_mm" in x else None,
                   "pad": r5(x["pad_world"]) if "pad_world" in x else None,
                   "asked_N": (round(float(np.linalg.norm(asked[x["station"]])), 4)
                               if x.get("station") in asked else None),
                   "force": r5(F) if F is not None else None,
                   "force_N": round(float(np.linalg.norm(F)), 4) if F is not None else None,
                   "apart_N": round(e["force_departure_N"], 4) if F is not None else None}
            if F is not None:
                given.append(np.array(F))
            fingers.append(row)
        hand_sum = sum(given, np.zeros(3))
        largest = max([largest] + [c["force_N"] for c in contacts] + [f["force_N"] or 0.0 for f in fingers])
        frames.append({"k": k, "t": round(float(lf["t"]), 3), "verdict": fr["verdict"], "reason": fr["reason"],
                       "unserved": fr.get("unserved") or [], "ball": s["ball"], "caps": s["caps"],
                       "boxes": s["boxes"], "marks": s["marks"], "contacts": contacts, "fingers": fingers,
                       "task_sum": {"force": r5(task_sum), "force_N": round(float(np.linalg.norm(task_sum)), 4),
                                    "of": len(touching)},
                       "hand_sum": {"force": r5(hand_sum), "force_N": round(float(np.linalg.norm(hand_sum)), 4),
                                    "of": len(given)} if given else None})
    return {"file": rel.replace(os.sep, "/"), "hand": src["hand"], "hand_name": hand_name,
            "task": src["leader"], "task_name": task_name, "task_caveat": L.get("caveat"),
            "caveat": src.get("caveat"), "radius": float(L["object"]["radius_m"]),
            "largest_contact_N": round(largest, 4), "frames": frames,
            "start": next((i for i, f in enumerate(frames) if f["hand_sum"]), 0)}


PAGE = r"""<meta charset="utf-8">
<meta name="claude:viewport-fit-sides" content="cover">
<title>Target beside attempt</title>
<style>
:root{--bg:#f5f6f4;--panel:#ffffff;--ink:#1c2420;--muted:#59645e;--line:#d9dfda;--accent:#1f6f5c;
 --green:#2f8f5b;--amber:#c98a1b;--red:#c2473a;--off:#9aa39d;--ask:#2c6fb3;--give:#b3572c}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#141816;--panel:#1c211e;--ink:#e6ebe7;
 --muted:#9ba69f;--line:#323a35;--accent:#5fc2a7;--green:#58b983;--amber:#e0a93f;--red:#e0705f;--off:#6d7771;
 --ask:#6aa8e6;--give:#e6936a;color-scheme:dark}}
:root[data-theme="dark"]{--bg:#141816;--panel:#1c211e;--ink:#e6ebe7;--muted:#9ba69f;--line:#323a35;
 --accent:#5fc2a7;--green:#58b983;--amber:#e0a93f;--red:#e0705f;--off:#6d7771;--ask:#6aa8e6;--give:#e6936a;
 color-scheme:dark}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--ink);font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif;margin:0;
 padding-block:16px 32px;padding-left:max(16px,env(safe-area-inset-left,0px));
 padding-right:max(16px,env(safe-area-inset-right,0px))}
main{max-width:1180px;margin:0 auto;display:flex;flex-direction:column;gap:14px}
h1{font-size:20px;margin:0;text-wrap:balance}h2{font-size:14px;margin:0}
p{margin:0;max-width:75ch}.muted{color:var(--muted)}
.caveat{border-left:3px solid var(--amber);padding:6px 10px;background:var(--panel)}
.two{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media (max-width:720px){.two{grid-template-columns:1fr}}
.pane{background:var(--panel);border:1px solid var(--line);border-radius:8px;min-width:0;display:flex;
 flex-direction:column}
.pane header{padding:8px 10px 0}.pane .small{font-size:12.5px;color:var(--muted)}
.gl{height:380px;position:relative}.gl canvas{display:block;width:100%;height:100%}
.pane footer{padding:6px 10px 9px;font-size:13px;border-top:1px solid var(--line)}
.bar{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
button{font:inherit;color:var(--ink);background:var(--panel);border:1px solid var(--line);border-radius:6px;
 padding:4px 10px;cursor:pointer}button:hover{border-color:var(--accent)}
#strip{display:flex;gap:2px;flex:1 1 320px;min-width:0}
#strip i{flex:1 1 0;height:22px;border-radius:2px;cursor:pointer;opacity:.55}
#strip i.on{opacity:1;outline:2px solid var(--ink);outline-offset:1px}
.green{background:var(--green)}.amber{background:var(--amber)}.red{background:var(--red)}.offc{background:var(--off)}
.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:13px;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:4px 10px 4px 0;border-bottom:1px solid var(--line);white-space:nowrap}
th{color:var(--muted);font-weight:500}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
.key{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:12.5px;color:var(--muted)}
.key b{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:5px}
code{font-size:12.5px;overflow-wrap:anywhere}
</style>
<main>
<div><h1 id="title"></h1>
<p class="muted">A moving task, checked frame by frame. Each bar of the strip is one checked moment. Drag either
picture to turn both.</p></div>
<p class="caveat" id="caveat" hidden></p>
<div class="bar"><button id="prev">Back</button><button id="play">Play</button><button id="next">Next</button>
<div id="strip"></div><span id="when" class="muted"></span></div>
<div class="two">
<section class="pane"><header><h2>Target: what the task asks</h2>
<div class="small">The object, the task's contact points and the force asked for at each.</div></header>
<div class="gl" id="glT"></div><footer id="footT"></footer></section>
<section class="pane"><header><h2 id="headA"></h2>
<div class="small" id="subA">The hand in the pose the check solved, where each finger touches, and the force it supplies. Only its capsule and box shapes are drawn.</div></header>
<div class="gl" id="glA"></div><footer id="footA"></footer></section>
</div>
<div class="key"><span><b style="background:var(--ask)"></b>force the task asks for at a contact</span>
<span id="keyGive"><b style="background:var(--give)"></b>force the finger supplies</span>
<span><b style="background:var(--ink)"></b>sum of the forces drawn, at the object's centre</span>
<span>finger colour: its verdict at this moment; grey is lifted</span></div>
<p id="verdict"></p>
<div class="scroll"><table id="table"></table></div>
<p class="muted">Every arrow is a force on the object, drawn to one scale in both pictures, with its head at the
point it acts on. The sums are of contact forces only: the object's weight is not in them. The attempt's sum counts
only the fingers the result stores a force for.</p>
<p class="muted" id="source"></p>
</main>
<script>__THREE__</script>
<script>__ORBIT__</script>
<script type="application/json" id="data">__DATA__</script>
<script>
(function(){
var D=JSON.parse(document.getElementById('data').textContent),F=D.frames,cur=D.start,timer=null;
function $(i){return document.getElementById(i)}
function esc(s){return String(s).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function css(n){return getComputedStyle(document.documentElement).getPropertyValue(n).trim()}
function V(a){return new THREE.Vector3(a[0],a[1],a[2])}
$('title').textContent=D.task_name+', tried by '+D.hand_name;
$('headA').textContent='Attempt: the hand chosen';
// the check stores a supplied force only for a finger with fewer inputs than sprung joints
var GIVES=D.frames.some(function(f){return f.hand_sum});
if(!GIVES){$('keyGive').hidden=true;
  $('subA').textContent='The hand in the pose the check solved, and where each finger touches. Only its capsule and box shapes are drawn. This result stores no supplied force.'}
if(D.caveat){$('caveat').hidden=false;$('caveat').textContent=D.caveat+'. So read the supplied forces as comparative, not as newtons to rely on.'}
$('source').innerHTML='Read from <code>'+esc(D.file)+'</code> and the task file <code>data/leaders/'+esc(D.task)+'.json</code>'+
  (D.task_caveat?' ('+esc(D.task_caveat)+')':'')+'. Made by <code>tools/render_landing_tasks.py</code>.';
var perN=D.largest_contact_N>0?0.8*D.radius/D.largest_contact_N:0;
function pane(id){
  var el=$(id),r=new THREE.WebGLRenderer({antialias:true,alpha:true,preserveDrawingBuffer:true});
  r.setPixelRatio(window.devicePixelRatio||1);el.appendChild(r.domElement);
  var s=new THREE.Scene(),c=new THREE.PerspectiveCamera(35,1,0.001,5);c.up.set(0,0,1);
  s.add(new THREE.AmbientLight(0xffffff,0.75));var l=new THREE.DirectionalLight(0xffffff,0.6);l.position.set(0.3,-0.5,1);s.add(l);
  var g=new THREE.Group();s.add(g);
  var o=new THREE.OrbitControls(c,r.domElement);o.enablePan=false;
  return{el:el,r:r,s:s,c:c,g:g,o:o}}
var T=pane('glT'),A=pane('glA');
function size(p){var w=p.el.clientWidth,h=p.el.clientHeight;p.r.setSize(w,h,false);p.c.aspect=w/h;p.c.updateProjectionMatrix()}
function draw(){T.r.render(T.s,T.c);A.r.render(A.s,A.c)}
function follow(a,b){a.o.addEventListener('change',function(){if(a.busy)return;b.busy=true;b.c.position.copy(a.c.position);
  b.c.quaternion.copy(a.c.quaternion);b.o.target.copy(a.o.target);b.o.update();b.busy=false;draw()})}
follow(T,A);follow(A,T);
function mat(col,op,top){return new THREE.MeshLambertMaterial({color:new THREE.Color(col),transparent:op<1||!!top,opacity:op,depthWrite:op>=1&&!top,depthTest:!top})}
function ball(g,c,r,col,op,top){var m=new THREE.Mesh(new THREE.SphereGeometry(r,32,20),mat(col,op,top));m.position.copy(V(c));if(top)m.renderOrder=5;g.add(m)}
function rod(g,a,b,r,col,op,top){a=V(a);b=V(b);var d=b.clone().sub(a),n=d.length();if(n<1e-7)return;
  var m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,n,14),mat(col,op,top));if(top)m.renderOrder=5;
  m.position.copy(a.clone().add(b).multiplyScalar(0.5));m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),d.normalize());g.add(m)}
function arrow(g,at,f,col,fromAt){ // head at `at`, or tail at `at` when fromAt
  var v=V(f).multiplyScalar(perN),n=v.length();if(n<1e-4)return;
  var p=V(at),a=fromAt?p:p.clone().sub(v),b=fromAt?p.clone().add(v):p,h=Math.min(0.006,0.45*n),u=v.clone().normalize();
  var e=b.clone().sub(u.clone().multiplyScalar(h));
  rod(g,[a.x,a.y,a.z],[e.x,e.y,e.z],0.0009,col,1,true);
  var m=new THREE.Mesh(new THREE.ConeGeometry(0.0024,h,14),mat(col,1,true));m.renderOrder=6;
  m.position.copy(e.clone().add(u.clone().multiplyScalar(h/2)));m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),u);g.add(m)}
function clear(g){while(g.children.length){var m=g.children.pop();if(m.geometry)m.geometry.dispose();if(m.material)m.material.dispose()}}
function stateCol(s){return css(s==='green'?'--green':s==='amber'?'--amber':s==='red'?'--red':'--off')}
function n3(x){return x==null?'not stored':x.toFixed(3)}
function show(i){
  cur=(i+F.length)%F.length;var f=F[cur],ask=css('--ask'),give=css('--give'),ink=css('--ink'),soft=css('--off');
  clear(T.g);clear(A.g);
  ball(T.g,f.ball,D.radius,soft,0.28);ball(A.g,f.ball,D.radius,soft,0.28);
  f.contacts.forEach(function(c){ball(T.g,c.pos,0.0022,c.touch?ask:soft,c.touch?1:0.6,true);if(c.touch)arrow(T.g,c.pos,c.force,ask)});
  arrow(T.g,f.ball,f.task_sum.force,ink,true);
  f.caps.forEach(function(c){var off=c.state==='off';rod(A.g,c.a,c.b,c.r,c.finger?stateCol(c.state):soft,off?0.35:1);
    ball(A.g,c.a,c.r,c.finger?stateCol(c.state):soft,off?0.35:1);ball(A.g,c.b,c.r,c.finger?stateCol(c.state):soft,off?0.35:1)});
  f.boxes.forEach(function(b){var m=new THREE.Mesh(new THREE.BoxGeometry(2*b.size[0],2*b.size[1],2*b.size[2]),mat(soft,0.8));
    m.position.copy(V(b.c));var R=b.R;m.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(
      R[0][0],R[0][1],R[0][2],0,R[1][0],R[1][1],R[1][2],0,R[2][0],R[2][1],R[2][2],0,0,0,0,1));A.g.add(m)});
  f.marks.forEach(function(m){ball(A.g,m.target,0.0022,ask,1,true);ball(A.g,m.pad,0.0022,give,1,true);rod(A.g,m.target,m.pad,0.0005,ink,1,true)});
  f.fingers.forEach(function(x){if(x.force)arrow(A.g,x.pad,x.force,give)});
  if(f.hand_sum)arrow(A.g,f.ball,f.hand_sum.force,ink,true);
  $('footT').innerHTML='Sum of the '+f.task_sum.of+' forces asked for: <b>'+n3(f.task_sum.force_N)+' N</b>';
  $('footA').innerHTML=f.hand_sum?'Sum of the '+f.hand_sum.of+' forces supplied: <b>'+n3(f.hand_sum.force_N)+' N</b>'
    :(GIVES?'No supplied force is stored for this hand at this moment.'
      :'This result stores no supplied force: the check stores one only for a finger with fewer inputs than sprung joints.');
  $('when').textContent='moment '+(cur+1)+' of '+F.length+', at '+f.t.toFixed(2)+' s';
  $('verdict').innerHTML='<span class="dot '+esc(f.verdict)+'"></span>This moment: <b>'+esc(f.verdict)+'</b>'+
    (f.reason?' ('+esc(f.reason)+')':'')+(f.unserved.length?'. No finger serves contact '+f.unserved.join(', ')+'.':'.');
  var h='<tr><th>Finger</th><th>Contact</th><th>Verdict</th><th>Off its contact by</th><th>Force asked</th><th>Force supplied</th><th>The two differ by</th></tr>';
  f.fingers.forEach(function(x){h+='<tr><td>'+esc(x.finger)+'</td><td>'+(x.station==null?'none':x.station)+'</td><td><span class="dot '+
    (x.state==='off'?'offc':esc(x.state))+'"></span>'+(x.state==='off'?'lifted':esc(x.state))+'</td><td>'+(x.off_mm==null?'':x.off_mm.toFixed(1)+' mm')+
    '</td><td>'+(x.asked_N==null?'':x.asked_N.toFixed(3)+' N')+'</td><td>'+(x.state==='off'?'':x.force_N==null?'not stored':x.force_N.toFixed(3)+' N')+
    '</td><td>'+(x.apart_N==null?'':x.apart_N.toFixed(3)+' N')+'</td></tr>'});
  $('table').innerHTML=h;
  [].forEach.call($('strip').children,function(b,j){b.className=F[j].verdict+(j===cur?' on':'')});
  draw()}
F.forEach(function(f,j){var b=document.createElement('i');b.title='moment '+(j+1)+': '+f.verdict;b.onclick=function(){stop();show(j)};$('strip').appendChild(b)});
function stop(){if(timer){clearInterval(timer);timer=null;$('play').textContent='Play'}}
$('prev').onclick=function(){stop();show(cur-1)};$('next').onclick=function(){stop();show(cur+1)};
$('play').onclick=function(){if(timer){stop();return}$('play').textContent='Pause';timer=setInterval(function(){show(cur+1)},450)};
// one view for the whole task: look at the middle of everything drawn, from the side and above
var lo=[1e9,1e9,1e9],hi=[-1e9,-1e9,-1e9];
function grow(p,r){for(var a=0;a<3;a++){lo[a]=Math.min(lo[a],p[a]-r);hi[a]=Math.max(hi[a],p[a]+r)}}
F.forEach(function(f){grow(f.ball,D.radius);f.caps.forEach(function(c){grow(c.a,c.r);grow(c.b,c.r)});
  // a box is taken as the ball round it: its centre, and half its diagonal
  f.boxes.forEach(function(b){grow(b.c,Math.sqrt(b.size[0]*b.size[0]+b.size[1]*b.size[1]+b.size[2]*b.size[2]))})});
var mid=[0,1,2].map(function(a){return(lo[a]+hi[a])/2}),span=Math.max(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]);
[T,A].forEach(function(p){p.busy=true;p.o.target.set(mid[0],mid[1],mid[2]);
  p.c.position.set(mid[0]+1.7*span,mid[1]-1.7*span,mid[2]+0.25*span);p.o.update();p.busy=false});
function fit(){size(T);size(A);draw()}
window.addEventListener('resize',fit);
if(window.matchMedia)window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change',function(){show(cur)});
var m=/^#moment-(\d+)$/.exec(location.hash||'');if(m&&+m[1]>=1&&+m[1]<=F.length)cur=+m[1]-1;
fit();show(cur);
})();
</script>
"""


# the view of a task on its own: the styles are PAGE's
TASK_BODY = r"""<main>
<div><h1 id="title"></h1>
<p class="muted">A moving task, checked frame by frame, shown with no hand chosen. Each bar of the strip is one of
the moments the check looks at. Drag the picture to turn it.</p></div>
<div class="bar" id="variants" hidden></div>
<p class="caveat" id="caveat" hidden></p>
<div class="bar"><button id="prev">Back</button><button id="play">Play</button><button id="next">Next</button>
<div id="strip"></div><span id="when" class="muted"></span></div>
<section class="pane"><header><h2>Target: what the task asks</h2>
<div class="small" id="what"></div></header>
<div class="gl" id="glT"></div><footer id="footT"></footer></section>
<div class="key"><span><b style="background:var(--ask)"></b>force the task asks for at a contact</span>
<span><b style="background:var(--ink)"></b>sum of the forces drawn, at the object's centre</span>
<span>grey point: a contact point that is lifted at this moment</span></div>
<div class="scroll"><table id="table"></table></div>
<p class="muted">Every arrow is a force on the object, with its head at the point it acts on. The sum is of
contact forces only: the object's weight is not in it. The picture is in the task's own frame; a check first
places the task in a hand.</p>
<p class="muted" id="source"></p>
</main>
<script>__THREE__</script>
<script>__ORBIT__</script>
<script type="application/json" id="data">__DATA__</script>
<script>
(function(){
var D=JSON.parse(document.getElementById('data').textContent),W,F,cur=0,timer=null,perN=0;
function $(i){return document.getElementById(i)}
function esc(s){return String(s).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function css(n){return getComputedStyle(document.documentElement).getPropertyValue(n).trim()}
function V(a){return new THREE.Vector3(a[0],a[1],a[2])}
$('title').textContent=D.name+', on its own';
var el=$('glT'),R=new THREE.WebGLRenderer({antialias:true,alpha:true,preserveDrawingBuffer:true});
R.setPixelRatio(window.devicePixelRatio||1);el.appendChild(R.domElement);
var S=new THREE.Scene(),C=new THREE.PerspectiveCamera(35,1,0.001,5);C.up.set(0,0,1);
S.add(new THREE.AmbientLight(0xffffff,0.75));var l=new THREE.DirectionalLight(0xffffff,0.6);l.position.set(0.3,-0.5,1);S.add(l);
var G=new THREE.Group();S.add(G);
var O=new THREE.OrbitControls(C,R.domElement);O.enablePan=false;
function draw(){R.render(S,C)}
O.addEventListener('change',draw);
function mat(col,op,top){return new THREE.MeshLambertMaterial({color:new THREE.Color(col),transparent:op<1||!!top,opacity:op,depthWrite:op>=1&&!top,depthTest:!top})}
function ball(c,r,col,op,top){var m=new THREE.Mesh(new THREE.SphereGeometry(r,32,20),mat(col,op,top));m.position.copy(V(c));if(top)m.renderOrder=5;G.add(m)}
function rod(a,b,r,col){var d=b.clone().sub(a),n=d.length();if(n<1e-7)return;
  var m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,n,14),mat(col,1,true));m.renderOrder=5;
  m.position.copy(a.clone().add(b).multiplyScalar(0.5));m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),d.normalize());G.add(m)}
function arrow(at,f,col,fromAt){ // head at `at`, or tail at `at` when fromAt
  var v=V(f).multiplyScalar(perN),n=v.length();if(n<1e-4)return;
  var p=V(at),a=fromAt?p:p.clone().sub(v),b=fromAt?p.clone().add(v):p,h=Math.min(0.006,0.45*n),u=v.clone().normalize();
  var e=b.clone().sub(u.clone().multiplyScalar(h));rod(a,e,0.0009,col);
  var m=new THREE.Mesh(new THREE.ConeGeometry(0.0024,h,14),mat(col,1,true));m.renderOrder=6;
  m.position.copy(e.clone().add(u.clone().multiplyScalar(h/2)));m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),u);G.add(m)}
function clear(){while(G.children.length){var m=G.children.pop();if(m.geometry)m.geometry.dispose();if(m.material)m.material.dispose()}}
function show(i){
  cur=(i+F.length)%F.length;var f=F[cur],ask=css('--ask'),ink=css('--ink'),soft=css('--off');
  clear();ball(f.ball,W.radius,soft,0.28);
  f.contacts.forEach(function(c){ball(c.pos,0.0022,c.touch?ask:soft,c.touch?1:0.6,true);if(c.touch)arrow(c.pos,c.force,ask)});
  arrow(f.ball,f.task_sum.force,ink,true);
  $('footT').innerHTML='Sum of the '+f.task_sum.of+' forces asked for: <b>'+f.task_sum.force_N.toFixed(3)+' N</b>';
  $('when').textContent='moment '+(cur+1)+' of '+F.length+', at '+f.t.toFixed(2)+' s (frame '+f.k+' of the task file)';
  var h='<tr><th>Contact point</th><th>At this moment</th><th>Force asked</th></tr>';
  f.contacts.forEach(function(c){h+='<tr><td>'+c.i+'</td><td>'+(c.touch?'touching':'lifted')+'</td><td>'+(c.touch?c.force_N.toFixed(3)+' N':'')+'</td></tr>'});
  $('table').innerHTML=h;
  [].forEach.call($('strip').children,function(b,j){b.className='ask'+(j===cur?' on':'')});
  draw()}
function stop(){if(timer){clearInterval(timer);timer=null;$('play').textContent='Play'}}
function size(){var w=el.clientWidth,h=el.clientHeight;R.setSize(w,h,false);C.aspect=w/h;C.updateProjectionMatrix()}
function choose(v){
  stop();W=D.variants[v];F=W.frames;perN=W.largest_contact_N>0?0.8*W.radius/W.largest_contact_N:0;
  [].forEach.call($('variants').querySelectorAll('button'),function(b,j){b.setAttribute('aria-pressed',String(j===v))});
  $('caveat').hidden=!W.caveat;$('caveat').textContent=W.caveat?'Caution carried by the task: '+W.caveat:'';
  $('what').textContent='The object ('+(2000*W.radius).toFixed(0)+' mm across'+(W.mass_kg==null?'':', '+(1000*W.mass_kg).toFixed(0)+' g')+
    '), its '+W.points+' contact points and the force asked for at each.';
  $('source').innerHTML='Read from the task file <code>data/leaders/'+esc(W.task)+'.json</code>: '+F.length+' of its '+W.all_frames+
    ' frames, spread evenly. Made by <code>tools/render_landing_tasks.py</code>.';
  $('strip').innerHTML='';
  F.forEach(function(f,j){var b=document.createElement('i');b.title='moment '+(j+1)+': '+f.task_sum.of+' touching';
    b.onclick=function(){stop();show(j)};$('strip').appendChild(b)});
  // one view for the whole task: look at the middle of everything drawn, from the side and above
  var lo=[1e9,1e9,1e9],hi=[-1e9,-1e9,-1e9];
  F.forEach(function(f){for(var a=0;a<3;a++){lo[a]=Math.min(lo[a],f.ball[a]-1.6*W.radius);hi[a]=Math.max(hi[a],f.ball[a]+1.6*W.radius)}});
  var mid=[0,1,2].map(function(a){return(lo[a]+hi[a])/2}),span=Math.max(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]);
  O.target.set(mid[0],mid[1],mid[2]);C.position.set(mid[0]+1.15*span,mid[1]-1.15*span,mid[2]+0.6*span);O.update();
  show(0)}
if(D.variants.length>1){$('variants').hidden=false;
  $('variants').innerHTML='<span class="muted">Variant:</span>'+D.variants.map(function(v,j){
    return '<button type="button" data-v="'+j+'" aria-pressed="false">'+esc(v.label)+'</button>'}).join('');
  $('variants').onclick=function(e){var b=e.target.closest('button');if(b)choose(+b.dataset.v)}}
$('prev').onclick=function(){stop();show(cur-1)};$('next').onclick=function(){stop();show(cur+1)};
$('play').onclick=function(){if(timer){stop();return}$('play').textContent='Pause';timer=setInterval(function(){show(cur+1)},450)};
window.addEventListener('resize',function(){size();draw()});
if(window.matchMedia)window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change',function(){show(cur)});
function fromHash(){var m=/^#variant-(\d+)(?:-moment-(\d+))?$/.exec(location.hash||'');
  choose(m&&+m[1]>=1&&+m[1]<=D.variants.length?+m[1]-1:0);if(m&&m[2]&&+m[2]>=1&&+m[2]<=F.length)show(+m[2]-1)}
window.addEventListener('hashchange',fromHash);
size();fromHash();
})();
</script>
"""
TASK_STYLE = ('.ask{background:var(--ask)}button[aria-pressed="true"]{border-color:var(--accent);'
              'color:var(--accent);font-weight:600}.pane{max-width:760px}.gl{height:400px}\n</style>')


def build_task(d):
    head = PAGE.split("<main>")[0].replace("<title>Target beside attempt</title>", "<title>Task on its own</title>")
    return (head.replace("</style>", TASK_STYLE) + TASK_BODY
            .replace("__THREE__", render3d.js("three.min.js"))
            .replace("__ORBIT__", render3d.js("OrbitControls.js"))
            .replace("__DATA__", json.dumps(d, separators=(",", ":")).replace("</", "<\\/")))


def build(d):
    return (PAGE.replace("__THREE__", render3d.js("three.min.js"))
            .replace("__ORBIT__", render3d.js("OrbitControls.js"))
            .replace("__DATA__", json.dumps(d, separators=(",", ":")).replace("</", "<\\/")))


def stored_pairs(repo):
    """(result, hand name, task name) for every stored result the landing
    page shows that is not a hand's own hold."""
    d = page.data(repo)
    names = {h["key"]: h["name"] for h in d["hands"]}
    out = []
    for pair, shown in sorted(d["results"].items()):
        hand, task = pair.split("|")
        if task == page.OWN:
            continue
        card = next(c for c in d["cards"] if any(v["id"] == task for v in c["variants"]))
        out += [(r["file"], names[hand], card["name"]) for r in shown]
    return out


def shoot(browser, window, shot, url):
    subprocess.run([browser, "--headless=new", "--hide-scrollbars", "--enable-unsafe-swiftshader",
                    "--window-size=" + window, "--virtual-time-budget=4000", "--screenshot=" + shot, url],
                   check=True, timeout=180)
    if not os.path.isfile(shot):
        raise SystemExit("the browser wrote no picture")


def write_tasks(a):
    listed = {"cards": []}
    for card in page.data(ROOT)["cards"]:
        d = card_data(ROOT, card)
        view = "task_%s.html" % d["card"].strip("_")
        with io.open(os.path.join(a.out, view), "x", encoding="utf-8", newline="\n") as f:
            f.write(build_task(d))
        print("wrote", view)
        entry = {"card": d["card"], "view": view, "variants": []}
        for n, v in enumerate(d["variants"]):
            most = max(f["task_sum"]["of"] for f in v["frames"])      # the still: the first moment
            row = {"task": v["task"], "moments": len(v["frames"]),    # with the most contacts touching
                   "shown": 1 + next(j for j, f in enumerate(v["frames"]) if f["task_sum"]["of"] == most)}
            if a.browser:
                row["still"] = "still_task_%s.png" % v["task"]
                shoot(a.browser, a.window or "760,880", os.path.abspath(os.path.join(a.out, row["still"])),
                      "file:///" + os.path.abspath(os.path.join(a.out, view)).replace(os.sep, "/")
                      + "#variant-%d-moment-%d" % (n + 1, row["shown"]))
                print("wrote", row["still"])
            entry["variants"].append(row)
        listed["cards"].append(entry)
    with io.open(os.path.join(a.out, "tasks.json"), "x", encoding="utf-8", newline="\n") as f:
        json.dump(listed, f, indent=1, sort_keys=True)
        f.write("\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="a new folder for the views")
    ap.add_argument("--tasks", action="store_true", help="write the view of each task on its own, "
                    "with no hand chosen, instead of the views of the stored results")
    ap.add_argument("--browser", help="a Chromium browser to take the still pictures with")
    ap.add_argument("--window", help="the browser window for a still, as WIDTH,HEIGHT "
                    "(default: 1180,700, or 760,880 with --tasks)")
    a = ap.parse_args(argv)
    os.makedirs(a.out)                       # fails if it is there: nothing is written over
    if a.tasks:
        return write_tasks(a)
    a.window = a.window or "1180,700"
    listed = {"views": []}
    for rel, hand_name, task_name in stored_pairs(ROOT):
        d = view_data(ROOT, rel, hand_name, task_name)
        stem = "%s__%s" % (d["hand"], d["task"])
        with io.open(os.path.join(a.out, "view_%s.html" % stem), "x", encoding="utf-8", newline="\n") as f:
            f.write(build(d))
        entry = {"result": d["file"], "hand": d["hand"], "task": d["task"], "view": "view_%s.html" % stem,
                 "moments": len(d["frames"]), "shown": d["start"] + 1}
        print("wrote", entry["view"])
        if a.browser:
            shoot(a.browser, a.window, os.path.abspath(os.path.join(a.out, "still_%s.png" % stem)),
                  "file:///" + os.path.abspath(os.path.join(a.out, entry["view"])).replace(os.sep, "/"))
            entry["still"] = "still_%s.png" % stem
            print("wrote", entry["still"])
        listed["views"].append(entry)
    with io.open(os.path.join(a.out, "renders.json"), "x", encoding="utf-8", newline="\n") as f:
        json.dump(listed, f, indent=1, sort_keys=True)
        f.write("\n")


if __name__ == "__main__":
    main()
