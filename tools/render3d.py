"""Interactive 3D view of a follow-the-leader result: every evaluated moment,
the hand in its solved pose, the ball, each finger coloured by its verdict,
the contact point each finger uses (with its surface normal) and the leader's
contact on the ball. Drag to orbit, scroll to zoom, a timeline to step or play.

    python tools/render3d.py outputs/follow_<hand>__<leader>__....json [STILLS_DIR]  ->  outputs/render3d_<hand>.html

STILLS_DIR (a folder with index.json and the JPEGs, e.g.
data/hands/measured/tennis_new_cup_take3_stills) adds the video still of each
moment, plain or with the keypoints drawn, as a panel on the left.

Re-evaluates the stored placement (follow(placement=...), same stiffness) to
recover the poses; no new search. three.js r128 and its OrbitControls are
inlined (fetched once into tools/vendor_js/), so the page works offline.
Lifted-off fingers are drawn faded in the model's rest pose (collisions are
not checked). SIMULATION ONLY.
"""
import json
import os
import sys
import urllib.request

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import FingerIK, Leader, follow  # noqa: E402
from gramian_workshop.hands import Hand, _mj  # noqa: E402

JS = {"three.min.js": "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js",
      "OrbitControls.js": "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"}
JS_DIR = os.path.join(ROOT, "tools", "vendor_js")


def js(name):
    p = os.path.join(JS_DIR, name)
    if not os.path.exists(p):
        os.makedirs(JS_DIR, exist_ok=True)
        urllib.request.urlretrieve(JS[name], p)
    return open(p, encoding="utf-8").read()


def scene(hand, fr):
    """Capsules, boxes and markers of one moment, in the hand frame (m)."""
    mj = _mj()
    m = hand.model
    d = mj.MjData(m)
    q = np.array(hand.base_q)
    state = {}
    for f, x in zip(hand.fingers, fr["fingers"]):
        state[f.label] = x["state"]
        if "q" in x:
            q[FingerIK(hand, f).qadr] = x["q"]
    d.qpos[:] = hand.apply_coupling(q)
    mj.mj_kinematics(m, d)
    owner = {}
    for f in hand.fingers:
        first = m.jnt_bodyid[f.joints[0]]
        b = f.tip_body
        while b > 0:
            owner[b] = f.label
            if b == first:
                break
            b = int(m.body_parentid[b])
    caps, boxes = [], []
    for g in range(m.ngeom):
        c, R = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3)
        lab = owner.get(int(m.geom_bodyid[g]))
        if int(m.geom_type[g]) == int(mj.mjtGeom.mjGEOM_CAPSULE):
            h, r = m.geom_size[g][1], m.geom_size[g][0]
            caps.append({"a": (c - R[:, 2] * h).round(5).tolist(), "b": (c + R[:, 2] * h).round(5).tolist(),
                         "r": round(float(r), 5), "finger": lab, "state": state.get(lab, "palm")})
        elif int(m.geom_type[g]) == int(mj.mjtGeom.mjGEOM_BOX) and lab is None:
            boxes.append({"c": c.round(5).tolist(), "R": R.round(5).tolist(),
                          "size": m.geom_size[g].round(5).tolist()})
    marks = []
    for x in fr["fingers"]:
        if "target" in x:
            marks.append({"finger": x["finger"], "station": x["station"], "state": x["state"],
                          "target": np.round(x["target"], 5).tolist(),
                          "pad": np.round(x["pad_world"], 5).tolist(), "normal": np.round(x["pad_normal"], 4).tolist()})
    return {"k": fr["k"], "verdict": fr["verdict"], "reason": fr["reason"], "untracked": fr.get("untracked", []),
            "ball": np.round(fr["ball_center"], 5).tolist(), "caps": caps, "boxes": boxes, "marks": marks,
            "fingers": [{"finger": x["finger"], "state": x["state"],
                         "reach_mm": round(x["reach_mm"], 1) if "reach_mm" in x else None,
                         "facing_deg": round(x["facing_deg"]) if "facing_deg" in x else None,
                         "pen_mm": round(x["penetration_mm"], 1) if "penetration_mm" in x else None,
                         "phalanx": x.get("phalanx"), "reason": x.get("reason")} for x in fr["fingers"]]}


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Follow in 3D</title><style>
:root{--bg:#f6f7f9;--panel:#fff;--ink:#1d232b;--muted:#5b6673;--line:#dde2e8}
@media (prefers-color-scheme:dark){:root{--bg:#14171b;--panel:#1d2127;--ink:#e6e9ed;--muted:#9aa5b1;--line:#333a43}}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--ink)}
header{padding:12px 16px}h1{font-size:17px;margin:0 0 4px}p{margin:0;color:var(--muted);font-size:13px;max-width:900px}
#wrap{display:flex;flex-wrap:wrap;gap:12px;padding:0 16px 16px}
#still{flex:0 1 300px;background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:8px}
#view{flex:1 1 560px;min-height:460px;height:62vh;background:var(--panel);border:1px solid var(--line);border-radius:8px;position:relative}
#side{flex:0 1 330px;background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px;font-size:13px}
#strip{display:flex;height:18px;border-radius:4px;overflow:hidden;margin:6px 0 10px;cursor:pointer}
#strip div{flex:1;border-right:1px solid var(--panel)} #strip div.cur{outline:2px solid var(--ink);outline-offset:-2px}
table{border-collapse:collapse;width:100%}td,th{padding:3px 4px;border-bottom:1px solid var(--line);text-align:left}
.sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:-1px}
button,input[type=range]{margin:2px 4px 2px 0} .legend span{margin-right:10px;white-space:nowrap}
#hint{position:absolute;left:10px;bottom:8px;font-size:12px;color:var(--muted);pointer-events:none}
.vlab{position:absolute;top:8px;left:10px;font-size:12px;font-weight:600;color:var(--muted);pointer-events:none;max-width:45%}
</style></head><body>
<header><h1 id="title"></h1><p>Each moment of the leader, with the hand in the pose the check solved for. Fingers are coloured
by their verdict. Each touching finger meets the ball tangentially: small black ball = where it touches, arrow = the
finger's outward normal there; coloured ball = the leader's contact. Green needs the touch within 2 mm of the leader's contact plus the soft pad's contact patch (about 6 mm across), on the pad or side of the finger (not the nail half), with the finger sunk into the ball by at most 3 mm (amber 6 mm).
"side" in the table is the angle from the middle of the pad (90&deg; = the side). Fingers are not checked against each
other. Lifted-off fingers are faded, in the model's rest pose. Simulation only.</p></header>
<div id="wrap"><div id="still" style="display:none"><div class="vlab" style="position:static;max-width:none;margin-bottom:4px">the video
 <label style="font-weight:400"><input type="checkbox" id="ovl" checked> keypoints drawn</label> <span id="cam"></span></div>
 <img id="img" alt="video still of the demonstration at this moment" style="width:100%;border-radius:6px"></div><div id="view"><div id="lab_l" class="vlab"></div><div id="lab_r" class="vlab" style="left:50%">the model hand, solved</div>
<div id="hint">drag to orbit (both views turn together) &middot; scroll to zoom &middot; right-drag to pan</div></div>
<div id="side"><div style="margin-bottom:6px">view: <button data-v="palm">palm</button><button data-v="thumb">thumb side</button><button data-v="little">little side</button><button data-v="tips">fingertips</button><button data-v="video" id="vidbtn" disabled title="needs the camera calibration">video camera</button></div>
<div><button id="prev">&#9664;</button><button id="play">play</button><button id="next">&#9654;</button>
<label><input type="checkbox" id="ghost" checked> lifted fingers</label></div>
<input type="range" id="slider" min="0" value="0" style="width:100%">
<div id="strip"></div><div id="frame" style="font-weight:600;margin-bottom:6px"></div>
<table id="tab"></table><div class="legend" style="margin-top:8px"></div></div></div>
<script>__THREE__</script><script>__ORBIT__</script>
<script>
const D=__DATA__;
const COL={green:0x2e9e5b,amber:0xe0a100,red:0xd1453b,off:0x9aa3ad,palm:0xc9b8a6};
const CSS={green:'#2e9e5b',amber:'#e0a100',red:'#d1453b',off:'#9aa3ad'};
document.getElementById('title').textContent=D.hand+' following '+D.leader;
document.querySelector('.legend').innerHTML=['green','amber','red','off'].map(s=>`<span><span class="sw" style="background:${CSS[s]}"></span>${s==='off'?'lifted':s}</span>`).join('');
const el=document.getElementById('view');
const R=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});R.setPixelRatio(window.devicePixelRatio);
el.appendChild(R.domElement);
const S=new THREE.Scene();S.background=new THREE.Color(getComputedStyle(document.body).getPropertyValue('--panel').trim()||'#fff');
const cam=new THREE.PerspectiveCamera(35,1,0.005,5);
S.add(new THREE.HemisphereLight(0xffffff,0x667788,0.9));const dl=new THREE.DirectionalLight(0xffffff,0.6);dl.position.set(0.3,0.5,0.8);S.add(dl);
const ctl=new THREE.OrbitControls(cam,R.domElement);
const G=new THREE.Group();S.add(G);
const SL=new THREE.Scene();SL.background=S.background;SL.add(new THREE.HemisphereLight(0xffffff,0x667788,0.9));
const dl2=new THREE.DirectionalLight(0xffffff,0.6);dl2.position.set(0.3,0.5,0.8);SL.add(dl2);
const GL=new THREE.Group();SL.add(GL);
document.getElementById('lab_l').textContent='what the hand copies: '+D.leader_info.label;
function capsule(a,b,r,color,op){const A=new THREE.Vector3(...a),B=new THREE.Vector3(...b),L=A.distanceTo(B);
 const m=new THREE.MeshStandardMaterial({color,roughness:0.6,transparent:op<1,opacity:op});const g=new THREE.Group();
 const cy=new THREE.Mesh(new THREE.CylinderGeometry(r,r,L,20),m);const s1=new THREE.Mesh(new THREE.SphereGeometry(r,20,14),m),s2=s1.clone();
 s1.position.y=L/2;s2.position.y=-L/2;g.add(cy,s1,s2);g.position.copy(A.clone().add(B).multiplyScalar(0.5));
 g.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),B.clone().sub(A).normalize());return g;}
function limb(grp,a,b,r,color,op){grp.add(capsule(a,b,r,color,op*0.35));grp.add(capsule(a,b,0.0022,color,1));
 for(const p of [a,b]){const m=new THREE.Mesh(new THREE.SphereGeometry(0.0038,12,8),new THREE.MeshStandardMaterial({color:0x39424d,roughness:0.6,transparent:op<1,opacity:op}));m.position.set(...p);grp.add(m);}}
let fi=0,timer=null;const ghost=document.getElementById('ghost');
const BONES=[[0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[0,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],[15,16],[0,17],[17,18],[18,19],[19,20],[5,9],[9,13],[13,17]];
const DIG=[null,'thumb','thumb','thumb','thumb','index','index','index','index','middle','middle','middle','middle','ring','ring','ring','ring','little','little','little','little'];
function drawLeader(f){while(GL.children.length)GL.remove(GL.children[0]);
 if(f.skeleton){const S=f.skeleton;const st={};for(const k of f.fingers)st[k.finger]=k.state;
  for(const [a,b] of BONES){if(!S[a]||!S[b])continue;const d=DIG[b]&&DIG[a]===DIG[b]?DIG[b]:(DIG[b]&&a===0?null:null);
   const dg=(DIG[a]===DIG[b]&&DIG[b])?DIG[b]:null;
   const col=dg?(COL[st[dg]]??COL.off):COL.palm;
   limb(GL,S[a],S[b],dg?(D.leader_info.pad_r[dg]??0.008):0.009,col,1);}}
 const ball=new THREE.Mesh(new THREE.SphereGeometry(D.radius,40,28),new THREE.MeshStandardMaterial({color:0xcfe37a,roughness:0.9,transparent:true,opacity:0.55}));
 ball.position.set(...f.ball);GL.add(ball);
 const st={};for(const k of f.marks)st[k.station]=k.state;
 for(const s of f.leader){const col=s.touch?(COL[st[s.i]]??0x6b7c93):0x9aa3ad;
  if(f.skeleton&&!s.touch)continue;
  const m=new THREE.Mesh(new THREE.SphereGeometry(D.leader_info.station_r,20,14),new THREE.MeshStandardMaterial({color:col,roughness:0.6,transparent:!s.touch,opacity:s.touch?1:0.3}));
  m.position.set(...s.pos);GL.add(m);}
 for(const k of f.marks){const t=new THREE.Mesh(new THREE.SphereGeometry(0.0022,14,10),new THREE.MeshBasicMaterial({color:COL[k.state]??COL.off}));t.position.set(...k.target);GL.add(t);}}
function draw(i){fi=i;while(G.children.length)G.remove(G.children[0]);const f=D.frames[i];drawLeader(f);
 for(const c of f.caps){if(c.state==='off'&&!ghost.checked)continue;limb(G,c.a,c.b,c.r,COL[c.state]??COL.palm,c.state==='off'?0.4:1);}
 for(const b of f.boxes){const m=new THREE.Mesh(new THREE.BoxGeometry(2*b.size[0],2*b.size[1],2*b.size[2]),new THREE.MeshStandardMaterial({color:COL.palm,roughness:0.8}));
  const M=new THREE.Matrix4().set(b.R[0][0],b.R[0][1],b.R[0][2],b.c[0],b.R[1][0],b.R[1][1],b.R[1][2],b.c[1],b.R[2][0],b.R[2][1],b.R[2][2],b.c[2],0,0,0,1);m.applyMatrix4(M);G.add(m);}
 const ball=new THREE.Mesh(new THREE.SphereGeometry(D.radius,40,28),new THREE.MeshStandardMaterial({color:0xcfe37a,roughness:0.9,transparent:true,opacity:0.55}));
 ball.position.set(...f.ball);G.add(ball);
 for(const k of f.marks){const t=new THREE.Mesh(new THREE.SphereGeometry(0.0022,14,10),new THREE.MeshBasicMaterial({color:COL[k.state]??COL.off}));t.position.set(...k.target);G.add(t);
  const p=new THREE.Mesh(new THREE.SphereGeometry(0.0016,12,8),new THREE.MeshBasicMaterial({color:0x111111}));p.position.set(...k.pad);G.add(p);
  G.add(new THREE.ArrowHelper(new THREE.Vector3(...k.normal),new THREE.Vector3(...k.pad),0.014,0x111111,0.004,0.0025));
  const gap=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...k.pad),new THREE.Vector3(...k.target)]);
  G.add(new THREE.Line(gap,new THREE.LineBasicMaterial({color:COL[k.state]??COL.off})));}
 document.getElementById('slider').value=i;
 if(D.stills){const st=D.stills[String(f.k)];document.getElementById('still').style.display=st?'block':'none';
  if(st){document.getElementById('img').src=document.getElementById('ovl').checked?st.overlay:st.plain;
   document.getElementById('cam').textContent='('+st.camera+' camera, t = '+st.t+' s)';}}
 document.getElementById('frame').innerHTML=`moment ${f.k} (${i+1} of ${D.frames.length}): <span style="color:${CSS[f.verdict]}">${f.verdict}</span>${f.reason?' ('+f.reason+')':''}`+
  (f.untracked&&f.untracked.length?`<div style="font-weight:400;color:var(--muted)">not well tracked: ${f.untracked.join(', ')} (no segment seen at both ends)</div>`:'');
 document.getElementById('tab').innerHTML='<tr><th>finger</th><th>state</th><th>off</th><th>side</th><th>sunk</th><th>phalanx</th></tr>'+f.fingers.map(x=>
  `<tr><td><span class="sw" style="background:${CSS[x.state]??CSS.off}"></span>${x.finger}</td><td>${x.state==='off'?'lifted':x.state}${x.reason?' ('+x.reason+')':''}</td>`+
  (x.reach_mm===null?'<td colspan="4"></td>':`<td>${x.reach_mm} mm</td><td>${x.facing_deg}&deg;</td><td>${x.pen_mm} mm</td><td>${x.phalanx}</td>`)+'</tr>').join('');
 [...document.querySelectorAll('#strip div')].forEach((d,j)=>d.classList.toggle('cur',j===i));}
const strip=document.getElementById('strip');D.frames.forEach((f,j)=>{const d=document.createElement('div');d.style.background=CSS[f.verdict];d.title=`moment ${f.k}: ${f.verdict}`;d.onclick=()=>draw(j);strip.appendChild(d);});
const sl=document.getElementById('slider');sl.max=D.frames.length-1;sl.oninput=()=>draw(+sl.value);
document.getElementById('prev').onclick=()=>draw((fi-1+D.frames.length)%D.frames.length);
document.getElementById('next').onclick=()=>draw((fi+1)%D.frames.length);
document.getElementById('play').onclick=e=>{if(timer){clearInterval(timer);timer=null;e.target.textContent='play';}else{timer=setInterval(()=>draw((fi+1)%D.frames.length),400);e.target.textContent='pause';}};
ghost.onchange=()=>draw(fi);document.getElementById('ovl').onchange=()=>draw(fi);
// fixed viewpoints, in the hand frame (x toward the thumb, y distal, z palmar)
const VIEWS={palm:[0,-0.25,1],thumb:[1,0,0.25],little:[-1,0,0.25],tips:[0,1,0.4]};
function setView(v){const c=new THREE.Vector3(...D.frames[fi].ball);const d=new THREE.Vector3(...VIEWS[v]).normalize();
 ctl.target.copy(c);cam.position.copy(c.clone().add(d.multiplyScalar(0.19)));cam.up.set(0,v==='tips'?0:1,v==='tips'?1:0);cam.lookAt(c);ctl.update();}
document.querySelectorAll('[data-v]').forEach(b=>{if(b.dataset.v!=='video')b.onclick=()=>setView(b.dataset.v);});
// camera: from the palm side, a little toward the thumb, looking at the ball
const c0=new THREE.Vector3(...D.frames[0].ball);ctl.target.copy(c0);cam.up.set(0,1,0);
cam.position.copy(c0.clone().add(new THREE.Vector3(0.07,-0.035,0.17)));ctl.update();
function size(){const w=el.clientWidth,h=el.clientHeight;R.setSize(w,h);cam.aspect=(w/2)/h;cam.updateProjectionMatrix();}
window.addEventListener('resize',size);size();draw(D.start??0);
(function loop(){requestAnimationFrame(loop);ctl.update();const w=el.clientWidth,h=el.clientHeight;R.setScissorTest(true);
 R.setViewport(0,0,w/2,h);R.setScissor(0,0,w/2,h);R.render(SL,cam);
 R.setViewport(w/2,0,w/2,h);R.setScissor(w/2,0,w/2,h);R.render(S,cam);R.setScissorTest(false);})();
</script></body></html>"""


def stills_for(frames, L, stills_dir):
    """{k: {plain, overlay (data URIs), camera, t}} for frames with a still
    within 5 ms of their time."""
    import base64
    I = json.load(open(os.path.join(stills_dir, "index.json")))
    by_t = {round(s_["t"], 3): s_ for s_ in I["stills"]}
    uri = lambda f: "data:image/jpeg;base64," + base64.b64encode(open(os.path.join(stills_dir, f), "rb").read()).decode()
    out = {}
    for fs in frames:
        t = L["frames"][fs["k"]]["t"]
        s_ = by_t.get(round(t, 3))
        if s_:
            out[str(fs["k"])] = {"plain": uri(s_["file"]), "overlay": uri(s_["overlay"]), "camera": s_["camera"],
                                 "t": round(t, 3)}
    return out


def main():
    src_p = sys.argv[1]
    stills_dir = sys.argv[2] if len(sys.argv) > 2 else None
    src = json.load(open(src_p))
    L = json.load(open(os.path.join(ROOT, "data", "leaders", src["leader"] + ".json")))
    kq = src.get("settings", {}).get("joint_stiffness_Nm_per_rad", 0.0)
    r = follow(src["hand"], L, n_eval=len(src["frames"]), placement=src, joint_stiffness=kq)
    hand = Hand(src["hand"])
    frames = [scene(hand, fr) for fr in r["frames"]]
    # what the hand copies: the leader's own stations, placed as the hand sees them
    leader = Leader(L)
    leader.phi = np.radians(src.get("respace_deg") or np.zeros(leader.n))
    Rw = np.array(src["placement"]["rotation_matrix"])
    t = np.array(src["placement"]["translation_m"])
    sim = "Wave_manip unified_v6" in str(L.get("kind", ""))
    names = L.get("stations") or [f"station {i}" for i in range(leader.n)]
    for fs in frames:
        lf = L["frames"][fs["k"]]
        oc = np.array(lf["obj_pos"])
        sts = []
        for i in range(leader.n):
            sp = lf["station_pos"][i]
            if sp is None:
                continue
            q = oc + leader._turn(i, False) @ (np.array(sp) - oc)
            sts.append({"i": i, "name": names[i], "pos": np.round(Rw @ (q - leader.c0) + t, 5).tolist(),
                        "touch": bool(lf["contact"][i])})
        fs["leader"] = sts
        if lf.get("skeleton"):
            fs["skeleton"] = [None if v is None else np.round(Rw @ (np.array(v) - leader.c0) + t, 5).tolist()
                              for v in lf["skeleton"]]
    data_leader = {"station_r": 0.008 if sim else 0.004,
                   "pad_r": dict(zip(L.get("stations") or [], (L.get("pad_offset_m") or
                                                            [0.0081, 0.0089, 0.0087, 0.0092, 0.0049]))),
                   "label": ("the robot cup's contact spheres (simulated leader)" if sim
                             else "the demonstrator's real hand: keypoint skeleton and contacts (recorded)")}
    start = next((i for i, f in enumerate(frames) if f["verdict"] != "green"), 0)
    data = {"hand": r["hand_name"], "leader": r["leader"], "radius": float(L["object"]["radius_m"]),
            "frames": frames, "start": start, "leader_info": data_leader,
            "stills": stills_for(frames, L, stills_dir) if stills_dir else None}
    page = (PAGE.replace("__THREE__", js("three.min.js")).replace("__ORBIT__", js("OrbitControls.js"))
            .replace("__DATA__", json.dumps(data, separators=(",", ":"))))
    out = os.path.join(ROOT, "outputs", f"render3d_{src['hand']}.html")
    open(out, "w", encoding="utf-8").write(page)
    print("wrote", out, f"({len(page) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
