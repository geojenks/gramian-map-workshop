"""Animate a leader file as a self-contained HTML page (top and side views).

    python tools/view_leader.py data/leaders/<id>.json [more ...]

Writes outputs/leader_<id>.html: the object, each station (filled while in
contact), and the force it puts on the object, with play/scrub controls.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>__TITLE__</title>
<style>
:root{color-scheme:light;--surface:#fcfcfb;--page:#f9f9f7;--ink:#0b0b0b;--ink2:#52514e;--muted:#898781;
--line:#e1e0d9;--ring:rgba(11,11,11,.1);--obj:#e1e0d9;--made:#2a78d6;--broken:#c3c2b7;--force:#eb6834}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;--surface:#1a1a19;--page:#0d0d0d;
--ink:#f0efec;--ink2:#c3c2b7;--line:#2c2c2a;--ring:rgba(255,255,255,.1);--obj:#2c2c2a;--made:#3987e5;--broken:#52514e;--force:#d95926}}
:root[data-theme="dark"]{color-scheme:dark;--surface:#1a1a19;--page:#0d0d0d;--ink:#f0efec;--ink2:#c3c2b7;--line:#2c2c2a;
--ring:rgba(255,255,255,.1);--obj:#2c2c2a;--made:#3987e5;--broken:#52514e;--force:#d95926}
body{margin:0;background:var(--page);color:var(--ink);font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:980px;margin:0 auto;padding:24px 16px 48px}h1{font-size:22px;margin:0 0 4px}
p{color:var(--ink2);margin:0 0 12px;max-width:760px}.card{background:var(--surface);border:1px solid var(--ring);border-radius:10px;padding:14px;margin:12px 0}
.views{display:flex;gap:16px;flex-wrap:wrap}.views figure{margin:0;flex:1 1 300px}figcaption{color:var(--ink2);font-size:13px}
svg{width:100%;height:auto;display:block}.ctl{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
button{font:inherit;border:1px solid var(--line);background:transparent;color:var(--ink);border-radius:6px;padding:6px 12px;cursor:pointer}
input[type=range]{flex:1 1 240px}.legend{display:flex;gap:16px;flex-wrap:wrap;color:var(--ink2);font-size:13px;margin-top:8px}
.sw{display:inline-block;width:12px;height:12px;border-radius:50%;vertical-align:-1px;margin-right:6px}
table{border-collapse:collapse;font-size:14px}td,th{padding:3px 10px 3px 0;text-align:left}th{color:var(--ink2);font-weight:600}
</style></head><body><main>
<h1>__TITLE__</h1><p>__SUB__</p>
<div class="card"><div class="views">
<figure><svg id="top" viewBox="-1 -1 2 2"></svg><figcaption>From above</figcaption></figure>
<figure><svg id="side" viewBox="-1 -1 2 2"></svg><figcaption>From the side (x horizontal, up vertical)</figcaption></figure>
</div>
<div class="ctl"><button id="play">Pause</button><input id="t" type="range" min="0" value="0"><span id="clock"></span></div>
<div class="legend"><span><span class="sw" style="background:var(--made)"></span>station in contact (drawn at its contact point)</span>
<span><span class="sw" style="background:var(--broken)"></span>station off the object</span>
<span><span class="sw" style="background:var(--force)"></span>force on the object (arrow)</span></div></div>
<div class="card"><table id="facts"></table></div>
<script>
const L=__DATA__;
const F=L.frames,R=L.object.radius_m;
let span=R;F.forEach(f=>f.station_pos.forEach(p=>{const d=Math.hypot(p[0]-f.obj_pos[0],p[1]-f.obj_pos[1],p[2]-f.obj_pos[2]);if(d>span)span=d;}));
const S=0.85/(span*1.15);let Fmax=1e-9;F.forEach(f=>f.force_on_object_N.forEach(v=>{Fmax=Math.max(Fmax,Math.hypot(...v));}));
const fs=0.35/Fmax;const ns="http://www.w3.org/2000/svg";
function el(t,a,p){const e=document.createElementNS(ns,t);for(const k in a)e.setAttribute(k,a[k]);p.appendChild(e);return e;}
function setup(id){const s=document.getElementById(id);el("circle",{cx:0,cy:0,r:R*S,fill:"var(--obj)",stroke:"var(--line)","stroke-width":.006},s);
 const mark=el("line",{stroke:"var(--ink2)","stroke-width":.008},s);
 const st=[],ar=[];for(let i=0;i<L.n_stations;i++){ar.push(el("line",{stroke:"var(--force)","stroke-width":.012,"stroke-linecap":"round"},s));
 st.push(el("circle",{r:.03,"stroke-width":.008},s));}return{mark,st,ar};}
const vTop=setup("top"),vSide=setup("side");
function qrot(q,v){const[w,x,y,z]=q;const t=[2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])];
 return[v[0]+w*t[0]+(y*t[2]-z*t[1]),v[1]+w*t[1]+(z*t[0]-x*t[2]),v[2]+w*t[2]+(x*t[1]-y*t[0])];}
function draw(k){const f=F[k],c=f.obj_pos;
 const P=(p,view)=>view==="top"?[(p[0]-c[0])*S,-(p[1]-c[1])*S]:[(p[0]-c[0])*S,-(p[2]-c[2])*S];
 for(const[view,g]of[["top",vTop],["side",vSide]]){
  const m=qrot(f.obj_quat,[R,0,0]);const e=P([c[0]+m[0],c[1]+m[1],c[2]+m[2]],view);g.mark.setAttribute("x1",0);g.mark.setAttribute("y1",0);g.mark.setAttribute("x2",e[0]);g.mark.setAttribute("y2",e[1]);
  f.station_pos.forEach((p,i)=>{const on=f.contact[i];const cp0=on&&f.contact_point&&f.contact_point[i]?f.contact_point[i]:p;const q=P(cp0,view);const s=g.st[i];s.setAttribute("cx",q[0]);s.setAttribute("cy",q[1]);
   s.setAttribute("fill",on?"var(--made)":"var(--broken)");s.setAttribute("stroke",on?"var(--made)":"var(--broken)");
   const a=g.ar[i],fv=f.force_on_object_N[i],cp=f.contact_point&&f.contact_point[i]?f.contact_point[i]:p;const o=P(cp,view);
   const d=view==="top"?[fv[0]*fs,-fv[1]*fs]:[fv[0]*fs,-fv[2]*fs];a.setAttribute("x1",o[0]);a.setAttribute("y1",o[1]);a.setAttribute("x2",o[0]+d[0]);a.setAttribute("y2",o[1]+d[1]);
   a.setAttribute("opacity",Math.hypot(...fv)>1e-6?1:0);});}
 document.getElementById("clock").textContent=`t = ${(f.t-F[0].t).toFixed(2)} s  ·  ${f.contact.filter(x=>x).length} of ${L.n_stations} in contact`;}
const sl=document.getElementById("t");sl.max=F.length-1;let k=0,playing=true;
sl.oninput=()=>{k=+sl.value;draw(k);};document.getElementById("play").onclick=e=>{playing=!playing;e.target.textContent=playing?"Pause":"Play";};
const dt=F.length>1?(F[1].t-F[0].t)*1000:40;setInterval(()=>{if(!playing)return;k=(k+1)%F.length;sl.value=k;draw(k);},Math.max(dt,30));draw(0);
const fsum=F.map(f=>f.force_on_object_N.reduce((a,v)=>[a[0]+v[0],a[1]+v[1],a[2]+v[2]],[0,0,0]));
const mz=fsum.reduce((a,v)=>a+v[2],0)/fsum.length;const nc=F.map(f=>f.contact.filter(x=>x).length);
const rows=[["Object",`${L.object.shape}, radius ${(R*1e3).toFixed(1)} mm, ${(L.object.mass_kg*1e3).toFixed(0)} g`],
["Stations",`${L.n_stations}; ${Math.min(...nc)}–${Math.max(...nc)} in contact at a time`],
["Mean upward force",`${mz.toFixed(3)} N (weight ${(L.object.mass_kg*9.81).toFixed(3)} N)`],
["Duration",`${(F[F.length-1].t-F[0].t).toFixed(2)} s, ${F.length} frames`],
["Source",`${L.kind}: ${L.source.repo} ${L.source.preset||""} @ ${(L.source.commit||"").slice(0,7)}; forces from ${L.source.force_source||"?"}`]];
document.getElementById("facts").innerHTML=rows.map(r=>`<tr><th>${r[0]}</th><td>${r[1]}</td></tr>`).join("");
</script></main></body></html>"""


def main():
    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)
    for p in sys.argv[1:]:
        with open(p) as f:
            L = json.load(f)
        sub = ("A leader: where each contact is, when it touches, and the force it "
               "puts on the object. A hand follows it if its fingers can be there "
               "and supply those forces. " + L.get("caveat", ""))
        html = (PAGE.replace("__TITLE__", "Leader: " + L["name"])
                .replace("__SUB__", sub)
                .replace("__DATA__", json.dumps(L, separators=(",", ":"))))
        out = os.path.join(ROOT, "outputs", f"leader_{L['id']}.html")
        with open(out, "w") as f:
            f.write(html)
        print("wrote", out)


if __name__ == "__main__":
    main()
