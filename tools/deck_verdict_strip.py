"""Draw the per-frame verdict strips used in the group-meeting deck, from
stored follow results in outputs/. Prints the colour counts per row.

    python tools/deck_verdict_strip.py <out.png>
"""
import json, sys, os, collections
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); OUT = sys.argv[1]
INK="#1f2933"; MUTE="#6b7682"
COL={"green":"#2e9e5b","amber":"#e0a100","red":"#d1453b"}
rows=[("Our measured hand, on our own demo","follow_george_measured_thumb__tennis_new_cup_take3_derived__tan.json"),
      ("Our measured hand, simulated cup","follow_george_measured_thumb__w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5__respaced__side_tan.json"),
      ("Literature-proportioned hand, simulated cup","follow_human_vc_arch_prop__w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5__respaced__side_tan.json"),
]
plt.rcParams.update({"font.family":"DejaVu Sans"})
fig, ax = plt.subplots(figsize=(16,3.6))
for j,(lab,p) in enumerate(rows):
    d=json.load(open(os.path.join(R,"outputs",p))); vs=[x["verdict"] for x in d["frames"]]; n=len(vs)
    for i,v in enumerate(vs): ax.add_patch(plt.Rectangle((i/n,-j-0.4),1/n*0.92,0.8,color=COL[v]))
    print(lab, dict(collections.Counter(vs)), "of", n)
    ax.text(-0.01,-j,lab,ha="right",va="center",fontsize=20,color=INK)
ax.set_xlim(0,1); ax.set_ylim(-len(rows)+0.4,0.6); ax.axis("off")
ax.text(1.0,0.75,"time along the task →",ha="right",fontsize=18,color=MUTE)
fig.subplots_adjust(left=0.40,right=0.99,top=0.9,bottom=0.02); fig.savefig(OUT,dpi=110,transparent=True); plt.close(fig)
