"""Print a summary line per leader file: stations, contacts, rotation rate,
force balance, object height.

    python tools/leader_stats.py data/leaders/*.json
"""
import json, sys, numpy as np
for p in sys.argv[1:]:
    L=json.load(open(p)); F=L['frames']; W=L['object']['mass_kg']*9.81
    S=np.array([np.sum(r['force_on_object_N'],axis=0) for r in F]); q=np.array([r['obj_quat'] for r in F]); t=np.array([r['t'] for r in F])
    yaw=np.unwrap(2*np.arctan2(q[:,3],q[:,0])); z=np.array([r['obj_pos'][2] for r in F])*1e3
    nc=[sum(r['contact']) for r in F]
    print(f"{L['id']}: {L['n_stations']} stations, {min(nc)}-{max(nc)} touching; rate {np.degrees(np.polyfit(t,yaw,1)[0]):.1f} deg/s; mean up-force {S[:,2].mean():.3f} vs weight {W:.3f}; ball z {z.min():.1f}-{z.max():.1f} mm (start {z[0]:.1f})")
