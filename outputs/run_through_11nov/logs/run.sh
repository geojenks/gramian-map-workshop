PM="<hands env>/Scripts/python.exe"
export GMW_ADDED_HANDS="$PWD/rt_out/added_hands"
step() { n="$1"; shift; echo "=== $n"; echo "\$ $*"; t=$(date +%s); "$@" 2>&1; echo "--- $n: exit $? in $(( $(date +%s)-t )) s"; }
step add_mosadegh "$PM" tools/add_hand.py data/hands/mosadegh2014_fPN_from_curves_x3_ring/hand.xml rt_out/mosadegh_x3_ring.json
step add_allegro "$PM" tools/add_hand.py external/mujoco_menagerie/wonik_allegro/left_hand.xml rt_out/allegro_left.json
step list "$PM" tools/add_hand.py --list
step check_leader_sim "$PM" tools/check_leader.py data/leaders/w10_cup_physical_x0.1_a0.15_n4__contactfix.json
step check_leader_human "$PM" tools/check_leader.py data/leaders/tennis_new_cup_take3_handframe__nframesfix.json
step gramians "$PM" tools/figure_gramians.py --records data/records --out-dir rt_out/gramians
step hold_mosadegh "$PM" tools/make_hold_leader.py mosadegh_x3_ring --out-dir rt_out/leaders --results-dir rt_out/results
step hold_allegro "$PM" tools/make_hold_leader.py allegro_left --out-dir rt_out/leaders --results-dir rt_out/results
ls rt_out/leaders rt_out/results
echo ALLDONE
