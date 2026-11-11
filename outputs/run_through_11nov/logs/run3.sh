PM="<hands env>/Scripts/python.exe"
PA="<analysis env>/Scripts/python.exe"
step() { n="$1"; shift; echo "=== $n"; echo "\$ $*"; t=$(date +%s); "$@" 2>&1; echo "--- $n: exit $? in $(( $(date +%s)-t )) s"; }
h=mosadegh_x3_ring_flipped; d=data/hands/mosadegh2014_fPN_from_curves_x3_ring_flipped
step build "$PM" tools/build_record_hand.py $d/hand.json
step add "$PM" tools/add_hand.py $d/hand.xml rt_out/$h.json
step hold "$PM" tools/make_hold_leader.py $h --out-dir rt_out/leaders --results-dir rt_out/results
cp rt_out/leaders/hold_${h}__self__selfoverlapfix.json data/leaders/
step follow "$PM" tools/run_follow.py --hands $h --leader-id hold_${h}__self__selfoverlapfix --placement-from rt_out/results/hold_placement_${h}__selfoverlapfix.json --tag fixed --margins --out-dir rt_out/results
step failures "$PA" tools/report_failures.py rt_out/results/follow_${h}__hold_${h}__self__selfoverlapfix__fixed.json --out-dir rt_out/results
echo ALLDONE
