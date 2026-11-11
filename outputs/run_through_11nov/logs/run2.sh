PM="<hands env>/Scripts/python.exe"
PA="<analysis env>/Scripts/python.exe"
export GMW_ADDED_HANDS="$PWD/rt_out/added_hands"
step() { n="$1"; shift; echo "=== $n"; echo "\$ $*"; t=$(date +%s); "$@" 2>&1; echo "--- $n: exit $? in $(( $(date +%s)-t )) s"; }
for h in mosadegh_x3_ring allegro_left; do
step follow_$h "$PM" tools/run_follow.py --hands $h --leader-id hold_${h}__self__selfoverlapfix --placement-from rt_out/results/hold_placement_${h}__selfoverlapfix.json --tag fixed --margins --out-dir rt_out/results
step failures_$h "$PA" tools/report_failures.py rt_out/results/follow_${h}__hold_${h}__self__selfoverlapfix__fixed.json --out-dir rt_out/results
done
step view "$PA" tools/view_follow.py rt_out/results/follow_mosadegh_x3_ring__hold_mosadegh_x3_ring__self__selfoverlapfix__fixed.json rt_out/results/follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json --out rt_out/results/follow_runthrough.html --table rt_out/results/follow_runthrough.md
ls rt_out/results
echo ALLDONE
