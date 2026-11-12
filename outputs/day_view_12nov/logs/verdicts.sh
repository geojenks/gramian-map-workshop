# Repeat the three verdict steps of the 11 Nov run with today's code, into scratch.
S="<scratch>"
PM="<hands env>/Scripts/python.exe"
PA="<analysis env>/Scripts/python.exe"
cd "<clone>" || exit 1
R=outputs/run_through_11nov
mkdir -p "$S/results" "$S/added_hands"
export GMW_ADDED_HANDS="$S/added_hands"
step() { n="$1"; shift; echo "=== $n"; echo "\$ $*"; t=$(date +%s); "$@" 2>&1; echo "--- $n: exit $? in $(( $(date +%s)-t )) s"; }
step add_allegro "$PM" tools/add_hand.py external/mujoco_menagerie/wonik_allegro/left_hand.xml $R/hand_descriptions/allegro_left.json
step add_mosadegh "$PM" tools/add_hand.py $R/hands/mosadegh2014_fPN_from_curves_x3_ring/hand.xml $R/hand_descriptions/mosadegh_x3_ring.json
step add_flipped "$PM" tools/add_hand.py $R/hands/mosadegh2014_fPN_from_curves_x3_ring_flipped/hand.xml $R/hand_descriptions/mosadegh_x3_ring_flipped.json
COPIED=""
for h in mosadegh_x3_ring mosadegh_x3_ring_flipped; do
  f=data/leaders/hold_${h}__self__selfoverlapfix.json
  if [ ! -e "$f" ]; then cp $R/leaders/hold_${h}__self__selfoverlapfix.json "$f"; COPIED="$COPIED $f"; fi
done
for h in allegro_left mosadegh_x3_ring mosadegh_x3_ring_flipped; do
  step follow_$h "$PM" tools/run_follow.py --hands $h --leader-id hold_${h}__self__selfoverlapfix --placement-from $R/results/hold_placement_${h}__selfoverlapfix.json --tag fixed --margins --out-dir "$S/results"
  step failures_$h "$PA" tools/report_failures.py "$S/results/follow_${h}__hold_${h}__self__selfoverlapfix__fixed.json" --out-dir "$S/results"
done
step view "$PA" tools/view_follow.py "$S"/results/follow_allegro_left__*__fixed.json "$S"/results/follow_mosadegh_x3_ring__*__fixed.json "$S"/results/follow_mosadegh_x3_ring_flipped__*__fixed.json --out "$S/results/follow_today.html" --table "$S/results/follow_today.md"
for f in $COPIED; do rm "$f"; done
git status --short data | head
echo ALLDONE
