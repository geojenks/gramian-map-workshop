PM="<hands env>/Scripts/python.exe"
PA="<analysis env>/Scripts/python.exe"
step() { n="$1"; shift; echo "=== $n"; echo "\$ $*"; t=$(date +%s); "$@" 2>&1; echo "--- $n: exit $? in $(( $(date +%s)-t )) s"; }
mkdir -p rt_out/r0
step curves_to_record "$PA" tools/curves_to_record.py data/test_curves/mosadegh2014_fPN.csv --out rt_out/r0/mosadegh_from_curves.json
step check_curves_record "$PA" tools/check_record.py rt_out/r0/mosadegh_from_curves.json
step table_to_record "$PA" tools/joint_table_to_record.py data/joint_tables/shadow_hand_finger.csv --out rt_out/r0/shadow_finger_from_table.json
step check_table_record "$PA" tools/check_record.py rt_out/r0/shadow_finger_from_table.json
mk() { mkdir -p data/hands/$1; sed "s#\"name\": \"wall_0.0010m_x3_ring\"#\"name\": \"$1\"#; s#\"record\": \"wall_0.0010m\"#\"record\": \"$2\"#" data/hands/wall_0.0010m_x3_ring/hand.json > data/hands/$1/hand.json; }
mk rt0_curves_by_id mosadegh2014_fPN_from_curves
mk rt0_curves_by_path rt_out/r0/mosadegh_from_curves.json
mk rt0_table_by_path rt_out/r0/shadow_finger_from_table.json
step build_curves_record_by_id "$PM" tools/build_record_hand.py data/hands/rt0_curves_by_id/hand.json
step build_curves_record_by_path "$PM" tools/build_record_hand.py data/hands/rt0_curves_by_path/hand.json
step build_table_record_by_path "$PM" tools/build_record_hand.py data/hands/rt0_table_by_path/hand.json
step gramians_mujoco_python "$PM" tools/figure_gramians.py --records data/records --out-dir rt_out/r0/gramians
step gramians_analysis_python "$PA" tools/figure_gramians.py --records data/records --out-dir rt_out/r0/gramians
echo ALLDONE
