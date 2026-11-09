S=C:/Users/geo/Documents/Projects/gramian-map-workshop/outputs/funder_film/inputs_7nov   # where the check results and the scratch parts live
PY=/c/Users/geo/Documents/Projects/ARIA/Wave_manip/inhand/Scripts/python.exe
WM=C:/Users/geo/Documents/Projects/ARIA/Wave_manip
cd /c/Users/geo/Documents/Projects/gramian-map-workshop
export PATH="$PATH:/c/Program Files/ffmpeg/bin" GMW_MENAGERIE="C:/Users/geo/Documents/Projects/gramian-map-workshop-install/external/mujoco_menagerie" PYTHONIOENCODING=utf-8
H=$S/human; R=$S/hora
$PY -u tools/render_force_film.py hora --wave-manip $WM --out $S/cut7_hora.mp4 --run $R/follow_shadow__hora_seed42_five_snapshots.json --leader-json $R/hora_seed42_five_snapshots.json
echo "=== hora finished"
$PY -u tools/render_force_film.py human --video --wave-manip $WM --out $S/cut7_human.mp4 --runs $H/follow_shadow__tennis_new_cup_take3_handframe__nframesfix.json $H/follow_allegro__tennis_new_cup_take3_handframe__nframesfix.json $H/follow_leap__tennis_new_cup_take3_handframe__nframesfix.json
echo "=== human finished"
cd "$(cygpath -u "$S")"
printf "file 'cut7_hora.mp4'\nfile 'cut7_human.mp4'\n" > cut7_list.txt
OUT=C:/Users/geo/Documents/Projects/gramian-map-workshop/outputs/funder_film/force_targets_film_rough_7nov.mp4
test -e "$OUT" && { echo "already exists, not overwriting"; exit 1; }
ffmpeg -loglevel error -f concat -safe 0 -i cut7_list.txt -c copy "$OUT"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT"
for t in 2 12 24 30 34 38 44 50 56 60 64; do ffmpeg -y -loglevel error -ss $t -i "$OUT" -frames:v 1 -vf scale=800:-1 still/cut7_$t.png; done
echo "=== all finished"
