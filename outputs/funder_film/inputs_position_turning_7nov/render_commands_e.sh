# Rebuilds position_turning_film_rough_9nov_e.mp4: cut d's Hora part (LEAP fit seen from the Hora replay camera),
# then the human part with Allegro checked against the four contacts it has fingers for, fingers paired by name
# (checks_9nov), seen from the phone's side. Also renders the alternative Shadow part, not in the film.
# Run from anywhere in Git Bash.
PY=/c/Users/geo/Documents/Projects/ARIA/Wave_manip/inhand/Scripts/python.exe
WM=C:/Users/geo/Documents/Projects/ARIA/Wave_manip
cd /c/Users/geo/Documents/Projects/gramian-map-workshop
export PATH="$PATH:/c/Program Files/ffmpeg/bin" GMW_MENAGERIE="C:/Users/geo/Documents/Projects/gramian-map-workshop-install/external/mujoco_menagerie" PYTHONIOENCODING=utf-8
F=outputs/funder_film; T=$F/inputs_position_turning_7nov/parts
OUT=$F/position_turning_film_rough_9nov_e.mp4
test -e "$OUT" && { echo "already exists, not overwriting"; exit 1; }
for H in allegro shadow; do
  P=$T/human_${H}_near_e2.mp4
  test -e $P && { echo "$P exists, not overwriting"; exit 1; }
  $PY -u tools/render_force_film.py human --video --checks $F/checks_9nov --fit-hand $H --hold 2.5 --fit-seconds 3 --end 7 --fit-dist .55 --phone-view near --fit-lift 0 --hide-turn-behind-ball --wave-manip $WM --out $P > $T/human_${H}_near_e2.log 2>&1
  grep -v "^frames" $T/human_${H}_near_e2.log | tail -n 2
  echo "=== $H finished"
done
printf "file 'hora_d.mp4'\nfile 'human_allegro_near_e2.mp4'\n" > $T/list_e.txt
ffmpeg -loglevel error -f concat -safe 0 -i $T/list_e.txt -c copy "$OUT"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT"
$PY tools/count_position_turning.py $F/checks_9nov
echo "=== all finished"
