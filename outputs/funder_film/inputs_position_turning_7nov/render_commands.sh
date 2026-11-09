# Rebuilds position_turning_film_rough_7nov.mp4. Run from anywhere in Git Bash.
# The turning check itself needs scipy alongside mujoco and numpy; the film does not. Its results are in checks_7nov_b.
PY=/c/Users/geo/Documents/Projects/ARIA/Wave_manip/inhand/Scripts/python.exe
WM=C:/Users/geo/Documents/Projects/ARIA/Wave_manip
cd /c/Users/geo/Documents/Projects/gramian-map-workshop
export PATH="$PATH:/c/Program Files/ffmpeg/bin" GMW_MENAGERIE="C:/Users/geo/Documents/Projects/gramian-map-workshop-install/external/mujoco_menagerie" PYTHONIOENCODING=utf-8
F=outputs/funder_film; R=$F/inputs_7nov/hora; T=$F/inputs_position_turning_7nov/parts; mkdir -p $T
$PY -u tools/render_force_film.py hora --wave-manip $WM --out $T/hora.mp4 --run $R/follow_leap__hora_seed42_five_snapshots.json --leader-json $R/hora_seed42_five_snapshots.json --fit-az 120 --fit-el 20 --fit-dist .56
echo "=== hora finished"
$PY -u tools/render_force_film.py human --video --checks $F/checks_7nov_b --fit-hand allegro --hold 2.5 --fit-seconds 3 --end 7 --fit-dist .55 --wave-manip $WM --out $T/human.mp4
echo "=== human finished"
printf "file 'hora.mp4'\nfile 'human.mp4'\n" > $T/list.txt
OUT=$F/position_turning_film_rough_7nov.mp4
test -e "$OUT" && { echo "already exists, not overwriting"; exit 1; }
ffmpeg -loglevel error -f concat -safe 0 -i $T/list.txt -c copy "$OUT"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT"
echo "=== all finished"
