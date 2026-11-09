# Rebuilds position_turning_film_rough_7nov_b.mp4: same as position_turning_film_rough_7nov.mp4, with the red dot on the Allegro snapshots explained on screen.
# The Hora part is unchanged, parts/hora.mp4, built by render_commands.sh. Run from anywhere in Git Bash.
PY=/c/Users/geo/Documents/Projects/ARIA/Wave_manip/inhand/Scripts/python.exe
WM=C:/Users/geo/Documents/Projects/ARIA/Wave_manip
cd /c/Users/geo/Documents/Projects/gramian-map-workshop
export PATH="$PATH:/c/Program Files/ffmpeg/bin" GMW_MENAGERIE="C:/Users/geo/Documents/Projects/gramian-map-workshop-install/external/mujoco_menagerie" PYTHONIOENCODING=utf-8
F=outputs/funder_film; T=$F/inputs_position_turning_7nov/parts
$PY -u tools/render_force_film.py human --video --checks $F/checks_7nov_b --fit-hand allegro --hold 2.5 --fit-seconds 3 --end 7 --fit-dist .55 --wave-manip $WM --out $T/human_b.mp4
echo "=== human finished"
printf "file 'hora.mp4'\nfile 'human_b.mp4'\n" > $T/list_b.txt
OUT=$F/position_turning_film_rough_7nov_b.mp4
test -e "$OUT" && { echo "already exists, not overwriting"; exit 1; }
ffmpeg -loglevel error -f concat -safe 0 -i $T/list_b.txt -c copy "$OUT"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT"
echo "=== all finished"
