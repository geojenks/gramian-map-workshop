# Renders cut e's three parts as layers for slides (tools/render_film_layers.py): pictures without the panel's words,
# plus a timeline of the words and the snapshot tiles. Same arguments as render_commands_e.sh and render_commands_d.sh.
PY=/c/Users/geo/Documents/Projects/ARIA/Wave_manip/inhand/Scripts/python.exe
WM=C:/Users/geo/Documents/Projects/ARIA/Wave_manip
cd /c/Users/geo/Documents/Projects/gramian-map-workshop
export PATH="$PATH:/c/Program Files/ffmpeg/bin" GMW_MENAGERIE="C:/Users/geo/Documents/Projects/gramian-map-workshop-install/external/mujoco_menagerie" PYTHONIOENCODING=utf-8
F=outputs/funder_film; L=$F/slides_layers_9nov; R=$F/inputs_7nov/hora
$PY -u tools/render_film_layers.py $L/hora hora --wave-manip $WM --out $L/hora/hora.mp4 --run $R/follow_leap__hora_seed42_five_snapshots.json --leader-json $R/hora_seed42_five_snapshots.json --fit-az 120 --fit-dist .34 > $L/hora.log 2>&1 &
for H in allegro shadow; do
  $PY -u tools/render_film_layers.py $L/human_$H human --video --checks $F/checks_9nov --fit-hand $H --hold 2.5 --fit-seconds 3 --end 7 --fit-dist .55 --phone-view near --fit-lift 0 --hide-turn-behind-ball --wave-manip $WM --out $L/human_$H/human_$H.mp4 > $L/human_$H.log 2>&1 &
done
wait
tail -n 2 $L/*.log
echo "=== all finished"
