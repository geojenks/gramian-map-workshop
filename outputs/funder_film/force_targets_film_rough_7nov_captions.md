# Captions: force_targets_film_rough_7nov.mp4

Built by `tools/render_force_film.py` from logged data. Rough cut, 65 s. Times are approximate.

| Time | Shot | Caption and data source |
|---|---|---|
| 0–20 s | Hora replay on Allegro, five snapshots saved | A published learned policy turns a cylinder in simulation. Hand and cylinder come from the replayed log `data/rl_logs/hora_seed42_30s*.npz`; normal forces come from the simulator's contacts; shear is derived from the object's motion and shared between fingers by `tools/hora_force_split.py`, and that share is one consistent split, not the only one. |
| 20–30 s | Shadow against the five Hora snapshots | Each snapshot is checked on its own by `tools/run_follow_snapshots.py`, which mirrors `follow.py`'s frame loop with a sphere at each contact. This is not the banked check. No hand is simulated doing the turn. The pose is schematic and parts pass through the cylinder and the hand itself. |
| 30–51 s | Phone video of a person turning a tennis ball | Recorded video, take 3 of `tennis_new_cup`, with tracked joints from `tennis_new_cup_take3_keypoints.json`. Forces are derived from the ball's tracked motion with an assumed mass and friction, from `data/leaders/tennis_new_cup_take3_handframe__nframesfix.json`. Nothing here is measured by a sensor. |
| 51–61 s | LEAP against the five video snapshots | The `follow.py` check on the palm-frame file above (the method is banked; these results are not). Frozen poses only. The check places the hand freely, so LEAP holds the ball from the opposite side to the person: its palm is 72 to 89 mm in front of the ball as the phone sees it, and the person's is 83 to 95 mm behind. The pose is schematic and parts pass through the ball and the hand itself. |
| 61–65 s | Result of every checked frame, three hands | Stored `follow.py` results for Shadow, Allegro and LEAP over 120 frames of the take. Red means not by this method at this pose, not that the hand can never do the task. |

Simulated and derived magnitudes are comparative, not predictive.

## Rebuilding

Check results and the five-snapshot Hora file: `outputs/funder_film/inputs_7nov/`. Render and join commands: `outputs/funder_film/inputs_7nov/render_commands_7nov.sh`.
