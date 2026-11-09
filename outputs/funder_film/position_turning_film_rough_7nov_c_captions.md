# Captions: position_turning_film_rough_7nov_c.mp4

Built by `tools/render_force_film.py` from logged data. Rough cut, about 75 s. Times are approximate.

The two halves use different checks. The Hora half checks position and force targets. The human half checks position, and separately whether the touching fingers can turn the ball, because the forces we can derive there show the weight shared out, not the turn.

| Time | Shot | Caption and data source |
|---|---|---|
| 0 to 20 s | Hora replay, with five snapshots saved | Hora, a published learned policy on an Allegro hand, turns a cylinder in simulation. We replay it from its log. Normal forces are read from the simulator's contacts. Shear is derived from the object's motion and shared between fingers; this is one consistent split, not the only one. Source: `data/rl_logs/hora_seed42_30s*.npz`, split by `tools/hora_force_split.py`. |
| 20 to 30 s | LEAP fitted to the five Hora snapshots | Each snapshot is frozen and checked on its own against the saved position and force targets. No hand is simulated doing the turn. Poses are schematic: parts of the hand pass through the cylinder and through each other. Source: `inputs_7nov/hora/follow_leap__hora_seed42_five_snapshots.json`, from `tools/run_follow_snapshots.py`, which mirrors the per-frame loop of `follow.py` with a sphere at each contact. It is not the banked check. |
| 30 to 53 s | Video of a person turning a tennis ball, pausing at five snapshots with the tracked hand and ball | Recorded video at normal speed. The wireframe is the joints the cameras tracked; hidden joints are left out. The turn is measured from the tracked ball, relative to the palm. No sensor measured forces here. Source: `data/leaders/tennis_new_cup_take3_handframe__nframesfix.json`. |
| 53 to 68 s | Allegro fitted to the five human snapshots | Two separate checks on each frozen snapshot. The position check is `follow.py` with the forces set to zero. The turning check asks whether the touching fingers can hold the ball inside friction and joint strength, and how much torque they have to spare about its turn. The ball's mass and friction are assumed. The check places the hand freely, so here it holds the ball from the opposite side to the person. Poses are schematic. Red dot: a contact with no finger to match. The panel warns that torque to spare can be overstated, because the model sets no limit on the load a joint end stop carries. Source: `checks_7nov_b/position_turning_allegro.json` and `checks_7nov_b/follow_allegro__*position_only.json`, from `tools/position_turning_check.py`. |
| 68 to 75 s | Counts for every frame, Allegro, LEAP and Shadow | 120 frames across the take, each frozen and checked on its own. Red means not by this method at this pose, not that the hand can never do the task. Green in the turning check is a low bar: the turn itself needs little torque. The turning check was built for this film and is not the banked method. Source: `checks_7nov_b/position_turning_{allegro,leap,shadow}.json`. |

Simulated and derived magnitudes are comparative, not predictive.

## Limits of the turning check

- The torque to spare is much larger than the torque the turn needs, so its colours are for comparing hands and poses, not for predicting performance.
- Joint end stops carry load without an upper limit in the model the check shares with `follow.py`. This can inflate the torque to spare. A touching finger has a joint at an end stop in 59 of the 93 frames Allegro can hold, 12 of 98 for LEAP and 33 of 36 for Shadow. These counts come from a separate test run of the check, not its results files.
- Contacts with no finger are left out of the turning check.

## Rebuilding

Inputs are in `inputs_7nov/` and `checks_7nov_b/`. Commands are in `inputs_position_turning_7nov/render_commands_c.sh`. The film needs no scipy. The turning check does; we installed it for that run only, into a temporary folder on `PYTHONPATH`.
