LEAP on the five Hora snapshots, with fingertip contacts limited to the soft pad mesh (tools/run_follow_snapshots_leap_pads.py, 9 Nov, 5,968 s).

Result: identical to the original check (inputs_7nov/hora/follow_leap__hora_seed42_five_snapshots.json): same placement, same pairing, same per-finger contact points and verdicts in every snapshot.
Why: the pad-only run can only touch the fingertip at pad points, and it chose exactly the points the original chose. So every fingertip touch in the original check was already on the pad. The pad rule did take effect (13 pad candidate points per finger, 16 on the thumb, against 23 and 27 on the whole fingertip), it just did not change the outcome.
The film keeps the original Hora part (parts/hora_d.mp4).
_unlabelled_follow_leap__... is the same result before the note on contact candidates was added.
