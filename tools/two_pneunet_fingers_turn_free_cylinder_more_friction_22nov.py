"""Free cylinder at the two masses that fell, with more friction between tip and cylinder (22 Nov).

SIMULATION ONLY (MuJoCo, contact and friction); comparative, not predictive.

Question: the free cylinder fell at the recorded mass (23.94 g) and at half of it (11.97 g) with the
recording's friction, 0.72. Is it held and turned at those masses if the tips grip better?

Fixed here before the first run, not tuned afterwards:
  - friction, tip to cylinder: 1.0 and 1.5 (ASSUMED values; 0.72 is the run already made);
  - masses: the recorded mass and half of it, the two that fell;
  - everything else, and the HELD and TURNS rules, exactly as in
    two_pneunet_fingers_turn_free_cylinder_lighter_22nov.py, whose main() is run unchanged with
    the friction and the mass list replaced. It also repeats its no-gravity gate at each friction.

Writes one new folder per friction:
  outputs/two_pneunet_fingers_turn_free_cylinder_lighter_22nov_friction_<value>/

Usage: python <this> --wave-manip <path> --gramian-map <path>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import two_pneunet_fingers_turn_free_cylinder_lighter_22nov as F   # noqa: E402

FRICTIONS = (1.0, 1.5)          # FIXED, see the header
F.MASS_SHARES = (1.0, 1 / 2)    # FIXED, see the header

base_args = sys.argv[1:]
real_setup = F.PI.setup
for mu in FRICTIONS:
    def setup(args, mu=mu):
        S = real_setup(args)
        S["L"] = dict(S["L"])
        S["L"]["mu"] = mu
        return S
    F.PI.setup = setup
    sys.argv = [sys.argv[0]] + base_args + ["--tag", f"friction_{mu:.2f}".replace(".", "p")]
    print(f"=== friction {mu:.2f} (ASSUMED) ===", flush=True)
    F.main()
