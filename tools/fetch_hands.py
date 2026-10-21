"""Fetch the rigid-hand models into external/ (not committed).

    python tools/fetch_hands.py [--wave-manip ../wave_manip]

- MuJoCo Menagerie (public, Apache-2.0 repository; each model keeps its own
  licence): a sparse, shallow clone of just the hands used here.
- The Linker L20 model lives in Wave_manip (built there from the vendor
  URDF); it is copied from a local Wave_manip checkout if one is given.
"""
import argparse
import os
import shutil
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXT = os.path.join(ROOT, "external")
MENAGERIE = "https://github.com/google-deepmind/mujoco_menagerie"
DIRS = ["wonik_allegro", "shadow_hand", "leap_hand"]


def run(*cmd, cwd=None):
    print("$", " ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True,
                   env={**os.environ, "GIT_LFS_SKIP_SMUDGE": "1"})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", default=None,
                    help="local Wave_manip checkout (for the L20 model)")
    a = ap.parse_args()
    os.makedirs(EXT, exist_ok=True)
    dst = os.path.join(EXT, "mujoco_menagerie")
    if not os.path.exists(os.path.join(dst, ".git")):
        run("git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
            MENAGERIE, dst)
        run("git", "sparse-checkout", "set", *DIRS, cwd=dst)
    else:
        print(f"{dst} exists; leaving it")
    if a.wave_manip:
        src = os.path.join(a.wave_manip, "experiment_log", "models",
                           "linker_l20_right")
        out = os.path.join(EXT, "wave_manip_models", "linker_l20_right")
        if os.path.exists(out):
            shutil.rmtree(out)
        shutil.copytree(src, out)
        print(f"copied {src} -> {out}")
    else:
        print("no --wave-manip given: the L20 will be unavailable")


if __name__ == "__main__":
    main()
