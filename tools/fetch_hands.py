"""Fetch the rigid-hand models into external/ (not committed).

    python tools/fetch_hands.py [--wave-manip ../wave_manip]

- MuJoCo Menagerie (public, Apache-2.0 repository; each model keeps its own
  licence): a sparse, shallow clone of just the hands used here, at the
  commit the stored results were computed with.
- Linker's public L20 URDF (linker-bot/linkerhand-urdf, Apache-2.0), at a
  pinned commit, built into a MuJoCo model with its <mimic> couplings kept
  (older MuJoCo's URDF import drops them). This is hand `l20_public`; it is not the
  lab's L20 (`l20`), whose vendor files differ (see docs/INSTALL_PLAN.md).
- The lab's Linker L20 model lives in Wave_manip (built there from the vendor
  files that came with the hand); it is copied from a local Wave_manip
  checkout if one is given.
"""
import argparse
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXT = os.path.join(ROOT, "external")
MENAGERIE = "https://github.com/google-deepmind/mujoco_menagerie"
MENAGERIE_COMMIT = "b846dd12bc459d776cccb3dee0b1d02acbf7a9c7"   # Wave_manip's sim, 20 May 2026
DIRS = ["wonik_allegro", "shadow_hand", "leap_hand"]
LINKER = "https://github.com/linker-bot/linkerhand-urdf"
LINKER_COMMIT = "49f26021b0a91ff5c33150df22970c07bb548672"
LINKER_DIR = "L20/right"


def run(*cmd, cwd=None):
    print("$", " ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True,
                   env={**os.environ, "GIT_LFS_SKIP_SMUDGE": "1"})


def sparse_clone(url, commit, dirs, dst):
    """Just `dirs` of `url` at `commit`, history depth 1."""
    if os.path.exists(os.path.join(dst, ".git")):
        print(f"{dst} exists; leaving it")
        return
    os.makedirs(dst)
    run("git", "init", "-q", cwd=dst)
    run("git", "remote", "add", "origin", url, cwd=dst)
    run("git", "sparse-checkout", "set", *dirs, cwd=dst)
    run("git", "fetch", "-q", "--depth", "1", "--filter=blob:none", "origin", commit, cwd=dst)
    run("git", "checkout", "-q", "FETCH_HEAD", cwd=dst)


def build_l20_public(src, out):
    """Linker's L20 right-hand URDF -> MJCF, keeping its joint couplings.

    The model is what the URDF says, nothing tuned: joint ranges and the
    effort (written as actuatorfrcrange) as given, collision meshes as
    given, and each <mimic> as a linear joint equality
    (dependent = offset + multiplier * master)."""
    import mujoco
    urdf = os.path.join(src, "linkerhand_l20_right.urdf")
    text = open(urdf, encoding="utf-8").read()
    mimic = {}
    for j in ET.fromstring(text.encode("utf-8")).findall("joint"):
        mm = j.find("mimic")
        if mm is not None:
            mimic[j.get("name")] = (mm.get("joint"), float(mm.get("offset", 0)),
                                    float(mm.get("multiplier", 1)))
    # mesh paths in the URDF are already relative to its folder
    text = re.sub(r"(<robot\b[^>]*>)", r'\1<mujoco><compiler '
                  r'balanceinertia="true" discardvisual="true"/></mujoco>', text, count=1)
    tmp = os.path.join(src, "_mujoco_import.urdf")
    with open(tmp, "w", encoding="utf-8") as fo:
        fo.write(text)
    spec = mujoco.MjSpec.from_file(tmp)
    os.remove(tmp)
    # newer MuJoCo imports <mimic> itself; older versions drop it
    have = {(e.name1, e.name2): list(e.data[:5]) for e in spec.equalities
            if e.type == mujoco.mjtEq.mjEQ_JOINT}
    for dep, (master, off, mul) in sorted(mimic.items()):
        if (dep, master) in have:
            if not all(abs(x - y) < 1e-9 for x, y in zip(have[dep, master], [off, mul, 0, 0, 0])):
                raise ValueError(f"MuJoCo's import of the {dep} coupling differs from the URDF")
            continue
        e = spec.add_equality()
        e.type = mujoco.mjtEq.mjEQ_JOINT
        e.name1, e.name2 = dep, master
        e.name = f"{dep}_coupling"
        e.data[:5] = [off, mul, 0, 0, 0]
    spec.compile()
    os.makedirs(out, exist_ok=True)
    meshes = os.path.join(out, "meshes")
    if os.path.exists(meshes):
        shutil.rmtree(meshes)
    shutil.copytree(os.path.join(src, "meshes"), meshes)
    shutil.copy(os.path.join(os.path.dirname(os.path.dirname(src)), "LICENSE"), out)
    xml = spec.to_xml()
    xml = re.sub(r' meshdir="[^"]*"', "", xml)
    with open(os.path.join(out, "l20_right.xml"), "w", encoding="utf-8") as fo:
        fo.write(f"<!-- Built by tools/fetch_hands.py from {LINKER} at {LINKER_COMMIT}, "
                 f"{LINKER_DIR} (Apache-2.0); couplings from its <mimic> tags. -->\n")
        fo.write(xml)
    m = mujoco.MjModel.from_xml_path(os.path.join(out, "l20_right.xml"))
    print(f"built {out}: {m.njnt} joints, {m.neq} couplings, {m.nu} actuators")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", default=None,
                    help="local Wave_manip checkout (for the lab's L20 model)")
    a = ap.parse_args()
    os.makedirs(EXT, exist_ok=True)
    sparse_clone(MENAGERIE, MENAGERIE_COMMIT, DIRS, os.path.join(EXT, "mujoco_menagerie"))
    linker = os.path.join(EXT, "linkerhand_urdf")
    sparse_clone(LINKER, LINKER_COMMIT, [LINKER_DIR], linker)
    try:
        import mujoco  # noqa: F401
    except ImportError:
        print("mujoco not installed (pip install -e .[hands]): l20_public not built")
    else:
        build_l20_public(os.path.join(linker, *LINKER_DIR.split("/")),
                         os.path.join(EXT, "linker_l20_public"))
    if a.wave_manip:
        src = os.path.join(a.wave_manip, "experiment_log", "models",
                           "linker_l20_right")
        out = os.path.join(EXT, "wave_manip_models", "linker_l20_right")
        if os.path.exists(out):
            shutil.rmtree(out)
        shutil.copytree(src, out)
        print(f"copied {src} -> {out}")
    else:
        print("no --wave-manip given: the lab's L20 (l20) is unavailable; "
              "l20_public is Linker's published model")


if __name__ == "__main__":
    main()
