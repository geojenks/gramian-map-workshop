"""Copy the gramian_map actuator records into data/records/ with a manifest.

    python tools/import_records.py --gramian-map ../Gramain_extraction/gramian_map

Records are copied byte for byte (never edited). The manifest pins each one
to its source commit and sha256, and carries the trust flags the source
project keeps in sim_status.json (e.g. the 55 deg McKibben's floor-noise
flag), so the workshop can show them rather than quietly clean them.
"""
import argparse
import glob
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _provenance import git_commit, source_entry  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "data", "records")


def quality_flags(gm):
    """id -> {regime: quality} for every regime whose quality is set."""
    path = os.path.join(gm, "sim_status.json")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        sims = json.load(f).get("simulations", [])
    out = {}
    for s in sims:
        q = {r: v["quality"] for r, v in (s.get("regimes") or {}).items()
             if isinstance(v, dict) and v.get("quality")}
        if q:
            out[s["id"]] = q
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gramian-map", required=True,
                    help="path to Gramain_extraction/gramian_map")
    a = ap.parse_args()
    gm = os.path.abspath(a.gramian_map)
    repo = os.path.dirname(gm)
    os.makedirs(OUT, exist_ok=True)
    flags = quality_flags(gm)
    manifest = {"source": "geojenks/Gramain_extraction gramian_map/records",
                "source_head": git_commit(repo),
                "sim_status": source_entry(repo, "gramian_map/sim_status.json"),
                "records": {}}
    for src in sorted(glob.glob(os.path.join(gm, "records", "*.json"))):
        name = os.path.basename(src)
        shutil.copyfile(src, os.path.join(OUT, name))
        with open(src) as f:
            rid = json.load(f)["id"]
        manifest["records"][rid] = {
            "file": name,
            **source_entry(repo, os.path.join("gramian_map", "records", name)),
            "quality": flags.get(rid, {}),
        }
    with open(os.path.join(OUT, "MANIFEST.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1)
    print(f"copied {len(manifest['records'])} records -> {OUT}")
    for rid, e in manifest["records"].items():
        if e["quality"]:
            print(f"  flag {rid}: {e['quality']}")


if __name__ == "__main__":
    main()
