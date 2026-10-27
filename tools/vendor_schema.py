"""Copy the record schema and its reference validator, byte for byte, from
Gramain_extraction into this repo, with a manifest of commit and sha256.

    python tools/vendor_schema.py --gramian-map ../Gramain_extraction/gramian_map

gramian_workshop/vendor/actuator_record.py  <- gramian_map/actuator_record.py
docs/RECORD_SCHEMA.md                       <- gramian_map/docs/RECORD_SCHEMA.md
Neither is edited here; tests/test_gates.py checks the hashes.
"""
import argparse
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _provenance import source_entry  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = {
    "gramian_workshop/vendor/actuator_record.py": "gramian_map/actuator_record.py",
    "docs/RECORD_SCHEMA.md": "gramian_map/docs/RECORD_SCHEMA.md",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gramian-map", required=True)
    a = ap.parse_args()
    repo = os.path.dirname(os.path.abspath(a.gramian_map))
    man = {}
    for dst, src in FILES.items():
        shutil.copyfile(os.path.join(repo, src), os.path.join(ROOT, dst))
        man[dst] = {"source": "geojenks/Gramain_extraction", **source_entry(repo, src)}
        print(f"copied {src} -> {dst}")
    with open(os.path.join(ROOT, "gramian_workshop", "vendor",
                           "SCHEMA_MANIFEST.json"), "w", encoding="utf-8") as f:
        json.dump(man, f, indent=1)


if __name__ == "__main__":
    main()
