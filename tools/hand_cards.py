"""Print and save a card per rigid hand: fingers, inputs, torque-limit
source, fingertip reach.

    python tools/hand_cards.py      (needs mujoco and tools/fetch_hands.py)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gramian_workshop.hands import HANDS, hand_card  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    cards, lines = [], []
    for k in HANDS:
        try:
            c = hand_card(k)
        except FileNotFoundError as e:
            lines.append(f"== {k}: skipped ({e})")
            continue
        cards.append(c)
        lines.append(f"== {c['name']}  ({c['model']}): {c['n_joints']} joints, "
                     f"{c['n_actuators']} actuators, {c['couplings']} couplings, "
                     f"shared {c['shared_joints'] or 'none'}")
        for f in c["fingers"]:
            lines.append(f"   {f['finger']:7s} {f['joints']} joints, {f['inputs']} inputs "
                         f"({f['inputs_source']}); torque limits: "
                         f"{f['torque_limits'] or 'none in model'}; reach {f['reach_mm']:.0f} mm")
    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)
    with open(os.path.join(ROOT, "outputs", "hand_cards.json"), "w", encoding="utf-8") as fo:
        json.dump(cards, fo, indent=1)
    txt = "\n".join(lines) + "\n"
    with open(os.path.join(ROOT, "outputs", "hand_cards.txt"), "w", encoding="utf-8") as fo:
        fo.write(txt)
    print(txt)


if __name__ == "__main__":
    main()
