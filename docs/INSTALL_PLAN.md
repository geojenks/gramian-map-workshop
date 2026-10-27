# Plan: installing the workshop tool from a link

Written 27 Oct 2026. It sits beside `docs/MINIMAL_PRODUCT_PLAN.md`, which says
what the product is; this says how an outside person gets it running.

**Decided and built (27 Oct 2026).** Licence Apache-2.0 (`LICENSE`). Linker's
public L20 is added as its own hand, `l20_public`, built by
`tools/fetch_hands.py`; `l20` still means the lab's hand. Routes: clone and
`pip install -e`, checked by `.github/workflows/install-check.yml` on Linux
and Windows, and a project page (`site/index.html`, published by
`.github/workflows/pages.yml`). No Docker image. The rest of this file is the
plan as first written; where it differs, this paragraph wins.

## Where it stands

- The core (Demos 1 to 3, the record form, the record checker) needs only
  numpy. It already runs from a clone.
- The rigid hands need `mujoco` (the `hands` extra) and the hand model files.
  `tools/fetch_hands.py` fetches the Menagerie hands from GitHub. The Linker
  L20 comes only from a local Wave_manip checkout.
- The human-finger hands are our own files in `data/hands/`. They need nothing
  extra.
- The repository is public but has **no licence file**. Without one, nobody
  outside may legally reuse the code.

## How the hand models are obtained

| Hand | Source | Licence | How an outsider gets it |
|---|---|---|---|
| Allegro | MuJoCo Menagerie, `wonik_allegro` | BSD (SimLab, 2016) | `fetch_hands.py`, no change |
| Shadow | MuJoCo Menagerie, `shadow_hand` | Apache-2.0 | `fetch_hands.py`, no change |
| LEAP | MuJoCo Menagerie, `leap_hand` | MIT | `fetch_hands.py`, no change |
| Human-finger hands | ours, `data/hands/` | our licence | in the repo already |
| Linker L20 (the lab's hand) | vendor folder that came with the hardware | none stated | not available; see below |

The Menagerie hands are downloaded on install, not copied into our
repository or image, so we redistribute nothing and need no notices. If the
Docker image bakes them in, the image must carry each model's `LICENSE` file
(the sparse clone already includes them).

**The L20 cannot be replaced by Linker's public model.** Linker publishes an
L20 URDF under Apache-2.0 (`github.com/linker-bot/linkerhand-urdf`, folder
`L20/right`). We compared it joint by joint with the vendor file that every
stored `follow_l20__*` result used. They are different hands on paper: each
finger's base sits 2 to 3 cm apart, the knuckle offsets have opposite sign,
the joint ranges differ (knuckle 1.22 against 1.4 rad, middle joint 1.75
against 1.57 rad), the thumb's last joint has another name, and the palm mass
differs. Swapping one for the other would silently change the L20 results.

Options for the L20:

- **(a) Keep it optional.** Outsiders get Allegro, Shadow, LEAP and the human
  hands. The L20 works only for us, through `fetch_hands.py --wave-manip`.
  This is what the code does today; the README says so.
- **(b) Add the public model as a separate hand**, for example `l20_public`,
  with its own stored results, built by moving Wave_manip's
  `experiment_log/scripts/w22s0_l20_model.py` (URDF to MJCF, with the joint
  couplings restored). Never a stand-in for the lab hand.
- **(c) Ask Linker** whether the files that came with our hand may be
  redistributed.

We recommend (a) now and (b) as later work.

## Install routes

**1. Clone and pip install (the main route).**

```bash
git clone https://github.com/geojenks/gramian-map-workshop
cd gramian-map-workshop
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[hands]"
python tools/fetch_hands.py
python demos/cup_hold.py
```

A plain `pip install git+https://...` is not enough. The package finds
`data/`, `demos/` and `outputs/` relative to the repository, and only the
Python package is installed. So the README offers clone then `pip install -e`.
Making a true pip install work would mean moving the data inside the package;
not worth it for a workshop tool.

**2. Docker image (optional).** A `Dockerfile` on `python:3.11-slim` that
clones nothing: it copies the repository, installs `.[hands]`, and runs
`fetch_hands.py` at build time, so the image works offline. Published to
GitHub's container registry (`ghcr.io/geojenks/gramian-map-workshop`) by a
GitHub Actions workflow. Use:

```bash
docker run --rm -v "$PWD/out:/app/outputs" ghcr.io/geojenks/gramian-map-workshop python demos/cup_hold.py
```

None of our three machines has Docker, so the image is built and tested only
in GitHub Actions. Its value is for people whose Python is awkward (Windows
without a compiler, locked-down machines). It adds one more thing to keep
working.

**3. Project page (GitHub Pages, optional).** One page: what the tool
answers, a picture of a verdict, the install commands, and links to the
stored HTML results (`outputs/cup_hold.html`, `outputs/follow_respaced.html`)
and to `forms/record_form.html`, which runs in the browser with no install at
all. Served from the repository with a small workflow. This is the link we
would give people.

## How we check it

A GitHub Actions workflow on a clean Ubuntu machine (and a Windows one, since
most visitors will be on Windows) that does exactly what the README says:
clone, new venv, `pip install -e ".[hands]"`, `fetch_hands.py`, then

- `python demos/cup_hold.py`, and
- `python tools/run_follow.py --hands allegro` on the stored tennis-ball
  example,

then compare those outputs with the committed files. (Built instead: all
three demos, compared by `tools/compare_outputs.py`, which allows round-off
of 1e-9; a byte comparison fails on round-off near 1e-16. The stored
tennis-ball `follow_allegro__*` result does not reproduce with today's code,
so it is not used as the check.) Tests run with `python -m unittest discover -s tests`; the
checks that need Wave_manip skip themselves.

The demos write to fixed names under `outputs/`, so these runs overwrite
stored files inside the throwaway clone. That is the point of the check; such
a run is never committed. I will also run the same steps once by hand in a
new venv on the laptop, in a scratch clone.

## Rulings needed

1. **Licence for the repository.** We recommend Apache-2.0: it matches the
   Shadow and Linker models, and it covers the code copied in from Wave_manip
   and Gramain_extraction, which are our own work. The records and hand data
   could take CC-BY-4.0 instead if you want citation for data. Without a
   licence the install route is moot.
2. **The L20:** (a), (b) or (c) above. We recommend (a) now.
3. **Which routes to build:** clone-and-pip with the CI check (recommended,
   always); the Docker image (recommended only if you expect visitors who
   cannot install Python); the project page (recommended: it is the link to
   send).

## Noticed, outside this plan

The hand names in `gramian_workshop/hands.py`, the files in
`data/hands/measured/` (including photographs of a hand) and several output
names carry a person's name in a public repository. Worth a decision before
the link goes out.
