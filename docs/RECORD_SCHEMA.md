# Actuator characterization record — schema v0.1

The simulation-independent data contract for the actuator map (Phase A of
`MAP_PLAN.md`). One record = one actuator characterization = one JSON
file, conventionally under `records/<id>.json`. Everything downstream
(metric, map, fingerprint cards) consumes records; `jacobian.py` /
`blocked_force_probe.py` / hand-typed literature data are just front-ends
that emit them.

Design rules the schema encodes:

- **Units are SI throughout** (m, kg, N, Pa, J, s). Unit strings appear in
  the record so a human can audit them, but conforming writers MUST use SI.
- **Outputs are port-based**: the core output is the motion/force of a
  named attachment port ("tip" in v0.1). Distributed shape is an optional
  extra block, expressed as low-order features — never raw meshes.
- **Tiers are derived, not declared.** A record's tier is computed from
  which blocks it carries:
  - Tier 0 — `scalars` only (datasheet numbers).
  - Tier 1 — at least one curve block (`free` / `blocked` / `stiffness`)
    with >= 3 input levels.
  - Tier 2 — `shape` block present (spatial mode fingerprint).
  Richer tiers refine a placement; they never redefine it.
- **Input energy is the canonical cross-physics coordinate** (mechanical
  work delivered at the input port: ∫p dV, ∫T ds, ∫τ dθ). v0.1 defines the
  slot (`input.work_J`) but existing converters leave it null — cavity
  volume per level is not yet extracted from sweeps. Native input units
  are always recorded alongside; nothing is lost by filling energy later.
- **Shape is polarity-blind and must be read with authority.** Gramian
  mode vectors are sign-invariant and direction becomes ill-defined as
  authority → 0 (empirical: the 55° McKibben, see MAP_PLAN.md). The
  record therefore stores `scalars.polarity_axial` (sign information)
  and `shape.authority` (magnitude) explicitly. How to render
  low-authority points (e.g. fading toward a "does nothing" origin) is a
  visualization decision — the record only guarantees the data to make it.
- **Uncertainty is data.** Every curve block may carry an `uncertainty`
  (fractional, 1-sigma-ish); `uncertainty.default_fraction` covers fields
  without their own. Tolerances in the similarity metric are calibrated
  from these — sparse literature points get wide tolerances by declaring
  large uncertainty, not by special-casing.

## Field reference

```jsonc
{
  "schema_version": "0.1",
  "id": "mckibben_v2_30deg",            // unique, filesystem-safe
  "name": "Two-body McKibben, braid 30 deg",

  "source": {
    "kind": "simulation",               // simulation | experiment | literature
    "detail": "SOFA v24.06, workflow_2 pipeline",
    "date": "2026-07-03",
    "reference": null                   // citation / DOI / datasheet URL
  },

  "metadata": {
    "actuation_physics": "pneumatic",   // pneumatic | cable | electric | thermal | ...
    "length_m": 0.073,                  // characteristic length (long axis)
    "mass_kg": null,
    "material_volume_m3": 1.2e-6,       // exact tet-sum where available
    "materials": {},                    // free-form pass-through (materials.json)
    "notes": null
  },

  "input": {
    "channels": [                       // one entry per independent input
      {"name": "chamber pressure", "unit": "Pa",
       "range": [0.0, 100.0], "rated": null}
    ],
    "energy_convention": "mechanical work delivered at the input port (J)",
    "work_J": null                      // per free-block level, once computed
  },

  "port": {"name": "tip", "description": "free-end centroid"},

  // ---- Tier 1 curve blocks (each optional, each independently useful) ----

  "free": {                             // free actuation: F = 0
    "u": [[0.0], [8.3], ...],           // (L, n_channels) native units
    "tip_displacement_m": [[dx,dy,dz], ...],  // (L, 3) vs rest
    "uncertainty": null
  },

  "blocked": {                          // held actuation: x ~= 0 at port
    "u": [[10.0], [20.0], ...],
    "force_axial_N": [0.0005, -0.042, ...],   // signed, along long axis
    "force_transverse_N": [0.0003, ...],      // magnitude, bending component
    "method": "tip spring k=1e6 N/m",
    "uncertainty": null,

    // ---- C2 optional per-axis force & moment fields (July 2026) ----
    // All three are optional/nullable; records without them (McKibbens,
    // literature) validate unchanged. force_axial_N and
    // force_transverse_N remain canonical for existing consumers.

    "force_N": [[0.051, 0.002, 0.0001], ...],  // (L, 3) full [Fx, Fy, Fz]
                                                // vector per input level.
                                                // Fx = axial (redundant with
                                                // force_axial_N for backward
                                                // compatibility).
    "moment_Nm": [[0.0001, -0.003, 0.004], ...], // (L, 3) bending moment
                                                  // about the inlet centroid:
                                                  // M = lever_vec x F_tip.
                                                  // [Mx, My, Mz] per level.
    "moment_method": "tip spring reaction x lever arm (blocked_moment_probe.py)",
    "twist_tip_Nm": [0.000353, ...]  // (L,) torsional moment about the
                                     // longitudinal axis from the tip force
                                     // distribution: sum_i (r_i - r_c) x
                                     // f_i . x_hat. Distinct from
                                     // moment_Nm[:,0]. Added C2, July 2026.
  },

  "stiffness": {                        // output impedance dF/dx vs
                                        // operating point (stiffness_probe.py)
    "u": [[0.0], [0.000375], ...],      // operating points (probe input units)
    "k_N_per_m": [9.1, ...],            // transverse stiffness at each point
    "k_axial_N_per_m": [412.0, ...],
    "direction": "transverse (+Y canonical)",
    "method": "two-sided tip force probe",
    "uncertainty": null
  },

  // ---- Tier 0 headline numbers (ALWAYS present; derived for tier >= 1) ----

  "scalars": {
    "free_stroke_m": 0.0035,            // |tip displacement| at max input
    "free_direction": [ -0.99, 0.1, 0.0 ],  // SMALL-SIGNAL unit direction
                                        // (first measurable motion, NOT
                                        // endpoint — endpoint direction
                                        // conflates shape with amplitude)
    "polarity_axial": -1,               // -1 contract / +1 extend / 0 neutral
    "blocked_force_N": 1.78,            // max |axial| blocked force
    "stiffness_N_per_m": null,
    "effective_dof": 1                  // rank(W_c) where known
  },

  // ---- Tier 2 spatial fingerprint (optional) ----

  "shape": {
    "convention": "spine centroids, point-major xyz; polarity-blind; read jointly with authority",
    "spine_rest_m": [[x,y,z], ...],     // (P, 3)
    "mode_eigenvalues": [ ... ],        // top-k eig(W_c), descending, k<=6
    "mode_vectors": [[...], ...],       // (3P, k) unit columns
    "authority": 1.3e-4                 // sqrt(trace W_c) — magnitude that
                                        // must accompany any shape distance
  },

  "uncertainty": {"default_fraction": 0.1},

  // ---- OPTIONAL provenance pin (simulation records; added C4, July 2026) ----
  // Ties the record to the exact CAD instantiation that produced it, so a
  // claim about this record is checkable against the specific mesh.
  // Approved shape: docs/PROPOSAL_C4_PROVENANCE.md. Loader-neutral (optional
  // top-level keys pass validate()).

  "provenance": {
    "generator": "make_bellows.py",     // or "A + B" for multi-step builds
    "generator_command": "python make_bellows.py --out cads/bellows/bellows1/mesh.msh; python preprocess.py ...",
    "cad_dir": "cads/bellows/bellows1", // repo-relative
    "mesh_file": "mesh.msh",
    "mesh_sha256": "…",                 // geometry source of truth
    "actuator_npz_sha256": "…",         // what the scene actually loads
    "sidecars_sha256": {"braid.npz": "…"},  // physics sidecars, when present
    "scene": "parametricGripper.py",
    "provenance_doc": "cads/bellows/PROVENANCE.md",  // the human-readable story
    "note": "…",                        // e.g. command reconstructed, not captured live
    "backfilled": "2026-07-12"          // present iff added after the fact
                                        // (backfill_provenance.py); absent when
                                        // emitted at build time
  }
}
```

Literature/experiment records carry no provenance block; `backfill_provenance.py`
instead checks their citation completeness and reports. Hash drift (a regenerated
mesh under an unchanged record) is a reportable MISMATCH — re-run
`backfill_provenance.py` (dry-run by default) any time generators change.

## Conformance

`actuator_record.py` is the reference implementation:

```
python actuator_record.py records/*.json        # validate + summary table
```

Validation errors (must fix): missing required fields, non-SI unit
strings on known quantities, array length mismatches within a block,
NaN/inf values, `u` outside declared channel range.
Warnings (should explain): tier 0 with source.kind=simulation (sims can
always do better), missing uncertainty, shape without authority.

## Producers

| Producer | Output tier | Status |
|---|---|---|
| `record_from_jacobian.py` (jacobian.npz [+ actuator.npz, materials.json, bforce log]) | 2 | Phase A; emits `provenance` at build time when given `--generator_command` |
| hand-typed JSON from paper/datasheet | 0–1 | template: `records/example_rigid_pneumatic_cylinder.json` |
| `record_from_experiment.py` (rig CSVs: free and/or blocked curves) | 1 | C1a, July 2026; refuses `records/` without `--confirm-canon` |
| `backfill_provenance.py` (adds `provenance` to existing simulation records) | n/a | C4, July 2026; dry-run by default |

## Known v0.1 limitations (candidates for v0.2)

- Single port, translation-only (`tip_displacement_m`); no rotation twist
  components yet. Multi-port + full 6-dim twist/wrench when a use case
  lands (multi-chamber, OSSA-like).
- `input.work_J` unfilled by the SOFA converter (needs per-level cavity
  volume from snapshots).
- No `cross_section_area_m2` → blocked stress (intensive) not derivable.
- Blocked force: scalar fields (`force_axial_N`, `force_transverse_N`)
  remain for backward compatibility; `force_N` (L,3) carries the full
  vector where available (C2, July 2026). `moment_Nm` (L,3) and
  `twist_tip_Nm` (L,) added in the same batch.
