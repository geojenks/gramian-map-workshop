# The pneu-net finger simulated from a mesh, as a draft record

`sample_pneunet_pressure_set.json` is a byte-for-byte copy of
`mesh_route/expected/sample_pneunet_pressure_set.json`, the record that the
mesh route in `mesh_route/` produces (see `mesh_route/PROVENANCE.md` for the
mesh, the simulation and the commands). It is copied here so that the map
builder, `tools/export_map_paths.py`, which reads every record under
`data/draft_records/`, draws it on the map with the other drafts.

It is a draft from a simulation, not a canonical record. Its inputs are in
the units the simulation scene took them in, so the map does not scale its
input by a body stiffness.

If the mesh route's expected record changes, copy it here again and rebuild
the map data (`python tools/export_map_paths.py --site`).
