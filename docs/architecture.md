# Architecture

The fixture `fixtures/rig.json` contains:

- `skeleton.bones` — each bone has `name`, `parent` (name or null), `bind_local` (t/r/s), and `length`
- `mesh.vertices` — bind-pose positions, normals, and up to 4 (bone_name, weight) pairs (weights already normalized)
- `clips` — named clips with per-bone keyframes. Times are seconds. Rotation is xyzw quaternion. Translation is world-parent local space.

`fixtures/plans/locomote_reach.json` is a bake plan:

- 0..1.6s: 1D blend of walk→run (parameter 0 at t=0, 1 at t=1.2) at 30 fps
- Upper-body additive layer: `reach_add` starting at t=0.8s, weight 0→1 over 0.3s
- IK on `hand` (two-bone: `upper_arm` / `lower_arm`), pole = `ik_pole`, target follows `ik_target` in the plan, weight 0 until t=0.8s then 1
- Root motion applied to the root bone's translation on XZ; Y stays on the authored curve
- Skin both LBS and DQS and dump vertices

See `docs/pose-dump-schema.md`.
