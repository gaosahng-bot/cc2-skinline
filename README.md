# skinline

A small skeletal animation / skinning runtime.

Already in the tree:

- `skinline.math3d` — vec3, quat, mat4
- `skinline.io` — load the JSON skeleton, mesh and clips under `fixtures/`
- `skinline.draw` — write a simple PPM of skinned vertices (debug view)
- Fixtures: a 9-bone arm+locomotion rig, walk/run clips, a reach clip, and a weighted mesh

Not implemented (the actual work):

- Clip sampling (linear / cubic, looping) in `skinline.clip`
- 1D blend tree + additive upper-body layer in `skinline.blend`
- Two-bone IK with pole vector in `skinline.ik`
- Linear blend skinning and dual-quaternion skinning in `skinline.skin`
- Root-motion extraction vs in-place in `skinline.motion`
- The `python -m skinline bake` CLI that writes a per-frame pose dump

```
python -m skinline bake --scene fixtures/rig.json --plan fixtures/plans/locomote_reach.json --out /tmp/skinline-out
```
