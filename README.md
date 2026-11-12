# skinline

A small skeletal animation / skinning runtime.

Already in the tree:

- `skinline.math3d` — vec3, quat, mat4
- `skinline.io` — load the JSON skeleton, mesh and clips under `fixtures/`
- `skinline.draw` — write a simple PPM of skinned vertices (debug view)
- Fixtures: a 13-bone arm+locomotion rig, walk/run clips, a reach clip, and a weighted mesh

Runtime pipeline (`skinline.bake` wires it together):

- `skinline.clip` — linear (lerp/slerp) and cubic (Catmull-Rom / squad) sampling, loop wrapping or clamping
- `skinline.blend` — 1D walk->run blend tree and the additive reach layer
- `skinline.ik` — analytic two-bone IK with a pole vector (law of cosines)
- `skinline.skin` — linear blend skinning and hemisphere-safe dual-quaternion skinning
- `skinline.motion` — XZ/yaw root-motion extraction and an in-place mode
- `python -m skinline bake` — evaluates a plan and writes `pose.jsonl`, preview PPMs and `summary.json`

```
python -m skinline bake --scene fixtures/rig.json --plan fixtures/plans/locomote_reach.json --out /tmp/skinline-out
```
