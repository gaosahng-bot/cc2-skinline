# pose dump

`bake` must write:

- `<out>/pose.jsonl` — one JSON object per frame
- `<out>/lbs_####.ppm` and `<out>/dqs_####.ppm` for frames listed in the plan's `preview_frames`
- `<out>/summary.json`

```json
{
  "frame": 0,
  "time": 0.0,
  "blend": {"walk": 1.0, "run": 0.0, "reach_add": 0.0, "ik": 0.0},
  "root_motion": {"dx": 0.0, "dz": 0.0, "yaw": 0.0},
  "bones": {
    "root": {"global": {"t": [0,0,0], "r": [0,0,0,1]}},
    "hand": {"global": {"t": [..], "r": [..]}}
  },
  "ik": {"target": [..], "pole": [..], "reached": false, "err": 0.0}
}
```

`summary.json` must include `frames`, `fps`, `duration`, `root_path_length` (sum of XZ root displacement), `ik_max_error` (metres, after IK is on), `lbs_vs_dqs_max_delta` (max vertex distance between the two skinning methods on preview frames).
