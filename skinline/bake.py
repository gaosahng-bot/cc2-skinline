"""Bake plan evaluator: sample, blend, IK, root motion, skin and dump."""
from __future__ import annotations

import json
import math
from pathlib import Path

from skinline.blend import apply_additive, blend_1d
from skinline.clip import sample_clip
from skinline.draw import write_ppm
from skinline.ik import two_bone_ik
from skinline.io import load_plan, load_scene
from skinline.math3d import Mat4, Quat, Vec3, trs
from skinline.motion import apply_root_motion, extract_root_delta
from skinline.skin import skin_dqs, skin_lbs


def _step_curve(curve: list[dict], time: float, key: str = "value") -> float:
    """Piecewise-linear evaluation of [{time, value}] style curves."""
    if time <= curve[0]["time"]:
        return float(curve[0][key])
    if time >= curve[-1]["time"]:
        return float(curve[-1][key])
    for i in range(len(curve) - 1):
        a, b = curve[i], curve[i + 1]
        if a["time"] <= time <= b["time"]:
            span = b["time"] - a["time"]
            u = 0.0 if span <= 0.0 else (time - a["time"]) / span
            return float(a[key]) + (float(b[key]) - float(a[key])) * u
    return float(curve[-1][key])


def _point_curve(curve: list[dict], time: float) -> Vec3:
    if time <= curve[0]["time"]:
        p = curve[0]["p"]
        return Vec3(float(p[0]), float(p[1]), float(p[2]))
    if time >= curve[-1]["time"]:
        p = curve[-1]["p"]
        return Vec3(float(p[0]), float(p[1]), float(p[2]))
    for i in range(len(curve) - 1):
        a, b = curve[i], curve[i + 1]
        if a["time"] <= time <= b["time"]:
            span = b["time"] - a["time"]
            u = 0.0 if span <= 0.0 else (time - a["time"]) / span
            pa, pb = a["p"], b["p"]
            return Vec3(
                pa[0] + (pb[0] - pa[0]) * u,
                pa[1] + (pb[1] - pa[1]) * u,
                pa[2] + (pb[2] - pa[2]) * u,
            )
    p = curve[-1]["p"]
    return Vec3(float(p[0]), float(p[1]), float(p[2]))


def _bind_pose(scene) -> dict:
    return {
        bone.name: {"t": bone.bind_t, "r": bone.bind_r, "s": bone.bind_s}
        for bone in scene.skeleton
    }


def _fill_bind(local: dict, bind: dict, bones: list[str]) -> dict:
    filled = {}
    for name in bones:
        channels = local.get(name, {})
        b = bind[name]
        filled[name] = {
            "t": channels.get("t") if channels.get("t") is not None else b["t"],
            "r": channels.get("r") if channels.get("r") is not None else b["r"],
            "s": channels.get("s") if channels.get("s") is not None else b["s"],
        }
    return filled


def _compute_globals(scene, local_pose: dict) -> tuple[dict[str, Mat4], dict[str, Quat], dict[str, Vec3]]:
    globals_mat: dict[str, Mat4] = {}
    globals_rot: dict[str, Quat] = {}
    globals_pos: dict[str, Vec3] = {}
    for bone in scene.skeleton:
        ch = local_pose[bone.name]
        local_mat = trs(ch["t"], ch["r"], ch["s"])
        if bone.parent is None:
            gmat = local_mat
            grot = ch["r"]
        else:
            gmat = globals_mat[bone.parent] * local_mat
            grot = globals_rot[bone.parent] * ch["r"]
        globals_mat[bone.name] = gmat
        globals_rot[bone.name] = grot
        globals_pos[bone.name] = gmat.transform_point(Vec3(0.0, 0.0, 0.0))
    return globals_mat, globals_rot, globals_pos


def _round_list(values: list[float], digits: int = 9) -> list[float]:
    return [round(float(v), digits) for v in values]


def bake(scene_path: str, plan_path: str, out_dir: str) -> None:
    """Evaluate the plan, write pose.jsonl, preview PPMs, and summary.json."""
    scene = load_scene(scene_path)
    plan = load_plan(plan_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    fps = int(plan["fps"])
    duration = float(plan["duration"])
    frames = int(round(duration * fps))
    in_place = bool(plan.get("in_place", False))
    bind = _bind_pose(scene)
    bone_names = [bone.name for bone in scene.skeleton]
    bind_inverse = {
        bone.name: trs(bone.bind_t, bone.bind_r, bone.bind_s).inverse()
        for bone in scene.skeleton
    }

    walk_clip = scene.clips[plan["blend"]["from"]]
    run_clip = scene.clips[plan["blend"]["to"]]
    add_clip = scene.clips[plan["additive"]["clip"]]
    ik_cfg = plan["ik"]
    upper_name = ik_cfg["upper"]
    lower_name = ik_cfg["lower"]
    hand_name = ik_cfg["end_effector"]
    upper_bone = scene.skeleton[scene.bone_index[upper_name]]
    upper_parent = upper_bone.parent
    upper_bone_len = upper_bone.length
    lower_bone_len = scene.skeleton[scene.bone_index[lower_name]].length

    preview = set(int(f) for f in plan.get("preview_frames", []))
    accumulated = {"dx": 0.0, "dz": 0.0, "yaw": 0.0}
    prev_root_t: Vec3 | None = None
    prev_root_r: Quat | None = None
    root_path_length = 0.0
    ik_max_error = 0.0
    skin_max_delta = 0.0
    pose_records: list[dict] = []

    for frame in range(frames):
        time = frame / fps

        alpha = _step_curve(plan["blend"]["param"], time)
        add_weight = _step_curve(plan["additive"]["weight"], time)
        ik_weight = _step_curve(ik_cfg["weight"], time)
        target = _point_curve(ik_cfg["target"], time)
        pole = _point_curve(ik_cfg["pole"], time)

        walk_pose = _fill_bind(sample_clip(walk_clip, time), bind, bone_names)
        run_pose = _fill_bind(sample_clip(run_clip, time), bind, bone_names)
        base_pose = blend_1d(walk_pose, run_pose, alpha)
        add_sampled = sample_clip(add_clip, time)
        local_pose = _fill_bind(apply_additive(base_pose, add_sampled, add_weight, bind), bind, bone_names)

        globals_mat, globals_rot, globals_pos = _compute_globals(scene, local_pose)

        ik_upper_local, ik_lower_local = two_bone_ik(
            globals_pos[upper_name],
            upper_bone_len,
            lower_bone_len,
            target,
            pole,
            upper_bone.bind_r,
            scene.skeleton[scene.bone_index[lower_name]].bind_r,
            globals_rot[upper_parent] if upper_parent else Quat.identity(),
        )
        if ik_weight > 0.0:
            local_pose[upper_name]["r"] = local_pose[upper_name]["r"].slerp(ik_upper_local, ik_weight)
            local_pose[lower_name]["r"] = local_pose[lower_name]["r"].slerp(ik_lower_local, ik_weight)
            globals_mat, globals_rot, globals_pos = _compute_globals(scene, local_pose)

        hand_pos = globals_pos[hand_name]
        ik_err = (hand_pos - target).norm()
        if ik_weight >= 1.0 - 1e-9:
            ik_max_error = max(ik_max_error, ik_err)

        root_t = globals_pos["root"]
        root_r = globals_rot["root"]
        if prev_root_t is None:
            delta = {"dx": 0.0, "dz": 0.0, "yaw": 0.0}
        else:
            delta = extract_root_delta(prev_root_t, root_t, prev_root_r, root_r)
        accumulated["dx"] += delta["dx"]
        accumulated["dz"] += delta["dz"]
        accumulated["yaw"] += delta["yaw"]
        if prev_root_t is not None:
            root_path_length += math.hypot(delta["dx"], delta["dz"])
        prev_root_t, prev_root_r = root_t, root_r

        final_root_t, final_root_r = apply_root_motion(
            accumulated, local_pose["root"]["t"], local_pose["root"]["r"], in_place
        )
        if in_place:
            local_pose["root"]["t"] = final_root_t
            local_pose["root"]["r"] = final_root_r
            globals_mat, globals_rot, globals_pos = _compute_globals(scene, local_pose)

        bones_dump = {}
        for name in bone_names:
            bones_dump[name] = {
                "global": {
                    "t": _round_list(globals_pos[name].as_list()),
                    "r": _round_list(globals_rot[name].as_list()),
                }
            }

        pose_records.append(
            {
                "frame": frame,
                "time": round(time, 9),
                "blend": {
                    walk_clip.name: round(1.0 - alpha, 9),
                    run_clip.name: round(alpha, 9),
                    add_clip.name: round(add_weight, 9),
                    "ik": round(ik_weight, 9),
                },
                "root_motion": {k: round(v, 9) for k, v in accumulated.items()},
                "bones": bones_dump,
                "ik": {
                    "target": _round_list(target.as_list()),
                    "pole": _round_list(pole.as_list()),
                    "reached": ik_err < 1e-3,
                    "err": round(ik_err, 9),
                },
            }
        )

        if frame in preview:
            lbs_points = skin_lbs(scene.mesh, globals_mat, bind_inverse)
            dqs_points = skin_dqs(scene.mesh, globals_mat, bind_inverse)
            write_ppm(out / f"lbs_{frame:04d}.ppm", lbs_points)
            write_ppm(out / f"dqs_{frame:04d}.ppm", dqs_points)
            for a, b in zip(lbs_points, dqs_points):
                skin_max_delta = max(skin_max_delta, (a - b).norm())

    with (out / "pose.jsonl").open("w") as f:
        for record in pose_records:
            f.write(json.dumps(record) + "\n")

    summary = {
        "frames": frames,
        "fps": fps,
        "duration": duration,
        "root_path_length": round(root_path_length, 9),
        "ik_max_error": round(ik_max_error, 9),
        "lbs_vs_dqs_max_delta": round(skin_max_delta, 9),
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
