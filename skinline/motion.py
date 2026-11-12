"""Root-motion extraction and in-place application."""
from __future__ import annotations

import math

from skinline.math3d import Vec3, Quat


def _yaw_of(q: Quat) -> float:
    q = q.normalized()
    return math.atan2(
        2.0 * (q.w * q.y + q.x * q.z),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def extract_root_delta(prev_root_t: Vec3, curr_root_t: Vec3, prev_root_r: Quat, curr_root_r: Quat) -> dict:
    """Return {"dx", "dz", "yaw"} between consecutive root poses, on XZ.

    Y is intentionally excluded from the translation delta. Yaw is the change
    around +Y, wrapped to (-pi, pi].
    """
    yaw = _yaw_of(curr_root_r) - _yaw_of(prev_root_r)
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw <= -math.pi:
        yaw += 2.0 * math.pi
    return {
        "dx": curr_root_t.x - prev_root_t.x,
        "dz": curr_root_t.z - prev_root_t.z,
        "yaw": yaw,
    }


def apply_root_motion(accumulated: dict, pose_root_t: Vec3, pose_root_r: Quat, in_place: bool) -> tuple[Vec3, Quat]:
    """Resolve the root transform for a frame.

    In-place mode removes authored XZ travel (Y stays on the authored curve) and
    replays the accumulated root motion instead. Otherwise the authored root
    transform is used verbatim while accumulation is still reported.
    """
    if not in_place:
        return pose_root_t, pose_root_r
    t = Vec3(accumulated["dx"], pose_root_t.y, accumulated["dz"])
    r = Quat.from_axis_angle(Vec3(0.0, 1.0, 0.0), accumulated["yaw"]) * pose_root_r
    return t, r
