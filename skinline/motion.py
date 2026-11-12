"""Root-motion extraction — NOT implemented yet."""
from __future__ import annotations

from skinline.math3d import Vec3, Quat


def extract_root_delta(prev_root_t: Vec3, curr_root_t: Vec3, prev_root_r: Quat, curr_root_r: Quat) -> dict:
    """Return {"dx","dz","yaw"} from consecutive sampled root keys, on XZ.

    Y is not part of locomotion delta. yaw is the change in the root quaternion
    around +Y, in radians, range (-pi, pi].
    """
    raise NotImplementedError("implement root-motion extraction")


def apply_root_motion(accumulated: dict, pose_root_t: Vec3, pose_root_r: Quat, in_place: bool) -> tuple[Vec3, Quat]:
    """If `in_place` is True, zero the authored XZ translation (keep Y) and only
    apply `accumulated`. If False, use the authored root translation as-is
    (accumulated should still be reported).
    """
    raise NotImplementedError("implement root-motion apply")
