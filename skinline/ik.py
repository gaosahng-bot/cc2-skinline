"""Two-bone IK with pole vector — NOT implemented yet."""
from __future__ import annotations

from skinline.math3d import Quat, Vec3


def two_bone_ik(
    root_global_pos: Vec3,
    mid_length: float,
    end_length: float,
    target: Vec3,
    pole: Vec3,
    root_bind_rot: Quat,
    mid_bind_rot: Quat,
    parent_rot: Quat,
) -> tuple[Quat, Quat]:
    """Return local rotations (root_local, mid_local) for a two-bone chain.

    The chain sits in the plane defined by root, target and pole. Use the law of
    cosines for the mid joint. If the target is farther than mid+end, fully
    stretch toward the target. If closer than |mid-end|, fold as far as possible.
    Rotations are local to the parent of the root bone.

    `parent_rot` is the global rotation of the parent of the upper bone.
    The returned quaternions are local.
    """
    raise NotImplementedError("implement two-bone IK")
