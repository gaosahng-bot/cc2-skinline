"""Analytic two-bone IK with a pole vector (law of cosines)."""
from __future__ import annotations

import math

from skinline.math3d import Quat, Vec3


def _shortest_arc(from_dir: Vec3, to_dir: Vec3) -> Quat:
    """Rotation taking normalized `from_dir` onto normalized `to_dir`."""
    a = from_dir.normalized()
    b = to_dir.normalized()
    dot = max(-1.0, min(1.0, a.dot(b)))
    axis = a.cross(b)
    if axis.norm() < 1e-9:
        if dot < 0.0:
            helper = Vec3(0.0, 1.0, 0.0) if abs(a.y) < 0.9 else Vec3(1.0, 0.0, 0.0)
            axis = a.cross(helper).normalized()
            return Quat.from_axis_angle(axis, math.pi)
        return Quat.identity()
    return Quat.from_axis_angle(axis.normalized(), math.acos(dot))


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
    """Return local rotations (upper_local, lower_local) for a two-bone chain.

    The solved chain runs root joint -> mid joint -> end point, with the end
    point placed at `target`. Both bones point along their local +Y axes; the
    returned global rotations are the shortest Y-axis alignments, so any bind
    roll is dropped (the pole vector selects the bend plane instead).

    Targets beyond mid+end straighten the chain toward the target; targets
    inside |mid-end| fold as tightly as possible. `parent_rot` is the global
    rotation of the parent of the upper bone; returned values are local.
    """
    reach = target - root_global_pos
    dist = reach.norm()
    if dist < 1e-9:
        return (
            parent_rot.conjugate() * root_bind_rot,
            root_bind_rot.conjugate() * mid_bind_rot,
        )

    max_reach = mid_length + end_length
    clamped_dist = max(abs(mid_length - end_length), min(dist, max_reach))
    forward = reach * (1.0 / dist)

    pole_dir = (pole - root_global_pos).normalized()
    bend = pole_dir - forward * pole_dir.dot(forward)
    if bend.norm() < 1e-9:
        helper = Vec3(0.0, 1.0, 0.0) if abs(forward.y) < 0.9 else Vec3(1.0, 0.0, 0.0)
        bend = helper - forward * helper.dot(forward)
    bend = bend.normalized()

    # Law of cosines: root-to-mid distance inside the solved triangle.
    cos_mid = (mid_length * mid_length + clamped_dist * clamped_dist - end_length * end_length) / (
        2.0 * mid_length * clamped_dist
    )
    cos_mid = max(-1.0, min(1.0, cos_mid))
    along = mid_length * cos_mid
    side = math.sqrt(max(0.0, mid_length * mid_length - along * along))
    mid_pos = root_global_pos + forward * along + bend * side
    end_pos = root_global_pos + forward * clamped_dist

    root_global = _shortest_arc(Vec3(0.0, 1.0, 0.0), (mid_pos - root_global_pos).normalized())
    mid_global = _shortest_arc(Vec3(0.0, 1.0, 0.0), (end_pos - mid_pos).normalized())

    root_local = parent_rot.conjugate() * root_global
    mid_local = root_global.conjugate() * mid_global
    return root_local, mid_local
