"""Mesh skinning: linear blend skinning and dual-quaternion skinning."""
from __future__ import annotations

import math

from skinline.io import Vertex
from skinline.math3d import Mat4, Quat, Vec3


class DualQuat:
    """Unit dual quaternion (real, dual) using math3d column product order."""

    __slots__ = ("real", "dual")

    def __init__(self, real: Quat, dual: Quat) -> None:
        self.real = real
        self.dual = dual

    @staticmethod
    def from_matrix(mat: Mat4) -> "DualQuat":
        m = mat.m
        t = Vec3(m[3], m[7], m[11])
        q = _quat_from_rotation_matrix(m)
        half = Quat(t.x * 0.5, t.y * 0.5, t.z * 0.5, 0.0)
        # math3d: (a*b) applies b then a. The matrix is T * R, so its dual
        # quaternion dual part is half_t * r (verified against Mat4 directly).
        return DualQuat(q, half * q)

    def transform_point(self, p: Vec3) -> Vec3:
        pq = Quat(p.x, p.y, p.z, 0.0)
        r = self.real
        rot_part = r * pq * r.conjugate()
        trans_part = self.dual * r.conjugate()
        return Vec3(
            rot_part.x + 2.0 * trans_part.x,
            rot_part.y + 2.0 * trans_part.y,
            rot_part.z + 2.0 * trans_part.z,
        )

    def scaled(self, factor: float) -> "DualQuat":
        return DualQuat(
            Quat(self.real.x * factor, self.real.y * factor, self.real.z * factor, self.real.w * factor),
            Quat(self.dual.x * factor, self.dual.y * factor, self.dual.z * factor, self.dual.w * factor),
        )

    def add(self, other: "DualQuat") -> "DualQuat":
        return DualQuat(
            Quat(
                self.real.x + other.real.x,
                self.real.y + other.real.y,
                self.real.z + other.real.z,
                self.real.w + other.real.w,
            ),
            Quat(
                self.dual.x + other.dual.x,
                self.dual.y + other.dual.y,
                self.dual.z + other.dual.z,
                self.dual.w + other.dual.w,
            ),
        )

    def normalized(self) -> "DualQuat":
        inv_len = 1.0 / math.sqrt(
            self.real.x ** 2 + self.real.y ** 2 + self.real.z ** 2 + self.real.w ** 2
        )
        return DualQuat(
            Quat(
                self.real.x * inv_len,
                self.real.y * inv_len,
                self.real.z * inv_len,
                self.real.w * inv_len,
            ),
            Quat(
                self.dual.x * inv_len,
                self.dual.y * inv_len,
                self.dual.z * inv_len,
                self.dual.w * inv_len,
            ),
        )


def _quat_from_rotation_matrix(m: list[float]) -> Quat:
    # Sign convention matches math3d.Quat.to_mat4 (off-diagonal pair signs).
    trace = m[0] + m[5] + m[10]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[9] - m[6]) / s
        y = (m[2] - m[8]) / s
        z = (m[4] - m[1]) / s
    elif m[0] > m[5] and m[0] > m[10]:
        s = math.sqrt(1.0 + m[0] - m[5] - m[10]) * 2.0
        w = (m[9] - m[6]) / s
        x = 0.25 * s
        y = (m[4] + m[1]) / s
        z = (m[2] + m[8]) / s
    elif m[5] > m[10]:
        s = math.sqrt(1.0 + m[5] - m[0] - m[10]) * 2.0
        w = (m[2] - m[8]) / s
        x = (m[4] + m[1]) / s
        y = 0.25 * s
        z = (m[6] + m[9]) / s
    else:
        s = math.sqrt(1.0 + m[10] - m[0] - m[5]) * 2.0
        w = (m[4] - m[1]) / s
        x = (m[2] + m[8]) / s
        y = (m[6] + m[9]) / s
        z = 0.25 * s
    return Quat(x, y, z, w).normalized()


def skin_lbs(vertices: list[Vertex], bone_mats: dict[str, Mat4], bind_inverse: dict[str, Mat4]) -> list[Vec3]:
    """Linear blend skinning: p' = sum_i w_i * bone_i * bind_inv_i * p."""
    skin_mats = {name: bone_mats[name] * bind_inverse[name] for name in bone_mats}
    out: list[Vec3] = []
    for vertex in vertices:
        acc = Vec3(0.0, 0.0, 0.0)
        for joint, weight in zip(vertex.joints, vertex.weights):
            if weight <= 0.0:
                continue
            acc += skin_mats[joint].transform_point(vertex.position) * weight
        out.append(acc)
    return out


def skin_dqs(vertices: list[Vertex], bone_mats: dict[str, Mat4], bind_inverse: dict[str, Mat4]) -> list[Vec3]:
    """Dual-quaternion skinning over the same bone * bind_inv transforms.

    Each skin matrix becomes a unit dual quaternion. Per vertex the influences
    are flipped into the hemisphere of the first influence, blended by weight,
    normalized and applied to the bind-pose position. Rigid blending preserves
    volume on the half-weight elbow band where LBS candy-wraps.
    """
    skin_dq = {
        name: DualQuat.from_matrix(bone_mats[name] * bind_inverse[name]) for name in bone_mats
    }
    out: list[Vec3] = []
    for vertex in vertices:
        ref = None
        blended: DualQuat | None = None
        total = 0.0
        for joint, weight in zip(vertex.joints, vertex.weights):
            if weight <= 0.0:
                continue
            dq = skin_dq[joint]
            if ref is not None:
                if dq.real.dot(ref.real) < 0.0:
                    dq = dq.scaled(-1.0)
            else:
                ref = dq
            term = dq.scaled(weight)
            blended = term if blended is None else blended.add(term)
            total += weight
        if total <= 0.0:
            out.append(vertex.position)
            continue
        out.append(blended.normalized().transform_point(vertex.position))
    return out
