"""Mesh skinning — NOT implemented yet."""
from __future__ import annotations

from skinline.io import Vertex
from skinline.math3d import Mat4, Vec3


def skin_lbs(vertices: list[Vertex], bone_mats: dict[str, Mat4], bind_inverse: dict[str, Mat4]) -> list[Vec3]:
    """Linear blend skinning: p' = Σ w_i * bone_i * bind_inv_i * p."""
    raise NotImplementedError("implement LBS")


def skin_dqs(vertices: list[Vertex], bone_mats: dict[str, Mat4], bind_inverse: dict[str, Mat4]) -> list[Vec3]:
    """Dual-quaternion skinning. Convert each skin matrix to a unit dual quaternion,
    blend with the vertex weights (flip to the same hemisphere), normalize, and
    transform the bind-pose position. This should reduce candy-wrapper collapse
    relative to LBS on the `twist_band` vertices in the fixture mesh.
    """
    raise NotImplementedError("implement DQS")
