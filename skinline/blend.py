"""Blend tree — NOT implemented yet."""
from __future__ import annotations


def blend_1d(pose_a: dict, pose_b: dict, alpha: float) -> dict:
    """1D blend between two sampled poses. `alpha` 0=A, 1=B.

    Interpolate present channels: vec lerp, quat slerp. Bones only in one pose are kept.
    """
    raise NotImplementedError("implement 1D pose blend")


def apply_additive(base: dict, additive: dict, weight: float, bind: dict) -> dict:
    """Apply an additive layer: result = base ⊕ weight * (additive ⊖ bind).

    For rotation this is base * slerp(identity, bind_inv * additive, weight).
    For translation: base + (additive - bind) * weight.
    """
    raise NotImplementedError("implement additive layer")
