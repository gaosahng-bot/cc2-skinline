"""Blend tree: 1D pose blend and an additive upper-body layer."""
from __future__ import annotations

from skinline.math3d import Quat, Vec3


def blend_1d(pose_a: dict, pose_b: dict, alpha: float) -> dict:
    """1D blend between two sampled poses, `alpha` 0 = A, 1 = B.

    Present channels are interpolated (vec lerp, quat slerp); a channel that
    exists only in one pose is kept. Missing channels stay missing (None) so the
    bind pose fill-in stage can handle them.
    """
    out: dict = {}
    for bone in pose_a.keys() | pose_b.keys():
        a = pose_a.get(bone, {})
        b = pose_b.get(bone, {})
        out[bone] = {
            "t": _blend_vec(a.get("t"), b.get("t"), alpha),
            "r": _blend_quat(a.get("r"), b.get("r"), alpha),
            "s": _blend_vec(a.get("s"), b.get("s"), alpha),
        }
    return out


def _blend_vec(a: Vec3 | None, b: Vec3 | None, alpha: float) -> Vec3 | None:
    if a is not None and b is not None:
        return a.lerp(b, alpha)
    return a if a is not None else b


def _blend_quat(a: Quat | None, b: Quat | None, alpha: float) -> Quat | None:
    if a is not None and b is not None:
        return a.slerp(b, alpha)
    return a if a is not None else b


def apply_additive(base: dict, additive: dict, weight: float, bind: dict) -> dict:
    """Apply an additive layer: result = base + weight * (additive - bind).

    Rotation: base * slerp(identity, bind^-1 * additive, weight).
    Translation/scale: base + (additive - bind) * weight.
    Bones untouched by the additive clip keep their base values.
    """
    out = {bone: dict(channels) for bone, channels in base.items()}
    for bone, add_channels in additive.items():
        base_channels = out.setdefault(bone, {"t": None, "r": None, "s": None})
        bind_channels = bind.get(bone, {})
        result = dict(base_channels)
        if add_channels.get("t") is not None and base_channels.get("t") is not None:
            delta = add_channels["t"] - bind_channels["t"]
            result["t"] = base_channels["t"] + delta * weight
        if add_channels.get("r") is not None and base_channels.get("r") is not None:
            relative = bind_channels["r"].conjugate() * add_channels["r"]
            result["r"] = base_channels["r"] * Quat.identity().slerp(relative, weight)
        if add_channels.get("s") is not None and base_channels.get("s") is not None:
            delta = add_channels["s"] - bind_channels["s"]
            result["s"] = base_channels["s"] + delta * weight
        out[bone] = result
    return out
