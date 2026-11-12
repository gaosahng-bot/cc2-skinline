"""Clip sampling — NOT implemented yet."""
from __future__ import annotations

from skinline.io import Clip, Key
from skinline.math3d import Quat, Vec3


def sample_clip(clip: Clip, time: float) -> dict[str, dict]:
    """Return {bone: {"t": Vec3|None, "r": Quat|None, "s": Vec3|None}} at `time`.

    Honour `clip.loop` (wrap) vs clamp, and `clip.interpolation`:
    - linear: vec lerp, quat slerp
    - cubic: Catmull-Rom on translations, quaternion squad or cubic slerp equivalent

    Times are in seconds. If a channel is missing from a key, keep the bind pose
    (the caller will fill bind). Additive clips should be sampled the same way;
    the blend stage is responsible for applying them as a delta from bind.
    """
    raise NotImplementedError("implement clip sampling")
