"""Clip sampling: linear / cubic interpolation with loop wrapping."""
from __future__ import annotations

import math

from skinline.io import Clip, Key
from skinline.math3d import Quat, Vec3


def sample_clip(clip: Clip, time: float) -> dict[str, dict]:
    """Return {bone: {"t": Vec3|None, "r": Quat|None, "s": Vec3|None}} at `time`.

    Honour `clip.loop` (wrap around `duration`) vs clamp to [0, duration].
    Linear clips use vec lerp and quat slerp; cubic clips use centripetal-free
    uniform Catmull-Rom on vec channels and Shoemake's squad on rotations.
    A channel missing from a key stays None so the caller can fill the bind pose.
    """
    result: dict[str, dict] = {}
    for bone, keys in clip.tracks.items():
        if not keys:
            result[bone] = {"t": None, "r": None, "s": None}
            continue
        i0, i1, u, wrapped = _segment(keys, clip.duration, clip.loop, time)
        k0, k1 = keys[i0], keys[i1 if i1 < len(keys) else 0]
        if clip.interpolation == "cubic" and len(keys) > 2:
            im = _neighbor(i0, -1, clip.loop, len(keys), wrapped)
            ip = _neighbor(i1, 1, clip.loop, len(keys), wrapped)
            pm = keys[im]
            pn = keys[ip if ip < len(keys) else 0]
            sampled = {
                "t": _sample_vec(pm.t, k0.t, k1.t, pn.t, u),
                "r": _sample_quat(pm.r, k0.r, k1.r, pn.r, u),
                "s": _sample_vec(pm.s, k0.s, k1.s, pn.s, u),
            }
        else:
            sampled = {
                "t": _lerp_or_hold(k0.t, k1.t, u),
                "r": _slerp_or_hold(k0.r, k1.r, u),
                "s": _lerp_or_hold(k0.s, k1.s, u),
            }
        result[bone] = sampled
    return result


def _segment(keys: list[Key], duration: float, loop: bool, time: float) -> tuple[int, int, float, bool]:
    """Return (left index, right index, local parameter u, wrapped flag)."""
    first, last = keys[0].time, keys[-1].time
    wrapped = False
    if loop:
        span = duration if duration > last else last
        if span <= 0.0:
            return 0, 0, 0.0, False
        # Half-open cycle [first, span): an exact multiple of `span` belongs
        # to the wrap segment, not to the first key of a new cycle.
        rel = (time - first) % span
        if rel == 0.0 and time > first:
            rel = span
        t = first + rel
        if t < first:
            t += span
        if t >= last:
            # Final key back to the first key of the next cycle. i1 == n is a
            # virtual index pointing at key 0 at time `span`.
            gap = span - last
            u = 0.0 if gap <= 0.0 else (t - last) / gap
            return len(keys) - 1, len(keys), u, True
    else:
        t = min(max(time, first), last)
        if t <= first:
            return 0, min(1, len(keys) - 1), 0.0, False
        if t >= last:
            return max(0, len(keys) - 2), len(keys) - 1, 1.0, False
    for i in range(len(keys) - 1):
        if keys[i].time <= t <= keys[i + 1].time:
            dt = keys[i + 1].time - keys[i].time
            return i, i + 1, (t - keys[i].time) / dt if dt > 0 else 0.0, False
    return max(0, len(keys) - 2), len(keys) - 1, 1.0, False


def _neighbor(i: int, step: int, loop: bool, count: int, wrapped: bool) -> int:
    j = i + step
    if 0 <= j < count:
        return j
    if loop or wrapped:
        return j % count
    return 0 if j < 0 else count - 1


def _lerp_or_hold(a: Vec3 | None, b: Vec3 | None, u: float) -> Vec3 | None:
    if a is None or b is None:
        return a if a is not None else b
    return a.lerp(b, u)


def _slerp_or_hold(a: Quat | None, b: Quat | None, u: float) -> Quat | None:
    if a is None or b is None:
        return a if a is not None else b
    return a.slerp(b, u)


def _sample_vec(pm: Vec3 | None, a: Vec3 | None, b: Vec3 | None, pn: Vec3 | None, u: float) -> Vec3 | None:
    if a is None or b is None:
        return _lerp_or_hold(a, b, u)
    p0 = pm if pm is not None else a
    p3 = pn if pn is not None else b
    u2, u3 = u * u, u * u * u
    return (
        p0 * (-0.5 * u3 + u2 - 0.5 * u)
        + a * (1.5 * u3 - 2.5 * u2 + 1.0)
        + b * (-1.5 * u3 + 2.0 * u2 + 0.5 * u)
        + p3 * (0.5 * u3 - 0.5 * u2)
    )


def _sample_quat(pm: Quat | None, a: Quat | None, b: Quat | None, pn: Quat | None, u: float) -> Quat | None:
    if a is None or b is None:
        return _slerp_or_hold(a, b, u)
    q0 = pm if pm is not None else a
    q3 = pn if pn is not None else b
    s0 = _squad_control(q0, a, b)
    s1 = _squad_control(a, b, q3)
    return _squad(a, s0, s1, b, u)


def _quat_log(q: Quat) -> Quat:
    q = q.normalized()
    v_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z)
    if v_norm < 1e-10:
        return Quat(0.0, 0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(v_norm, q.w)
    scale = angle / v_norm
    return Quat(q.x * scale, q.y * scale, q.z * scale, 0.0)


def _quat_exp(l: Quat) -> Quat:
    angle = math.sqrt(l.x * l.x + l.y * l.y + l.z * l.z)
    if angle < 1e-10:
        return Quat.identity()
    s = math.sin(angle / 2.0) / angle
    return Quat(l.x * s, l.y * s, l.z * s, math.cos(angle / 2.0))


def _quat_mul_scaled(a: Quat, s: float) -> Quat:
    return Quat(a.x * s, a.y * s, a.z * s, a.w * s)


def _quat_add(a: Quat, b: Quat) -> Quat:
    return Quat(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w)


def _squad_control(prev: Quat, q: Quat, nxt: Quat) -> Quat:
    # s_i = q * exp(-(log(q^-1 * prev) + log(q^-1 * nxt)) / 4)
    q_inv = q.conjugate()
    term = _quat_mul_scaled(
        _quat_add(_quat_log(q_inv * prev), _quat_log(q_inv * nxt)),
        -0.25,
    )
    return q * _quat_exp(term)


def _squad(q0: Quat, s0: Quat, s1: Quat, q1: Quat, u: float) -> Quat:
    a = q0.slerp(q1, u)
    b = s0.slerp(s1, u)
    return a.slerp(b, 2.0 * u * (1.0 - u))
