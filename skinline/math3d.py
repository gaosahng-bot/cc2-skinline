"""Small 3D math helpers. Complete."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, s: float) -> "Vec3":
        return Vec3(self.x * s, self.y * s, self.z * s)

    __rmul__ = __mul__

    def __neg__(self) -> "Vec3":
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, o: "Vec3") -> float:
        return self.x * o.x + self.y * o.y + self.z * o.z

    def cross(self, o: "Vec3") -> "Vec3":
        return Vec3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> "Vec3":
        n = self.norm()
        if n < 1e-12:
            return Vec3(0, 0, 0)
        return self * (1.0 / n)

    def lerp(self, o: "Vec3", t: float) -> "Vec3":
        return self + (o - self) * t

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]


@dataclass(frozen=True)
class Quat:
    x: float
    y: float
    z: float
    w: float

    @staticmethod
    def identity() -> "Quat":
        return Quat(0, 0, 0, 1)

    @staticmethod
    def from_axis_angle(axis: Vec3, radians: float) -> "Quat":
        a = axis.normalized()
        s = math.sin(radians / 2.0)
        return Quat(a.x * s, a.y * s, a.z * s, math.cos(radians / 2.0)).normalized()

    def conjugate(self) -> "Quat":
        return Quat(-self.x, -self.y, -self.z, self.w)

    def normalized(self) -> "Quat":
        n = math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2)
        if n < 1e-12:
            return Quat.identity()
        return Quat(self.x / n, self.y / n, self.z / n, self.w / n)

    def __mul__(self, o: "Quat") -> "Quat":
        return Quat(
            self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
            self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
        ).normalized()

    def rotate(self, v: Vec3) -> Vec3:
        qv = Quat(v.x, v.y, v.z, 0)
        r = self * qv * self.conjugate()
        return Vec3(r.x, r.y, r.z)

    def slerp(self, o: "Quat", t: float) -> "Quat":
        a = self.normalized()
        b = o.normalized()
        dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
        if dot < 0:
            b = Quat(-b.x, -b.y, -b.z, -b.w)
            dot = -dot
        if dot > 0.9995:
            return Quat(
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z),
                a.w + t * (b.w - a.w),
            ).normalized()
        theta = math.acos(max(-1.0, min(1.0, dot)))
        s = math.sin(theta)
        wa = math.sin((1 - t) * theta) / s
        wb = math.sin(t * theta) / s
        return Quat(a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb).normalized()

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.w]

    def to_mat4(self, t: Vec3 | None = None, s: Vec3 | None = None) -> "Mat4":
        x, y, z, w = self.normalized().as_list()
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        m = [
            1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0,
            2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0,
            2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0,
            0, 0, 0, 1,
        ]
        mat = Mat4(m)
        if s is not None:
            mat = Mat4.scale(s) * mat if False else mat.scale_left(s)
        if t is not None:
            mat = Mat4.translation(t) * mat
        return mat


class Mat4:
    def __init__(self, m: list[float]) -> None:
        if len(m) != 16:
            raise ValueError("mat4 needs 16 values, row-major")
        self.m = [float(x) for x in m]

    @staticmethod
    def identity() -> "Mat4":
        return Mat4([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])

    @staticmethod
    def translation(t: Vec3) -> "Mat4":
        return Mat4([1, 0, 0, t.x, 0, 1, 0, t.y, 0, 0, 1, t.z, 0, 0, 0, 1])

    @staticmethod
    def scale(s: Vec3) -> "Mat4":
        return Mat4([s.x, 0, 0, 0, 0, s.y, 0, 0, 0, 0, s.z, 0, 0, 0, 0, 1])

    def scale_left(self, s: Vec3) -> "Mat4":
        return Mat4.scale(s) * self

    def __mul__(self, o: "Mat4") -> "Mat4":
        a, b = self.m, o.m
        r = [0.0] * 16
        for i in range(4):
            for j in range(4):
                r[i * 4 + j] = (
                    a[i * 4 + 0] * b[0 * 4 + j]
                    + a[i * 4 + 1] * b[1 * 4 + j]
                    + a[i * 4 + 2] * b[2 * 4 + j]
                    + a[i * 4 + 3] * b[3 * 4 + j]
                )
        return Mat4(r)

    def transform_point(self, v: Vec3) -> Vec3:
        m = self.m
        x = m[0] * v.x + m[1] * v.y + m[2] * v.z + m[3]
        y = m[4] * v.x + m[5] * v.y + m[6] * v.z + m[7]
        z = m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]
        w = m[12] * v.x + m[13] * v.y + m[14] * v.z + m[15]
        if abs(w) > 1e-12:
            x, y, z = x / w, y / w, z / w
        return Vec3(x, y, z)

    def transform_vector(self, v: Vec3) -> Vec3:
        m = self.m
        return Vec3(
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[4] * v.x + m[5] * v.y + m[6] * v.z,
            m[8] * v.x + m[9] * v.y + m[10] * v.z,
        )

    def inverse(self) -> "Mat4":
        # general 4x4 inverse (Gauss-Jordan)
        a = self.m[:] + Mat4.identity().m
        # We store two 4x4 side by side conceptually — use 4x8
        m = [[0.0] * 8 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                m[i][j] = self.m[i * 4 + j]
                m[i][j + 4] = 1.0 if i == j else 0.0
        for i in range(4):
            pivot = i
            for r in range(i + 1, 4):
                if abs(m[r][i]) > abs(m[pivot][i]):
                    pivot = r
            if abs(m[pivot][i]) < 1e-12:
                raise ValueError("singular matrix")
            m[i], m[pivot] = m[pivot], m[i]
            div = m[i][i]
            for j in range(8):
                m[i][j] /= div
            for r in range(4):
                if r == i:
                    continue
                f = m[r][i]
                for j in range(8):
                    m[r][j] -= f * m[i][j]
        out = []
        for i in range(4):
            out.extend(m[i][4:])
        return Mat4(out)


def trs(t: Vec3, r: Quat, s: Vec3 | None = None) -> Mat4:
    sc = s if s is not None else Vec3(1, 1, 1)
    return Mat4.translation(t) * r.to_mat4() * Mat4.scale(sc)
