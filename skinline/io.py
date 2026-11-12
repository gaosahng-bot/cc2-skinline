"""Load the fixture JSON formats. Complete."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from skinline.math3d import Quat, Vec3


@dataclass
class Bone:
    name: str
    parent: str | None
    bind_t: Vec3
    bind_r: Quat
    bind_s: Vec3
    length: float


@dataclass
class Vertex:
    position: Vec3
    normal: Vec3
    joints: list[str]
    weights: list[float]


@dataclass
class Key:
    time: float
    t: Vec3 | None
    r: Quat | None
    s: Vec3 | None


@dataclass
class Clip:
    name: str
    duration: float
    loop: bool
    interpolation: str  # "linear" or "cubic"
    is_additive: bool
    tracks: dict[str, list[Key]]  # bone -> keys sorted by time
    root_motion: bool


@dataclass
class Scene:
    skeleton: list[Bone]
    mesh: list[Vertex]
    clips: dict[str, Clip]
    bone_index: dict[str, int] = field(default_factory=dict)

    def children(self, name: str) -> list[str]:
        return [b.name for b in self.skeleton if b.parent == name]


def _vec(v) -> Vec3:
    return Vec3(float(v[0]), float(v[1]), float(v[2]))


def _quat(v) -> Quat:
    return Quat(float(v[0]), float(v[1]), float(v[2]), float(v[3])).normalized()


def load_scene(path: str | Path) -> Scene:
    data = json.loads(Path(path).read_text())
    bones = []
    for b in data["skeleton"]["bones"]:
        trs = b["bind_local"]
        bones.append(
            Bone(
                name=b["name"],
                parent=b.get("parent"),
                bind_t=_vec(trs["t"]),
                bind_r=_quat(trs["r"]),
                bind_s=_vec(trs.get("s", [1, 1, 1])),
                length=float(b.get("length", 0.0)),
            )
        )
    mesh = []
    for v in data["mesh"]["vertices"]:
        joints = [j[0] for j in v["skin"]]
        weights = [float(j[1]) for j in v["skin"]]
        mesh.append(Vertex(_vec(v["p"]), _vec(v.get("n", [0, 1, 0])), joints, weights))
    clips = {}
    for c in data["clips"]:
        tracks = {}
        for bone, keys in c["tracks"].items():
            tracks[bone] = [
                Key(
                    time=float(k["time"]),
                    t=_vec(k["t"]) if "t" in k else None,
                    r=_quat(k["r"]) if "r" in k else None,
                    s=_vec(k["s"]) if "s" in k else None,
                )
                for k in keys
            ]
            tracks[bone].sort(key=lambda x: x.time)
        clips[c["name"]] = Clip(
            name=c["name"],
            duration=float(c["duration"]),
            loop=bool(c.get("loop", True)),
            interpolation=c.get("interpolation", "linear"),
            is_additive=bool(c.get("additive", False)),
            tracks=tracks,
            root_motion=bool(c.get("root_motion", False)),
        )
    scene = Scene(skeleton=bones, mesh=mesh, clips=clips)
    scene.bone_index = {b.name: i for i, b in enumerate(bones)}
    return scene


def load_plan(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())
