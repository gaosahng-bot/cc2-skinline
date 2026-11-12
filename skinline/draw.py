"""Tiny software preview: project skinned vertices into a PPM. Complete."""
from __future__ import annotations

from pathlib import Path

from skinline.math3d import Vec3


def write_ppm(path: str | Path, points: list[Vec3], size: int = 256, scale: float = 40.0, origin: tuple[float, float] = (0.0, 1.0)) -> None:
    w = h = size
    img = [[(24, 24, 28) for _ in range(w)] for _ in range(h)]
    ox, oy = origin
    for p in points:
        x = int(w / 2 + (p.x - ox) * scale)
        y = int(h / 2 - (p.y - oy) * scale)
        if 1 <= x < w - 1 and 1 <= y < h - 1:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    img[y + dy][x + dx] = (230, 220, 80)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write(f"P3\n{w} {h}\n255\n")
        for row in img:
            f.write(" ".join(f"{r} {g} {b}" for r, g, b in row) + "\n")
