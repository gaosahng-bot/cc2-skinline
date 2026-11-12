"""CLI. `bake` is the work to implement."""
from __future__ import annotations

import argparse


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="skinline")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("inspect", help="print skeleton and clip names (already works)")
    s.add_argument("--scene", required=True)
    b = sub.add_parser("bake", help="evaluate a plan and write pose dumps")
    b.add_argument("--scene", required=True)
    b.add_argument("--plan", required=True)
    b.add_argument("--out", required=True)
    args = p.parse_args(argv)
    if args.cmd == "inspect":
        from skinline.io import load_scene
        scene = load_scene(args.scene)
        print(f"bones={len(scene.skeleton)} verts={len(scene.mesh)} clips={list(scene.clips)}")
        for bone in scene.skeleton:
            print(f"  {bone.name:12} parent={bone.parent} length={bone.length:.3f}")
        return 0
    if args.cmd == "bake":
        from skinline.bake import bake
        bake(args.scene, args.plan, args.out)
        return 0
    return 2
