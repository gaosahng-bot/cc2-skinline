"""Bake plan evaluator — NOT implemented yet."""
from __future__ import annotations


def bake(scene_path: str, plan_path: str, out_dir: str) -> None:
    """Evaluate the plan, write pose.jsonl, preview PPMs, and summary.json.

    See docs/pose-dump-schema.md and docs/architecture.md.
    Reuse inspect-able loaders and math; implement the missing modules.
    """
    raise NotImplementedError("implement bake")
