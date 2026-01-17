from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

import yaml

from .schema import ArmControllerConfig


def read_yaml(path: Path) -> dict[str, Any]:
    """Read a YAML file into a dict; empty files return {}."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping at {path}, got {type(data).__name__}")
    return data


def merge_dicts(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Deep-merge override into base, preferring override values."""
    merged: dict[str, Any] = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_dicts(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_arm_controller_config(
    override_paths: Iterable[str | Path] | None = None,
    default_path: str | Path | None = None,
) -> ArmControllerConfig:
    if default_path is None:
        default_path = Path(__file__).resolve().parent / "arm_controller_default.yaml"

    merged = read_yaml(Path(default_path))

    if override_paths:
        for path in override_paths:
            merged = merge_dicts(merged, read_yaml(Path(path)))

    return ArmControllerConfig(**merged)
