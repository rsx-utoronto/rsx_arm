from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

import yaml
from rclpy.node import Node

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


def normalize_override_paths(
    override_paths: Iterable[str | Path] | str | Path | None,
) -> list[str | Path]:
    if override_paths is None:
        return []
    if isinstance(override_paths, Path):
        return [override_paths]
    if isinstance(override_paths, str):
        parts = [part.strip() for part in override_paths.split(",")]
        return [part for part in parts if part]
    return list(override_paths)


def load_arm_controller_config(
    override_paths: Iterable[str | Path] | str | Path | None = None,
    default_path: str | Path | None = None,
) -> ArmControllerConfig:
    if default_path is None:
        default_path = Path(__file__).resolve().parent / "arm_controller_default.yaml"

    merged = read_yaml(Path(default_path))

    for path in normalize_override_paths(override_paths):
        merged = merge_dicts(merged, read_yaml(Path(path)))

    return ArmControllerConfig(**merged)


def load_arm_controller_config_from_node(
    node: Node,
    *,
    default_path: str | Path | None = None,
) -> ArmControllerConfig:
    node.declare_parameter("config_file", "")
    node.declare_parameter("config_overrides", "")

    config_file = node.get_parameter("config_file").value or None
    override_paths = node.get_parameter("config_overrides").value

    return load_arm_controller_config(
        override_paths=override_paths,
        default_path=config_file or default_path,
    )
