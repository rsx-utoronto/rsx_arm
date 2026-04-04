from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

import yaml
from ament_index_python.packages import get_package_share_directory as _get_share
from rclpy.node import Node

_SHARE = Path(_get_share("arm_utilities")) / "arm_configs"


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


def load_config(
    default_path: str | Path,
    override_paths: Iterable[str | Path] | str | Path | None = None,
) -> dict[str, Any]:
    """Load and deep-merge one or more YAML files, returning a plain dict.

    Args:
        default_path: Path to the base YAML file.
        override_paths: One or more override YAML files deep-merged on top of the base.
    """
    merged = read_yaml(Path(default_path))
    for path in normalize_override_paths(override_paths):
        merged = merge_dicts(merged, read_yaml(Path(path)))
    return merged


def load_config_from_node(
    node: Node,
    *,
    default_path: str | Path,
    param_prefix: str = "",
) -> dict[str, Any]:
    """Load a config from ROS 2 node parameters, returning a plain dict.

    Declares and reads two node parameters (optionally prefixed to avoid
    collisions when loading multiple configs from the same node):
        {param_prefix}_config_file:      override for the base YAML path (optional).
        {param_prefix}_config_overrides: comma-separated override YAML paths (optional).
    """
    p = f"{param_prefix}_" if param_prefix else ""
    file_param      = f"{p}config_file"
    overrides_param = f"{p}config_overrides"

    if not node.has_parameter(file_param):
        node.declare_parameter(file_param, "")
    if not node.has_parameter(overrides_param):
        node.declare_parameter(overrides_param, "")

    config_file    = node.get_parameter(file_param).value or None
    override_paths = node.get_parameter(overrides_param).value

    return load_config(
        default_path=config_file or default_path,
        override_paths=override_paths,
    )


# --- named config helpers ---

def load_arm_controller_config(
    override_paths: Iterable[str | Path] | str | Path | None = None,
    default_path: str | Path | None = None,
) -> dict[str, Any]:
    return load_config(
        default_path=default_path or _SHARE / "arm_controller_default.yaml",
        override_paths=override_paths,
    )


def load_arm_controller_config_from_node(
    node: Node,
    *,
    default_path: str | Path | None = None,
) -> dict[str, Any]:
    return load_config_from_node(
        node,
        default_path=default_path or _SHARE / "arm_controller_default.yaml",
        param_prefix="arm_controller",
    )


def load_keyboard_config(
    override_paths: Iterable[str | Path] | str | Path | None = None,
    default_path: str | Path | None = None,
) -> dict[str, Any]:
    return load_config(
        default_path=default_path or _SHARE / "redragon_k552.yaml",
        override_paths=override_paths,
    )


def load_keyboard_config_from_node(
    node: Node,
    *,
    default_path: str | Path | None = None,
) -> dict[str, Any]:
    return load_config_from_node(
        node,
        default_path=default_path or _SHARE / "redragon_k552.yaml",
        param_prefix="keyboard",
    )