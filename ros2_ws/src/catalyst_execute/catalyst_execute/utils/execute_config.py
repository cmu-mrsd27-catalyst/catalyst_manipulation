"""Load package YAML defaults for catalyst_execute nodes (install or source tree)."""

from __future__ import annotations

import os
from typing import Any, Dict

import yaml

_CONFIG_CACHE: Dict[str, Any] | None = None


def _resolve_yaml_path() -> str | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory('catalyst_execute')
        p = os.path.join(share, 'config', 'catalyst_execute_params.yaml')
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    # utils/ -> inner catalyst_execute/ -> package root (config/ lives here)
    src = os.path.normpath(
        os.path.join(here, '..', '..', 'config', 'catalyst_execute_params.yaml'))
    if os.path.isfile(src):
        return src
    return None


def load_execute_params() -> dict:
    global _CONFIG_CACHE
    if _CONFIG_CACHE is not None:
        return _CONFIG_CACHE
    path = _resolve_yaml_path()
    if path and os.path.isfile(path):
        with open(path, 'r', encoding='utf-8') as f:
            _CONFIG_CACHE = yaml.safe_load(f) or {}
    else:
        _CONFIG_CACHE = {}
    return _CONFIG_CACHE


def section(name: str) -> dict:
    d = load_execute_params().get(name)
    return dict(d) if isinstance(d, dict) else {}
