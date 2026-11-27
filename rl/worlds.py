"""Load a named obstacle layout ("world") from a YAML file.

A world file is a mapping with an ``obstacles`` key holding a list of
``[x, y, r]`` triples: the centre and radius of each circular obstacle,
in metres, inside the 10x10 area. See ``rl/configs/worlds/`` for examples.
"""

from __future__ import annotations

from pathlib import Path

import yaml


def load_world(path: str | Path) -> list[tuple[float, float, float]]:
    """Read the ``obstacles`` list from a world YAML file.

    Returns a list of ``(x, y, r)`` tuples; ``[]`` for an explicitly
    empty world. Raises ``ValueError`` on a malformed file.
    """
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if not isinstance(data, dict) or "obstacles" not in data:
        raise ValueError(f"{path}: expected a mapping with an 'obstacles' key")
    obstacles = data["obstacles"]
    if obstacles is None:
        return []
    if not isinstance(obstacles, list):
        raise ValueError(f"{path}: 'obstacles' must be a list of [x, y, r] triples")

    parsed: list[tuple[float, float, float]] = []
    for entry in obstacles:
        if not isinstance(entry, (list, tuple)) or len(entry) != 3:
            raise ValueError(
                f"{path}: each obstacle must be an [x, y, r] triple, got {entry!r}"
            )
        x, y, r = (float(value) for value in entry)
        if r <= 0:
            raise ValueError(
                f"{path}: obstacle radius must be positive, got {entry!r}"
            )
        parsed.append((x, y, r))
    return parsed
