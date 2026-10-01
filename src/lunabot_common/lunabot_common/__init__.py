"""Dependency-light shared LunaBot definitions."""

from .terrain_config import TerrainClass, TerrainConfig, TerrainConfigError, load_terrain_config

__all__ = [
    "TerrainClass",
    "TerrainConfig",
    "TerrainConfigError",
    "load_terrain_config",
]
