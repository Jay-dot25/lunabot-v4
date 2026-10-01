"""Source-checkout compatibility facade for the ROS `lunabot_common` package."""

from .terrain_config import TerrainClass, TerrainConfig, TerrainConfigError, load_terrain_config

__all__ = ["TerrainClass", "TerrainConfig", "TerrainConfigError", "load_terrain_config"]
