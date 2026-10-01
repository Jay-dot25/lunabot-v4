"""Source-checkout compatibility facade for the ROS `lunabot_common` package."""

from .terrain_config import TerrainClass, TerrainConfig, TerrainConfigError, load_terrain_config
from .status_compat import LegacyStatus, mission_fields, parse_legacy_status, planner_fields, replan_fields

__all__ = [
    "TerrainClass", "TerrainConfig", "TerrainConfigError", "load_terrain_config",
    "LegacyStatus", "parse_legacy_status", "planner_fields", "replan_fields",
    "mission_fields",
]
