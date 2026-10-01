"""Installation smoke-test entry point for the shared package."""


def main() -> None:
    from .terrain_config import load_terrain_config

    config = load_terrain_config()
    print(f"lunabot_common 0.1.0: {len(config.classes)} terrain classes loaded")
