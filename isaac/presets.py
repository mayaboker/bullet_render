"""Per-world defaults for the Isaac renderer; importable without Isaac or its dependencies."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD_PRESETS = ROOT / 'isaac' / 'world_presets.json'


def world_preset(world):
    """Preset entry for the world's file name, or an empty dict without a world or entry."""
    if world is None:
        return {}
    presets = json.loads(WORLD_PRESETS.read_text())
    entry = presets.get(Path(world).name, {})
    if not isinstance(entry, dict):
        raise ValueError(f'invalid world preset for {Path(world).name}')
    return entry
