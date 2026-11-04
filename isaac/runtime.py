"""Runtime locations shared by the renderer and its launch probes."""
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
# SDK 6.2 embeds Python 3.12; keep its binary wheels separate from SDK 5.0/3.11.
DEPENDENCIES = ROOT / 'build' / 'isaac6.2-deps'


def isaac_directory():
    return Path(os.environ.get('ISAAC_SIM_DIR') or Path.home() / 'isaacsim6.2').expanduser()
