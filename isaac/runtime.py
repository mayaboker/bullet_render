"""Runtime locations shared by the renderer and its launch probes."""
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEPENDENCIES = ROOT / 'build' / 'isaac6.2-deps'


def isaac_directory():
    return Path(os.environ.get('ISAAC_SIM_DIR') or Path.home() / 'isaacsim6.2').expanduser()
