"""Repeated configure/build must not rewrite an unchanged Godot API or extension."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
API = ROOT / 'build/deps/godot_cpp-build/extension_api.json'
LIBRARY = ROOT / 'godot/bin/libzmq_camera.so'


def cmake(*args):
    result = subprocess.run(['cmake', *args], cwd=ROOT, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=180)
    assert result.returncode == 0, result.stdout
    return result.stdout


def test_unchanged_configure_build_does_not_recompile():
    cmake('--preset', 'debug')
    cmake('--build', '--preset', 'debug')
    before = (API.stat().st_mtime_ns, LIBRARY.stat().st_mtime_ns)
    cmake('--preset', 'debug')
    assert API.stat().st_mtime_ns == before[0], 'Configure rewrote the unchanged trimmed Godot API'
    output = cmake('--build', '--preset', 'debug')
    assert LIBRARY.stat().st_mtime_ns == before[1], output
    assert 'no work to do' in output, output
