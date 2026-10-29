"""The launch command must select NVIDIA PRIME and preserve Godot arguments."""
import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def test_godot_launch_selects_nvidia_prime(tmp_path):
    godot = tmp_path / 'godot'
    godot.write_text('#!/bin/sh\nprintf "%s\\n" "$__NV_PRIME_RENDER_OFFLOAD" "$__GLX_VENDOR_LIBRARY_NAME" "$@"\n')
    godot.chmod(0o755)
    result = subprocess.run([str(ROOT / 'tools/run_godot.sh'), '--path', 'project with spaces', '--script', 'test_scene.gd'],
                            env=dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ['PATH'],
                                     __NV_PRIME_RENDER_OFFLOAD='0', __GLX_VENDOR_LIBRARY_NAME='mesa'),
                            capture_output=True, text=True, check=True)
    assert result.stdout.splitlines() == ['1', 'nvidia', '--display-driver', 'x11', '--rendering-method',
                                        'gl_compatibility', '--rendering-driver', 'opengl3', '--disable-vsync',
                                        '--path', 'project with spaces', '--script', 'test_scene.gd']
