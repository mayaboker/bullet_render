#!/usr/bin/env bash
set -euo pipefail

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

exec godot --display-driver x11 --rendering-method gl_compatibility --rendering-driver opengl3 --disable-vsync "$@"
