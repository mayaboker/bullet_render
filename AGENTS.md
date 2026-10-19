# Repository Guidelines

## Project structure

`src/native/` owns the C++ ZmqCameraAdapter GDExtension. `godot/` owns the scene,
landscape, camera and real-render checks. Sibling `bullet_msgs` owns shared JSON
contracts and the SHM wire specification.
`configs/` contains physics, transport and recorder profiles. Python under `tests/`
and `tools/` is test tooling, not a runtime bridge. Create directories with content.

## Build and checks

Use C++20, GCC, CMake/Ninja presets and Godot 4.7 on Linux x86_64. Run
`cmake --preset debug` and `cmake --build --preset debug` from this root; CMake
fetches pinned sources into ignored build/deps, links installed libzmq, and imports
the extension into Godot. Do not silently upgrade pinned native dependencies.

Use `uv sync --locked` and `uv run --locked pytest -q` for native transport tests.
`godot --headless --path godot --script test_frames.gd` checks transforms/FOV;
`godot --path godot --script test_scene.gd` needs a real graphics display.
See README.md for complete launch, MCAP and fault-check commands. Keep uv.lock
aligned with pyproject.toml. Do not install system tools automatically.

## Style and boundaries

Use four spaces in C++/Python and tabs in GDScript; follow .editorconfig.
Keep ZeroMQ receiver workers independent of Godot objects: copy bounded raw
messages into protected slots, then parse/validate and mutate scene objects on the main thread.
Godot owns rendering; physics owns startup settings and capture cadence. Use
bounded queues and nonblocking image publication.
Every YAML field needs an adjacent usage/units/default comment where relevant.

## Tests and generated data

Add focused checks for behavior, malformed input, stale/duplicate data, failure and
cleanup. TCP tests must use real sockets and verify exact topic matching.
Headless Godot cannot validate camera pixels; render tests need a display. Keep native build
outputs, .godot caches, environments and recordings untracked. Arrival-timestamp
MCAP does not provide exact image/pose association.

## Changes and PRs

Keep changes focused. PRs should describe behavior, validation and limitations,
with screenshots for visual changes. List checks not run. Sibling changes follow
their own AGENTS.md. Commit and push only when requested.
