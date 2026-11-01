# Shared terrain heightmap

## Goal

Export the existing Godot procedural terrain into shared heightmap/mesh assets consumed by PyBullet and use the same samples for the rendered terrain mesh and collider.

## Implementation steps

1. Add a headless exporter that samples the existing terrain function.
2. Store the binary heightmap and manifest with coordinate and scale metadata.
3. Load the shared asset for Godot mesh/collider generation and add parity checks.

## Interface changes

- Add a versioned terrain heightmap asset and manifest shared with `bullet_drone`.
- Preserve the current Godot terrain dimensions and coordinate conventions.

## Verification

- Headless exporter output is finite and deterministic.
- Godot terrain samples match the exported asset at representative coordinates.

## Scope/defaults

- Keep one 500 m heightfield; no streaming tiles in this milestone.

## Status

Implemented and verified

## Implementation commit

`471576d`
