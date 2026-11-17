# Safe terrain spawn picker clearance

## Goal

Keep picked and default Godot spawn positions compatible with the physics preflight terrain-clearance check.

## Changes

- Raise the picker/preset terrain offset from 0.10 m to 0.50 m.
- Keep the physics-side AABB validation authoritative and preserve rejected-spawn feedback.
- Update the headless spawn-panel checks for the new automatic Z value.

## Verification

- Godot spawn tests pass with default and picked positions.
- Existing position bounds and draft/applied separation remain unchanged.

## Status

Implemented; Godot spawn test passed

## Implementation commit

`adf9c73`
