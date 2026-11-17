# Run-all background service launcher

## Goal

Make the Run all workflow start Bullet View and Bullet Visual in the background,
then run the flight supervisor in the foreground with reliable cleanup.

## Implementation

- Add `tools/run_all.sh` with backend/drone profile selection, build steps,
  background service PIDs, and an exit cleanup trap.
- Keep `tools/run_flight.py` responsible for recorder, Godot, physics, app, SITL,
  logging, and MCAP lifecycle supervision.
- Update the VS Code Run all task to invoke the shell launcher directly.
- Keep Bullet Visual Open as a standalone task without making Run all depend on it.

## Verification

- Validate shell syntax and workspace JSON.
- Run bullet_visual and bullet_app test suites plus diff checks.

## Status

Complete.

## Implementation commit

`1b5378f`
