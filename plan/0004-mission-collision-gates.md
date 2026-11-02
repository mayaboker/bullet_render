# Mission start and liftoff collision gates

- Status: Implemented; verified 2026-11-02
- Implementation commit: Pending

Before Start mission ignore collision termination. Enable target hits at mission
start and terrain-crash detection only after a post-start sample has at least
0.20 m geometry-based clearance and no contact. Keep physical collisions/gravity.
Latch gates for the session, clearing on stop/reset/spawn/owner/lease termination.
RC v2 carries mission_started; simulation state v3 carries mission_start_step.
Legacy RC v1 uses armed as automatic mission start; legacy state behavior stays.
Godot and physics both reject pre-start poses/reports and clear detector history.
Support manual and automatic internal/Betaflight simulation, preserving recording.

Verify shared schemas, lease/scenario gates, mission commands, native adapter,
Godot collision flow and real headless manual takeoff without motor actuation.
Preserve unrelated changes; no commits or pushes. Record final results here.

## Implementation and verification

Implemented RC v2 mission_started and state v3 mission_start_step, preserving
legacy contracts. The external runner uses the valid owner lease, publishes the
next-step start boundary promptly, gates physics target-source history, and
shares geometry-based terrain gating across internal and Betaflight paths.
Godot rejects foreign/pre-start poses before collision sweep/latch, drains/reset
state while idle, and native C++ validates the new simulation state. Stop/reset/
spawn/owner/lease termination clears gates without discarding terminal results.
Physical collision bodies, contact forces, gravity, sensors and takeoff watchdogs
are unchanged. Design/protocol/user documentation was updated.

567 unique tests passed across affected checks: bullet_app 197, bullet_msgs 184,
bullet_render 29, mcap_bagger 92, workspace 15, and 50 focused bullet_drone tests
(lease, scenario, external RC configuration, real manual takeoff, simulation
control, spawn selection, ZMQ rendering and live wind). Native CMake debug build
passed; Godot headless test_collision.gd passed existing geometry/sweep/latch
checks plus new mission boundary/legacy/foreign-episode checks. The racer and
heavy quad real PyBullet regressions hold armed idle against terrain, start
mission, cross 0.20 m clearance, and detect an actual later return to terrain.
Both internal expiry and Betaflight hold-last lease policies are tested, including
withdrawal. Terminal state v3 validates against the same shared terminal schema.
Whitespace checks passed.

The reproduction log was recordings/launch_20261102T085544.487571Z/app.log:
MANUAL_IDLE → stopped/terrain_collision → Arm confirmation timeout. The new gates
remove this path before mission start and during ground takeoff. Restart the
entire Run all stack to load the rebuilt native extension and both new protocol
writers/readers. No running flight session was restarted; no physical motors were
actuated. Actual Betaflight/SITL flight was not run; its shared gate and lease
policy were verified with fake commands. Existing unrelated work was preserved.
No commits or pushes were made; implementation commit remains Pending.
