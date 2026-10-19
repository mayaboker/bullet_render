# Bullet Render — native Godot / ZeroMQ camera

This project is a member of the [Bullet workspace](../README.md). All six
projects share `../.venv` and `../uv.lock`; run uv commands with the
workspace flags below to retain every project and optional dependency.

Godot renders a landscape and forward camera from `bullet_drone` ground-truth
poses. The native C++ adapter receives JSON telemetry over loopback ZMQ TCP and
publishes image descriptors over TCP, with RGB8 pixels in POSIX shared memory.
Sibling `bullet_msgs` owns the shared schemas, wire contract and SHM reader.
Sibling `bullet_app` provides the live camera/bounding-box preview with `--debug`.

A parked red delivery truck sits **50 m ahead of the physics origin** (horizontal
ground distance, Bullet X=50 m / Godot Z=−50 m), following the terrain slope. The
drone spawn stays at its physics position. The default spectator view is wide
enough to include both; right-drag or scroll to inspect the truck. Scenery,
including the truck, has no Bullet collision body. Godot detects truck contact
to end app-owned episodes without applying collision forces.

## Build

Prerequisites on Ubuntu 24.04: Godot 4.7, GCC with C++20, CMake, Ninja, pkg-config,
libzmq development headers (tested with 4.3.5), Python 3.12 and uv. No Rust build is
needed. If libzmq headers are missing, install `libzmq3-dev` through your package
manager. Godot-cpp remains pinned to Godot 4.5 stable and is fetched by CMake.

```bash
uv sync --locked --all-packages --all-extras --all-groups
cmake --preset debug
cmake --build --preset debug
```

Unchanged configure/build runs reuse the existing bindings and extension; real
source, profile or build-setting changes still trigger the required rebuild.

The output is `godot/bin/libzmq_camera.so`, linked to system libzmq. CMake imports
the extension into Godot; a one-second frame delay avoids Godot's cold-import
shutdown race (upstream issue #111048). Generated files remain ignored.

Keep the sibling `bullet_msgs` checkout alongside this repository. Prepare consumers:

```bash
cd ../bullet_drone
uv sync --locked --all-packages --all-extras --all-groups
cd ../mcap_bagger
uv sync --locked --all-packages --all-extras --all-groups
cd ../bullet_app
uv sync --locked --all-packages --all-extras --all-groups
```

## Launch

Run these in separate terminals, from the indicated directories. The recorder
waits for physics to report a running simulation before creating a file.

```bash
# mcap_bagger
uv run --locked --all-packages --all-extras --all-groups mcap-bagger --config ../bullet_render/configs/recorder.yaml
```

```bash
# bullet_render; NVIDIA PRIME offload verified on the RTX 5070 Laptop GPU
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia godot --path godot
```

```bash
# bullet_drone
uv run --locked --all-packages --all-extras --all-groups python -m examples.external_rc.main \
  --render-config ../bullet_render/configs/physics.json
```

For PlotJuggler, run the workspace `Bullet: Plotter (live position)` task and connect
its ZMQ Subscriber to TCP transport, address `127.0.0.1`, port `5555`, topic
`render/drone_1/pose`, using JSON and `simulation_time_s` as the embedded timestamp. The workspace
`Bullet: Run all` task opens renderer, physics, MCAP recorder, plotter and camera/bbox app in dedicated terminals;
the renderer uses NVIDIA PRIME. MCAP recording is automatic; PlotJuggler needs its connection. `Bullet: Open tools` lets
you prepare them before starting `Bullet: Position flight`.

Use workspace task **Bullet: Physics + render + app** to start app-owned takeoff/LOS physics,
Godot renderer, automatic MCAP recorder and camera/bbox app together in dedicated terminals.

## Simulation toolbar

All four physics example CLIs start **Stopped** and stay alive between runs.
In Godot's bottom-left toolbar, the play triangle starts a fresh episode, the
stop square ends the run while keeping its final pose, and the reset arrow
restores the initial pose at time zero and waits. Normal mission completion also
returns to Stopped. Click or tab to a button and press Enter/Space.

Godot broadcasts commands on loopback TCP **5557**, and physics acknowledges actual lifecycle state. The external-RC example also
requires fresh app readiness before Start. The toolbar shows actual physics state from telemetry, not
an optimistic click result. It remains disabled until physics sees this Godot
controller's heartbeat. A missed command may require another click. Three seconds
without Godot's 500 ms heartbeat ends an active run; reconnecting never auto-starts.

The held pose is republished once per second while stopped, including for late
viewers. Physics steps and simulation time remain frozen. Errors end the episode
and appear in status; Start/Reset attempts a fresh reset.

The MCAP service creates one file when physics reports a running episode, including
when it joins an ongoing run. Stop, Reset, mission completion, physics errors, or
three seconds without valid physics state drains and closes the file. Reset does
not open another file. Repeated Start keeps the current file. A closed or failed
episode is never reopened by delayed state messages; the next episode gets a new
file. Recording failures leave `.mcap.partial` files and do not stop physics. The
manual recorder prompt remains available by omitting `--config`. Recording is best
effort: waiting for running state and PUB/SUB delivery can miss initial/final samples.

## Transport and interfaces

`configs/transport.json` holds distinct loopback TCP endpoints for telemetry, images,
and control (`control_endpoint`). Publishers bind;
subscribers connect and automatically reconnect. No broker is needed.

| Publisher | Endpoint | Topic and payload | Subscribers |
| --- | --- | --- | --- |
| Physics | `tcp://127.0.0.1:5555` | `render/drone_1/config`, JSON repeated every second | Godot |
| Physics | Same port 5555 | `render/drone_1/pose`, JSON at 30 Hz simulation time | Godot, MCAP, PlotJuggler |
| Godot | `tcp://127.0.0.1:5557` | `simulation/command` and `simulation/heartbeat`, JSON | Physics, future consumers |
| Physics | Same port 5555 | `simulation/state`, JSON | Godot, future consumers |
| Godot | `tcp://127.0.0.1:5556` | `camera/front/image`, SHM descriptor | MCAP, video viewer |

Physics runs at 240 Hz. Its configured capture/pose rate remains 30 Hz; physics
owns camera settings and cadence. Config/pose telemetry retains exactly two parts,
topic and JSON. Image notifications are a single message containing exact topic
plus NUL, little-endian uint32 JSON length, and descriptor JSON, with no pixels.
The old IPC/socket-pixel wire format is replaced and all components must migrate
together. Configurable TCP ports are supported; all image consumers must share
the renderer's Linux host/user.

Pose messages now use **schema version 2** and include
`linear_velocity_m_s: [vx, vy, vz]`: Bullet world-frame ground-truth velocity in
m/s, Z upward, from the same physics snapshot as position. PlotJuggler can plot
`linear_velocity_m_s[0]`, `[1]`, and `[2]`; MCAP records the vector with the pose.
Rebuild/restart the native renderer and restart physics/recorder together after
updating the shared schema. Strict consumers reject old pose v1 or missing
velocity. Existing MCAP files retain their original embedded schemas. Config,
control, image versions and TCP endpoints are unchanged.

Godot allocates three 0600 POSIX SHM slots after learning camera dimensions.
Writers take nonblocking exclusive locks; readers take nonblocking shared locks
and copy pixels before unlocking. Busy, incomplete, reused and missing slots are
skipped, so neither recorder nor viewer blocks rendering. Unique sessions permit
renderer restart. Normal close unlinks owned objects; abrupt kill can leave
orphans requiring exact-session cleanup. See [bullet_msgs](../bullet_msgs/README.md)
for the header layout, validation rules and cleanup instructions.

`ZmqCameraAdapter` provides `start(transport_path)`, `poll_request()`,
`publish_frame(pixels, request)`, `get_stats()` and idempotent `close()`.
`publish_control(action)` only enqueues a local event; `get_control_state()` returns
the latest valid physics state or an empty dictionary after three seconds without
a valid state. Socket ownership remains on the native worker.
Statistics add `shm_session`, `busy_slots` and `publication_drops`; `frames` counts
accepted notification submissions, not consumer deliveries. Worker threads never
access Godot objects. Camera dimensions remain fixed for a renderer run.

Telemetry high-water marks remain 64 messages, image descriptors 8. Zero linger
and nonblocking publication remain in place. The recorder uses `input: shm_image`,
copies pixels into its bounded 64 MiB queue, and reports skipped SHM images.
Malformed data and queue overflow fail recording rather than hiding accepted-data
loss. ZMQ can also drop notifications before receipt, which counters cannot fully
observe. Arrival-time MCAP does not provide exact image/pose association.

Start the camera/bbox preview (or use workspace **Bullet: App (debug)**):

```bash
cd ../bullet_app
uv run --locked --all-packages --all-extras --all-groups bullet-app --transport-config ../bullet_render/configs/transport.json \
  --mission-config ../los_tracking/examples/bullet_profile.json --debug
```

The app supports one exact topic per process, dimensions from descriptors,
bounding boxes and stream status, reconnect, and Q/Esc/window-close exit. Godot still performs
GPU readback; scenery remains visual with no Bullet collision geometry.

## Checks and benchmark

```bash
uv run --locked --all-packages --all-extras --all-groups pytest -q
godot --headless --path godot --script test_frames.gd
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  godot --path godot --script test_scene.gd
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  uv run --locked --all-packages --all-extras --all-groups \
  python tools/check_pipeline.py --seconds 60 --report-dir /tmp/bullet-zmq-check
# Add --fault-checks for late recorder startup, physics pause, slow Godot and restart.
# Check the automatic recorder service with Start, Stop, Reset and physics loss:
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  uv run --locked --all-packages --all-extras --all-groups \
  python tools/check_recording_controls.py --report-dir /tmp/bullet-recording-controls
```

The pipeline check independently reads MCAP images/poses, checks camera bytes and
arrival timestamps, and reports FPS, CPU usage (100% = one core), queue peak and
last observed native counters. Logs and a camera preview remain in the report
directory; temporary recordings are deleted. Mean capture/readback and publication
timings are sampled in periodic Godot logs. The final snapshot may precede shutdown.

Historical socket-pixel performance results are recorded in [BENCHMARK.md](BENCHMARK.md); they are not SHM measurements. The source rate
caps delivered FPS at 30, and mission restart gaps reduce the full-run average.

Migration validation results are recorded in sibling
[plan 0023](../bullet_drone/plan/0023-zmq-tcp-shm.md). The real-render pipeline check
writes its logs, independent MCAP readback report and camera PNG under the report
directory. Full performance tuning is outside this migration.

Simulation-control validation and limits are recorded in
[plan 0024](../bullet_drone/plan/0024-godot-simulation-controls.md).

### Bullet application detections

The sibling `bullet_app` receives camera images, calls `los_tracking`, and publishes
`tracking/front/bbox` JSON on loopback port 5558. The recorder profile records it
with `bullet_msgs:bbox.json` during each episode. **Bullet: Physics + render + app**
and **Bullet: Run all** launch the application in debug mode with bounding boxes.
Run `uv sync --locked --all-packages --all-extras --all-groups` in `../bullet_app` before using these tasks; see the
[application README](../bullet_app/README.md) for the standalone command.

## App flight verification

The combined tasks launch the app with `--mission-config
../los_tracking/examples/bullet_profile.json`. Wait for app READY, then press
Godot Start: warmup, arm, climb 20 m relative to the initial barometer, settle
for 3 s and wait for a fresh post-hover bbox before LOS. Missing bbox times out
after 5 s; expired RC or LOS faults freeze physics. Renderer-confirmed truck
contact completes the scenario; landing is not inferred. See [app mission details](../bullet_app/README.md). The separate
Position flight task still runs the square.

With a real graphics display, run the isolated-port check:

```bash
uv run --locked --all-packages --all-extras --all-groups python tools/check_app_flight.py
```

It launches physics, app, Godot and MCAP, checks the mission through LOS, kills
the app to verify the expiry watchdog freezes physics, and reads back the bag.
Logs/report are retained in `/tmp/bullet-app-flight`; temporary bags are removed.
The native adapter preserves source pose/feedback identity in descriptor v2,
using the measurement timestamp for capture_ns; binary SHM remains v1.

## Truck collision and completion

In app-owned flight, Godot checks a **0.42 m drone sphere** against convex
colliders derived from individual red-truck meshes. Initial overlap and swept
queries detect contact even when poses skip past the truck. Collision queries run
on the main physics thread, independently of GPU readback. Terrain, trees and
rocks remain outside detection; there is no impact force, bounce or landing.

Godot reports `render/drone_1/collision` on existing port 5557. Physics validates
controller/episode identity and the stamped source against its recent history,
freezes before another step and publishes `scenario/ended` on port 5555.
Simulation state v2 embeds the same success/target_hit result. The app latches
COMPLETED, withdraws RC intent, and Godot shows **Completed / Target hit**. All
processes stay open. Start or Reset clears the result for a fresh episode.

Reports retry every 100 ms. Without confirmation after one second, Godot displays
an acknowledgement failure and requests ordinary Stop; it does not claim success.
The confirmed result repeats for late subscribers. Its contact source and final
frozen physics source are separate: physics may advance during report delivery
and is not rewound to the contact point.

The recorder stores raw proposals and one confirmed event per event ID, then
allows 250 ms for final feedback/app diagnostics before closing. A stopped-state
snapshot also supplies the result if the standalone event was missed. Arrival
RawImage timestamps still do not establish exact image/pose association.

```bash
# Real-display deterministic source trajectory through production RC/lifecycle wiring.
uv run --locked --all-packages --all-extras --all-groups python tools/check_collision.py
# Discard all collision reports: verify timeout, Stop and absence of false success.
uv run --locked --all-packages --all-extras --all-groups python tools/check_collision.py \
  --reject-collisions --report-dir /tmp/bullet-collision-timeout
# Geometry alone (headless does not validate pixels).
godot --headless --path godot --script test_collision.gd
```

The integration probe substitutes deterministic observations for physical motion;
collision detection, transport, physics-loop termination, app and recorder use
production code. It deliberately drops the first report to verify retry. This
checks termination without requiring LOS tuning to navigate into the truck.
Logs/report and a completion screenshot remain in `/tmp/bullet-collision/`;
temporary recordings are removed. The separate square examples retain their
legacy lifecycle and do not enable this stamped-pose collision flow.

## LOS graph and MCAP topic

The mission app publishes **tracking/front/los** on the existing port5558, and
`configs/recorder.yaml` records it alongside the other app/physics streams.
It contains bearing in **degrees**, LOS rate in **degrees/second**, TTC in seconds,
fit validity/reason/residual, timestamps and mission/navigation state. Steady
samples are30Hz; state changes are immediate. Existing mission diagnostics remain
in radians. See [app telemetry fields](../bullet_app/README.md).

The real-display checks validate the stream in MCAP:
`tools/check_app_flight.py` compares sampled degrees with existing radian mission
diagnostics, and `tools/check_collision.py` checks the final inactive LOS marker
is saved within the250ms collision-finalization drain. Test bags are temporary;
normal task recordings still save to `recordings/`.
