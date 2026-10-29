# Isaac Sim renderer

Use `/home/user/isaacsim5.0` (Isaac 5.0, Python 3.11) as an alternate renderer.
PyBullet owns drone motion, RC stabilization, episode state and takeoff settings.
Isaac owns the visible world, camera images, spectator controls and truck-contact queries.
The app and MCAP recorder retain their existing transport interfaces.
[docs/isaac-renderer-map.html](docs/isaac-renderer-map.html) draws the processes,
threads, ports and per-frame timing described below; open it in a browser.

## Launch

Stop Godot or the previous Isaac renderer first: they use the same image/control ports.
Keep the usual physics, mission app and recorder processes running.

```bash
cd /home/user/git/bullet/bullet_render
/home/user/isaacsim5.0/python.sh isaac/main.py
# Load the requested world and select a relative takeoff height:
/home/user/isaacsim5.0/python.sh isaac/main.py \
  --world /home/user/ov/rivermark.usd --takeoff-height 6
```

Other options: `--spawn X Y Z`, `--target X Y Z` (truck origin), `--transport-config
PATH`, `--headless`, and `--seconds N` for a bounded probe. Startup spawn/height
overrides are applied once through stopped physics, without auto-starting flight.
Omitting them preserves physics' applied settings.

Without `--target`, the truck origin comes from `isaac/world_presets.json`, keyed
by the world's file name, and otherwise defaults to `40 0 0`. Rivermark's preset
puts the truck on the road at `71.313 -26.666 5.643`; edit that file to add a
world or change its spot. The preset only sets the truck; the starting position
stays whatever physics has applied.

A complete command-line session, each in its own terminal, is physics, then the
app with the Isaac profile, then the recorder, then this renderer. Physics and
the recorder commands are the ones in the [main README](../README.md); the app
command is below. Stop Godot first if it is running.

Launch the mission app with the Isaac navigator profile rather than the Godot
one; it differs only in the image-age bound (see Camera timing):

```bash
cd ../bullet_app
uv run --locked --all-packages --all-extras --all-groups bullet-app \
  --transport-config ../bullet_render/configs/transport.json \
  --mission-config ../bullet_render/isaac/mission_profile.json --debug
```

For a fresh checkout, install transport dependencies separately from Isaac and
the shared Python environment (no system installation):

```bash
/home/user/isaacsim5.0/python.sh -m pip install \
  --target build/isaac-deps -r isaac/requirements.txt
```

## Controls and units

The **Bullet simulation controls** panel provides Start/Stop/Reset, authoritative
physics state, Perspective/Front camera/Focus drone, a USD path and Load button,
starting X/Y/Z, takeoff height and Apply, plus red-truck X/Y/Z and Move target.
World/target/scenario changes require stopped physics. Applied values and
acknowledgement status are displayed separately from editable drafts.

**Snap starting Z to world surface** and **Snap target Z to world surface** fill
the draft Z from the highest visible surface under the entered X/Y: the starting
Z gets the 0.015 m spawn clearance rounded up to the next millimetre, the target
Z is the surface itself. The notice names the prim that was hit, because a roof
counts as a surface; check it is ground before applying. Point-instanced
vegetation is ignored. Snapping only edits drafts; Apply and Move target still
send them. Nothing is snapped automatically. After Apply, **Applied XYZ** and
**Drone XYZ** both show the confirmed start; the drone marker moves only then.
Imported worlds give PyBullet no ground, so a start on the visual surface drops
briefly after Start until the app arms and climbs.

**Perspective**, **Front camera** and **Focus drone** act on the main `Viewport`
window, never on a second viewport you open. Kit otherwise treats the last
clicked viewport as active, which made Perspective appear to do nothing when a
second window showed the front camera.

All inputs use Bullet metres, Z up. Starting X/Y are limited to +/-250 m;
starting Z is absolute world altitude, 0.015-500 m. Takeoff height is a **relative
climb above the initial barometer altitude**, 0.1-100 m. For example, Z=5 with
height=6 gives an altitude goal near Z=11. This is not height above imported terrain;
an elevated drone can fall briefly before arming.

Apply uses the existing versioned spawn/height request and renderer controller identity.
Only a matching physics response changes applied settings. Start/Reset are blocked
while Apply is pending. Requests retry for two seconds; an unresolved request is
reported as unconfirmed rather than assumed successful. The app receives the
selected height from physics feedback for each fresh episode. Wait for confirmed
settings and app READY, then Start. Reset/Start clears a prior fault or completion.

## Perspective marker

A cyan cross and **DRONE / FRONT CAMERA** label identify the drone; a yellow arrow
shows camera-forward direction. Focus drone recentres the free perspective view.
This is a viewport overlay, absent from the camera RGB pixels. It cannot obscure
or confuse the detector. `/World/Bullet/Drone` also provides a named scene transform.
The camera render product is independent of the perspective viewport: the app can
show the front camera while Isaac displays another viewing angle.

## USD worlds

The source default prim is referenced into an in-memory environment root. Source
files are never saved or changed, and saved viewport/render settings are not imported.
Y-up scenes and declared units are converted to Z-up/metres. Invalid paths, absent
default prims and worlds without loaded geometry are rejected, retaining the old world.
Referenced rigid bodies are disabled in the rendering copy; flight stays in PyBullet.

Rivermark is a small layer referencing online NVIDIA assets. Its main environment
is available, but 39 distinct external prop/vegetation references were unresolved;
an affected NVIDIA URL returned HTTP 404. Available scenery loads with a visible
missing-assets notice and console warnings. Empty instance batches with entirely
missing prototypes are disabled only in the in-memory copy, avoiding invalid
bounds queries and per-instance warning floods. First downloads/shader compilation may
take time and require internet access or a populated asset cache.

Imported terrain/buildings remain **visual geometry**; they do not become PyBullet
ground or obstacles. Choose start/target positions accordingly. Only the generated
red truck completes this scenario. Detection still selects the largest qualifying
red object, rather than semantically distinguishing trucks from other red scenery.

A world's surface need not sit at Bullet Z=0. Rivermark's lot is about 5.75 m
above Z=0 at the origin and 5.85 m at X=40, with storefront roofs near 14 m from
X=60 to 70 and the plaza at 6.6 m by X=80. The default spawn (0, 0, 0.015) and a
truck at Z=0 are therefore underground there, and a front camera below the
surface renders dark. Use the Snap buttons, or enter Z by hand, before Apply and
Move target. Rivermark also paints curbs red: from a low hover the detector can
select a curb instead of a distant truck, so keep the truck large in frame.

## Confirmed contact

A 0.42 m drone sphere is checked against the same boxes used to draw the truck.
Initial overlap and exact swept contact are supported, including skipped poses,
without expanded-box corner false positives. Duplicate/stale/foreign sources are
excluded; a long pose gap does not authorize a sweep. Pure geometry checks and
report retries run in the transport worker, independently of GPU readback.

Reports use `render/drone_1/collision` with controller/source identity and event ID.
Physics validates its recent source history, freezes and publishes `scenario/ended`
and terminal state. Only that confirmation displays **COMPLETED / Target hit** and
latches completion in the app. Reports retry every 100 ms. Without confirmation after
one second, Isaac displays acknowledgement failure and requests ordinary Stop.
Reset/new episodes clear the latch. Physics may advance during report delivery;
its final pose is not rewound to first contact. The existing MCAP profile records
proposals, confirmed events, scenario settings and final app diagnostics.

## Camera timing

Dimensions/FOV, RGB8 SHM and stamped v2 descriptors retain the existing contracts.
Isaac's supplied `isaacsim.exp.base.zero_delay.kit` experience matches each capture
to its source pose. Idle rendering is limited to avoid delaying new poses; active
captures pump the UI. Original physics timestamps, sensor freshness limits and RC
expiry remain unchanged. LOS command expiry depends on image age: slow rendering
can stop physics even while the preview shows a detected box. Rich-world timing
must be measured separately; these are soft real-time processes. Arrival-time MCAP
does not provide exact image/pose association.

Measured physics-measurement-to-app image age was about 24 ms on the starter
slab but a median of 59 to 89 ms, with peaks above 120 ms, on Rivermark at about
22 delivered frames per second. The Godot profile's 0.1 s `max_image_age_s`
then rejects frames every few ticks; each rejection clears the navigator's
acquisition candidate, so the mission reports `los_acquisition_timeout` while
the preview still draws a box. `isaac/mission_profile.json` is the Godot profile
with `max_image_age_s` raised to 0.2 s. RC lease and command validity are
unchanged, so the physics expiry watchdog still freezes on app loss; the
bearing filter and TTC fit use measurement timestamps, so added age is tolerated
rather than hidden. Use that profile with Isaac and keep the Godot one for Godot.

## Validation

Focused tests cover actual TCP/SHM, exact topics, malformed input, stale/duplicate
poses, source identities/timestamps, cleanup, matching scenario acknowledgement,
contact geometry, retry, timeout and episode reset.

```bash
uv run --locked --all-packages --all-extras --all-groups \
  pytest -q tests/test_isaac_transport.py tests/test_isaac_scenario.py tests/test_isaac_surface.py
```

GPU probes use isolated ports and separate processes, leaving a user's run alone:

```bash
# Camera/FOV/source correspondence with both alternating camera positions.
uv run --locked --all-packages --all-extras --all-groups \
  python tools/check_isaac_renderer.py --gui --moving-camera
# Real flight with a confirmed 6 m climb, hover and LOS approach.
uv run --locked --all-packages --all-extras --all-groups \
  python tools/check_isaac_flight.py --gui --app-debug --takeoff-height 6 \
  --report-dir /tmp/bullet-isaac-custom-height
# Rivermark on its surface with the Isaac profile: acquisition and approach with measured image ages.
uv run --locked --all-packages --all-extras --all-groups \
  python tools/check_isaac_flight.py --gui --world /home/user/ov/rivermark.usd \
  --spawn 0 0 5.765 --takeoff-height 11 --target 40 0 5.852 \
  --mission-config isaac/mission_profile.json --report-dir /tmp/bullet-isaac-rivermark
# Dropped-first-report recovery, confirmed completion, late app, reset and MCAP.
uv run --locked --all-packages --all-extras --all-groups \
  python tools/check_isaac_scenario.py
# All reports rejected: timeout/Stop without false success.
uv run --locked --all-packages --all-extras --all-groups \
  python tools/check_isaac_scenario.py --reject-collisions \
  --report-dir /tmp/bullet-isaac-scenario-timeout
# Real Rivermark, unit/up-axis conversion, surface heights and unchanged source hash.
/home/user/isaacsim5.0/python.sh tools/check_isaac_world.py
```

Report directories retain logs, JSON results and screenshots. The collision probe
substitutes deterministic motion while retaining production geometry, RC/lifecycle,
source validation, app completion and recorder paths. It is not a navigation-tuning
test. The selected-height check uses real flight physics. `check_rc_deadlines.py`
replays camera latency against the real mission/RC lease; `check_rc_timing.py` is
an eight-second passive sampler of default local telemetry.
