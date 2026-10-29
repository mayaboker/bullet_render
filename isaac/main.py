"""Isaac Sim 5.0 render-only backend. Launch with Isaac's python.sh."""
import argparse
import os
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
IDLE_UI_PERIOD_S = 0.1
POSE_POLL_PERIOD_S = 0.002
# Isaac has its own Python 3.11; never import the workspace's 3.12 site-packages.
sys.path.insert(0, str(ROOT / 'build' / 'isaac-deps'))
sys.path.insert(0, str(ROOT.parent / 'bullet_msgs' / 'src'))
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transport-config', type=Path, default=ROOT / 'configs/transport.json')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seconds', type=float, help='Exit after this many seconds after initialization')
    parser.add_argument('--world', type=Path, help='USD world file; source is referenced without saving changes')
    parser.add_argument('--spawn', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='Initial Bullet world position in metres; requires stopped physics. Default is the world preset, if any')
    parser.add_argument('--takeoff-height', type=float, help='Relative climb in metres (0.1–100), confirmed by physics before Start')
    parser.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='Red truck origin in Bullet metres; default is the world preset or 40 0 0')
    args = parser.parse_args()

    from bullet_msgs import load_transport
    from transport import Receiver, Publisher, camera_pose
    from bullet_msgs.spawn import validate_position, validate_takeoff_height
    from isaac.scenario import DEFAULT_TARGET_M, vector
    from isaac.presets import world_preset
    try:
        if args.spawn is not None:
            validate_position(args.spawn)
        if args.takeoff_height is not None:
            validate_takeoff_height(args.takeoff_height)
        preset = world_preset(args.world)
        if args.spawn is None and 'spawn_m' in preset:
            args.spawn = list(vector(preset['spawn_m']))
            validate_position(args.spawn)
        if args.target is None:
            args.target = preset.get('target_m', DEFAULT_TARGET_M)
        args.target = vector(args.target)
    except ValueError as error:
        parser.error(str(error))
    profile = load_transport(args.transport_config)

    from isaacsim import SimulationApp
    # Isaac's zero-delay experience waits for render/readback completion in the
    # current Kit update. The default experience pipelines older camera frames.
    experience = Path(os.environ['EXP_PATH']) / 'isaacsim.exp.base.zero_delay.kit'
    application = SimulationApp({'headless': args.headless, 'renderer': 'RayTracedLighting'},
                                experience=str(experience))
    receiver = publisher = controls = None
    try:
        import numpy as np
        import omni.timeline
        from isaacsim.sensors.camera import Camera
        from isaac.scene import Scene, FRONT_CAMERA
        scene = Scene(args.world, args.target)
        receiver = Receiver(profile, target_position=args.target)
        if not args.headless:
            from isaac.controls import Controls
            controls = Controls(receiver, scene, args.spawn, args.takeoff_height)

        camera = None
        last_stats = 0.0
        captures = published = 0
        last_ui_update = 0.0
        startup_applied = False
        deadline = time.monotonic() + args.seconds if args.seconds is not None else float('inf')
        print('Isaac renderer: waiting for physics; use Bullet controls after Mission READY', flush=True)
        while application.is_running() and time.monotonic() < deadline:
            sample = receiver.poll()
            if controls is not None:
                controls.update()
            if not startup_applied and (args.spawn is not None or args.takeoff_height is not None):
                state, spawn = receiver.state, receiver.spawn_state
                if (state and spawn and state['state'] == 'stopped' and state['controller_id'] == receiver.controller_id
                        and state['physics_id'] == spawn['physics_id'] and state['episode_id'] == spawn['episode_id']
                        and time.monotonic() - receiver.state_received < 3 and not receiver.pending_spawn):
                    position = args.spawn if args.spawn is not None else spawn['position_m']
                    height = args.takeoff_height if args.takeoff_height is not None else spawn.get('takeoff_height_m', 20.)
                    receiver.apply_spawn(position, height, 'CLI scenario')
                    startup_applied = True
            if sample is None:
                # An idle render can consume the entire arrival-to-capture
                # budget while a new pose waits in the latest slot. Pump UI at
                # 10 Hz while stopped; active captures already pump it at 30 Hz.
                now = time.monotonic()
                if now - last_ui_update >= IDLE_UI_PERIOD_S:
                    application.update()
                    last_ui_update = time.monotonic()
                else:
                    time.sleep(POSE_POLL_PERIOD_S)
                continue
            pose, received = sample
            capture_started = time.monotonic()
            if camera is None:
                settings = receiver.config['camera']
                camera = Camera(FRONT_CAMERA, resolution=(settings['width_px'], settings['height_px']))
                camera.initialize()
                # Physical aperture and focal length use the same units; ratio defines horizontal FOV.
                # Keep USD focalLength away from the renderer's small-value limits.
                focal = 1.0
                camera.set_focal_length(focal)
                camera.set_horizontal_aperture(2 * focal * np.tan(np.deg2rad(settings['fov_x_deg']) / 2))
                camera.set_clipping_range(0.01, 900.0)
                # Sensor annotators need a playing render timeline. This stage has
                # no PhysicsScene or rigid bodies; PyBullet still owns all dynamics.
                omni.timeline.get_timeline_interface().play()
                publisher = Publisher(profile['image_endpoint'], settings)
            position, orientation = camera_pose(pose)
            scene.update_pose(pose)
            if controls is not None:
                controls.pose(pose)
            camera.set_world_pose(np.asarray(position), np.asarray(orientation), camera_axes='world')
            # SimulationApp gathers render results synchronously (asyncRendering=false).
            # Zero-delay rendering completes the held pose and RGB readback in
            # this update. This stage has no PhysX scene or World.step.
            application.update()
            last_ui_update = time.monotonic()
            rgba = camera.get_rgba()
            settings = receiver.config['camera']
            if rgba is not None and rgba.shape == (settings['height_px'], settings['width_px'], 4):
                if rgba.dtype != np.uint8:
                    raise RuntimeError('Isaac camera returned unexpected pixel type')
                accepted = publisher.publish(np.ascontiguousarray(rgba[:, :, :3]).tobytes(), pose, received)
                captures += 1
                published += int(accepted)
                now = time.monotonic()
                if now - last_stats >= 5:
                    source = pose.get('source')
                    print('Isaac camera stats:', dict(captures=captures, published=published,
                          render_ms=round((now - capture_started) * 1000, 1),
                          receipt_age_ms=round((now - received) * 1000, 1),
                          source_age_ms=round((now - source['measurement_us'] / 1e6) * 1000, 1) if source else None), flush=True)
                    last_stats = now
    except Exception as error:
        print(f'Isaac renderer failed: {error}', file=sys.stderr, flush=True)
        raise
    finally:
        if controls is not None:
            controls.close()
        if receiver is not None:
            receiver.close()
        if publisher is not None:
            publisher.close()
        application.close()


if __name__ == '__main__':
    main()
