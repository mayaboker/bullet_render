"""Isaac Sim 6.2 render-only backend. Launch with Isaac's python.sh."""
import argparse
import os
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
IDLE_UI_PERIOD_S = 0.1
POSE_POLL_PERIOD_S = 0.002
sys.path.insert(0, str(ROOT))
from isaac.runtime import DEPENDENCIES
if DEPENDENCIES.is_dir():
    sys.path[:0] = [str(DEPENDENCIES)]
_LIBS = ROOT.parent / 'bullet_drone' / 'libs'
for _pkg in ('bullet_simulation_protocol', 'bullet_image_transport'):
    _src = _LIBS / _pkg / 'src'
    if _src.is_dir() and str(_src) not in sys.path:
        sys.path.insert(0, str(_src))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transport-config', type=Path, required=True,
                        help='Transport endpoint JSON (telemetry + image endpoints)')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--world', type=Path,
                        help='USD world file; source is referenced without saving changes')
    parser.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='Red truck origin in Bullet metres; default is 40 0 0')
    parser.add_argument('--seconds', type=float,
                        help='Exit after this many seconds after initialization')
    args = parser.parse_args()

    from bullet_simulation_protocol import load_transport
    from isaac.scenario import DEFAULT_TARGET_M, vector
    from isaac.presets import world_preset
    try:
        preset = world_preset(args.world)
        if args.target is None:
            args.target = preset.get('target_m', DEFAULT_TARGET_M)
        args.target = vector(args.target)
        spawn_offset = preset.get('spawn_m', [0, 0, 0])
    except ValueError as error:
        parser.error(str(error))
    profile = load_transport(args.transport_config)

    from isaacsim import SimulationApp
    experience = Path(os.environ['EXP_PATH']) / 'isaacsim.exp.base.zero_delay.kit'
    application = SimulationApp({'headless': args.headless, 'renderer': 'RayTracedLighting',
                                 'extra_args': ['--/rtx/hydra/supportMultiTickRate=false',
                                                '--/rtx/rendering/perSensorTickTlas=false']},
                                experience=str(experience))
    import carb.settings
    _settings = carb.settings.get_settings()
    _settings.set('/rtx/post/tonemap/op', 0)
    _settings.set('/rtx/post/lensFlare/enabled', False)
    _settings.set('/rtx/post/bloom/enabled', False)
    _settings.set('/rtx/post/chromaticAberration/enabled', False)
    _settings.set('/rtx/post/fxaa/enabled', False)
    receiver = publisher = camera = marker = None
    try:
        import numpy as np
        import omni.timeline
        from isaac.camera import Camera
        from isaac.scene import Scene, DroneMarker, FRONT_CAMERA
        from isaac.transport import Receiver, Publisher, camera_pose
        scene = Scene(args.world, args.target)
        if not args.headless:
            try:
                marker = DroneMarker()
                scene.focus_drone()
            except Exception:
                marker = None
        else:
            marker = None
        receiver = Receiver(profile)

        last_stats = 0.0
        captures = published = 0
        last_ui_update = 0.0
        deadline = time.monotonic() + args.seconds if args.seconds is not None else float('inf')
        print('Isaac renderer: waiting for poses from launcher', flush=True)
        while application.is_running() and time.monotonic() < deadline:
            sample = receiver.poll()
            if sample is None:
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
                import math
                config = receiver.config
                ci = config['camera_info']
                width, height = ci['width'], ci['height']
                fx = ci['k'][0]
                fov_x_deg = 2 * math.degrees(math.atan(width / (2 * fx)))
                camera = Camera(FRONT_CAMERA, width=width, height=height, fov_x_deg=fov_x_deg)
                position, orientation = camera_pose(pose)
                position = [position[i] + spawn_offset[i] for i in range(3)]
                camera.set_world_pose(position, orientation)
                omni.timeline.get_timeline_interface().play()
                for warmup in range(64):
                    application.update()
                    test = camera.get_rgba()
                    if test is not None and test[:, :, :3].max() > 0:
                        break
                publisher = Publisher(profile['image_endpoint'],
                                     dict(width_px=width, height_px=height))
                receiver.flush()
                print('Isaac renderer: camera ready', flush=True)
                continue
            position, orientation = camera_pose(pose)
            position = [position[i] + spawn_offset[i] for i in range(3)]
            offset_pose = dict(pose, position_m=position)
            scene.update_pose(offset_pose)
            if marker is not None:
                marker.update(offset_pose)
            camera.set_world_pose(position, orientation)
            application.update()
            application.update()
            last_ui_update = time.monotonic()
            rgba = camera.get_rgba()
            if rgba is None or rgba.shape != (height, width, 4):
                continue
            if rgba.dtype != np.uint8:
                raise RuntimeError('Isaac camera returned unexpected pixel type')
            accepted = publisher.publish(np.ascontiguousarray(rgba[:, :, :3]).tobytes(), pose, received)
            captures += 1
            published += int(accepted)
            if published == 1:
                print(f'Isaac renderer: streaming on {profile["image_endpoint"]}', flush=True)
            now = time.monotonic()
            if now - last_stats >= 5:
                print('Isaac camera stats:', dict(captures=captures, published=published,
                      render_ms=round((now - capture_started) * 1000, 1),
                      receipt_age_ms=round((now - received) * 1000, 1),
                      fid=pose.get('header', {}).get('fid')), flush=True)
                last_stats = now
    except Exception as error:
        print(f'Isaac renderer failed: {error}', file=sys.stderr, flush=True)
        raise
    finally:
        if marker is not None:
            marker.close()
        if receiver is not None:
            receiver.close()
        if publisher is not None:
            publisher.close()
        camera = None
        application.close()


if __name__ == '__main__':
    main()
