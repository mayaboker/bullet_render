"""Load a real USD environment and validate unit conversion/source preservation.

Run with Isaac's python.sh. No physics/control ports are opened.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'build/isaac-deps'), str(ROOT.parent / 'bullet_msgs/src')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=Path('/home/user/ov/rivermark.usd'))
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-isaac-world'))
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    before = hashlib.sha256(args.world.read_bytes()).hexdigest()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': False, 'renderer': 'RayTracedLighting'},
                        experience=os.environ['EXP_PATH'] + '/isaacsim.exp.base.zero_delay.kit')
    marker = None
    report = dict(passed=False)
    try:
        import omni.renderer_capture
        from pxr import Usd, UsdGeom, Gf
        from isaac.scene import Scene, DroneMarker
        scene = Scene()
        fixture = args.report_dir / 'centimetres-y-up.usda'
        fixture.unlink(missing_ok=True)
        test_stage = Usd.Stage.CreateNew(str(fixture))
        UsdGeom.SetStageUpAxis(test_stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(test_stage, .01)
        test_stage.SetDefaultPrim(UsdGeom.Xform.Define(test_stage, '/World').GetPrim())
        cube = UsdGeom.Cube.Define(test_stage, '/World/Test')
        cube.CreateSizeAttr(100.)
        cube.AddTranslateOp().Set(Gf.Vec3d(0, 200, 0))
        test_stage.GetRootLayer().Save()
        scene.load_world(fixture)
        bound = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']).ComputeWorldBound(
            scene.stage.GetPrimAtPath('/World/Environment/Content/Test')).ComputeAlignedRange()
        assert abs(bound.GetMidpoint()[2] - 2) < 1e-6
        cube_top = scene.surface_height(0., 0.)
        assert cube_top is not None and abs(cube_top[0] - 2.5) < 1e-6, cube_top
        assert scene.surface_height(5., 0.) is None
        assert scene.world_path == str(fixture)
        previous = scene.world_path
        try:
            scene.load_world(args.report_dir / 'does-not-exist.usd')
        except ValueError:
            pass
        else:
            raise AssertionError('invalid world silently accepted')
        assert scene.world_path == previous
        print('Loading actual world:', args.world, flush=True)
        scene.load_world(args.world)
        environment = scene.stage.GetPrimAtPath('/World/Environment')
        prim_count = sum(1 for _ in Usd.PrimRange(environment))
        assert prim_count > 10, f'world payload appears empty ({prim_count} prims)'
        assert scene.world_path == str(args.world.expanduser().resolve())
        # Rivermark's lot sits well above Bullet Z=0; record what the Snap buttons would use.
        surface_heights = {f'{x},{y}': scene.surface_height(float(x), float(y)) for x, y in ((0, 0), (40, 0), (80, 0))}
        pose = dict(position_m=[0., 0., 3.], orientation_xyzw=[0., 0., 0., 1.])
        scene.update_pose(pose)
        marker = DroneMarker()
        marker.update(pose)
        scene.focus_drone()
        import numpy as np
        import omni.timeline
        from PIL import Image
        from isaacsim.sensors.camera import Camera
        camera = Camera('/World/FrontCamera', resolution=(640, 480))
        camera.initialize()
        camera.set_focal_length(1.)
        camera.set_horizontal_aperture(2.)
        camera.set_clipping_range(.01, 900.)
        camera.set_world_pose(np.array([0., 0., 3.]), np.array([1., 0., 0., 0.]), camera_axes='world')
        omni.timeline.get_timeline_interface().play()
        deadline = time.monotonic() + 90
        for _ in range(20):
            app.update()
        while scene.context.get_stage_loading_status()[2] and time.monotonic() < deadline:
            app.update()
        assert not scene.context.get_stage_loading_status()[2], 'world assets still loading after 90 seconds'
        rgba = camera.get_rgba()
        assert rgba is not None and rgba.shape == (480, 640, 4) and rgba.dtype == np.uint8
        assert np.std(rgba[:, :, :3]) > 2, 'world camera returned an empty/flat image'
        Image.fromarray(rgba[:, :, :3]).save(args.report_dir / 'front-camera.png')
        started = time.monotonic()
        for _ in range(20):
            app.update()
        render_ms = (time.monotonic() - started) * 1000 / 20
        screenshot = args.report_dir / 'rivermark.png'
        screenshot.unlink(missing_ok=True)
        omni.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(screenshot))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            app.update()
            if screenshot.exists() and screenshot.read_bytes().endswith(bytes.fromhex('49454e44ae426082')):
                break
            time.sleep(.02)
        assert screenshot.exists(), 'perspective screenshot did not complete'
        after = hashlib.sha256(args.world.read_bytes()).hexdigest()
        assert before == after, 'source world changed on disk'
        report = dict(passed=True, world=str(args.world), environment_prim_count=prim_count,
                      source_unchanged=True, centimetre_y_up_conversion=True, invalid_load_retains_world=True,
                      missing_external_assets=scene.world_warnings, disabled_empty_instancers=scene.disabled_instancers,
                      front_camera_rgb=True, mean_render_update_ms=render_ms, screenshot=str(screenshot),
                      surface_heights=surface_heights)
    except Exception as error:
        report['error'] = str(error)
        traceback.print_exc()
        raise
    finally:
        if marker:
            marker.close()
        (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        app.close()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
