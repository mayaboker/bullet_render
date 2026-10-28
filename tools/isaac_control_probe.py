"""Test-only file-driven lifecycle controls around the production Isaac renderer."""
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'isaac'), str(ROOT / 'build/isaac-deps'), str(ROOT.parent / 'bullet_msgs/src')]
command_path = Path(sys.argv.pop(1))

import transport
import isaac.scene as scene_module
import isaac.controls as controls_module

scene = controls = None


class ProbeScene(scene_module.Scene):
    def __init__(self, *args, **kwargs):
        global scene
        super().__init__(*args, **kwargs)
        scene = self


class ProbeControls(controls_module.Controls):
    def __init__(self, *args, **kwargs):
        global controls
        super().__init__(*args, **kwargs)
        controls = self


class ProbeReceiver(transport.Receiver):
    command_sequence = 0
    snapshot_at = 0

    def poll(self):
        result = super().poll()
        if command_path.exists():
            command = json.loads(command_path.read_text())
            if command['sequence'] > self.command_sequence:
                action = command['action']
                if action in ('start', 'stop', 'reset'):
                    self.command(action)
                elif action == 'apply_spawn':
                    if controls:
                        for field, value in zip(controls.spawn_fields, command['position_m']):
                            field.model.set_value(value)
                        controls.height_field.model.set_value(command['takeoff_height_m'])
                        controls.apply_scenario()
                    else:
                        self.apply_spawn(command['position_m'], command['takeoff_height_m'])
                elif action == 'set_target':
                    if controls:
                        for field, value in zip(controls.target_fields, command['position_m']):
                            field.model.set_value(value)
                    self.set_target(command['position_m'])
                    scene.set_target(command['position_m'])
                elif action == 'load_world':
                    scene.load_world(command['path'])
                elif action == 'screenshot':
                    import omni.renderer_capture
                    omni.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(command['path'])
                elif action == 'focus':
                    scene.focus_drone()
                elif action == 'snap_spawn':
                    controls.snap_spawn_height()
                elif action == 'second_viewport':
                    # Reproduce a user layout with a second full-size viewport on the front camera.
                    from omni.kit.viewport.utility import create_viewport_window
                    window = create_viewport_window('Viewport 2', width=1280, height=720)
                    window.viewport_api.set_active_camera(scene_module.FRONT_CAMERA)
                elif action == 'perspective':
                    controls.show_perspective()
                elif action == 'front_camera':
                    controls.show_front_camera()
                elif action == 'snap_target':
                    controls.snap_target_height()
                else:
                    raise ValueError('unsupported probe command')
                self.command_sequence = command['sequence']
        if time.monotonic() >= self.snapshot_at:
            state_path = command_path.with_name('isaac_state.json')
            viewports = {}
            if controls:
                from omni.kit.viewport.window import get_viewport_window_instances
                viewports = {window.title: str(window.viewport_api.camera_path) for window in get_viewport_window_instances()}
            value = dict(command_sequence=self.command_sequence, state=self.state, spawn=self.spawn_state, pending=bool(self.pending_spawn),
                         spawn_notice=self.spawn_notice, contact=self.contact_status,
                         world=scene.world_path if scene else '', position=scene.last_position if scene else None,
                         marker=bool(controls and controls.marker), editable=bool(controls and controls.editable()),
                         notice=controls.notice.text if controls else '',
                         spawn_draft=controls.values(controls.spawn_fields) if controls else None, viewports=viewports,
                         target=list(scene.target_position) if scene else None,
                         target_draft=controls.values(controls.target_fields) if controls else None)
            pending = state_path.with_suffix('.tmp')
            pending.write_text(json.dumps(value))
            pending.replace(state_path)
            self.snapshot_at = time.monotonic() + .1
        return result


transport.Receiver = ProbeReceiver
scene_module.Scene = ProbeScene
controls_module.Controls = ProbeControls
from main import main
main()
