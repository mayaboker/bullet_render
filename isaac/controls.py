"""Scenario inputs and spectator controls; all USD/UI access stays on Kit thread."""
import math
import time

from bullet_msgs.spawn import MIN_ALTITUDE_M
from isaac.scene import FRONT_CAMERA, PERSPECTIVE_CAMERA, DroneMarker

# Starting Z sits this far above the visual surface, matching the Godot picker and the spawn contract floor.
SPAWN_CLEARANCE_M = MIN_ALTITUDE_M
# Snapped heights are rounded to the millimetre input step; the spawn rounds upward to keep its clearance.
SNAP_STEP_M = 0.001


class Controls:
    def __init__(self, receiver, scene, spawn=None, takeoff_height=None):
        import omni.ui as ui
        self.receiver, self.scene = receiver, scene
        self.marker = DroneMarker()
        self.synced = False
        self.initial_spawn, self.initial_height = spawn, takeoff_height
        self.window = ui.Window('Bullet simulation controls', width=450, height=900, position_x=20, position_y=65)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=7, height=0):
                    self.status = ui.Label('Waiting for physics', height=45, word_wrap=True)
                    with ui.HStack(height=28):
                        self.start = ui.Button('Start', clicked_fn=lambda: self.run(lambda: receiver.command('start')))
                        self.stop = ui.Button('Stop', clicked_fn=lambda: self.run(lambda: receiver.command('stop')))
                        self.reset = ui.Button('Reset', clicked_fn=lambda: self.run(lambda: receiver.command('reset')))
                    with ui.HStack(height=28):
                        ui.Button('Perspective', clicked_fn=self.show_perspective)
                        self.front = ui.Button('Front camera', clicked_fn=self.show_front_camera)
                        ui.Button('Focus drone', clicked_fn=lambda: self.run(scene.focus_drone))
                    ui.Label('Cyan marker: drone. Yellow arrow: camera forward.', height=22)
                    self.position = ui.Label('Drone XYZ: waiting for pose', height=22)
                    ui.Separator(height=5)
                    ui.Label('WORLD - USD file', height=22)
                    self.world = ui.StringField(height=26).model
                    self.world.set_value(scene.world_path)
                    self.load_world = ui.Button('Load USD world (while stopped)', height=28, clicked_fn=self.load)
                    self.world_status = ui.Label(self.world_description(), height=45, word_wrap=True)
                    ui.Label('World geometry is visual; only red-truck contact ends the scenario.', height=32, word_wrap=True)
                    ui.Separator(height=5)
                    ui.Label('SCENARIO - Bullet metres, Z up', height=22)
                    self.spawn_fields = self.xyz(spawn or (0, 0, .015))
                    self.snap_spawn = ui.Button('Snap starting Z to world surface', height=26, clicked_fn=self.snap_spawn_height)
                    with ui.HStack(height=26):
                        ui.Label('Takeoff height (m)', width=155)
                        self.height_field = ui.FloatField()
                        self.height_field.model.set_value(takeoff_height if takeoff_height is not None else 20.)
                    ui.Label('Climb above starting barometer altitude (0.1-100 m).', height=22)
                    self.apply = ui.Button('Apply starting position and takeoff height', height=30, clicked_fn=self.apply_scenario)
                    self.applied = ui.Label('Waiting for physics scenario settings', height=52, word_wrap=True)
                    self.spawn_status = ui.Label('', height=42, word_wrap=True)
                    ui.Separator(height=5)
                    ui.Label('RED TRUCK - target XYZ (metres)', height=22)
                    self.target_fields = self.xyz(scene.target_position)
                    self.snap_target = ui.Button('Snap target Z to world surface', height=26, clicked_fn=self.snap_target_height)
                    self.move_target = ui.Button('Move target (while stopped)', height=28, clicked_fn=self.apply_target)
                    self.notice = ui.Label('', height=45, word_wrap=True)
        scene.focus_drone()

    @staticmethod
    def xyz(values):
        import omni.ui as ui
        fields = []
        with ui.HStack(height=26):
            for axis, value in zip('XYZ', values):
                ui.Label(axis, width=18)
                field = ui.FloatField()
                field.model.set_value(float(value))
                fields.append(field)
        return fields

    @staticmethod
    def values(fields):
        return [field.model.get_value_as_float() for field in fields]

    def run(self, action):
        try:
            self.notice.text = action() or ''
        except Exception as error:
            self.notice.text = str(error)
            print(f'Scenario input: {error}', flush=True)

    def show_perspective(self):
        self.run(lambda: self.scene.set_view_camera(PERSPECTIVE_CAMERA))

    def show_front_camera(self):
        self.run(lambda: self.scene.set_view_camera(FRONT_CAMERA))

    def load(self):
        def action():
            if self.receiver.state and self.receiver.state['state'] == 'running':
                raise ValueError('Stop physics before loading a world')
            self.world_status.text = 'Loading world and referenced assets…'
            self.scene.load_world(self.world.get_value_as_string())
            self.world_status.text = self.world_description()
            self.scene.focus_drone()
        self.run(action)
        if self.notice.text:
            self.world_status.text = 'World load failed; previous world retained'

    def world_description(self):
        text = self.scene.world_path or 'Starter world'
        if self.scene.world_warnings:
            text += f'\nLoaded with {len(self.scene.world_warnings)} missing external assets (see log)'
        return text

    def apply_scenario(self):
        self.run(lambda: self.receiver.apply_spawn(self.values(self.spawn_fields), self.height_field.model.get_value_as_float()))

    def snap_height(self, fields, clearance, label):
        """Set a draft Z field from the highest visual surface under its X/Y; never touches physics."""
        x, y, _ = self.values(fields)
        hit = self.scene.surface_height(x, y)
        if hit is None:
            raise ValueError(f'No world surface under X {x:g}, Y {y:g}; enter Z manually')
        height, path = hit
        value = math.ceil((height + clearance) / SNAP_STEP_M) * SNAP_STEP_M if clearance else round(height, 3)
        fields[2].model.set_value(value)
        return f'{label} Z {value:.3f} m from surface {height:.3f} m ({path.rsplit("/", 1)[-1]}); check it is ground, not a roof'

    def snap_spawn_height(self):
        self.run(lambda: self.snap_height(self.spawn_fields, SPAWN_CLEARANCE_M, 'Starting'))

    def snap_target_height(self):
        self.run(lambda: self.snap_height(self.target_fields, 0.0, 'Target'))

    def apply_target(self):
        def action():
            if not self.editable():
                raise ValueError('Stop physics and wait for any pending Apply before moving the target')
            position = self.values(self.target_fields)
            self.receiver.set_target(position)
            self.scene.set_target(position)
        self.run(action)

    def editable(self):
        r = self.receiver
        return bool(r.state and time.monotonic() - r.state_received < 3
                    and r.state['controller_id'] == r.controller_id and r.state['state'] == 'stopped'
                    and not r.pending_spawn)

    def update(self):
        r, now = self.receiver, time.monotonic()
        state = r.state
        connected = bool(state and now - r.state_received < 3 and state['controller_id'] == r.controller_id)
        running = connected and state['state'] == 'running'
        self.status.text = (f"{state['state'].upper()}: {state['reason']} · {state['simulation_time_s']:.2f} s"
                            if connected else 'Waiting for connected physics')
        if r.contact_status:
            self.status.text += '\n' + r.contact_status
        self.start.enabled = connected and not running and not r.pending_spawn
        self.stop.enabled = bool(running)
        self.reset.enabled = connected and not r.pending_spawn
        editable = self.editable()
        for widget in (*self.spawn_fields, self.height_field, self.apply, self.snap_spawn,
                       *self.target_fields, self.move_target, self.snap_target):
            widget.enabled = editable
        self.load_world.enabled = not running and not r.pending_spawn
        self.front.enabled = bool(self.scene.stage.GetPrimAtPath(FRONT_CAMERA))
        spawn = r.spawn_state
        if spawn and state and spawn['physics_id'] == state['physics_id'] and now - r.spawn_received < 3:
            height = spawn.get('takeoff_height_m', 20.)
            self.applied.text = f"Applied XYZ: {spawn['position_m']}\nApplied takeoff height: {height:g} m"
            if not self.synced:
                if self.initial_spawn is None:
                    for field, value in zip(self.spawn_fields, spawn['position_m']):
                        field.model.set_value(value)
                if self.initial_height is None:
                    self.height_field.model.set_value(height)
                self.synced = True
        self.spawn_status.text = r.spawn_notice

    def pose(self, pose):
        self.marker.update(pose)
        self.position.text = 'Drone XYZ: ' + ', '.join(f'{x:.2f}' for x in pose['position_m'])

    def close(self):
        self.marker.close()
        self.window.destroy()
