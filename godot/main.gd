extends Node3D

const Frames = preload("res://frames.gd")
const SpawnPanel = preload("res://spawn_panel.gd")
const WindPanel = preload("res://wind_panel.gd")
const Windsock = preload("res://windsock.gd")
const TruckCollision = preload("res://truck_collision.gd")
const COLLISION_RETRY_MS := 100
const COLLISION_ACK_TIMEOUT_MS := 1000
const WORLD_SIZE := 500.0
const GRID := 120
const STALE_MS := 1000
# Horizontal metres from the physics origin, in Godot's X/Z ground plane.
const TRUCK_POSITION_M := Vector2(0.0, -50.0)
const TRUCK_CLEARANCE_M := 6.0
var spawn_panel: PanelContainer
var wind_panel: PanelContainer
var windsock: Node3D
var wind_spawn_position: Array = []
var collision_detector: RefCounted
var pending_collision: Dictionary = {}
var collision_started_ms := 0
var collision_retry_ms := 0
var collision_episode := ""
var collision_failed := false
var adapter: ZmqCameraAdapter
var busy := false
var capture_only := false
var viewport: SubViewport
var fpv: Camera3D
var spectator: Camera3D
var drone: Node3D
var truck: Node3D
var status: Label
var preview: TextureRect
var noise := FastNoiseLite.new()
var yaw := 0.7
var pitch := 0.45
var distance := 65.0
var last_request := 0
var total_frames := 0
var fps_frames := 0
var fps_started := 0
var measured_fps := 0.0
var active_config: Dictionary = {}
var last_stats_ms := 0
var timing_samples := {"capture_ms": [], "render_wait_ms": [], "readback_ms": [], "publish_ms": [], "source_age_ms": []}
var control_buttons: Dictionary = {}
var control_label: Label
var control_display := "DISCONNECTED"
var control_notice := ""
var control_notice_until := 0

@export var transport_config_path := "res://../configs/transport.json"

func _ready() -> void:
	if not capture_only and "nvidia" not in RenderingServer.get_video_adapter_vendor().to_lower():
		push_error("NVIDIA is required. Launch with tools/run_godot.sh --path godot")
		process_mode = Node.PROCESS_MODE_DISABLED
		get_tree().quit(1)
		return
	drone = Node3D.new()
	truck = Node3D.new()
	Engine.max_fps = 90
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	noise.seed = 271828
	noise.frequency = 0.012
	_world()
	collision_detector = TruckCollision.new(get_world_3d(), truck)
	_cameras()
	_ui()
	fps_started = Time.get_ticks_msec()
	adapter = ZmqCameraAdapter.new()
	add_child(adapter)
	spawn_panel.setup(self, spectator, adapter)
	wind_panel.adapter = adapter
	if not capture_only:
		var transport_path := ProjectSettings.globalize_path(transport_config_path)
		if not adapter.start(transport_path):
			get_tree().quit(1)
			return
	print("Godot ready with native ZeroMQ camera adapter")

func height_at(x: float, z: float) -> float:
	var clearing := smoothstep(22.0, 70.0, Vector2(x, z).length())
	return (noise.get_noise_2d(x, z) + 0.65) * 24.0 * clearing

func material(color: Color) -> StandardMaterial3D:
	var result := StandardMaterial3D.new()
	result.albedo_color = color
	result.roughness = 0.9
	return result

func mesh_node(parent: Node3D, mesh: Mesh, location: Vector3, color: Color) -> MeshInstance3D:
	var node := MeshInstance3D.new()
	node.mesh = mesh
	node.position = location
	node.material_override = material(color)
	parent.add_child(node)
	return node

func box(parent: Node3D, location: Vector3, size: Vector3, color: Color) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = size
	return mesh_node(parent, mesh, location, color)

func _world() -> void:
	var environment := WorldEnvironment.new()
	var settings := Environment.new()
	settings.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var sky_material := ProceduralSkyMaterial.new()
	sky_material.sky_top_color = Color("527eaa")
	sky_material.sky_horizon_color = Color("cadbdc")
	sky_material.ground_horizon_color = Color("cadbdc")
	sky_material.ground_bottom_color = Color("566448")
	sky.sky_material = sky_material
	settings.sky = sky
	settings.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	settings.ambient_light_energy = 0.35
	settings.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	environment.environment = settings
	add_child(environment)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-35, -40, 0)
	sun.light_color = Color("fff0cf")
	sun.light_energy = 0.9
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 100
	add_child(sun)
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	for z in range(GRID + 1):
		for x in range(GRID + 1):
			var px := (float(x) / GRID - 0.5) * WORLD_SIZE
			var pz := (float(z) / GRID - 0.5) * WORLD_SIZE
			surface.add_vertex(Vector3(px, height_at(px, pz), pz))
	for z in range(GRID):
		for x in range(GRID):
			var i := z * (GRID + 1) + x
			for index in [i, i + 1, i + GRID + 1, i + 1, i + GRID + 2, i + GRID + 1]:
				surface.add_index(index)
	surface.generate_normals()
	var terrain := MeshInstance3D.new()
	terrain.mesh = surface.commit()
	var ground_material := ShaderMaterial.new()
	ground_material.shader = preload("res://ground.gdshader")
	terrain.material_override = ground_material
	add_child(terrain)
	# Picking only: isolated from the truck contact layer and all physics response.
	var terrain_body := StaticBody3D.new()
	terrain_body.collision_layer = 2
	terrain_body.collision_mask = 0
	var terrain_shape := CollisionShape3D.new()
	terrain_shape.shape = terrain.mesh.create_trimesh_shape()
	terrain_body.add_child(terrain_shape)
	terrain.add_child(terrain_body)
	_path()
	_truck()
	var rng := RandomNumberGenerator.new()
	rng.seed = 314159
	for i in range(240):
		var x := rng.randf_range(-240, 240)
		var z := rng.randf_range(-240, 240)
		if Vector2(x, z).length() < 20 or absf(x - path_x(z)) < 5 or Vector2(x, z).distance_to(TRUCK_POSITION_M) < TRUCK_CLEARANCE_M:
			continue
		var h := rng.randf_range(4, 10)
		var trunk := CylinderMesh.new()
		trunk.top_radius = 0.15
		trunk.bottom_radius = 0.28
		trunk.height = h * 0.55
		trunk.radial_segments = 6
		mesh_node(self, trunk, Vector3(x, height_at(x, z) + h * 0.275, z), Color("63513c"))
		var crown := CylinderMesh.new()
		crown.top_radius = 0
		crown.bottom_radius = h * 0.32
		crown.height = h * 0.8
		crown.radial_segments = 7
		mesh_node(self, crown, Vector3(x, height_at(x, z) + h * 0.65, z), Color(0.12 + rng.randf() * 0.08, 0.25 + rng.randf() * 0.12, 0.16))
	for i in range(60):
		var x := rng.randf_range(-210, 210)
		var z := rng.randf_range(-210, 210)
		if Vector2(x, z).length() < 15 or Vector2(x, z).distance_to(TRUCK_POSITION_M) < TRUCK_CLEARANCE_M:
			continue
		var rock := SphereMesh.new()
		rock.radial_segments = 7
		rock.rings = 3
		var node := mesh_node(self, rock, Vector3(x, height_at(x, z), z), Color("8c9183"))
		node.scale = Vector3(rng.randf_range(1, 4), rng.randf_range(1, 3), rng.randf_range(1, 4))
	box(self, Vector3(0, 0.006, 0), Vector3(4, 0.01, 4), Color("b4ac8a"))
	for x in [-1.6, 1.6]:
		box(self, Vector3(x, 0.018, 0), Vector3(0.1, 0.012, 3.3), Color("f2df9c"))

func _truck() -> void:
	truck.name = "RedTruck"
	truck.position = Vector3(TRUCK_POSITION_M.x, height_at(TRUCK_POSITION_M.x, TRUCK_POSITION_M.y), TRUCK_POSITION_M.y)
	# Keep all four wheels on the local slope; the front (+Z) faces the spawn.
	var slope_x := height_at(truck.position.x + 0.5, truck.position.z) - height_at(truck.position.x - 0.5, truck.position.z)
	var slope_z := height_at(truck.position.x, truck.position.z + 0.5) - height_at(truck.position.x, truck.position.z - 0.5)
	var up := Vector3(-slope_x, 1.0, -slope_z).normalized()
	var front := Vector3(0.0, slope_z, 1.0).normalized()
	truck.basis = Basis(up.cross(front).normalized(), up, front)
	add_child(truck)
	var red := Color("cf252a")
	var dark := Color("20272d")
	var glass := Color("25485c")
	box(truck, Vector3(0, 0.85, 0), Vector3(2.2, 0.35, 6.0), dark)
	box(truck, Vector3(0, 2.1, -0.95), Vector3(2.4, 2.2, 4.2), red).name = "CargoBox"
	box(truck, Vector3(0, 1.9, 2.1), Vector3(2.3, 2.0, 1.8), red).name = "Cab"
	box(truck, Vector3(0, 2.45, 3.01), Vector3(1.95, 0.65, 0.04), glass)
	box(truck, Vector3(0, 1.55, 3.02), Vector3(1.05, 0.45, 0.06), dark)
	box(truck, Vector3(0, 0.95, 3.08), Vector3(2.5, 0.2, 0.2), Color("adb3b5"))
	box(truck, Vector3(0, 1.05, -3.13), Vector3(2.5, 0.2, 0.18), dark)
	for side in [-1.0, 1.0]:
		box(truck, Vector3(side * 1.16, 2.45, 2.1), Vector3(0.04, 0.65, 1.15), glass)
		box(truck, Vector3(side * 1.38, 2.35, 2.75), Vector3(0.2, 0.35, 0.16), dark)
		box(truck, Vector3(side * 0.86, 1.55, 3.06), Vector3(0.35, 0.28, 0.06), Color("fff1c4"))
		for axle_z in [-2.1, 1.9]:
			var tire := CylinderMesh.new()
			tire.top_radius = 0.55
			tire.bottom_radius = 0.55
			tire.height = 0.34
			tire.radial_segments = 16
			var wheel := mesh_node(truck, tire, Vector3(side * 1.2, 0.55, axle_z), dark)
			wheel.rotation.z = PI / 2
			var hub := CylinderMesh.new()
			hub.top_radius = 0.25
			hub.bottom_radius = 0.25
			hub.height = 0.02
			hub.radial_segments = 12
			var rim := mesh_node(truck, hub, Vector3(side * 1.38, 0.55, axle_z), Color("b2b9bc"))
			rim.rotation.z = PI / 2

func path_x(z: float) -> float:
	return 25.0 + sin(z * 0.018) * 24.0

func _path() -> void:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	for i in range(160):
		var z0 := -249.0 + i * 3.1
		var z1 := z0 + 3.1
		for point in [Vector2(path_x(z0)-1.6, z0), Vector2(path_x(z0)+1.6, z0), Vector2(path_x(z1)-1.6, z1), Vector2(path_x(z0)+1.6, z0), Vector2(path_x(z1)+1.6, z1), Vector2(path_x(z1)-1.6, z1)]:
			surface.add_vertex(Vector3(point.x, height_at(point.x, point.y) + 0.12, point.y))
	surface.generate_normals()
	mesh_node(self, surface.commit(), Vector3.ZERO, Color("9b8d64"))

func _cameras() -> void:
	add_child(drone)
	drone.position.y = 2
	var body := box(drone, Vector3.ZERO, Vector3(0.24, 0.08, 0.32), Color("e9a753"))
	body.layers = 2
	for x in [-0.22, 0.22]:
		for z in [-0.22, 0.22]:
			var arm := box(drone, Vector3(x/2, 0, z/2), Vector3(0.045, 0.035, 0.35), Color("263a42"))
			arm.rotation.y = -signf(x*z) * PI/4
			arm.layers = 2
			var prop := CylinderMesh.new()
			prop.height = 0.012
			prop.top_radius = 0.1
			prop.bottom_radius = 0.1
			var motor := mesh_node(drone, prop, Vector3(x, 0.04, z), Color("e1e8db"))
			motor.layers = 2
	spectator = Camera3D.new()
	spectator.far = 1500
	add_child(spectator)
	spectator.current = true
	viewport = SubViewport.new()
	viewport.size = Vector2i(640, 480)
	viewport.world_3d = get_viewport().world_3d
	viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	add_child(viewport)
	fpv = Camera3D.new()
	fpv.keep_aspect = Camera3D.KEEP_WIDTH
	fpv.fov = 90
	fpv.near = 0.01
	fpv.far = 900
	fpv.cull_mask = 1
	viewport.add_child(fpv)
	fpv.current = true
	fpv.position = drone.position

func _ui() -> void:
	var canvas := CanvasLayer.new()
	add_child(canvas)
	var band := ColorRect.new()
	band.color = Color(0.035, 0.07, 0.08, 0.88)
	band.set_anchors_and_offsets_preset(Control.PRESET_TOP_WIDE)
	band.offset_bottom = 92
	canvas.add_child(band)
	var title := Label.new()
	title.text = "MEADOW FLIGHT  /  BULLET + GODOT"
	title.position = Vector2(28, 15)
	title.add_theme_font_size_override("font_size", 23)
	canvas.add_child(title)
	status = Label.new()
	status.position = Vector2(28, 51)
	status.modulate = Color("bfd8cc")
	canvas.add_child(status)
	var panel := PanelContainer.new()
	panel.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_RIGHT)
	panel.offset_left = -432
	panel.offset_top = -368
	panel.offset_right = -24
	panel.offset_bottom = -24
	canvas.add_child(panel)
	var column := VBoxContainer.new()
	panel.add_child(column)
	var label := Label.new()
	label.text = "  FRONT CAMERA  ·  RGB8"
	label.custom_minimum_size.y = 30
	column.add_child(label)
	preview = TextureRect.new()
	preview.custom_minimum_size = Vector2(400, 300)
	preview.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	preview.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	preview.texture = viewport.get_texture()
	column.add_child(preview)
	var hint := Label.new()
	hint.text = "Right-drag to orbit   ·   Scroll to zoom   /   Truck contact ends the scenario"
	hint.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_LEFT)
	hint.position = Vector2(28, -108)
	canvas.add_child(hint)
	var toolbar := HBoxContainer.new()
	toolbar.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_LEFT)
	toolbar.position = Vector2(24, -80)
	toolbar.add_theme_constant_override("separation", 8)
	canvas.add_child(toolbar)
	for action: String in ["start", "stop", "reset"]:
		var button := Button.new()
		button.name = action.capitalize() + "Button"
		button.icon = load("res://icons/%s.svg" % action)
		button.custom_minimum_size = Vector2(48, 48)
		button.tooltip_text = {"start": "Start a fresh episode", "stop": "Stop and keep the final pose", "reset": "Reset to the initial state and wait"}[action]
		button.accessibility_name = action.capitalize() + " simulation"
		button.accessibility_description = button.tooltip_text
		button.focus_mode = Control.FOCUS_ALL
		button.disabled = true
		for style_name: String in ["normal", "hover", "pressed", "disabled", "focus"]:
			var style := StyleBoxFlat.new()
			style.bg_color = Color("193b44") if style_name == "hover" else Color("102c35")
			style.border_color = Color("a9dfbd") if style_name == "focus" else Color("36545e")
			style.set_border_width_all(2 if style_name == "focus" else 1)
			style.set_corner_radius_all(6)
			button.add_theme_stylebox_override(style_name, style)
		button.pressed.connect(_send_control.bind(action))
		control_buttons[action] = button
		toolbar.add_child(button)
	control_label = Label.new()
	control_label.custom_minimum_size = Vector2(220, 48)
	control_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	control_label.add_theme_font_size_override("font_size", 16)
	control_label.modulate = Color("d0e0db")
	toolbar.add_child(control_label)
	spawn_panel = SpawnPanel.new()
	canvas.add_child(spawn_panel)
	spawn_panel.hide()
	var scenario_button := Button.new()
	scenario_button.text = "Scenario"
	scenario_button.custom_minimum_size = Vector2(100, 48)
	scenario_button.pressed.connect(func():
		spawn_panel.visible = not spawn_panel.visible
		wind_panel.hide())
	toolbar.add_child(scenario_button)
	wind_panel = WindPanel.new()
	canvas.add_child(wind_panel)
	wind_panel.hide()
	var wind_button := Button.new()
	wind_button.name = "WindButton"
	wind_button.text = "Wind"
	wind_button.custom_minimum_size = Vector2(100, 48)
	wind_button.accessibility_name = "Wind settings"
	wind_button.pressed.connect(func():
		wind_panel.visible = not wind_panel.visible
		spawn_panel.hide())
	toolbar.add_child(wind_button)
	windsock = Windsock.new()
	add_child(windsock)
	windsock.place_at_spawn([0., 0., 0.], height_at)

func _send_control(action: String) -> void:
	if (not spawn_panel.pending.is_empty() or not wind_panel.pending.is_empty()) and action != "stop":
		return
	if adapter == null or not adapter.publish_control(action):
		control_notice = "Could not send command"
	else:
		control_notice = action.capitalize() + " sent"
	control_notice_until = Time.get_ticks_msec() + 1500

func _update_controls() -> void:
	var state: Dictionary = {} if capture_only or adapter == null else adapter.get_control_state()
	var connected := not state.is_empty()
	var ready: bool = connected and state.controller_id == adapter.get_stats().controller_id
	var running: bool = connected and state.state == "running"
	control_buttons.start.disabled = not ready or running
	control_buttons.stop.disabled = not ready or not running
	control_buttons.reset.disabled = not ready
	var spawn: Dictionary = {} if capture_only or adapter == null else adapter.get_spawn_state()
	spawn_panel.update_state(state, spawn, "" if adapter == null else String(adapter.get_stats().controller_id))
	var wind: Dictionary = {} if capture_only or adapter == null else adapter.get_wind_state()
	wind_panel.update_state(state, wind, "" if adapter == null else String(adapter.get_stats().controller_id))
	windsock.update_wind(wind)
	if not spawn.is_empty() and spawn.position_m != wind_spawn_position:
		wind_spawn_position = spawn.position_m.duplicate()
		windsock.place_at_spawn(wind_spawn_position, height_at)
	if not spawn_panel.pending.is_empty() or not wind_panel.pending.is_empty():
		control_buttons.start.disabled = true
		control_buttons.reset.disabled = true
	control_display = String(state.state).to_upper() if connected else "DISCONNECTED"
	var detail := "Waiting for physics" if not connected else "%s · %.2f s" % [state.reason, state.simulation_time_s]
	if Time.get_ticks_msec() < control_notice_until:
		detail = control_notice
	if state.get("terminal_result") != null:
		control_display = "COMPLETED"
		detail = "Target hit · %.2f s" % state.simulation_time_s
	elif collision_failed:
		detail = "Collision acknowledgement failed"
	elif not pending_collision.is_empty():
		detail = "Contact detected · waiting for confirmation"
	control_label.text = "%s\n%s" % [control_display.capitalize(), detail]

func _physics_process(_delta: float) -> void:
	spawn_panel.physics_update()
	if capture_only or adapter == null:
		return
	var state: Dictionary = adapter.get_control_state()
	if state.is_empty():
		return
	var episode_key := "%s/%s" % [state.physics_id, state.episode_id]
	if episode_key != collision_episode:
		collision_episode = episode_key
		pending_collision = {}
		collision_failed = false
		collision_detector.reset()
	if state.state != "running":
		pending_collision = {}
		return
	var now := Time.get_ticks_msec()
	if pending_collision.is_empty():
		var pose: Dictionary = adapter.poll_collision_pose()
		if not pose.is_empty():
			pending_collision = collision_detector.check(pose)
			if not pending_collision.is_empty():
				pending_collision.event_id = "%s-%s-hit-%s" % [adapter.get_stats().controller_id, pose.source.episode_id, int(pose.source.step_index)]
				collision_started_ms = now
				collision_retry_ms = 0
	if not pending_collision.is_empty() and now >= collision_retry_ms:
		if now - collision_started_ms >= COLLISION_ACK_TIMEOUT_MS:
			collision_failed = true
			adapter.publish_control("stop")
		else:
			adapter.publish_collision(pending_collision)
		collision_retry_ms = now + COLLISION_RETRY_MS

func _process(_delta: float) -> void:
	_update_controls()
	if not capture_only and not busy:
		var request := adapter.poll_request()
		if not adapter.get_stats().error.is_empty():
			get_tree().quit(1)
			return
		if not request.is_empty():
			_capture(request)
	var target := drone.global_position
	spectator.position = target + Vector3(sin(yaw)*cos(pitch), sin(pitch), cos(yaw)*cos(pitch)) * distance
	spectator.look_at(target)
	var now := Time.get_ticks_msec()
	if now - fps_started >= 1000:
		measured_fps = fps_frames * 1000.0 / (now - fps_started)
		fps_frames = 0
		fps_started = now
	var state := control_display
	var drops := 0
	if adapter != null:
		adapter.publish_health()
		var stats := adapter.get_stats()
		drops = stats.sequence_gaps + stats.stale
	if not capture_only and now - last_stats_ms >= 5000:
		var sample := adapter.get_stats()
		sample["timing_frames"] = timing_samples.capture_ms.size()
		for metric: String in timing_samples:
			var summary := timing_summary(timing_samples[metric])
			for statistic: String in summary:
				sample[metric + "_" + statistic] = summary[statistic]
			timing_samples[metric].clear()
		print("Native camera stats: ", JSON.stringify(sample))
		last_stats_ms = now
	status.text = "%s   ·   %.1f FPS   ·   %d captures   ·   %d gaps/stale" % [state, measured_fps, total_frames, drops]

static func timing_summary(samples: Array) -> Dictionary:
	if samples.is_empty():
		return {}
	var sorted := samples.duplicate()
	sorted.sort()
	var total := 0.0
	for value: float in sorted:
		total += value
	return {"mean": total / sorted.size(), "p95": sorted[ceili(sorted.size() * 0.95) - 1], "max": sorted.back()}

func _capture(request: Dictionary) -> void:
	var capture_started := Time.get_ticks_usec()
	busy = true
	last_request = Time.get_ticks_msec()
	var camera: Dictionary = request.config.camera
	if active_config != camera:
		viewport.size = Vector2i(camera.width_px, camera.height_px)
		fpv.fov = camera.fov_x_deg
		active_config = camera.duplicate()
	drone.transform = Transform3D(Frames.orientation(request.pose.orientation_xyzw), Frames.position(request.pose.position_m))
	fpv.global_transform = drone.global_transform
	viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	drone.force_update_transform()
	fpv.force_update_transform()
	# Publish camera pixels before window presentation can stall the frame.
	RenderingServer.force_draw(false)
	var readback_started := Time.get_ticks_usec()
	var image := viewport.get_texture().get_image()
	if image == null or image.is_empty():
		push_error("Camera readback returned an empty image")
		get_tree().quit(1)
		return
	image.convert(Image.FORMAT_RGB8)
	var pixels := image.get_data()
	var publish_started := Time.get_ticks_usec()
	if not capture_only and not adapter.publish_frame(pixels, request):
		get_tree().quit(1)
		return
	timing_samples.capture_ms.append((publish_started - capture_started) / 1000.0)
	timing_samples.render_wait_ms.append((readback_started - capture_started) / 1000.0)
	timing_samples.readback_ms.append((publish_started - readback_started) / 1000.0)
	timing_samples.publish_ms.append((Time.get_ticks_usec() - publish_started) / 1000.0)
	if not capture_only:
		timing_samples.source_age_ms.append(adapter.get_stats().source_age_ms)
	total_frames += 1
	fps_frames += 1
	busy = false

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if event.pressed else Input.MOUSE_MODE_VISIBLE
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_UP:
			distance = maxf(2, distance * 0.85)
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			distance = minf(650, distance * 1.15)
	elif event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		yaw -= event.relative.x * 0.005
		pitch = clampf(pitch + event.relative.y * 0.005, 0.05, 1.45)

func _exit_tree() -> void:
	if adapter != null:
		print("Native camera final: ", JSON.stringify(adapter.get_stats()))
		adapter.close()
