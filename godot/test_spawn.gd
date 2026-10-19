extends SceneTree
const Presets = preload("res://spawn_presets.gd")
var scene: Node3D
var failed := false

func require(value: bool, detail: String) -> void:
	if not value:
		failed = true
		push_error(detail)
		quit(1)


func _initialize() -> void:
	root.content_scale_size = Vector2i(1280, 800)
	root.content_scale_mode = Window.CONTENT_SCALE_MODE_VIEWPORT
	_run.call_deferred()

func _run() -> void:
	var saved := Presets.new()
	var path := "user://test_spawn_%s.json" % Time.get_ticks_usec()
	require(saved.save_preset("Hill", [120.25, -90.125, 60.015], path, 7.5), "save failed")
	var restored := Presets.new()
	require(restored.load_file(path), "load failed")
	require(restored.presets.Hill.position_m == [120.25, -90.125, 60.015] and restored.presets.Hill.takeoff_height_m == 7.5, "preset round trip")
	require(not restored.save_preset("Default", [0, 0, 2], path), "Default writable")
	require(restored.save_preset("Hill", [1, 2, 3], path), "update failed")
	require(not restored.save_preset("bad", [0, 0, 501], path), "bad altitude saved")
	require(not restored.save_preset("bad", [0, 0, 1], "user://missing-directory/spawn.json"), "write error ignored")
	var file := FileAccess.open(path, FileAccess.WRITE)
	file.store_string('{"schema_version":1,"presets":{"bad":[0,0,-1]}}')
	file.close()
	require(not restored.load_file(path), "malformed file accepted")
	require(restored.presets.Hill.position_m == [1, 2, 3], "failed load replaced previous entries")
	file = FileAccess.open(path, FileAccess.WRITE)
	file.store_string('{"schema_version":1,"presets":{"Legacy":[10,20,30]}}')
	file.close()
	require(restored.load_file(path) and restored.presets.Legacy.takeoff_height_m == 20.0, "legacy preset migration failed")
	require(not restored.save_preset("bad", [0, 0, 1], path, 0.0), "invalid takeoff saved")
	DirAccess.remove_absolute(ProjectSettings.globalize_path(path))
	scene = load("res://main.tscn").instantiate()
	scene.capture_only = true
	root.add_child(scene)
	scene.set_process(false)
	await physics_frame
	await physics_frame
	var panel: PanelContainer = scene.spawn_panel
	panel.show()
	panel.physics_update()
	require(panel.valid_draft(), "default rejected")
	var actual: Vector3 = scene.drone.position
	panel.draft = [170.0, 140.0, 0.015]
	panel._draft_changed()
	panel.physics_update()
	require(not panel.valid_draft(), "below hills allowed")
	var ground: float = panel.terrain_height
	panel.draft[2] = ground + 0.015
	require(panel.valid_draft(), "exact local clearance rejected")
	panel.draft[0] = 251.0
	panel._draft_changed()
	panel.physics_update()
	require(not panel.valid_draft(), "outside scene allowed")
	# Real camera ray into rendered terrain, far outside the central flat square.
	var target := Vector3(-140, 0, -170)
	scene.spectator.position = target + Vector3(0, 200, 100)
	scene.spectator.look_at(target)
	panel.editable = true
	panel.picking = true
	var ui_click := InputEventMouseButton.new()
	ui_click.button_index = MOUSE_BUTTON_LEFT
	ui_click.pressed = true
	ui_click.position = panel.global_position + Vector2(10, 20)
	root.push_input(ui_click, true)
	require(panel.pick_screen == null and panel.picking, "UI click selected world")
	ui_click.pressed = false
	root.push_input(ui_click, true)
	var world_click := InputEventMouseButton.new()
	world_click.button_index = MOUSE_BUTTON_LEFT
	world_click.pressed = true
	world_click.position = scene.spectator.unproject_position(target)
	root.push_input(world_click, true)
	require(panel.pick_screen != null and not panel.picking, "world click not consumed by picking")
	panel.physics_update()
	require(absf(float(panel.draft[0])) > 100 and absf(float(panel.draft[1])) > 100, "selection limited to clearing")
	require(panel.terrain_valid, "terrain ray missed")
	require(panel.valid_draft() and float(panel.draft[2]) >= panel.terrain_height + 0.015 and float(panel.draft[2]) < panel.terrain_height + 0.0161, "pick did not automatically set terrain clearance")
	require(scene.drone.position == actual, "draft teleported actual drone")
	# Truck colliders and terrain picking occupy separate collision layers.
	var truck_ray: Dictionary = panel._terrain_ray(scene.truck.position + Vector3(0, 100, 0), scene.truck.position - Vector3(0, 100, 0))
	require(not truck_ray.is_empty() and truck_ray.position.y < scene.truck.position.y + 0.001, "pick ray hit truck")
	panel.presets.presets["Test"] = {"position_m": [12.0, 13.0, 80.0], "takeoff_height_m": 8.5}
	panel._refresh_presets("Test")
	panel.load_selected()
	require(panel.draft == [12.0, 13.0, 80.0] and panel.takeoff.value == 8.5 and scene.drone.position == actual, "Load must edit only draft")
	panel.pending = {"request_id": "pending", "physics_id": "old"}
	panel.update_state({"physics_id": "new", "state": "stopped", "controller_id": "controller"}, {}, "controller")
	require(panel.pending.is_empty(), "physics restart left Apply stuck")
	var fake_spawn := {"physics_id": "physics", "controller_id": "controller", "episode_id": "episode", "request_id": "other", "result": "applied", "reason": "spawn_applied", "scenario_name": "Hill", "position_m": [10.0, 20.0, 40.0]}
	var fake_control := {"physics_id": "physics", "controller_id": "controller", "episode_id": "episode", "state": "stopped"}
	panel.pending = {"physics_id": "physics", "request_id": "waiting"}
	panel.started_ms = Time.get_ticks_msec() - 2100
	panel.last_sent_ms = Time.get_ticks_msec() - 2000
	panel.update_state(fake_control, fake_spawn, "controller")
	require(not panel.pending.is_empty() and panel.apply_button.disabled, "timeout enabled commands before reconciliation")
	panel.last_sent_ms = Time.get_ticks_msec() - 2600
	panel.update_state(fake_control, fake_spawn, "controller")
	require(panel.pending.is_empty() and panel.editable, "authoritative timeout reconciliation failed")
	panel.pending = {"physics_id": "physics", "request_id": "ack"}
	fake_spawn.request_id = "ack"
	panel.update_state(fake_control, fake_spawn, "controller")
	require(panel.pending.is_empty(), "matching acknowledgement not accepted")
	fake_control.state = "running"
	panel.update_state(fake_control, fake_spawn, "controller")
	require(not panel.editable and not panel.takeoff.editable and panel.load_button.disabled and panel.apply_button.disabled, "running edits enabled")
	print("Spawn: presets, write failures, full-landscape ray picking, frame mapping, altitude, clearance, bounds, layer separation, draft isolation and restart passed")
	scene.queue_free()
	quit(1 if failed else 0)
