extends SceneTree
# Test-only driver: exercise the actual toolbar handler, never a production bridge.
var scene: Node3D
var command_path := ""
var report_path := ""
var last_command := -1
var last_report := 0

func _initialize() -> void:
	_start.call_deferred()

func _start() -> void:
	var args := OS.get_cmdline_user_args()
	command_path = args[0]
	report_path = args[1]
	scene = load("res://main.tscn").instantiate()
	if args.size() > 2:
		scene.transport_config_path = args[2]
	root.add_child(scene)

func _process(_delta: float) -> bool:
	if scene == null or scene.adapter == null:
		return false
	if FileAccess.file_exists(command_path):
		var parser := JSON.new()
		var command = parser.data if parser.parse(FileAccess.get_file_as_string(command_path)) == OK else null
		if command is Dictionary and command.get("sequence", -1) != last_command:
			last_command = command.get("sequence", -1)
			var action: String = command.get("action", "")
			var panel = scene.spawn_panel
			if action == "search_setup":
				scene.truck.visible = command.get("target_visible", true)
				scene.distance = 180.0
				panel.show()
				panel.draft = [100.0, 0.0, 0.015]
				panel.preset_name.text = "Yaw search test"
				panel.takeoff.value = 6.0
				panel.auto_z_pending = true
				panel._draft_changed()
			elif action == "spawn_pick":
				panel.show()
				panel.takeoff.value = command.get("takeoff_height_m", 20.0)
				panel.preset_name.text = "Integration test"
				panel._begin_pick()
				var event := InputEventMouseButton.new()
				event.button_index = MOUSE_BUTTON_LEFT
				event.pressed = true
				event.position = scene.spectator.unproject_position(scene.Frames.position(command.get("position_m", [10, 5, 0])))
				root.push_input(event)
			elif action == "spawn_apply":
				panel.apply_button.pressed.emit()
			elif action == "spawn_screenshot":
				_capture_screenshot.call_deferred(command.path)
			if scene.control_buttons.has(action) and not scene.control_buttons[action].disabled:
				scene.control_buttons[action].pressed.emit()
	var now := Time.get_ticks_msec()
	if now - last_report >= 100:
		var disabled := {}
		for action: String in scene.control_buttons:
			disabled[action] = scene.control_buttons[action].disabled
		var report := {"state": scene.adapter.get_control_state(), "disabled": disabled,
			"target_visible": scene.truck.visible, "spawn": scene.adapter.get_spawn_state(), "draft": scene.spawn_panel.draft,
			"spawn_pending": scene.spawn_panel.pending, "spawn_message": scene.spawn_panel.message.text,
			"spawn_apply_disabled": scene.spawn_panel.apply_button.disabled, "terrain_height": scene.spawn_panel.terrain_height,
			"stats": scene.adapter.get_stats(), "collision_failed": scene.collision_failed, "display": scene.control_display, "position": [scene.drone.position.x, scene.drone.position.y, scene.drone.position.z]}
		var file := FileAccess.open(report_path, FileAccess.WRITE)
		if file != null:
			file.store_string(JSON.stringify(report))
		last_report = now
	return false

func _capture_screenshot(path: String) -> void:
	# Keep SceneTree._process synchronous: its bool return controls tree lifetime.
	await RenderingServer.frame_post_draw
	var saved := root.get_texture().get_image().save_png(path)
	var done := FileAccess.open(path + ".done", FileAccess.WRITE)
	done.store_string(str(saved))
	done.close()

func _finalize() -> void:
	if scene != null and scene.adapter != null:
		scene.adapter.close()
