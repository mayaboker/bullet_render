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
			if scene.control_buttons.has(action) and not scene.control_buttons[action].disabled:
				scene.control_buttons[action].pressed.emit()
	var now := Time.get_ticks_msec()
	if now - last_report >= 100:
		var disabled := {}
		for action: String in scene.control_buttons:
			disabled[action] = scene.control_buttons[action].disabled
		var report := {"state": scene.adapter.get_control_state(), "disabled": disabled,
			"stats": scene.adapter.get_stats(), "collision_failed": scene.collision_failed, "display": scene.control_display, "position": [scene.drone.position.x, scene.drone.position.y, scene.drone.position.z]}
		var file := FileAccess.open(report_path, FileAccess.WRITE)
		if file != null:
			file.store_string(JSON.stringify(report))
		last_report = now
	return false

func _finalize() -> void:
	if scene != null and scene.adapter != null:
		scene.adapter.close()
