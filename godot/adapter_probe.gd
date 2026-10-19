extends SceneTree
var adapter: ZmqCameraAdapter
var busy := false
var delay := 0.0
var close_after := 0
var previous: Dictionary = {}
var control_action := ""
var control_sent := false
var previous_spawn: Dictionary = {}
var previous_control: Dictionary = {}

func _initialize() -> void:
	_start.call_deferred()

func _start() -> void:
	var args := OS.get_cmdline_user_args()
	adapter = ZmqCameraAdapter.new()
	root.add_child(adapter)
	if args.size() > 2:
		delay = args[2].to_float()
	if args.size() > 3:
		close_after = args[3].to_int()
	if args.size() > 4:
		control_action = args[4]
	if not adapter.start(args[0]):
		print("PROBE ", JSON.stringify(adapter.get_stats()))
		quit(1)
		return
	print("PROBE ", JSON.stringify({"ready":true}))

func _process(_delta: float) -> bool:
	if adapter == null:
		return false
	var control_state := adapter.get_control_state()
	if control_state != previous_control:
		print("PROBE ", JSON.stringify({"control_state": control_state}))
		previous_control = control_state.duplicate()
	var spawn := adapter.get_spawn_state()
	if spawn != previous_spawn:
		print("PROBE ", JSON.stringify({"spawn_state": spawn}))
		previous_spawn = spawn.duplicate(true)
	if not control_sent and control_action == "spawn" and not spawn.is_empty():
		control_sent = adapter.publish_spawn({"physics_id": spawn.physics_id, "episode_id": spawn.episode_id,
			"request_id": "native-pick", "scenario_name": "Hill", "position_m": [12.0, -34.0, 56.789], "takeoff_height_m": 7.5})
	if not control_sent and control_action != "spawn" and not control_action.is_empty() and not control_state.is_empty():
		control_sent = adapter.publish_control(control_action)
	if not busy:
		var request := adapter.poll_request()
		if not request.is_empty():
			_publish(request)
	var stats := adapter.get_stats()
	if stats != previous:
		print("PROBE ", JSON.stringify(stats))
		previous = stats.duplicate()
	if not stats.error.is_empty():
		quit(1)
	return false

func _publish(request: Dictionary) -> void:
	busy = true
	print("PROBE ", JSON.stringify({"capturing":request.pose.sequence}))
	if delay > 0:
		await create_timer(delay).timeout
	var camera: Dictionary = request.config.camera
	var pixels := PackedByteArray()
	pixels.resize(int(camera.width_px) * int(camera.height_px) * 3)
	pixels.fill(42)
	var success := adapter.publish_frame(pixels, request)
	print("PROBE ", JSON.stringify({"sequence":request.pose.sequence, "stream_id":request.pose.stream_id, "success":success, "linear_velocity_m_s":request.pose.linear_velocity_m_s, "stats":adapter.get_stats()}))
	busy = false
	if close_after > 0 and adapter.get_stats().frames >= close_after:
		adapter.close()
		adapter.close()
		print("PROBE ", JSON.stringify({"closed":true, "stats":adapter.get_stats()}))
		quit()

func _finalize() -> void:
	if adapter != null:
		adapter.close()
