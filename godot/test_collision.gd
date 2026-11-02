extends SceneTree
const Frames = preload("res://frames.gd")
var sequence := 0
var scene: Node3D

func _initialize() -> void:
	_run.call_deferred()

func pose(local_position: Vector3, episode := "test") -> Dictionary:
	sequence += 1
	var p: Vector3 = scene.truck.global_transform * local_position
	return {"source": {"physics_id": "physics", "episode_id": episode, "stream_id": "stream",
		"step_index": sequence, "simulation_time_s": sequence / 240.0, "measurement_us": sequence * 10000},
		"position_m": [-p.z, -p.x, p.y]}

func require(value: bool, message: String) -> void:
	if not value:
		push_error(message)
		quit(1)
		assert(value, message)

func _run() -> void:
	scene = load("res://main.tscn").instantiate()
	scene.capture_only = true
	root.add_child(scene)
	await physics_frame
	await physics_frame
	var sample := pose(Vector3(0, 2.1, -0.95))
	var state := {"schema_version": 3, "physics_id": "physics", "episode_id": "test",
		"state": "running", "mission_start_step": null}
	require(not scene._collision_pose_eligible(state, sample), "collision enabled during manual idle")
	state.mission_start_step = sample.source.step_index + 1
	require(not scene._collision_pose_eligible(state, sample), "delayed pre-start pose eligible")
	sample.source.step_index += 1
	require(scene._collision_pose_eligible(state, sample), "post-start target collision disabled")
	sample.source.episode_id = "old"
	require(not scene._collision_pose_eligible(state, sample), "foreign episode eligible")
	sample.source.episode_id = "test"
	state.state = "stopped"
	require(not scene._collision_pose_eligible(state, sample), "stopped collision eligible")
	state.state = "running"
	state.schema_version = 2
	state.erase("mission_start_step")
	require(scene._collision_pose_eligible(state, sample), "legacy collision behavior changed")
	var detector: RefCounted = scene.collision_detector
	require(not detector.check(pose(Vector3(0, 2.1, -0.95))).is_empty(), "initial overlap missed")
	require(detector.check(pose(Vector3(0, 2.1, -0.95))).is_empty(), "duplicate collision")
	detector.reset()
	require(detector.check(pose(Vector3(2.0, 2.1, -0.95))).is_empty(), "near miss hit")
	detector.reset()
	require(not detector.check(pose(Vector3(1.5, 2.1, -0.95))).is_empty(), "envelope contact missed")
	detector.reset()
	require(detector.check(pose(Vector3(0, 2.1, -10))).is_empty(), "start unexpectedly hit")
	require(not detector.check(pose(Vector3(0, 2.1, 10))).is_empty(), "sweep tunneled through truck")
	detector.reset()
	require(detector.check(pose(Vector3(0, 2.1, -10), "old")).is_empty(), "old start hit")
	require(detector.check(pose(Vector3(0, 2.1, 10), "new")).is_empty(), "swept across reset")
	print("Truck collision: overlap, envelope, near miss, tilted geometry, sweep, latch and reset checks passed")
	scene.queue_free()
	quit(0)
