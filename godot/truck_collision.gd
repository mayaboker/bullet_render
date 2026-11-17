extends RefCounted
# Queries only: no Godot rigid-body simulation or physical response.
const Frames = preload("res://frames.gd")
const DRONE_RADIUS_M := 0.42
const TARGET_LAYER := 1
const MAX_SWEEP_GAP_US := 1000000
var world: World3D
var previous: Dictionary = {}
var identity := ""
var latched := false
var sphere := SphereShape3D.new()

func _init(space: World3D, truck: Node3D) -> void:
	world = space
	sphere.radius = DRONE_RADIUS_M
	var body := StaticBody3D.new()
	body.name = "TruckDetectionOnly"
	body.collision_layer = TARGET_LAYER
	body.collision_mask = 0
	truck.add_child(body)
	for part in truck.get_children():
		if part is MeshInstance3D:
			var collider := CollisionShape3D.new()
			collider.shape = part.mesh.create_convex_shape()
			collider.transform = part.transform
			body.add_child(collider)

func reset() -> void:
	previous = {}
	identity = ""
	latched = false

func check(pose: Dictionary) -> Dictionary:
	var source: Dictionary = pose.source
	var key := "%s/%s/%s" % [source.physics_id, source.episode_id, source.stream_id]
	if key != identity:
		reset()
		identity = key
	if latched:
		return {}
	if not previous.is_empty() and source.step_index <= previous.source.step_index:
		return {}
	var finish := Frames.position(pose.position_m)
	var start := finish
	if not previous.is_empty() and source.measurement_us - previous.source.measurement_us <= MAX_SWEEP_GAP_US:
		start = Frames.position(previous.position_m)
	var query := PhysicsShapeQueryParameters3D.new()
	query.shape = sphere
	query.collision_mask = TARGET_LAYER
	query.margin = 0.0
	query.transform = Transform3D(Basis.IDENTITY, start)
	var space := world.direct_space_state
	var hit := not space.intersect_shape(query, 1).is_empty()
	var fraction := 0.0
	if not hit:
		query.motion = finish - start
		var fractions := space.cast_motion(query)
		if fractions.size() == 2 and fractions[0] < 1.0:
			hit = true
			fraction = fractions[1]
		else:
			query.motion = Vector3.ZERO
			query.transform.origin = finish
			hit = not space.intersect_shape(query, 1).is_empty()
			fraction = 1.0
	previous = pose.duplicate(true)
	if not hit:
		return {}
	latched = true
	var position := start.lerp(finish, fraction)
	return {"source": source.duplicate(true), "target_id": "red_truck",
		"drone_position_m": [-position.z, -position.x, position.y]}
