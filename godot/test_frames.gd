extends SceneTree
const Frames = preload("res://frames.gd")
func _initialize() -> void:
	_check.call_deferred()

func _check() -> void:
	assert(Frames.position([1, 2, 3]).is_equal_approx(Vector3(-2, 3, -1)))
	assert(Frames.orientation([0, 0, 0, 1]).is_equal_approx(Basis.IDENTITY))
	var s := sqrt(0.5)
	assert((Frames.orientation([0, 0, s, s]) * Vector3.FORWARD).is_equal_approx(Vector3.LEFT))
	assert((Frames.orientation([0, s, 0, s]) * Vector3.FORWARD).is_equal_approx(Vector3.DOWN))
	assert((Frames.orientation([s, 0, 0, s]) * Vector3.UP).is_equal_approx(Vector3.RIGHT))
	var viewport := SubViewport.new()
	viewport.size = Vector2i(640, 480)
	root.add_child(viewport)
	var camera := Camera3D.new()
	camera.keep_aspect = Camera3D.KEEP_WIDTH
	camera.fov = 90
	viewport.add_child(camera)
	assert(is_equal_approx(camera.get_camera_projection().get_fov(), 90.0))
	print("Frame conversion and horizontal FOV checks passed")
	quit()
