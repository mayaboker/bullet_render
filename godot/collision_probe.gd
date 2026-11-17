extends "res://control_probe.gd"
# Test-only trajectory and screenshot; production collision detection is unchanged.
var screenshot_saved := false

func _start() -> void:
	super._start()
	var args := OS.get_cmdline_user_args()
	var points := []
	for z in [-10.0, 10.0]:
		var point: Vector3 = scene.truck.global_transform * Vector3(0, 2.1, z)
		points.append([-point.z, -point.x, point.y])
	var file := FileAccess.open(args[3], FileAccess.WRITE)
	file.store_string(JSON.stringify(points))

func _process(delta: float) -> bool:
	super._process(delta)
	if not screenshot_saved and scene != null and scene.control_display == "COMPLETED":
		screenshot_saved = true
		_screenshot.call_deferred()
	return false

func _screenshot() -> void:
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png(OS.get_cmdline_user_args()[4])
