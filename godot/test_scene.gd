extends SceneTree
const Main = preload("res://main.gd")

func _initialize() -> void:
	_check.call_deferred()

func _check() -> void:
	var scene := Node3D.new()
	scene.set_script(Main)
	scene.capture_only = true
	root.add_child(scene)
	await RenderingServer.frame_post_draw
	assert(scene.fpv.cull_mask == 1)
	assert(is_equal_approx(Vector2(scene.truck.position.x, scene.truck.position.z).length(), 50.0))
	assert(is_equal_approx(scene.truck.position.y, scene.height_at(scene.truck.position.x, scene.truck.position.z)))
	assert(scene.truck.get_node("Cab").material_override.albedo_color == Color("cf252a"))
	assert(scene.truck.get_node("CargoBox").material_override.albedo_color == Color("cf252a"))
	scene._build_drone_visual("heavy_quad")
	var front_props := 0
	var rear_props := 0
	for child in scene.drone.get_children():
		if child is MeshInstance3D and child.mesh is CylinderMesh:
			if child.position.z < 0:
				front_props += 1
				assert(child.material_override.albedo_color == Color("f36f3b"))
			else:
				rear_props += 1
				assert(child.material_override.albedo_color == Color("2f7de1"))
	assert(front_props == 2 and rear_props == 2)
	var upward_normals := false
	for child in scene.get_children():
		if child is MeshInstance3D and child.material_override is ShaderMaterial:
			var arrays: Array = child.mesh.surface_get_arrays(0)
			assert(arrays[Mesh.ARRAY_NORMAL][0].y > 0)
			upward_normals = true
	assert(upward_normals)
	# Unlit reference cards verify row orientation and channel order through the real camera readback.
	var cards: Array[MeshInstance3D] = []
	for item in [[Vector3(-2, 3, -5), Color.RED], [Vector3(2, 3, -5), Color.GREEN], [Vector3(0, 5, -5), Color.BLUE]]:
		var card: MeshInstance3D = scene.box(scene, item[0], Vector3(1, 1, 0.1), item[1])
		card.material_override.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		cards.append(card)
	# Let newly created materials compile before testing immediate camera capture.
	await RenderingServer.frame_post_draw
	await process_frame
	var camera := {"width_px":640, "height_px":480, "fov_x_deg":90}
	scene._capture({"config":{"camera":camera}, "pose":{"position_m":[0,0,3], "orientation_xyzw":[0,0,0,1], "sequence":1, "stream_id":"test"}})
	assert(not scene.busy, "Camera capture must complete before window presentation")
	while scene.busy:
		await process_frame
	var rendered: Image = scene.viewport.get_texture().get_image()
	rendered.convert(Image.FORMAT_RGB8)
	var raw := rendered.get_data()
	assert(raw.size() == 640*480*3)
	var image := Image.create_from_data(640, 480, false, Image.FORMAT_RGB8, raw)
	assert(image.get_pixel(191, 240).r > 0.9 and image.get_pixel(191, 240).g < 0.1)
	assert(image.get_pixel(449, 240).g > 0.9 and image.get_pixel(449, 240).r < 0.1)
	assert(image.get_pixel(320, 111).b > 0.9 and image.get_pixel(320, 111).r < 0.1)
	assert(image.get_pixel(28, 28).b > 0.2 and image.get_pixel(28, 28).b > image.get_pixel(28, 28).r)
	assert(scene.fpv.get_camera_projection().get_fov() > 89.99 and scene.fpv.get_camera_projection().get_fov() < 90.01)
	# A second capture must use the new pose immediately, without another scene frame.
	scene._capture({"config":{"camera":camera}, "pose":{"position_m":[0,0,4], "orientation_xyzw":[0,0,0,1], "sequence":2, "stream_id":"test"}})
	assert(not scene.busy)
	var moved: Image = scene.viewport.get_texture().get_image()
	assert(moved.get_pixel(191, 304).r > 0.9 and moved.get_pixel(191, 304).g < 0.1)
	assert(moved.get_pixel(191, 240).r < 0.9)
	for card in cards:
		card.queue_free()
	await process_frame
	scene._capture({"config":{"camera":camera}, "pose":{"position_m":[0,0,3], "orientation_xyzw":[0,0,0,1], "sequence":3, "stream_id":"test"}})
	while scene.busy:
		await process_frame
	var truck_image: Image = scene.viewport.get_texture().get_image()
	var center: Vector2 = scene.fpv.unproject_position(scene.truck.global_position + Vector3.UP * 1.8)
	var red_pixels := 0
	for y in range(maxi(0, int(center.y) - 24), mini(480, int(center.y) + 24)):
		for x in range(maxi(0, int(center.x) - 24), mini(640, int(center.x) + 24)):
			var pixel := truck_image.get_pixel(x, y)
			if pixel.r > 0.25 and pixel.r > pixel.g * 2.0 and pixel.r > pixel.b * 2.0:
				red_pixels += 1
	assert(red_pixels > 20, "Red truck must be visible in the drone's forward camera")
	truck_image.save_png("/tmp/bullet-render-truck-fpv.png")
	scene.set_process(false)
	var screen := Rect2(Vector2.ZERO, Vector2(root.size))
	assert(not scene.spectator.is_position_behind(scene.truck.global_position))
	assert(screen.has_point(scene.spectator.unproject_position(scene.truck.global_position + Vector3.UP * 1.5)))
	assert(screen.has_point(scene.spectator.unproject_position(scene.drone.global_position)))
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png("/tmp/bullet-render-truck-default.png")
	scene.spectator.position = Vector3(38, 30, 30)
	scene.spectator.look_at(Vector3(0, 2, -20))
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png("/tmp/bullet-render-scene.png")
	scene.spectator.position = scene.truck.global_position + Vector3(9, 6, 11)
	scene.spectator.look_at(scene.truck.global_position + Vector3.UP * 1.5)
	var pan_start: Vector3 = scene.spectator_pan
	var left_down := InputEventMouseButton.new()
	left_down.button_index = MOUSE_BUTTON_LEFT
	left_down.pressed = true
	scene._unhandled_input(left_down)
	var pan_motion := InputEventMouseMotion.new()
	pan_motion.relative = Vector2(10, -20)
	scene._unhandled_input(pan_motion)
	assert(scene.spectator_pan.x > pan_start.x and scene.spectator_pan.y > pan_start.y, "left drag did not pan world X/Y")
	var left_up := InputEventMouseButton.new()
	left_up.button_index = MOUSE_BUTTON_LEFT
	left_up.pressed = false
	scene._unhandled_input(left_up)
	var ignored_motion := InputEventMouseMotion.new()
	ignored_motion.relative = Vector2(100, 100)
	scene._unhandled_input(ignored_motion)
	assert(scene.spectator_pan.x < 20.0 and scene.spectator_pan.y < 20.0, "released pan kept moving")
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png("/tmp/bullet-render-truck-detail.png")
	print("Scene normals, RGB/channel orientation, immediate pose/readback, FOV, UI exclusion and red truck at 50 m checks passed")
	root.remove_child(scene)
	scene.free()
	quit()
