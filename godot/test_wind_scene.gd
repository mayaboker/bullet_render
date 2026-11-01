extends SceneTree
const Main = preload("res://main.gd")

func _initialize() -> void:
	_check.call_deferred()

func _check() -> void:
	var scene := Node3D.new()
	scene.set_script(Main)
	scene.capture_only = true
	root.add_child(scene)
	scene.set_process(false)
	await RenderingServer.frame_post_draw
	var settings := {"enabled":true,"behavior":"gusts","velocity_world_m_s":[3.,-1.,0.],"gust_stddev_m_s":[0.3,0.3,0.1],"correlation_time_s":1.,"seed":2}
	var control := {"physics_id":"physics","episode_id":"episode","controller_id":"godot","state":"running"}
	var wind := {"physics_id":"physics","episode_id":"episode","controller_id":"godot","request_id":null,"result":"idle","reason":"startup","settings":settings,"velocity_world_m_s":[3.2,-0.8,0.1]}
	scene.wind_panel.update_state(control,wind,"godot")
	scene.find_child("WindButton",true,false).pressed.emit()
	assert(scene.wind_panel.visible and not scene.spawn_panel.visible)
	scene.windsock.position = Vector3(0.,scene.height_at(0.,-8.),-8.)
	scene.windsock.update_wind(wind)
	var request := {"config":{"camera":{"width_px":640,"height_px":480,"fov_x_deg":90}},"pose":{"position_m":[0,0,3],"orientation_xyzw":[0,0,0,1],"sequence":1,"stream_id":"test"}}
	scene._capture(request)
	var with_sock: PackedByteArray = scene.viewport.get_texture().get_image().get_data()
	scene.windsock.hide()
	scene._capture(request)
	var without_sock: PackedByteArray = scene.viewport.get_texture().get_image().get_data()
	assert(with_sock == without_sock,"Spectator windsock or its shadows leaked into drone image")
	scene.windsock.show()
	scene.spectator.position = Vector3(15.,11.,15.)
	scene.spectator.look_at(Vector3(0.,1.,-6.))
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png("/tmp/bullet-wind-controls.png")
	print("Wind toolbar, spectator scene and exact FPV exclusion checks passed")
	root.remove_child(scene)
	scene.free()
	quit()
