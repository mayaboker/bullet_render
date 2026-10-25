extends SceneTree
const WindPanel = preload("res://wind_panel.gd")
const Sock = preload("res://windsock.gd")

func _initialize() -> void:
	_check.call_deferred()

func _check() -> void:
	var panel := WindPanel.new()
	root.add_child(panel)
	var settings := {"enabled":true, "behavior":"gusts", "velocity_world_m_s":[2.,0.,0.],
		"gust_stddev_m_s":[0.3,0.3,0.1], "correlation_time_s":1., "seed":2}
	var control := {"physics_id":"physics", "episode_id":"episode", "controller_id":"godot", "state":"running"}
	var wind := {"physics_id":"physics", "episode_id":"episode", "controller_id":"godot", "settings":settings,
		"velocity_world_m_s":[2.,1.,0.], "request_id":null, "result":"idle", "reason":"startup"}
	panel.update_state(control, wind, "godot")
	assert(not panel.apply_button.disabled)
	assert(panel.draft().velocity_world_m_s == settings.velocity_world_m_s)
	assert(is_equal_approx(panel.draft().gust_stddev_m_s[0], 0.3))
	assert(panel.draft().seed == 2)
	panel.inputs.velocity_world_m_s[0].value = 4.
	panel.update_state(control, wind, "godot")
	assert(panel.draft().velocity_world_m_s[0] == 4.)
	panel._apply()
	assert(not panel.pending.is_empty())
	wind.request_id = panel.pending.request_id
	wind.result = "applied"
	wind.settings = panel.draft()
	panel.hide()
	panel.update_state(control, wind, "godot")
	assert(panel.pending.is_empty() and not panel.visible)
	assert(panel.message.text == "Wind applied")
	panel.update_state(control, {}, "godot")
	assert(panel.apply_button.disabled and panel.readback.text == "Wind unavailable")
	var sock := Sock.new()
	root.add_child(sock)
	sock.place_at_spawn([10.,20.,3.], func(_x:float, _z:float): return 7.)
	assert(sock.position == Vector3(-28.,7.,-10.))
	for sample: Array in [[1.,0.,0.],[-1.,0.,0.],[0.,1.,0.],[0.,-1.,0.],[0.,0.,1.],[0.,0.,-1.]]:
		sock.update_wind({"velocity_world_m_s":sample})
		assert((-sock.sock.basis.z).is_equal_approx(Vector3(-sample[1],sample[2],-sample[0])))
	sock.update_wind({"velocity_world_m_s":[0.,0.,0.]})
	assert((-sock.sock.basis.z).is_equal_approx(Vector3.DOWN))
	sock.update_wind({})
	assert(sock.indicator.text == "Wind unavailable")
	for node in sock.find_children("*", "GeometryInstance3D", true, false):
		assert(node.layers == 2)
		if node is MeshInstance3D:
			assert(node.cast_shadow == GeometryInstance3D.SHADOW_CASTING_SETTING_OFF)
	panel.queue_free()
	sock.queue_free()
	print("Wind panel and windsock checks passed")
	quit()
