extends PanelContainer

const Presets = preload("res://spawn_presets.gd")
const Frames = preload("res://frames.gd")
const UNSUPPORTED_MESSAGE := "Spawn selection requires connected external-RC physics."
const PICK_LAYER := 2
const CLEARANCE_M := 0.015
const INPUT_STEP_M := 0.001
const TERRAIN_RAY_EPSILON_M := 0.00001
const RETRY_MS := 100
const ACK_TIMEOUT_MS := 2000
# Wait beyond the last request's 1 s lifetime and the status cache's 1 s lifetime.
const RECONCILE_MS := 2500
var presets := Presets.new()
var camera: Camera3D
var adapter: ZmqCameraAdapter
var draft: Array = [0.0, 0.0, 0.015]
var applied: Dictionary = {}
var pending: Dictionary = {}
var picking := false
var pick_screen: Variant = null
var auto_z_pending := false
var terrain_dirty := true
var terrain_height := 0.0
var terrain_valid := false
var editable := false
var started_ms := 0
var retry_ms := 0
var last_sent_ms := 0
var serial := 0
var draft_marker: MeshInstance3D
var applied_marker: MeshInstance3D
var xy_label: Label
var applied_label: Label
var ground_label: Label
var takeoff: SpinBox
var altitude: SpinBox
var preset_list: OptionButton
var preset_name: LineEdit
var message: Label
var pick_button: Button
var load_button: Button
var save_button: Button
var apply_button: Button

func _ready() -> void:
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.025, 0.065, 0.08, 0.96)
	style.border_color = Color("36545e")
	style.set_border_width_all(1)
	style.set_corner_radius_all(6)
	style.content_margin_left = 12
	style.content_margin_right = 12
	style.content_margin_top = 10
	style.content_margin_bottom = 10
	add_theme_stylebox_override("panel", style)
	position = Vector2(24, 108)
	custom_minimum_size = Vector2(355, 0)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 8)
	add_child(column)
	var title := Label.new()
	title.text = "  SCENARIO START  ·  WORLD METRES"
	column.add_child(title)
	var legend := Label.new()
	legend.text = "Yellow: draft · Green: applied"
	legend.modulate = Color("bfd8cc")
	column.add_child(legend)
	pick_button = _button(column, "Pick position on landscape", _begin_pick)
	xy_label = Label.new()
	column.add_child(xy_label)
	var altitude_row := HBoxContainer.new()
	column.add_child(altitude_row)
	var z_label := Label.new()
	z_label.text = "World Z (m)   "
	altitude_row.add_child(z_label)
	altitude = SpinBox.new()
	altitude.min_value = 0.015
	altitude.max_value = 500.0
	altitude.step = INPUT_STEP_M
	altitude.value = draft[2]
	altitude.custom_minimum_size.x = 180
	altitude.tooltip_text = "Automatically filled from terrain when picking; adjust to start higher."
	altitude_row.add_child(altitude)
	altitude.value_changed.connect(func(value: float): draft[2] = value; _draft_changed())
	ground_label = Label.new()
	column.add_child(ground_label)
	var takeoff_row := HBoxContainer.new()
	column.add_child(takeoff_row)
	var takeoff_label := Label.new()
	takeoff_label.text = "Takeoff height (m)   "
	takeoff_row.add_child(takeoff_label)
	takeoff = SpinBox.new()
	takeoff.min_value = 0.1
	takeoff.max_value = 100.0
	takeoff.step = 0.1
	takeoff.value = Presets.DEFAULT_TAKEOFF_M
	takeoff.tooltip_text = "Climb above initial barometer altitude, then hover for 3 seconds."
	takeoff.custom_minimum_size.x = 150
	takeoff_row.add_child(takeoff)
	preset_list = OptionButton.new()
	column.add_child(preset_list)
	preset_name = LineEdit.new()
	preset_name.placeholder_text = "Scenario name"
	preset_name.max_length = 64
	preset_name.text = "Custom"
	column.add_child(preset_name)
	var row := HBoxContainer.new()
	column.add_child(row)
	load_button = _button(row, "Load draft", load_selected)
	save_button = _button(row, "Save preset", save_draft)
	apply_button = _button(column, "Apply · reset and preview", apply_draft)
	applied_label = Label.new()
	applied_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	applied_label.custom_minimum_size.x = 340
	column.add_child(applied_label)
	message = Label.new()
	message.custom_minimum_size = Vector2(340, 72)
	message.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	column.add_child(message)
	presets.load_file()
	_refresh_presets()
	message.text = presets.error
	_draft_changed()

func _button(parent: Node, label: String, callback: Callable) -> Button:
	var button := Button.new()
	button.text = label
	button.pressed.connect(callback)
	parent.add_child(button)
	return button

func setup(scene: Node3D, view_camera: Camera3D, transport: ZmqCameraAdapter) -> void:
	camera = view_camera
	adapter = transport
	draft_marker = _marker(scene, Color("ffd454"), 0.65)
	applied_marker = _marker(scene, Color("5aefbe"), 0.35)
	_draft_changed()

func _marker(scene: Node3D, color: Color, radius: float) -> MeshInstance3D:
	var marker := MeshInstance3D.new()
	var mesh := SphereMesh.new()
	mesh.radius = radius
	mesh.height = radius * 2
	marker.mesh = mesh
	marker.layers = 2 # Spectator only; excluded from the front camera.
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.albedo_color.a = 0.5
	marker.material_override = material
	scene.add_child(marker)
	marker.hide()
	return marker

func _draft_changed() -> void:
	terrain_dirty = true
	if xy_label != null:
		xy_label.text = "  Bullet X: %.3f    Y: %.3f" % [draft[0], draft[1]]
	if draft_marker != null:
		draft_marker.position = Frames.position(draft)
		draft_marker.visible = visible

func _begin_pick() -> void:
	if editable:
		picking = true
		message.text = "Click the landscape. Yellow = draft; green = applied."

func _unhandled_input(event: InputEvent) -> void:
	if visible and picking and editable and event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		pick_screen = event.position
		picking = false
		get_viewport().set_input_as_handled()

func _terrain_ray(from: Vector3, to: Vector3) -> Dictionary:
	var query := PhysicsRayQueryParameters3D.create(from, to, PICK_LAYER)
	return camera.get_world_3d().direct_space_state.intersect_ray(query)

func physics_update() -> void:
	if camera == null:
		return
	if pick_screen != null:
		if editable:
			var origin := camera.project_ray_origin(pick_screen)
			var hit := _terrain_ray(origin, origin + camera.project_ray_normal(pick_screen) * 2000.0)
			if hit.is_empty():
				message.text = "Click inside the 500 × 500 m landscape."
			else:
				var point: Vector3 = hit.position
				draft[0] = snappedf(-point.z, 0.001)
				draft[1] = snappedf(-point.x, 0.001)
				auto_z_pending = true
				_draft_changed()
				message.text = "World Z set from terrain. Set takeoff height, then Apply."
		pick_screen = null
	if terrain_dirty:
		var point := Frames.position(draft)
		var hit := _terrain_ray(Vector3(point.x, 100, point.z), Vector3(point.x, -100, point.z))
		terrain_valid = not hit.is_empty()
		terrain_height = hit.position.y if terrain_valid else 0.0
		if absf(terrain_height) < TERRAIN_RAY_EPSILON_M:
			terrain_height = 0.0
		if auto_z_pending and terrain_valid:
			draft[2] = maxf(CLEARANCE_M, ceilf((terrain_height + CLEARANCE_M) / INPUT_STEP_M) * INPUT_STEP_M)
			altitude.set_value_no_signal(draft[2])
			_draft_changed()
		if auto_z_pending:
			auto_z_pending = false
		terrain_dirty = false
		ground_label.text = "  Terrain %.3f m · minimum Z %.3f m" % [terrain_height, ceilf((terrain_height + CLEARANCE_M) / INPUT_STEP_M) * INPUT_STEP_M] if terrain_valid else "  Position outside landscape"

func valid_draft() -> bool:
	return Presets.valid_position(draft) and terrain_valid and not terrain_dirty and float(draft[2]) + 0.0000001 >= terrain_height + CLEARANCE_M

func _refresh_presets(select_name: String = "Default") -> void:
	preset_list.clear()
	for key in presets.presets:
		preset_list.add_item(key)
		if key == select_name:
			preset_list.select(preset_list.item_count - 1)

func load_selected() -> void:
	if not editable:
		return
	var selected := preset_list.get_item_text(preset_list.selected)
	draft = presets.presets[selected].position_m.duplicate()
	takeoff.set_value_no_signal(presets.presets[selected].takeoff_height_m)
	auto_z_pending = false
	preset_name.text = selected
	altitude.set_value_no_signal(draft[2])
	_draft_changed()
	message.text = "Loaded draft; Apply to move physics."

func save_draft() -> void:
	if not editable:
		return
	if not valid_draft():
		message.text = "Set Z at or above the local terrain minimum."
		return
	if presets.save_preset(preset_name.text.strip_edges(), draft, Presets.PATH, takeoff.value):
		_refresh_presets(preset_name.text.strip_edges())
		message.text = "Preset saved locally."
	else:
		message.text = presets.error

func apply_draft() -> void:
	if not editable or not valid_draft() or not Presets.valid_name(preset_name.text.strip_edges()) or applied.is_empty():
		return
	serial += 1
	pending = {"physics_id": applied.physics_id, "episode_id": applied.episode_id,
		"request_id": "%s-spawn-%s-%s" % [adapter.get_stats().controller_id, Time.get_ticks_msec(), serial],
		"scenario_name": preset_name.text.strip_edges(), "position_m": draft.duplicate(), "takeoff_height_m": takeoff.value}
	started_ms = Time.get_ticks_msec()
	retry_ms = 0
	message.text = "Applying · waiting for physics confirmation…"
	editable = false
	picking = false

func update_state(control: Dictionary, spawn: Dictionary, controller_id: String) -> void:
	var now := Time.get_ticks_msec()
	if not spawn.is_empty():
		if message.text.begins_with(UNSUPPORTED_MESSAGE):
			message.text = presets.error if not presets.error.is_empty() else "Pick a position or load a preset, then Apply."
		applied = spawn.duplicate(true)
		applied_label.text = "Applied: %s · X %.3f / Y %.3f / Z %.3f m" % [spawn.scenario_name, spawn.position_m[0], spawn.position_m[1], spawn.position_m[2]] + " · climb %.1f m" % spawn.get("takeoff_height_m", Presets.DEFAULT_TAKEOFF_M)
		if applied_marker != null:
			applied_marker.position = Frames.position(spawn.position_m)
			applied_marker.visible = visible
	if not pending.is_empty():
		if not control.is_empty() and control.physics_id != pending.physics_id:
			pending = {}
			message.text = "Physics restarted; review the current spawn and Apply again."
		elif not spawn.is_empty() and spawn.request_id == pending.request_id:
			message.text = "Applied · %s at %s m" % [spawn.scenario_name, spawn.position_m] if spawn.result == "applied" else "Apply rejected: " + spawn.reason
			pending = {}
		elif now - started_ms >= ACK_TIMEOUT_MS:
			message.text = "Apply unconfirmed · reconciling authoritative spawn…"
			if not spawn.is_empty() and now - last_sent_ms >= RECONCILE_MS:
				pending = {}
				message.text = "Apply was not confirmed. Current applied spawn: %s m. Retry when ready." % [spawn.position_m]
		elif now >= retry_ms:
			adapter.publish_spawn(pending)
			last_sent_ms = now
			retry_ms = now + RETRY_MS
	editable = pending.is_empty() and not control.is_empty() and control.state == "stopped" and control.controller_id == controller_id and not spawn.is_empty()
	if not editable:
		picking = false
	pick_button.disabled = not editable
	load_button.disabled = not editable
	save_button.disabled = not editable
	preset_list.disabled = not editable
	preset_name.editable = editable
	altitude.editable = editable
	takeoff.editable = editable
	ground_label.modulate = Color("bfd8cc") if valid_draft() else Color("ffad85")
	apply_button.disabled = not editable or not valid_draft() or not Presets.valid_name(preset_name.text.strip_edges())
	if spawn.is_empty() and pending.is_empty():
		message.text = UNSUPPORTED_MESSAGE + ("\n" + presets.error if not presets.error.is_empty() else "")
	if draft_marker != null:
		draft_marker.visible = visible
		applied_marker.visible = visible and not applied.is_empty()
