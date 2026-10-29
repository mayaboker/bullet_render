extends PanelContainer

const RETRY_MS := 100
const ACK_TIMEOUT_MS := 2000
const RECONCILE_MS := 2500
const MAX_SEED := 9007199254740991
var adapter: ZmqCameraAdapter
var pending: Dictionary = {}
var applied: Dictionary = {}
var inputs: Dictionary = {}
var wind_enabled: CheckBox
var gust_enabled: CheckBox
var apply_button: Button
var message: Label
var readback: Label
var dirty := false
var loading := false
var identity := ""
var loaded := false
var available := false
var serial := 0
var started_ms := 0
var retry_ms := 0
var last_sent_ms := 0

func _ready() -> void:
	position = Vector2(24, 108)
	custom_minimum_size = Vector2(395, 0)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.025, 0.065, 0.08, 0.96)
	style.content_margin_left = 12
	style.content_margin_right = 12
	style.content_margin_top = 10
	style.content_margin_bottom = 10
	style.set_corner_radius_all(6)
	add_theme_stylebox_override("panel", style)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 8)
	add_child(column)
	_label(column, "WIND · WORLD X / Y / Z · m/s")
	_label(column, "Vector points toward air travel. +Z is up.")
	wind_enabled = CheckBox.new()
	wind_enabled.text = "Enable wind"
	column.add_child(wind_enabled)
	wind_enabled.toggled.connect(func(_value: bool): _changed())
	_vector_row(column, "Mean velocity", "velocity_world_m_s", false)
	gust_enabled = CheckBox.new()
	gust_enabled.text = "Enable correlated gusts"
	column.add_child(gust_enabled)
	gust_enabled.toggled.connect(func(_value: bool): _changed())
	_vector_row(column, "Gust stddev", "gust_stddev_m_s", true)
	_label(column, "Stddev is typical variation, not a maximum.")
	_scalar_row(column, "Gust correlation time (s)", "correlation_time_s", 0.001, 60., 0.1)
	_scalar_row(column, "Random seed", "seed", -MAX_SEED, MAX_SEED, 1.)
	apply_button = Button.new()
	apply_button.text = "Apply wind"
	apply_button.accessibility_name = "Apply wind settings"
	apply_button.pressed.connect(_apply)
	column.add_child(apply_button)
	readback = _label(column, "Waiting for physics wind state")
	message = _label(column, "Wind controls require external-RC physics.")
	message.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_set_editable(false)

func _label(parent: Node, text: String) -> Label:
	var label := Label.new()
	label.text = text
	parent.add_child(label)
	return label

func _spin(minimum: float, maximum: float, step: float, title: String) -> SpinBox:
	var spin := SpinBox.new()
	spin.min_value = minimum
	spin.max_value = maximum
	spin.step = step
	spin.allow_greater = true
	spin.allow_lesser = minimum < 0
	spin.custom_minimum_size.x = 110
	spin.accessibility_name = title
	spin.value_changed.connect(func(_value: float): _changed())
	return spin

func _vector_row(parent: Node, title: String, key: String, nonnegative: bool) -> void:
	_label(parent, title + " (X / Y / Z)")
	var row := HBoxContainer.new()
	parent.add_child(row)
	var values: Array[SpinBox] = []
	for axis: String in ["X", "Y", "Z"]:
		var spin := _spin(0. if nonnegative else -50., 50., 0.1, title + " " + axis)
		row.add_child(spin)
		values.append(spin)
	inputs[key] = values

func _scalar_row(parent: Node, title: String, key: String, minimum: float, maximum: float, step: float) -> void:
	var row := HBoxContainer.new()
	parent.add_child(row)
	_label(row, title)
	var spin := _spin(minimum, maximum, step, title)
	if key == "seed":
		spin.allow_greater = false
		spin.allow_lesser = false
	row.add_child(spin)
	inputs[key] = spin

func _vector_text(value: Array) -> String:
	return "[%+.2f, %+.2f, %+.2f]" % [value[0], value[1], value[2]]

func _changed() -> void:
	if not loading:
		dirty = true

func _load(settings: Dictionary) -> void:
	loading = true
	wind_enabled.button_pressed = settings.enabled
	gust_enabled.button_pressed = settings.behavior == "gusts"
	for key: String in ["velocity_world_m_s", "gust_stddev_m_s"]:
		for i in range(3):
			inputs[key][i].value = settings[key][i]
	inputs.correlation_time_s.value = settings.correlation_time_s
	inputs.seed.value = settings.seed
	loading = false
	dirty = false
	loaded = true

func draft() -> Dictionary:
	var velocity: Array = []
	var deviations: Array = []
	for i in range(3):
		velocity.append(inputs.velocity_world_m_s[i].value)
		deviations.append(inputs.gust_stddev_m_s[i].value)
	return {"enabled": wind_enabled.button_pressed, "behavior": "gusts" if gust_enabled.button_pressed else "constant",
		"velocity_world_m_s": velocity, "gust_stddev_m_s": deviations,
		"correlation_time_s": inputs.correlation_time_s.value, "seed": int(inputs.seed.value)}

func _set_editable(editable: bool) -> void:
	wind_enabled.disabled = not editable
	gust_enabled.disabled = not editable
	apply_button.disabled = not editable
	for key: String in inputs:
		if inputs[key] is Array:
			for spin: SpinBox in inputs[key]:
				spin.editable = editable
		else:
			inputs[key].editable = editable

func _apply() -> void:
	if not available or not pending.is_empty() or applied.is_empty():
		return
	serial += 1
	pending = {"physics_id": applied.physics_id, "episode_id": applied.episode_id,
		"request_id": "wind-%d-%d" % [Time.get_ticks_usec(), serial], "settings": draft()}
	started_ms = Time.get_ticks_msec()
	retry_ms = started_ms
	message.text = "Applying wind…"

func update_state(control: Dictionary, wind: Dictionary, controller_id: String) -> void:
	var key := "" if control.is_empty() else "%s/%s/%s" % [control.physics_id, control.episode_id, control.controller_id]
	if key != identity:
		identity = key
		pending = {}
		loaded = false
		dirty = false
	available = not control.is_empty() and control.controller_id == controller_id and not wind.is_empty() and control.state in ["running", "stopped"]
	if not wind.is_empty():
		applied = wind.duplicate(true)
		readback.text = "Applied mean: %s m/s\nCurrent wind: %s m/s" % [_vector_text(wind.settings.velocity_world_m_s), _vector_text(wind.velocity_world_m_s)]
		if not loaded or (not dirty and pending.is_empty()):
			_load(wind.settings)
	else:
		readback.text = "Wind unavailable"
	if not pending.is_empty():
		var now := Time.get_ticks_msec()
		if not wind.is_empty() and wind.request_id == pending.request_id:
			message.text = "Wind applied" if wind.result == "applied" else "Wind rejected: %s" % wind.reason
			pending = {}
			if wind.result == "applied":
				_load(wind.settings)
		elif now - started_ms >= ACK_TIMEOUT_MS:
			message.text = "Apply unconfirmed · waiting for physics readback"
			if not wind.is_empty() and now - last_sent_ms >= RECONCILE_MS:
				pending = {}
				message.text = "Apply unconfirmed. Review applied values before retrying."
		elif available and now >= retry_ms:
			adapter.publish_wind(pending)
			last_sent_ms = now
			retry_ms = now + RETRY_MS
	elif not available:
		message.text = "Wind controls require connected external-RC physics."
	elif message.text.begins_with("Wind controls require"):
		message.text = "Edit settings, then Apply to update physics."
	_set_editable(available and pending.is_empty())
