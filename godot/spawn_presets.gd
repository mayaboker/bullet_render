extends RefCounted

const PATH := "user://spawn_scenarios.json"
const MAX_PRESETS := 128
const MAX_BYTES := 65536
const DEFAULT_TAKEOFF_M := 20.0
var presets: Dictionary = {"Default": {"position_m": [0.0, 0.0, 0.015], "takeoff_height_m": DEFAULT_TAKEOFF_M}}
var error := ""

static func valid_name(value: Variant) -> bool:
	if not value is String or value.is_empty() or value.length() > 64 or value.strip_edges() != value:
		return false
	for i in value.length():
		if value.unicode_at(i) < 32:
			return false
	return true

static func valid_position(value: Variant) -> bool:
	if not value is Array or value.size() != 3:
		return false
	for n in value:
		if (not n is float and not n is int) or not is_finite(float(n)):
			return false
	return absf(value[0]) <= 250.0 and absf(value[1]) <= 250.0 and value[2] >= 0.015 and value[2] <= 500.0

static func valid_takeoff(value: Variant) -> bool:
	return (value is float or value is int) and is_finite(float(value)) and value >= 0.1 and value <= 100.0

func load_file(path: String = PATH) -> bool:
	error = ""
	if not FileAccess.file_exists(path):
		return true
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null or file.get_length() > MAX_BYTES:
		error = "Could not read presets"
		return false
	var value: Variant = JSON.parse_string(file.get_as_text())
	if not value is Dictionary or value.size() != 2 or not value.get("presets") is Dictionary:
		error = "Invalid preset file"
		return false
	var version: Variant = value.get("schema_version")
	if (not version is int and not version is float) or (version != 1 and version != 2):
		error = "Unsupported preset version"
		return false
	var entries: Dictionary = value.presets
	if entries.size() > MAX_PRESETS:
		error = "Too many presets"
		return false
	var converted := {}
	for key in entries:
		var entry: Variant = entries[key]
		if version == 1:
			entry = {"position_m": entry, "takeoff_height_m": DEFAULT_TAKEOFF_M}
		if not valid_name(key) or key == "Default" or not entry is Dictionary or entry.size() != 2 or not valid_position(entry.get("position_m")) or not valid_takeoff(entry.get("takeoff_height_m")):
			error = "Invalid preset entry"
			return false
		converted[key] = entry.duplicate(true)
	presets = {"Default": {"position_m": [0.0, 0.0, 0.015], "takeoff_height_m": DEFAULT_TAKEOFF_M}}
	presets.merge(converted, true)
	return true

func save_preset(preset_name: String, position: Array, path: String = PATH, takeoff_height_m: float = DEFAULT_TAKEOFF_M) -> bool:
	error = ""
	if not valid_takeoff(takeoff_height_m) or not valid_name(preset_name) or preset_name == "Default" or not valid_position(position):
		error = "Use a custom name, valid position and takeoff height; Default is read-only"
		return false
	var entries := presets.duplicate(true)
	entries.erase("Default")
	entries[preset_name] = {"position_m": position.duplicate(), "takeoff_height_m": takeoff_height_m}
	if entries.size() > MAX_PRESETS:
		error = "Too many presets"
		return false
	var temporary := path + ".tmp-%s-%s" % [OS.get_process_id(), Time.get_ticks_usec()]
	var file := FileAccess.open(temporary, FileAccess.WRITE)
	if file == null:
		error = "Could not write presets"
		return false
	file.store_string(JSON.stringify({"schema_version": 2, "presets": entries}, "\t", true, true))
	file.flush()
	var write_error := file.get_error()
	file.close()
	if write_error != OK or DirAccess.rename_absolute(ProjectSettings.globalize_path(temporary), ProjectSettings.globalize_path(path)) != OK:
		DirAccess.remove_absolute(ProjectSettings.globalize_path(temporary))
		error = "Could not save presets; previous file kept"
		return false
	presets[preset_name] = entries[preset_name].duplicate(true)
	return true
