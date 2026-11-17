extends SceneTree

const WORLD_SIZE := 500.0
const GRID := 120
const OUTPUT_DIR := "res://../../terrain"
const HEIGHTMAP_PATH := OUTPUT_DIR + "/terrain_heightmap.f32"
const MANIFEST_PATH := OUTPUT_DIR + "/terrain_manifest.json"
const MESH_PATH := OUTPUT_DIR + "/terrain.obj"

var noise := FastNoiseLite.new()

func _initialize() -> void:
	noise.seed = 271828
	noise.frequency = 0.012
	var workspace_root := ProjectSettings.globalize_path("res://../../")
	var directory := DirAccess.open(workspace_root)
	if directory == null:
		push_error("Cannot open workspace root")
		quit(1)
		return
	directory.make_dir_recursive(workspace_root.path_join("terrain"))
	var heightmap_path := workspace_root.path_join("terrain/terrain_heightmap.f32")
	var manifest_path := workspace_root.path_join("terrain/terrain_manifest.json")
	var heightmap := FileAccess.open(heightmap_path, FileAccess.WRITE)
	if heightmap == null:
		push_error("Cannot create terrain heightmap")
		quit(1)
		return
	for z in range(GRID + 1):
		for x in range(GRID + 1):
			var px := (float(x) / GRID - 0.5) * WORLD_SIZE
			var pz := (float(z) / GRID - 0.5) * WORLD_SIZE
			heightmap.store_float(height_at(px, pz))
	heightmap.close()
	var mesh := FileAccess.open(workspace_root.path_join("terrain/terrain.obj"), FileAccess.WRITE)
	for z in range(GRID + 1):
		for x in range(GRID + 1):
			var px := (float(x) / GRID - 0.5) * WORLD_SIZE
			var pz := (float(z) / GRID - 0.5) * WORLD_SIZE
			# Bullet X/Y maps to Godot -Z/-X; keep the shared terrain in the Bullet frame.
			mesh.store_line("v %f %f %f" % [-pz, -px, height_at(px, pz)])
	for z in range(GRID):
		for x in range(GRID):
			var i := z * (GRID + 1) + x + 1
			mesh.store_line("f %d %d %d" % [i, i + 1, i + GRID + 1])
			mesh.store_line("f %d %d %d" % [i + 1, i + GRID + 2, i + GRID + 1])
	mesh.close()
	var manifest := {"world_size_m": WORLD_SIZE, "samples_x": GRID + 1, "samples_z": GRID + 1,
		"origin_m": [-WORLD_SIZE / 2.0, -WORLD_SIZE / 2.0], "height_units": "m",
		"sample_format": "float32_le", "collision_mesh": "terrain.obj"}
	var metadata := FileAccess.open(manifest_path, FileAccess.WRITE)
	metadata.store_string(JSON.stringify(manifest, "  "))
	metadata.close()
	print("Exported shared terrain heightmap: ", heightmap_path)
	quit(0)

func height_at(x: float, z: float) -> float:
	var clearing := smoothstep(22.0, 70.0, Vector2(x, z).length())
	return (noise.get_noise_2d(x, z) + 0.65) * 24.0 * clearing
