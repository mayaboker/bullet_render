extends Node3D

const Frames = preload("res://frames.gd")
const CALM_SPEED_M_S := 0.1
const SPAWN_OFFSET_WORLD_Y_M := 8.0
const POLE_HEIGHT_M := 3.0
const SOCK_LENGTH_M := 2.0
var sock: Node3D
var indicator: Label3D
var fabrics: Array[StandardMaterial3D] = []

func _ready() -> void:
	var pole := CylinderMesh.new()
	pole.height = POLE_HEIGHT_M
	pole.top_radius = 0.035
	pole.bottom_radius = 0.045
	_mesh(self, pole, Vector3(0, POLE_HEIGHT_M / 2, 0), Color("b8c8ce"))
	sock = Node3D.new()
	sock.position = Vector3(0, POLE_HEIGHT_M, 0)
	add_child(sock)
	for i in range(4):
		var segment := CylinderMesh.new()
		segment.height = SOCK_LENGTH_M / 4
		segment.top_radius = lerpf(0.32, 0.12, float(i) / 4)
		segment.bottom_radius = lerpf(0.32, 0.12, float(i + 1) / 4)
		var fabric := _mesh(sock, segment, Vector3(0, 0, -(i + 0.5) * segment.height), Color("ee652b") if i % 2 == 0 else Color("f2eee0"))
		fabric.rotation.x = PI / 2
		fabrics.append(fabric.material_override)
	indicator = Label3D.new()
	indicator.position = Vector3(0, POLE_HEIGHT_M + 0.7, 0)
	indicator.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	indicator.font_size = 40
	indicator.pixel_size = 0.008
	indicator.layers = 2
	add_child(indicator)
	update_wind({})

func _mesh(parent: Node3D, mesh: Mesh, at: Vector3, color: Color) -> MeshInstance3D:
	var node := MeshInstance3D.new()
	node.mesh = mesh
	node.position = at
	node.layers = 2
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 0.9
	node.material_override = material
	parent.add_child(node)
	return node

func place_at_spawn(spawn: Array, height_at: Callable) -> void:
	var point := Frames.position([spawn[0], spawn[1] + SPAWN_OFFSET_WORLD_Y_M, 0.])
	point.y = height_at.call(point.x, point.z)
	position = point

func update_wind(state: Dictionary) -> void:
	if state.is_empty():
		indicator.text = "Wind unavailable"
		for fabric in fabrics:
			fabric.albedo_color = Color("78878b")
		return
	for i in range(fabrics.size()):
		fabrics[i].albedo_color = Color("ee652b") if i % 2 == 0 else Color("f2eee0")
	var velocity := Frames.position(state.velocity_world_m_s)
	var speed := velocity.length()
	if speed < CALM_SPEED_M_S:
		sock.basis = Basis(Quaternion(Vector3.FORWARD, Vector3.DOWN))
		indicator.text = "Wind calm"
	else:
		sock.basis = Basis(Quaternion(Vector3.FORWARD, velocity.normalized()))
		indicator.text = "Wind %.1f m/s" % speed
