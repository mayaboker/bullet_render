class_name BulletFrames
extends RefCounted

# Bullet X forward/Y left/Z up -> Godot -Z forward/-X left/Y up.
static func position(value: Array) -> Vector3:
	return Vector3(-value[1], value[2], -value[0])

static func orientation(value: Array) -> Basis:
	return Basis(Quaternion(-value[1], value[2], -value[0], value[3]).normalized())
