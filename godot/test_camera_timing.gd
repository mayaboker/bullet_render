extends SceneTree
const Main = preload("res://main.gd")

func _initialize() -> void:
	assert(Main.timing_summary([]).is_empty())
	var samples := [100.0, 1.0, 2.0, 3.0]
	var summary := Main.timing_summary(samples)
	assert(summary == {"mean": 26.5, "p95": 100.0, "max": 100.0})
	assert(samples == [100.0, 1.0, 2.0, 3.0])
	assert(Main.timing_summary([2.0]) == {"mean": 2.0, "p95": 2.0, "max": 2.0})
	print("Camera timing summary checks passed")
	quit()
