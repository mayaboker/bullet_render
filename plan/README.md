# Plans

| Plan | Description | Status | Implementation commit |
| --- | --- | --- | --- |
| [0001 Run-all launcher](0001-run-all-launcher.md) | Start Bullet View and Bullet Visual in the background before the flight supervisor. | Complete | `1b5378f` |
| [0002 Shared terrain heightmap](0002-shared-terrain-heightmap.md) | Export and render the shared terrain asset used by PyBullet collision. | Implemented and verified | `471576d` |
| [0003 Safe terrain spawn picker](0003-spawn-picker-clearance.md) | Keep picked/default positions above the physics vehicle-clearance margin. | Implemented; Godot spawn test passed | `adf9c73` |
