#pragma once
namespace bullet_render {
inline constexpr char SPAWN_REQUEST_TOPIC[] = "simulation/spawn_request";
inline constexpr char SPAWN_STATE_TOPIC[] = "simulation/spawn_state";
inline constexpr double SPAWN_HORIZONTAL_LIMIT_M = 250.0;
inline constexpr double SPAWN_MIN_ALTITUDE_M = 0.015;
inline constexpr double SPAWN_MAX_ALTITUDE_M = 500.0;
inline constexpr double DEFAULT_TAKEOFF_HEIGHT_M = 20.0;
inline constexpr double MIN_TAKEOFF_HEIGHT_M = 0.1;
inline constexpr double MAX_TAKEOFF_HEIGHT_M = 100.0;
inline constexpr int SPAWN_STATE_TIMEOUT_MS = 1000;
}
