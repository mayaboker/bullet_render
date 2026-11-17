#pragma once
#include <cstddef>
namespace bullet_render {
inline constexpr char COMMAND_TOPIC[] = "simulation/command";
inline constexpr char HEARTBEAT_TOPIC[] = "simulation/heartbeat";
inline constexpr char STATE_TOPIC[] = "simulation/state";
inline constexpr int HEARTBEAT_INTERVAL_MS = 500;
inline constexpr int HEARTBEAT_TIMEOUT_MS = 3000;
inline constexpr size_t CONTROL_QUEUE_SIZE = 8;
}
