#include "zmq_camera_adapter.h"
#include <bullet_msgs/shm.hpp>
#include <bullet_msgs/control.hpp>
#include <bullet_msgs/spawn.hpp>
#include <bullet_msgs/wind.hpp>
#include <deque>
#include <godot_cpp/classes/json.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <zmq.h>
#include <godot_cpp/classes/file_access.hpp>
#include <future>
#include <thread>
#include <stdexcept>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <set>
#include <string>

using namespace godot;
namespace {
using Clock = std::chrono::steady_clock;
constexpr size_t MAX_MESSAGE_BYTES = 16384;
constexpr int64_t MAX_WIRE_BYTES = 65536;
constexpr int64_t MAX_IMAGE_BYTES = 64 * 1024 * 1024;
constexpr int64_t MAX_SEQUENCE = 9007199254740991;
constexpr int TELEMETRY_HWM = 64;
constexpr int IMAGE_HWM = 8;
constexpr double STALE_SECONDS = 1.0;

bool number(const Variant &v, double low, double high, bool integer = false) {
    if (v.get_type() != Variant::INT && v.get_type() != Variant::FLOAT) return false;
    const double n = v;
    return std::isfinite(n) && n >= low && n <= high && (!integer || std::trunc(n) == n);
}
bool text(const Variant &v) {
    if (v.get_type() != Variant::STRING) return false;
    const String s = v;
    return s.length() > 0 && s.length() <= 128;
}
bool control_identity(const Variant &value) {
    if (!text(value)) return false;
    const String name = value;
    for (int64_t i = 0; i < name.length(); ++i) {
        const auto c = name[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_')) return false;
    }
    return true;
}
bool shape(const Dictionary &d, std::initializer_list<const char *> keys) {
    if (d.size() != static_cast<int64_t>(keys.size())) return false;
    for (const auto key : keys) if (!d.has(key)) return false;
    return true;
}
bool vector(const Variant &v, int count, double bound) {
    if (v.get_type() != Variant::ARRAY) return false;
    const Array array = v;
    if (array.size() != count) return false;
    for (int i = 0; i < count; ++i) if (!number(array[i], -bound, bound)) return false;
    return true;
}
bool spawn_name(const Variant &v) {
    if (v.get_type() != Variant::STRING) return false;
    const String name = v;
    if (name.is_empty() || name.length() > 64 || name.strip_edges() != name) return false;
    for (int64_t i = 0; i < name.length(); ++i) if (name[i] < 32) return false;
    return true;
}
bool spawn_position(const Variant &v) {
    if (!vector(v, 3, 500)) return false;
    const Array p = v;
    return number(p[0], -bullet_msgs::SPAWN_HORIZONTAL_LIMIT_M, bullet_msgs::SPAWN_HORIZONTAL_LIMIT_M) &&
        number(p[1], -bullet_msgs::SPAWN_HORIZONTAL_LIMIT_M, bullet_msgs::SPAWN_HORIZONTAL_LIMIT_M) &&
        number(p[2], bullet_msgs::SPAWN_MIN_ALTITUDE_M, bullet_msgs::SPAWN_MAX_ALTITUDE_M);
}
bool valid_takeoff_height(const Variant &v) {
    return number(v, bullet_msgs::MIN_TAKEOFF_HEIGHT_M, bullet_msgs::MAX_TAKEOFF_HEIGHT_M);
}
bool valid_spawn_state(const Dictionary &d) {
    Dictionary legacy = d.duplicate();
    const bool v2 = number(d.get("schema_version", 0), 2, 2, true);
    if (v2) {
        if (!d.has("takeoff_height_m") || !valid_takeoff_height(d["takeoff_height_m"])) return false;
        legacy.erase("takeoff_height_m");
    }
    if (!shape(legacy, {"schema_version", "physics_id", "controller_id", "episode_id", "scenario_name", "position_m", "request_id", "result", "reason"}) ||
        !number(d["schema_version"], v2 ? 2 : 1, v2 ? 2 : 1, true) || !control_identity(d["physics_id"]) || !control_identity(d["episode_id"]) ||
        (d["controller_id"].get_type() != Variant::NIL && !control_identity(d["controller_id"])) ||
        (d["request_id"].get_type() != Variant::NIL && !control_identity(d["request_id"])) ||
        !spawn_name(d["scenario_name"]) || !spawn_position(d["position_m"]) || d["result"].get_type() != Variant::STRING ||
        d["reason"].get_type() != Variant::STRING) return false;
    const String result = d["result"], reason = d["reason"];
    return (result == "idle" || result == "applied" || result == "rejected") && reason.length() > 0 && reason.length() <= 256 &&
        ((d["request_id"].get_type() == Variant::NIL) == (result == "idle"));
}
bool valid_wind_settings(const Dictionary &d) {
    if (!shape(d, {"enabled", "behavior", "velocity_world_m_s", "gust_stddev_m_s", "correlation_time_s", "seed"}) ||
        d["enabled"].get_type() != Variant::BOOL || d["behavior"].get_type() != Variant::STRING ||
        !vector(d["velocity_world_m_s"], 3, 1e12) || !vector(d["gust_stddev_m_s"], 3, 1e12) ||
        !number(d["correlation_time_s"], 0, 1e12) || double(d["correlation_time_s"]) <= 0 ||
        !number(d["seed"], -MAX_SEQUENCE, MAX_SEQUENCE, true)) return false;
    const String behavior = d["behavior"];
    const Array deviations = d["gust_stddev_m_s"];
    for (int i = 0; i < 3; ++i) if (double(deviations[i]) < 0) return false;
    return behavior == "constant" || behavior == "gusts";
}
bool valid_wind_state(const Dictionary &d) {
    if (!shape(d, {"schema_version", "physics_id", "episode_id", "controller_id", "request_id", "settings", "velocity_world_m_s", "simulation_time_s", "result", "reason"}) ||
        !number(d["schema_version"], 1, 1, true) || !control_identity(d["physics_id"]) || !control_identity(d["episode_id"]) ||
        (d["controller_id"].get_type() != Variant::NIL && !control_identity(d["controller_id"])) ||
        (d["request_id"].get_type() != Variant::NIL && !control_identity(d["request_id"])) ||
        d["settings"].get_type() != Variant::DICTIONARY || !valid_wind_settings(d["settings"]) ||
        !vector(d["velocity_world_m_s"], 3, 1e12) || !number(d["simulation_time_s"], 0, 1e12) ||
        d["result"].get_type() != Variant::STRING || d["reason"].get_type() != Variant::STRING) return false;
    const String result = d["result"], reason = d["reason"];
    return (result == "idle" || result == "applied" || result == "rejected") && reason.length() > 0 && reason.length() <= 256 &&
        ((d["request_id"].get_type() == Variant::NIL) == (result == "idle"));
}
bool valid_config(const Dictionary &d) {
    if (!shape(d, {"schema_version", "stream_id", "camera", "fps"}) ||
        !number(d["schema_version"], 1, 1, true) || !text(d["stream_id"]) ||
        !number(d["fps"], 0, 240) || double(d["fps"]) <= 0 ||
        d["camera"].get_type() != Variant::DICTIONARY) return false;
    const Dictionary camera = d["camera"];
    return shape(camera, {"width_px", "height_px", "fov_x_deg"}) &&
        number(camera["width_px"], 1, 8192, true) && number(camera["height_px"], 1, 8192, true) &&
        number(camera["fov_x_deg"], 0, 180) && double(camera["fov_x_deg"]) > 0 &&
        double(camera["fov_x_deg"]) < 180 &&
        int64_t(camera["width_px"]) * int64_t(camera["height_px"]) * 3 <= MAX_IMAGE_BYTES;
}
bool valid_stamp(const Dictionary &s) {
    if (!shape(s, {"physics_id", "episode_id", "stream_id", "step_index", "simulation_time_s", "measurement_us"})) return false;
    for (const auto key : {"physics_id", "episode_id", "stream_id"}) if (!control_identity(s[key])) return false;
    for (const auto key : {"step_index", "measurement_us"}) if (!number(s[key], 0, MAX_SEQUENCE, true)) return false;
    return number(s["simulation_time_s"], 0, 1e12);
}
bool valid_source(const Dictionary &s, const Dictionary &pose) {
    if (!valid_stamp(s)) return false;
    for (const auto key : {"episode_id", "stream_id", "step_index", "simulation_time_s"})
        if (!pose.has(key) || s[key] != pose[key]) return false;
    return true;
}
Dictionary integer_stamp(const Dictionary &s) {
    Dictionary result = s.duplicate();
    for (const auto key : {"step_index", "measurement_us"}) result[key] = int64_t(s[key]);
    return result;
}
bool valid_collision(const Dictionary &d) {
    if (!shape(d, {"schema_version", "event_id", "controller_id", "issued_us", "source", "target_id", "drone_position_m"}) ||
        !number(d["schema_version"], 1, 1, true) || !control_identity(d["event_id"]) || !control_identity(d["controller_id"]) ||
        !number(d["issued_us"], 0, MAX_SEQUENCE, true) || d["source"].get_type() != Variant::DICTIONARY ||
        !valid_stamp(d["source"]) || d["target_id"].get_type() != Variant::STRING || String(d["target_id"]) != "red_truck" ||
        !vector(d["drone_position_m"], 3, 1e12)) return false;
    const Dictionary source = d["source"];
    return int64_t(d["issued_us"]) >= int64_t(source["measurement_us"]);
}
bool valid_end(const Dictionary &d) {
    if (!shape(d, {"schema_version", "event_id", "collision", "final_source", "outcome", "reason", "ended_us"}) ||
        !number(d["schema_version"], 1, 1, true) || !control_identity(d["event_id"]) ||
        d["collision"].get_type() != Variant::DICTIONARY || !valid_collision(d["collision"]) ||
        d["final_source"].get_type() != Variant::DICTIONARY || !valid_stamp(d["final_source"]) ||
        d["outcome"].get_type() != Variant::STRING || String(d["outcome"]) != "success" ||
        d["reason"].get_type() != Variant::STRING || String(d["reason"]) != "target_hit" ||
        !number(d["ended_us"], 0, MAX_SEQUENCE, true)) return false;
    const Dictionary collision = d["collision"], source = collision["source"], final_source = d["final_source"];
    for (const auto key : {"physics_id", "episode_id", "stream_id"}) if (source[key] != final_source[key]) return false;
    return d["event_id"] == collision["event_id"] &&
        double(final_source["simulation_time_s"]) >= double(source["simulation_time_s"]) &&
        int64_t(final_source["step_index"]) >= int64_t(source["step_index"]) &&
        int64_t(final_source["measurement_us"]) >= int64_t(source["measurement_us"]) &&
        int64_t(d["ended_us"]) >= int64_t(final_source["measurement_us"]) &&
        int64_t(d["ended_us"]) >= int64_t(collision["issued_us"]);
}
bool valid_pose(const Dictionary &d) {
    Dictionary legacy = d.duplicate();
    const bool stamped = number(d.get("schema_version", 0), 3, 3, true);
    if (stamped) {
        if (d.get("source", Variant()).get_type() != Variant::DICTIONARY || !valid_source(d["source"], d)) return false;
        legacy.erase("source");
    }
    if (!shape(legacy, {"schema_version", "stream_id", "drone_id", "episode_id", "sequence", "publisher_drops",
                  "step_index", "simulation_time_s", "position_m", "orientation_xyzw", "linear_velocity_m_s"}) ||
        !number(d["schema_version"], stamped ? 3 : 2, stamped ? 3 : 2, true) || !text(d["stream_id"]) ||
        d["drone_id"].get_type() != Variant::STRING || String(d["drone_id"]) != "drone_1" ||
        !text(d["episode_id"]) || !number(d["simulation_time_s"], 0, 1e12) ||
        !vector(d["position_m"], 3, 1e9) || !vector(d["orientation_xyzw"], 4, 1) ||
        !vector(d["linear_velocity_m_s"], 3, 1e9)) return false;
    for (const auto key : {"sequence", "publisher_drops", "step_index"}) {
        if (!number(d[key], 0, MAX_SEQUENCE, true)) return false;
    }
    const Array q = d["orientation_xyzw"];
    double norm = 0;
    for (int i = 0; i < 4; ++i) norm += double(q[i]) * double(q[i]);
    return norm >= 1e-24;
}
Dictionary parse(const std::string &raw) {
    Ref<JSON> json;
    json.instantiate();
    if (json->parse(String::utf8(raw.data(), static_cast<int64_t>(raw.size()))) != OK ||
        json->get_data().get_type() != Variant::DICTIONARY) return {};
    return json->get_data();
}
}

struct ZmqCameraAdapter::Impl {
    std::mutex mutex;
    std::string config_slot, pose_slot, collision_pose_slot, state_slot, collision_outgoing, spawn_slot, spawn_outgoing, wind_slot, wind_outgoing;
    Clock::time_point state_received{}, valid_state_received{}, spawn_received{}, valid_spawn_received{}, wind_received{}, valid_wind_received{};
    std::deque<std::pair<std::string, int64_t>> commands;
    std::string controller_id = std::to_string(getpid()) + "-" + std::to_string(Clock::now().time_since_epoch().count());
    Dictionary control_state, spawn_state, wind_state;
    Clock::time_point next_health{};
    String control_error;
    Clock::time_point pose_received{}, capture_received{};
    std::atomic<int64_t> invalid{0}, superseded{0};
    int64_t frames = 0, sequence_gaps = 0, stale = 0, last_sequence = -1, capture_sequence = -1;
    int64_t collision_sequence = -1;
    String collision_episode, collision_stream;
    int64_t frame_bytes = 0, busy_slots = 0, publication_drops = 0;
    double source_age_ms = -1.0;
    bullet_msgs::FrameWriter images;
    bool running = false;
    String error, capture_stream;
    Dictionary config, camera;
    std::set<String> retired;
    void *context = nullptr, *publisher = nullptr;
    std::thread receiver;
    std::atomic<bool> stop{false};
    std::string worker_error;

    void receive_loop(std::string endpoint, std::string control_endpoint, std::promise<void> ready) {
        void *socket = nullptr, *control = nullptr;
        bool started = false;
        try {
            socket = zmq_socket(context, ZMQ_SUB);
            if (!socket) throw std::runtime_error(zmq_strerror(zmq_errno()));
            int linger = 0;
            int64_t max_size = MAX_WIRE_BYTES;
            if (zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger)) ||
                zmq_setsockopt(socket, ZMQ_RCVHWM, &TELEMETRY_HWM, sizeof(TELEMETRY_HWM)) ||
                zmq_setsockopt(socket, ZMQ_MAXMSGSIZE, &max_size, sizeof(max_size)) ||
                zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "render/drone_1/", 15) ||
                zmq_setsockopt(socket, ZMQ_SUBSCRIBE, bullet_msgs::WIND_STATE_TOPIC, sizeof(bullet_msgs::WIND_STATE_TOPIC) - 1) ||
                zmq_setsockopt(socket, ZMQ_SUBSCRIBE, bullet_msgs::SPAWN_STATE_TOPIC, sizeof(bullet_msgs::SPAWN_STATE_TOPIC) - 1) ||
                zmq_setsockopt(socket, ZMQ_SUBSCRIBE, bullet_msgs::STATE_TOPIC, sizeof(bullet_msgs::STATE_TOPIC) - 1) ||
                zmq_connect(socket, endpoint.c_str())) throw std::runtime_error(zmq_strerror(zmq_errno()));
            control = zmq_socket(context, ZMQ_PUB);
            const int hwm = bullet_msgs::CONTROL_QUEUE_SIZE;
            if (!control || zmq_setsockopt(control, ZMQ_LINGER, &linger, sizeof(linger)) ||
                zmq_setsockopt(control, ZMQ_SNDHWM, &hwm, sizeof(hwm)) ||
                zmq_bind(control, control_endpoint.c_str())) throw std::runtime_error(zmq_strerror(zmq_errno()));
            uint64_t control_sequence = 0;
            auto next_heartbeat = Clock::now();
            auto publish = [&](const std::string &action, int64_t issued_ms) {
                const auto payload = "{\"schema_version\":1,\"controller_id\":\"" + controller_id +
                    "\",\"sequence\":" + std::to_string(++control_sequence) + ",\"action\":\"" + action +
                    "\",\"issued_ms\":" + std::to_string(issued_ms) + "}";
                const char *topic = action == "heartbeat" ? bullet_msgs::HEARTBEAT_TOPIC : bullet_msgs::COMMAND_TOPIC;
                if (zmq_send(control, topic, std::strlen(topic), ZMQ_SNDMORE | ZMQ_DONTWAIT) < 0 ||
                    zmq_send(control, payload.data(), payload.size(), ZMQ_DONTWAIT) < 0)
                    throw std::runtime_error(zmq_strerror(zmq_errno()));
            };
            ready.set_value();
            started = true;
            while (!stop) {
                const auto now = Clock::now();
                if (now >= next_heartbeat) {
                    publish("heartbeat", std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
                    next_heartbeat = now + std::chrono::milliseconds(bullet_msgs::HEARTBEAT_INTERVAL_MS);
                }
                std::deque<std::pair<std::string, int64_t>> outgoing;
                { std::lock_guard lock(mutex); outgoing.swap(commands); }
                for (const auto &[action, issued_ms] : outgoing) publish(action, issued_ms);
                std::string collision;
                { std::lock_guard lock(mutex); collision.swap(collision_outgoing); }
                if (!collision.empty()) {
                    constexpr char topic[] = "render/drone_1/collision";
                    if (zmq_send(control, topic, sizeof(topic) - 1, ZMQ_SNDMORE | ZMQ_DONTWAIT) < 0 ||
                        zmq_send(control, collision.data(), collision.size(), ZMQ_DONTWAIT) < 0)
                        throw std::runtime_error(zmq_strerror(zmq_errno()));
                }
                std::string spawn;
                { std::lock_guard lock(mutex); spawn.swap(spawn_outgoing); }
                if (!spawn.empty()) {
                    const auto &topic = bullet_msgs::SPAWN_REQUEST_TOPIC;
                    if (zmq_send(control, topic, sizeof(topic) - 1, ZMQ_SNDMORE | ZMQ_DONTWAIT) < 0 ||
                        zmq_send(control, spawn.data(), spawn.size(), ZMQ_DONTWAIT) < 0)
                        throw std::runtime_error(zmq_strerror(zmq_errno()));
                }
                std::string wind;
                { std::lock_guard lock(mutex); wind.swap(wind_outgoing); }
                if (!wind.empty()) {
                    const auto &topic = bullet_msgs::WIND_REQUEST_TOPIC;
                    if (zmq_send(control, topic, sizeof(topic) - 1, ZMQ_SNDMORE | ZMQ_DONTWAIT) < 0 ||
                        zmq_send(control, wind.data(), wind.size(), ZMQ_DONTWAIT) < 0)
                        throw std::runtime_error(zmq_strerror(zmq_errno()));
                }
                zmq_pollitem_t item{socket, 0, ZMQ_POLLIN, 0};
                int result = zmq_poll(&item, 1, 10);
                if (result < 0) {
                    if (zmq_errno() == EINTR) continue;
                    throw std::runtime_error(zmq_strerror(zmq_errno()));
                }
                if (!(item.revents & ZMQ_POLLIN)) continue;
                std::string topic, raw;
                int parts = 0;
                bool malformed = false, more;
                do {
                    zmq_msg_t message;
                    zmq_msg_init(&message);
                    if (zmq_msg_recv(&message, socket, 0) < 0) {
                        zmq_msg_close(&message);
                        throw std::runtime_error(zmq_strerror(zmq_errno()));
                    }
                    const size_t size = zmq_msg_size(&message);
                    more = zmq_msg_more(&message);
                    if (size == 0 || size > MAX_MESSAGE_BYTES || parts >= 2) malformed = true;
                    else if (parts == 0) topic.assign(static_cast<char *>(zmq_msg_data(&message)), size);
                    else raw.assign(static_cast<char *>(zmq_msg_data(&message)), size);
                    ++parts;
                    zmq_msg_close(&message);
                } while (more);
                if (malformed || parts != 2) { ++invalid; continue; }
                const bool is_config = topic == "render/drone_1/config";
                const bool is_state = topic == bullet_msgs::STATE_TOPIC;
                const bool is_spawn = topic == bullet_msgs::SPAWN_STATE_TOPIC;
                const bool is_wind = topic == bullet_msgs::WIND_STATE_TOPIC;
                if (!is_wind && !is_spawn && !is_config && !is_state && topic != "render/drone_1/pose") continue;
                std::lock_guard lock(mutex);
                if (is_wind) { wind_slot = std::move(raw); wind_received = Clock::now(); continue; }
                if (is_spawn) { spawn_slot = std::move(raw); spawn_received = Clock::now(); continue; }
                if (is_state) { state_slot = std::move(raw); state_received = Clock::now(); continue; }
                if (is_config) config_slot = std::move(raw);
                else {
                    if (!pose_slot.empty()) ++superseded;
                    collision_pose_slot = raw;
                    pose_slot = std::move(raw);
                    pose_received = Clock::now();
                }
            }
        } catch (const std::exception &e) {
            if (!started) ready.set_exception(std::current_exception());
            else { std::lock_guard lock(mutex); worker_error = e.what(); }
        }
        if (control) zmq_close(control);
        if (socket) zmq_close(socket);
    }

};

ZmqCameraAdapter::ZmqCameraAdapter() : impl(std::make_unique<Impl>()) {}
ZmqCameraAdapter::~ZmqCameraAdapter() { close(); }

void ZmqCameraAdapter::_bind_methods() {
    ClassDB::bind_method(D_METHOD("start", "transport_path"), &ZmqCameraAdapter::start);
    ClassDB::bind_method(D_METHOD("poll_request"), &ZmqCameraAdapter::poll_request);
    ClassDB::bind_method(D_METHOD("publish_frame", "pixels", "request"), &ZmqCameraAdapter::publish_frame);
    ClassDB::bind_method(D_METHOD("get_stats"), &ZmqCameraAdapter::get_stats);
    ClassDB::bind_method(D_METHOD("publish_health"), &ZmqCameraAdapter::publish_health);
    ClassDB::bind_method(D_METHOD("publish_control", "action"), &ZmqCameraAdapter::publish_control);
    ClassDB::bind_method(D_METHOD("poll_collision_pose"), &ZmqCameraAdapter::poll_collision_pose);
    ClassDB::bind_method(D_METHOD("publish_collision", "report"), &ZmqCameraAdapter::publish_collision);
    ClassDB::bind_method(D_METHOD("get_control_state"), &ZmqCameraAdapter::get_control_state);
    ClassDB::bind_method(D_METHOD("publish_wind", "request"), &ZmqCameraAdapter::publish_wind);
    ClassDB::bind_method(D_METHOD("get_wind_state"), &ZmqCameraAdapter::get_wind_state);
    ClassDB::bind_method(D_METHOD("publish_spawn", "request"), &ZmqCameraAdapter::publish_spawn);
    ClassDB::bind_method(D_METHOD("get_spawn_state"), &ZmqCameraAdapter::get_spawn_state);
    ClassDB::bind_method(D_METHOD("close"), &ZmqCameraAdapter::close);
}

bool ZmqCameraAdapter::start(const String &transport_path) {
    if (impl->running) return false;
    close();
    impl = std::make_unique<Impl>();
    try {
        const auto file = FileAccess::open(transport_path, FileAccess::READ);
        if (file.is_null()) throw std::runtime_error("cannot read transport profile");
        const Dictionary profile = parse(file->get_as_text().utf8().get_data());
        Dictionary legacy_profile = profile.duplicate();
        legacy_profile.erase("flight_command_endpoint");
        if (!shape(legacy_profile, {"telemetry_endpoint", "image_endpoint", "control_endpoint"})) throw std::runtime_error("invalid transport profile");
        for (const auto key : {"telemetry_endpoint", "image_endpoint", "control_endpoint"}) {
            const auto endpoint = String(profile[key]).utf8();
            if (profile[key].get_type() != Variant::STRING ||
                !bullet_msgs::valid_endpoint(std::string_view(endpoint.get_data(), endpoint.length())))
                throw std::runtime_error("expected loopback TCP endpoint tcp://127.0.0.1:<port 1-65535>");
        }
        if (profile["telemetry_endpoint"] == profile["image_endpoint"] ||
            profile["control_endpoint"] == profile["image_endpoint"] || profile["control_endpoint"] == profile["telemetry_endpoint"]) throw std::runtime_error("endpoints must differ");
        if (profile.has("flight_command_endpoint")) {
            const auto endpoint = String(profile["flight_command_endpoint"]).utf8();
            if (profile["flight_command_endpoint"].get_type() != Variant::STRING ||
                !bullet_msgs::valid_endpoint(std::string_view(endpoint.get_data(), endpoint.length())))
                throw std::runtime_error("invalid flight command endpoint");
            for (const auto key : {"telemetry_endpoint", "image_endpoint", "control_endpoint"})
                if (profile[key] == profile["flight_command_endpoint"]) throw std::runtime_error("endpoints must differ");
        }
        impl->context = zmq_ctx_new();
        if (!impl->context) throw std::runtime_error(zmq_strerror(zmq_errno()));
        impl->publisher = zmq_socket(impl->context, ZMQ_PUB);
        int linger = 0;
        if (!impl->publisher ||
            zmq_setsockopt(impl->publisher, ZMQ_LINGER, &linger, sizeof(linger)) ||
            zmq_setsockopt(impl->publisher, ZMQ_SNDHWM, &IMAGE_HWM, sizeof(IMAGE_HWM)) ||
            zmq_bind(impl->publisher, String(profile["image_endpoint"]).utf8().get_data()))
            throw std::runtime_error(zmq_strerror(zmq_errno()));
        std::promise<void> ready;
        auto future = ready.get_future();
        impl->receiver = std::thread(&Impl::receive_loop, impl.get(),
            String(profile["telemetry_endpoint"]).utf8().get_data(),
            String(profile["control_endpoint"]).utf8().get_data(), std::move(ready));
        future.get();
        impl->running = true;
        return true;
    } catch (const std::exception &e) {
        impl->error = String("ZeroMQ adapter startup failed: ") + e.what();
        UtilityFunctions::push_error(impl->error);
        close();
        return false;
    }
}

Dictionary ZmqCameraAdapter::poll_request() {
    if (!impl->running || !impl->error.is_empty()) return {};
    std::string raw_config, raw_pose;
    Clock::time_point received;
    {
        std::lock_guard lock(impl->mutex);
        if (!impl->worker_error.empty()) { impl->error = String::utf8(impl->worker_error.c_str()); return {}; }
        raw_config.swap(impl->config_slot);
        raw_pose.swap(impl->pose_slot);
        received = impl->pose_received;
    }
    if (!raw_config.empty()) {
        const Dictionary config = parse(raw_config);
        if (!valid_config(config)) {
            ++impl->invalid;
            UtilityFunctions::push_warning("Rejected invalid ZeroMQ camera config");
        } else {
            const String stream = config["stream_id"];
            if (!impl->retired.contains(stream)) {
                const bool same = !impl->config.is_empty() && String(impl->config["stream_id"]) == stream;
                if (same && impl->config != config) {
                    ++impl->invalid;
                    UtilityFunctions::push_warning("Rejected camera configuration change within a run");
                } else if (!same) {
                    const Dictionary camera = config["camera"];
                    if (!impl->camera.is_empty() && impl->camera != camera) {
                        impl->error = "Camera settings changed: restart Godot and reconfigure recorder";
                        UtilityFunctions::push_error(impl->error);
                        return {};
                    }
                    if (!impl->config.is_empty()) impl->retired.insert(impl->config["stream_id"]);
                    impl->config = config;
                    impl->camera = camera;
                    impl->last_sequence = -1;
                    impl->frame_bytes = int64_t(camera["width_px"]) * int64_t(camera["height_px"]) * 3;
                    if (impl->images.session.empty()) {
                        try {
                            impl->images.open(int64_t(camera["width_px"]), int64_t(camera["height_px"]));
                            UtilityFunctions::print("SHM publisher session: ", impl->images.session.c_str());
                        } catch (const std::exception &e) {
                            impl->error = String("SHM allocation failed: ") + e.what();
                            return {};
                        }
                    }
                }
            }
        }
    }
    if (raw_pose.empty()) return {};
    const Dictionary pose = parse(raw_pose);
    if (!valid_pose(pose)) {
        ++impl->invalid;
        UtilityFunctions::push_warning("Rejected invalid ZeroMQ capture pose");
        return {};
    }
    if (impl->config.is_empty() || String(pose["stream_id"]) != String(impl->config["stream_id"])) return {};
    const int64_t sequence = pose["sequence"];
    if (sequence <= impl->last_sequence) return {};
    if (impl->last_sequence >= 0) impl->sequence_gaps += sequence - impl->last_sequence - 1;
    impl->last_sequence = sequence;
    if (std::chrono::duration<double>(Clock::now() - received).count() > STALE_SECONDS) {
        ++impl->stale;
        return {};
    }
    impl->capture_received = received;
    impl->capture_sequence = sequence;
    impl->capture_stream = pose["stream_id"];
    Dictionary request;
    request["config"] = impl->config;
    request["pose"] = pose;
    return request;
}

bool ZmqCameraAdapter::publish_frame(const PackedByteArray &pixels, const Dictionary &request) {
    try {
        if (!impl->running || !impl->frame_bytes || !impl->error.is_empty()) {
            throw std::runtime_error("adapter is not ready to publish");
        }
        const Dictionary pose = request.get("pose", Dictionary());
        if (int64_t(pose.get("sequence", -1)) != impl->capture_sequence ||
            String(pose.get("stream_id", "")) != impl->capture_stream || impl->capture_sequence < 0) {
            throw std::runtime_error("frame does not match the requested pose");
        }
        impl->capture_sequence = -1;
        if (!impl->control_state.is_empty() && String(pose.get("episode_id", "")) != String(impl->control_state["episode_id"])) {
            ++impl->stale;
            return true;
        }
        if (pixels.size() != impl->frame_bytes) throw std::runtime_error("RGB frame size mismatch");
        if (std::chrono::duration<double>(Clock::now() - impl->capture_received).count() > STALE_SECONDS) {
            ++impl->stale;
            return true;
        }
        auto capture_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            impl->capture_received.time_since_epoch()).count();
        std::string source_json;
        if (number(pose.get("schema_version", 0), 3, 3, true)) {
            if (!valid_pose(pose)) throw std::runtime_error("invalid stamped capture pose");
            Dictionary source = Dictionary(pose["source"]).duplicate();
            source["measurement_us"] = int64_t(source["measurement_us"]);
            source["step_index"] = int64_t(source["step_index"]);
            capture_ns = int64_t(source["measurement_us"]) * 1000;
            source_json = JSON::stringify(source, "", true, true).utf8().get_data();
        }
        impl->source_age_ms = std::chrono::duration<double, std::milli>(Clock::now().time_since_epoch()).count()
            - static_cast<double>(capture_ns) / 1e6;
        const auto message = impl->images.write(pixels.ptr(), pixels.size(), capture_ns, source_json);
        if (message.empty()) { ++impl->busy_slots; return true; }
        if (zmq_send(impl->publisher, message.data(), message.size(), ZMQ_DONTWAIT) < 0) {
            if (zmq_errno() == EAGAIN) { ++impl->publication_drops; return true; }
            throw std::runtime_error(zmq_strerror(zmq_errno()));
        }
        ++impl->frames;
        return true;
    } catch (const std::exception &e) {
        impl->error = String("ZeroMQ image pipeline failed: ") + e.what();
        UtilityFunctions::push_error(impl->error);
        return false;
    }
}

Dictionary ZmqCameraAdapter::get_stats() const {
    Dictionary stats;
    stats["frames"] = impl->frames;
    stats["source_age_ms"] = impl->source_age_ms;
    stats["controller_id"] = String(impl->controller_id.c_str());
    stats["control_error"] = impl->control_error;
    stats["shm_session"] = String(impl->images.session.c_str());
    stats["busy_slots"] = impl->busy_slots;
    stats["publication_drops"] = impl->publication_drops;
    stats["invalid"] = impl->invalid.load();
    stats["superseded"] = impl->superseded.load();
    stats["sequence_gaps"] = impl->sequence_gaps;
    stats["stale"] = impl->stale;
    stats["error"] = impl->error;
    stats["running"] = impl->running;
    return stats;
}

void ZmqCameraAdapter::close() {
    impl->stop = true;
    if (impl->receiver.joinable()) impl->receiver.join();
    if (impl->publisher) { zmq_close(impl->publisher); impl->publisher = nullptr; }
    if (impl->context) { zmq_ctx_term(impl->context); impl->context = nullptr; }
    impl->images.close();
    impl->running = false;
}


bool ZmqCameraAdapter::publish_control(const String &action) {
    if (!impl->running || (action != "start" && action != "stop" && action != "reset")) return false;
    std::lock_guard lock(impl->mutex);
    if (impl->commands.size() >= bullet_msgs::CONTROL_QUEUE_SIZE) return false;
    impl->commands.emplace_back(action.utf8().get_data(),
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count());
    return true;
}

Dictionary ZmqCameraAdapter::poll_collision_pose() {
    const Dictionary state = get_control_state();
    std::string raw;
    { std::lock_guard lock(impl->mutex); raw.swap(impl->collision_pose_slot); }
    if (raw.empty() || state.is_empty() || String(state["state"]) != "running") return {};
    const Dictionary pose = parse(raw);
    if (!valid_pose(pose) || !number(pose["schema_version"], 3, 3, true)) return {};
    const Dictionary source = pose["source"];
    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();
    const auto age_us = now_us - int64_t(source["measurement_us"]);
    if (source["physics_id"] != state["physics_id"] || source["episode_id"] != state["episode_id"] ||
        age_us < 0 || age_us > 1000000) return {};
    const String episode = source["episode_id"], stream = source["stream_id"];
    if (episode != impl->collision_episode || stream != impl->collision_stream) {
        impl->collision_sequence = -1;
        impl->collision_episode = episode;
        impl->collision_stream = stream;
    }
    if (int64_t(pose["sequence"]) <= impl->collision_sequence) return {};
    impl->collision_sequence = int64_t(pose["sequence"]);
    return pose;
}

bool ZmqCameraAdapter::publish_collision(const Dictionary &report) {
    if (!impl->running) return false;
    Dictionary packet = report.duplicate(true);
    packet["schema_version"] = int64_t(1);
    packet["controller_id"] = String(impl->controller_id.c_str());
    packet["issued_us"] = int64_t(std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count());
    if (packet.get("source", Variant()).get_type() != Variant::DICTIONARY) return false;
    const Dictionary source = packet["source"];
    if (!valid_stamp(source)) return false;
    packet["source"] = integer_stamp(source);
    if (!valid_collision(packet)) return false;
    const auto raw = JSON::stringify(packet, "", true, true).utf8();
    if (raw.length() > 4096) return false;
    std::lock_guard lock(impl->mutex);
    impl->collision_outgoing.assign(raw.get_data(), raw.length());
    return true;
}

Dictionary ZmqCameraAdapter::get_control_state() {
    std::string raw;
    Clock::time_point received;
    { std::lock_guard lock(impl->mutex); raw.swap(impl->state_slot); received = impl->state_received; }
    if (!raw.empty()) {
        const auto state = parse(raw);
        Dictionary legacy = state.duplicate();
        const bool v2 = number(state.get("schema_version", 0), 2, 2, true);
        if (v2) legacy.erase("terminal_result");
        bool valid = raw.size() <= 4096 && (!v2 || state.has("terminal_result")) && shape(legacy, {"schema_version", "physics_id", "controller_id", "episode_id", "state", "reason", "simulation_time_s"});
        if (valid) {
            const String mode = state["state"];
            const String reason = state["reason"];
            valid = number(state["schema_version"], v2 ? 2 : 1, v2 ? 2 : 1, true) && control_identity(state["physics_id"]) && control_identity(state["episode_id"]) &&
                (state["controller_id"].get_type() == Variant::NIL || control_identity(state["controller_id"])) &&
                state["state"].get_type() == Variant::STRING && (mode == "running" || mode == "stopped" || mode == "error") &&
                state["reason"].get_type() == Variant::STRING && reason.length() > 0 && reason.length() <= 256 &&
                number(state["simulation_time_s"], 0, 1e12);
        }
        if (valid && v2 && state["terminal_result"].get_type() != Variant::NIL) {
            valid = state["terminal_result"].get_type() == Variant::DICTIONARY;
            if (valid) {
                const Dictionary end = state["terminal_result"], final_source = end.get("final_source", Dictionary());
                valid = valid_end(end) && String(state["state"]) == "stopped" && state["reason"] == end["reason"];
                for (const auto key : {"physics_id", "episode_id", "simulation_time_s"})
                    if (!final_source.has(key) || state[key] != final_source[key]) valid = false;
            }
        }
        if (valid) { impl->control_state = state; impl->valid_state_received = received; impl->control_error = ""; }
        else { ++impl->invalid; impl->control_error = "Rejected invalid simulation state"; }
    }
    if (!impl->running || std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - impl->valid_state_received).count() > bullet_msgs::HEARTBEAT_TIMEOUT_MS) return {};
    return impl->control_state;
}


bool ZmqCameraAdapter::publish_spawn(const Dictionary &request) {
    Dictionary legacy = request.duplicate();
    legacy.erase("takeoff_height_m");
    const Variant height = request.get("takeoff_height_m", bullet_msgs::DEFAULT_TAKEOFF_HEIGHT_M);
    if (!impl->running || !valid_takeoff_height(height) || !shape(legacy, {"physics_id", "episode_id", "request_id", "scenario_name", "position_m"}) ||
        !control_identity(request["physics_id"]) || !control_identity(request["episode_id"]) || !control_identity(request["request_id"]) ||
        !spawn_name(request["scenario_name"]) || !spawn_position(request["position_m"])) return false;
    Dictionary packet = request.duplicate(true);
    packet["schema_version"] = int64_t(2);
    packet["takeoff_height_m"] = height;
    packet["controller_id"] = String(impl->controller_id.c_str());
    packet["issued_ms"] = int64_t(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count());
    const auto raw = JSON::stringify(packet, "", true, true).utf8();
    if (raw.length() > 4096) return false;
    std::lock_guard lock(impl->mutex);
    impl->spawn_outgoing.assign(raw.get_data(), raw.length());
    return true;
}

Dictionary ZmqCameraAdapter::get_spawn_state() {
    const Dictionary control = get_control_state();
    std::string raw;
    Clock::time_point received;
    { std::lock_guard lock(impl->mutex); raw.swap(impl->spawn_slot); received = impl->spawn_received; }
    if (!raw.empty()) {
        const auto state = parse(raw);
        if (raw.size() <= 4096 && valid_spawn_state(state)) {
            impl->spawn_state = state;
            impl->valid_spawn_received = received;
        } else ++impl->invalid;
    }
    if (!impl->running || control.is_empty() || impl->spawn_state.is_empty() ||
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - impl->valid_spawn_received).count() > bullet_msgs::SPAWN_STATE_TIMEOUT_MS) return {};
    for (const auto key : {"physics_id", "episode_id", "controller_id"})
        if (impl->spawn_state[key] != control[key]) return {};
    return impl->spawn_state;
}


bool ZmqCameraAdapter::publish_wind(const Dictionary &request) {
    if (!impl->running || !shape(request, {"physics_id", "episode_id", "request_id", "settings"}) ||
        !control_identity(request["physics_id"]) || !control_identity(request["episode_id"]) || !control_identity(request["request_id"]) ||
        request["settings"].get_type() != Variant::DICTIONARY || !valid_wind_settings(request["settings"])) return false;
    Dictionary packet = request.duplicate(true);
    Dictionary settings = packet["settings"];
    settings["seed"] = int64_t(settings["seed"]);
    packet["settings"] = settings;
    packet["schema_version"] = int64_t(1);
    packet["controller_id"] = String(impl->controller_id.c_str());
    packet["issued_ms"] = int64_t(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count());
    const auto raw = JSON::stringify(packet, "", true, true).utf8();
    if (raw.length() > 4096) return false;
    std::lock_guard lock(impl->mutex);
    impl->wind_outgoing.assign(raw.get_data(), raw.length());
    return true;
}

Dictionary ZmqCameraAdapter::get_wind_state() {
    const Dictionary control = get_control_state();
    std::string raw;
    Clock::time_point received;
    { std::lock_guard lock(impl->mutex); raw.swap(impl->wind_slot); received = impl->wind_received; }
    if (!raw.empty()) {
        const auto state = parse(raw);
        if (raw.size() <= 4096 && valid_wind_state(state)) {
            impl->wind_state = state;
            impl->valid_wind_received = received;
        } else ++impl->invalid;
    }
    if (!impl->running || control.is_empty() || impl->wind_state.is_empty() ||
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - impl->valid_wind_received).count() > bullet_msgs::WIND_STATE_TIMEOUT_MS) return {};
    for (const auto key : {"physics_id", "episode_id", "controller_id"})
        if (impl->wind_state[key] != control[key]) return {};
    return impl->wind_state;
}


bool ZmqCameraAdapter::publish_health() {
    const auto now = Clock::now();
    if (!impl->running || !impl->publisher || now < impl->next_health) return false;
    const auto state = get_control_state();
    if (state.is_empty()) return false;
    impl->next_health = now + std::chrono::seconds(1);
    Dictionary packet;
    packet["schema_version"] = int64_t(1);
    packet["physics_id"] = state["physics_id"];
    packet["episode_id"] = state["episode_id"];
    packet["generated_us"] = int64_t(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
    packet["stats"] = get_stats();
    const auto raw = JSON::stringify(packet, "", true, true).utf8();
    constexpr char topic[] = "analysis/render/health";
    return zmq_send(impl->publisher, topic, sizeof(topic) - 1, ZMQ_SNDMORE | ZMQ_DONTWAIT) >= 0 &&
        zmq_send(impl->publisher, raw.get_data(), raw.length(), ZMQ_DONTWAIT) >= 0;
}
