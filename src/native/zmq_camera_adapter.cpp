#include "zmq_camera_adapter.h"
#include <bullet_msgs/shm.hpp>
#include <bullet_msgs/control.hpp>
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
bool valid_source(const Dictionary &s, const Dictionary &pose) {
    if (!shape(s, {"physics_id", "episode_id", "stream_id", "step_index", "simulation_time_s", "measurement_us"})) return false;
    for (const auto key : {"physics_id", "episode_id", "stream_id"}) if (!control_identity(s[key])) return false;
    for (const auto key : {"step_index", "measurement_us"}) if (!number(s[key], 0, MAX_SEQUENCE, true)) return false;
    if (!number(s["simulation_time_s"], 0, 1e12)) return false;
    for (const auto key : {"episode_id", "stream_id", "step_index", "simulation_time_s"}) if (s[key] != pose[key]) return false;
    return true;
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
    std::string config_slot, pose_slot, state_slot;
    Clock::time_point state_received{}, valid_state_received{};
    std::deque<std::pair<std::string, int64_t>> commands;
    std::string controller_id = std::to_string(getpid()) + "-" + std::to_string(Clock::now().time_since_epoch().count());
    Dictionary control_state;
    String control_error;
    Clock::time_point pose_received{}, capture_received{};
    std::atomic<int64_t> invalid{0}, superseded{0};
    int64_t frames = 0, sequence_gaps = 0, stale = 0, last_sequence = -1, capture_sequence = -1;
    int64_t frame_bytes = 0, busy_slots = 0, publication_drops = 0;
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
                if (!is_config && !is_state && topic != "render/drone_1/pose") continue;
                std::lock_guard lock(mutex);
                if (is_state) { state_slot = std::move(raw); state_received = Clock::now(); continue; }
                if (is_config) config_slot = std::move(raw);
                else {
                    if (!pose_slot.empty()) ++superseded;
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
    ClassDB::bind_method(D_METHOD("publish_control", "action"), &ZmqCameraAdapter::publish_control);
    ClassDB::bind_method(D_METHOD("get_control_state"), &ZmqCameraAdapter::get_control_state);
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
            source_json = JSON::stringify(source).utf8().get_data();
        }
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

Dictionary ZmqCameraAdapter::get_control_state() {
    std::string raw;
    Clock::time_point received;
    { std::lock_guard lock(impl->mutex); raw.swap(impl->state_slot); received = impl->state_received; }
    if (!raw.empty()) {
        const auto state = parse(raw);
        bool valid = raw.size() <= 4096 && shape(state, {"schema_version", "physics_id", "controller_id", "episode_id", "state", "reason", "simulation_time_s"});
        if (valid) {
            const String mode = state["state"];
            const String reason = state["reason"];
            valid = number(state["schema_version"], 1, 1, true) && control_identity(state["physics_id"]) && control_identity(state["episode_id"]) &&
                (state["controller_id"].get_type() == Variant::NIL || control_identity(state["controller_id"])) &&
                state["state"].get_type() == Variant::STRING && (mode == "running" || mode == "stopped" || mode == "error") &&
                state["reason"].get_type() == Variant::STRING && reason.length() > 0 && reason.length() <= 256 &&
                number(state["simulation_time_s"], 0, 1e12);
        }
        if (valid) { impl->control_state = state; impl->valid_state_received = received; impl->control_error = ""; }
        else { ++impl->invalid; impl->control_error = "Rejected invalid simulation state"; }
    }
    if (!impl->running || std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - impl->valid_state_received).count() > bullet_msgs::HEARTBEAT_TIMEOUT_MS) return {};
    return impl->control_state;
}
