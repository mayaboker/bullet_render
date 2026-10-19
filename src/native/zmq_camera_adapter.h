#pragma once
#include <godot_cpp/classes/node.hpp>
#include <memory>

class ZmqCameraAdapter : public godot::Node {
    GDCLASS(ZmqCameraAdapter, godot::Node)
    struct Impl;
    std::unique_ptr<Impl> impl;
protected:
    static void _bind_methods();
public:
    ZmqCameraAdapter();
    ~ZmqCameraAdapter();
    bool start(const godot::String &transport_path);
    godot::Dictionary poll_request();
    bool publish_frame(const godot::PackedByteArray &pixels, const godot::Dictionary &request);
    godot::Dictionary get_stats() const;
    bool publish_control(const godot::String &action);
    godot::Dictionary get_control_state();
    void close();
};
