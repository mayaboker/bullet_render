#pragma once
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/random.h>
#include <sys/stat.h>
#include <unistd.h>

namespace bullet_render {
inline constexpr size_t HEADER_BYTES = 128;
inline constexpr size_t SLOT_COUNT = 3;
inline constexpr uint32_t VERSION = 1;
inline constexpr size_t MAX_IMAGE_BYTES = 64 * 1024 * 1024;
inline constexpr size_t MAX_METADATA_BYTES = 1024;
inline constexpr char MAGIC[8] = "BMSHM01";

inline void put_le(uint8_t *data, uint64_t value, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i) data[i] = (value >> (8 * i)) & 0xff;
}
inline bool valid_endpoint(std::string_view endpoint) {
    constexpr std::string_view prefix = "tcp://127.0.0.1:";
    if (!endpoint.starts_with(prefix)) return false;
    auto port = endpoint.substr(prefix.size());
    if (port.empty() || port.size() > 5 || port[0] == '0') return false;
    unsigned value = 0;
    auto parsed = std::from_chars(port.data(), port.data() + port.size(), value);
    return parsed.ec == std::errc() && parsed.ptr == port.data() + port.size() && value >= 1 && value <= 65535;
}
inline void check_posix(int result, const char *operation) {
    if (result < 0) throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}

class FrameWriter {
    struct Slot {
        int fd = -1;
        uint8_t *data = nullptr;
        std::string name;
    };
    std::array<Slot, SLOT_COUNT> slots;
    size_t next_slot = 0;
    uint32_t width = 0, height = 0;
    size_t frame_bytes = 0;
public:
    std::string session;
    uint64_t sequence = 0;
    FrameWriter() = default;
    FrameWriter(const FrameWriter &) = delete;
    FrameWriter &operator=(const FrameWriter &) = delete;
    ~FrameWriter() { close(); }

    void open(uint32_t w, uint32_t h) {
        close();
        if (!w || !h || uint64_t(w) * h * 3 > MAX_IMAGE_BYTES)
            throw std::runtime_error("invalid SHM image dimensions");
        width = w; height = h; frame_bytes = size_t(w) * h * 3;
        std::array<uint8_t, 16> random{};
        size_t offset = 0;
        while (offset < random.size()) {
            const auto count = getrandom(random.data() + offset, random.size() - offset, 0);
            if (count < 0 && errno == EINTR) continue;
            check_posix(count, "getrandom");
            if (count == 0) throw std::runtime_error("getrandom returned no bytes");
            offset += count;
        }
        session.clear();
        constexpr char hex[] = "0123456789abcdef";
        for (const auto byte : random) { session += hex[byte >> 4]; session += hex[byte & 15]; }
        try {
            for (size_t i = 0; i < SLOT_COUNT; ++i) {
                auto &slot = slots[i];
                const auto name = "/bullet_msgs_" + session + "_s" + std::to_string(i);
                slot.fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0600);
                check_posix(slot.fd, "shm_open");
                slot.name = name;
                check_posix(fchmod(slot.fd, 0600), "fchmod");
                check_posix(ftruncate(slot.fd, HEADER_BYTES + frame_bytes), "ftruncate");
                const int allocated = posix_fallocate(slot.fd, 0, HEADER_BYTES + frame_bytes);
                if (allocated != 0) throw std::runtime_error(std::string("SHM allocation: ") + std::strerror(allocated));
                void *mapping = mmap(nullptr, HEADER_BYTES + frame_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, slot.fd, 0);
                if (mapping == MAP_FAILED) check_posix(-1, "mmap");
                slot.data = static_cast<uint8_t *>(mapping);
            }
        } catch (...) { close(); throw; }
    }

    // Empty notification means every slot is busy, never wait for readers.
    std::string write(const uint8_t *pixels, size_t size, uint64_t capture_ns,
                      const std::string &source_json = {}, const std::string &header_json = {},
                      const std::string &drone_id_json = {}, uint64_t host_capture_ns = 0) {
        if (!frame_bytes || size != frame_bytes) throw std::runtime_error("SHM RGB frame size mismatch");
        ++sequence;
        for (size_t attempt = 0; attempt < SLOT_COUNT; ++attempt) {
            const size_t index = (next_slot + attempt) % SLOT_COUNT;
            auto &slot = slots[index];
            if (flock(slot.fd, LOCK_EX | LOCK_NB) != 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
                check_posix(-1, "flock writer");
            }
            struct Lock { int fd; ~Lock() { flock(fd, LOCK_UN); } } lock{slot.fd};
            std::array<uint8_t, HEADER_BYTES> header{};
            std::memcpy(header.data(), MAGIC, 8);
            const std::array<uint32_t, 6> fields{VERSION, uint32_t(index), width, height, 1, width * 3};
            for (size_t i = 0; i < fields.size(); ++i) put_le(header.data() + 8 + i * 4, fields[i], 4);
            put_le(header.data() + 32, frame_bytes, 8);
            put_le(header.data() + 40, sequence, 8);
            put_le(header.data() + 48, capture_ns, 8);
            std::memcpy(header.data() + 64, session.data(), session.size());
            const uint8_t incomplete[8]{};
            if (pwrite(slot.fd, incomplete, 8, 56) != 8) throw std::runtime_error("could not invalidate SHM slot");
            std::atomic_thread_fence(std::memory_order_seq_cst);
            std::memcpy(slot.data, header.data(), header.size());
            std::memcpy(slot.data + HEADER_BYTES, pixels, frame_bytes);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            const uint8_t complete[8]{1};
            if (pwrite(slot.fd, complete, 8, 56) != 8) throw std::runtime_error("could not commit SHM slot");
            if (header_json.empty() != drone_id_json.empty()) throw std::runtime_error("incomplete pose identity");
            const auto descriptor_version = !header_json.empty() ? "3" : (source_json.empty() ? "1" : "2");
            const auto metadata = "{\"version\":" + std::string(descriptor_version) + ",\"session\":\"" + session + "\",\"slot\":" + std::to_string(index) +
                ",\"shm_name\":\"" + slot.name + "\",\"width\":" + std::to_string(width) +
                ",\"height\":" + std::to_string(height) + ",\"encoding\":\"rgb8\",\"step\":" + std::to_string(width * 3) +
                ",\"bytes\":" + std::to_string(frame_bytes) + ",\"sequence\":" + std::to_string(sequence) +
                ",\"capture_ns\":" + std::to_string(capture_ns) +
                (source_json.empty() ? "" : ",\"source\":" + source_json) +
                (header_json.empty() ? "" : ",\"header\":" + header_json + ",\"drone_id\":" + drone_id_json) +
                (host_capture_ns == 0 ? "" : ",\"host_capture_ns\":" + std::to_string(host_capture_ns)) + "}";
            if (metadata.size() > MAX_METADATA_BYTES) throw std::runtime_error("SHM descriptor too large");
            constexpr char topic[] = "camera/front/image";
            std::string message(topic, sizeof(topic));
            std::array<uint8_t, 4> length{};
            put_le(length.data(), metadata.size(), 4);
            message.append(reinterpret_cast<const char *>(length.data()), length.size());
            message += metadata;
            next_slot = (index + 1) % SLOT_COUNT;
            return message;
        }
        return {};
    }

    void close() noexcept {
        for (auto &slot : slots) {
            if (slot.data) { munmap(slot.data, HEADER_BYTES + frame_bytes); slot.data = nullptr; }
            if (slot.fd >= 0) { ::close(slot.fd); slot.fd = -1; }
            if (!slot.name.empty()) { shm_unlink(slot.name.c_str()); slot.name.clear(); }
        }
        frame_bytes = 0; next_slot = 0; sequence = 0;
    }
};
}
