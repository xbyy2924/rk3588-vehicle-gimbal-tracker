#include "gimbal_udp_controller.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <thread>

namespace {

constexpr uint32_t kMagic = UINT32_C(0x31424D47);
constexpr uint8_t kVersion = 1;
constexpr uint8_t kMessageCommand = 1;
constexpr uint8_t kMessageTelemetry = 3;
constexpr uint16_t kCommandHeartbeat = 1;
constexpr uint16_t kCommandSetTargets = 2;
constexpr uint16_t kCommandHoldCurrent = 3;
constexpr uint16_t kCommandSubscribe = 6;
constexpr uint16_t kFlagM1Valid = 1U << 0;
constexpr uint16_t kFlagM2Valid = 1U << 1;
constexpr uint16_t kFlagEnableWatchdog = 1U << 2;
constexpr uint32_t kSystemRemoteActive = 1U << 5;
constexpr uint32_t kSystemWatchdogExpired = 1U << 6;
constexpr uint64_t kHeartbeatPeriodMs = 100;
constexpr uint64_t kTargetPeriodMs = 33;

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "Gimbal UDP protocol requires a little-endian target"
#endif

#define PACKED __attribute__((packed))

struct PACKED PacketHeader {
    uint32_t magic;
    uint8_t version;
    uint8_t message_type;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t timestamp_ms;
};

struct PACKED CommandPacket {
    PacketHeader header;
    uint16_t command_id;
    uint16_t flags;
    int32_t m1_target_mdeg;
    int32_t m2_target_mdeg;
};

// Only the fields needed by visual control are named; padding preserves the
// exact 624-byte STM32 telemetry wire layout.
struct PACKED AxisTelemetryWire {
    uint8_t before_target[44];
    int32_t target_position_mdeg;
    uint8_t before_position[12];
    int32_t motor_position_mdeg;
    uint8_t remaining[144];
};

struct PACKED TelemetryPacketWire {
    PacketHeader header;
    uint32_t system_flags;
    uint8_t before_motor1[188];
    AxisTelemetryWire motor1;
    AxisTelemetryWire motor2;
};

static_assert(sizeof(PacketHeader) == 16, "PacketHeader layout mismatch");
static_assert(sizeof(CommandPacket) == 28, "CommandPacket layout mismatch");
static_assert(sizeof(AxisTelemetryWire) == 208, "Axis layout mismatch");
static_assert(sizeof(TelemetryPacketWire) == 624, "Telemetry layout mismatch");
static_assert(offsetof(AxisTelemetryWire, target_position_mdeg) == 44,
              "Axis target offset mismatch");
static_assert(offsetof(AxisTelemetryWire, motor_position_mdeg) == 60,
              "Axis position offset mismatch");
static_assert(offsetof(TelemetryPacketWire, motor1) == 208,
              "M1 telemetry offset mismatch");
static_assert(offsetof(TelemetryPacketWire, motor2) == 416,
              "M2 telemetry offset mismatch");

uint64_t monotonic_ms()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint32_t monotonic_ms32()
{
    return static_cast<uint32_t>(monotonic_ms());
}

int32_t degrees_to_mdeg(double degrees)
{
    const double scaled = degrees * 1000.0;
    const double bounded = std::max(
        static_cast<double>(std::numeric_limits<int32_t>::min()),
        std::min(static_cast<double>(std::numeric_limits<int32_t>::max()), scaled));
    return static_cast<int32_t>(bounded >= 0.0 ? bounded + 0.5 : bounded - 0.5);
}

int set_nonblocking(int fd)
{
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

}  // namespace

struct GimbalUdpController::Impl {
    Impl(const std::string& local_ip,
         const std::string& remote_ip,
         uint16_t remote_port,
         uint16_t local_port)
        : bind_ip(local_ip), stm32_ip_text(remote_ip),
          command_port(remote_port), telemetry_port(local_port)
    {
    }

    ~Impl()
    {
        stop();
    }

    bool start()
    {
        if (is_running.load(std::memory_order_acquire)) return true;

        if (inet_pton(AF_INET, stm32_ip_text.c_str(), &stm32_ip) != 1) {
            set_error("invalid STM32 IP: " + stm32_ip_text);
            return false;
        }

        std::memset(&stm32_address, 0, sizeof(stm32_address));
        stm32_address.sin_family = AF_INET;
        stm32_address.sin_port = htons(command_port);
        stm32_address.sin_addr = stm32_ip;

        command_fd = create_socket(0);
        if (command_fd < 0) {
            close_sockets();
            return false;
        }
        telemetry_fd = create_socket(telemetry_port);
        if (telemetry_fd < 0) {
            close_sockets();
            return false;
        }

        next_sequence = monotonic_ms32();
        is_running.store(true, std::memory_order_release);
        send_command(kCommandSubscribe, 0, 0, 0);
        worker = std::thread(&Impl::worker_loop, this);
        return true;
    }

    void stop()
    {
        if (!is_running.exchange(false, std::memory_order_acq_rel)) {
            close_sockets();
            return;
        }
        if (worker.joinable()) worker.join();
        close_sockets();
    }

    int create_socket(uint16_t port)
    {
        const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (fd < 0) {
            set_errno_error("socket");
            return -1;
        }

        const int enabled = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled,
                       sizeof(enabled)) != 0) {
            set_errno_error("setsockopt(SO_REUSEADDR)");
            close(fd);
            return -1;
        }

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(port);
        if (inet_pton(AF_INET, bind_ip.c_str(), &local.sin_addr) != 1) {
            set_error("invalid bind IP: " + bind_ip);
            close(fd);
            return -1;
        }
        if (bind(fd, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
            set_errno_error("bind(" + bind_ip + ":" + std::to_string(port) + ")");
            close(fd);
            return -1;
        }
        if (set_nonblocking(fd) != 0) {
            set_errno_error("fcntl(O_NONBLOCK)");
            close(fd);
            return -1;
        }
        return fd;
    }

    bool send_command(uint16_t command_id, uint16_t flags,
                      int32_t m1_mdeg, int32_t m2_mdeg)
    {
        if (command_fd < 0) return false;
        CommandPacket packet{};
        packet.header.magic = kMagic;
        packet.header.version = kVersion;
        packet.header.message_type = kMessageCommand;
        packet.header.payload_size = sizeof(packet) - sizeof(packet.header);
        packet.header.sequence = ++next_sequence;
        packet.header.timestamp_ms = monotonic_ms32();
        packet.command_id = command_id;
        packet.flags = flags;
        packet.m1_target_mdeg = m1_mdeg;
        packet.m2_target_mdeg = m2_mdeg;

        const ssize_t sent = sendto(
            command_fd, &packet, sizeof(packet), 0,
            reinterpret_cast<const sockaddr*>(&stm32_address),
            sizeof(stm32_address));
        if (sent != static_cast<ssize_t>(sizeof(packet))) {
            set_errno_error("sendto(command)");
            return false;
        }
        return true;
    }

    void set_targets(double m1_deg, double m2_deg)
    {
        std::lock_guard<std::mutex> lock(request_mutex);
        requested_m1_deg = m1_deg;
        requested_m2_deg = m2_deg;
        target_pending = true;
    }

    void hold()
    {
        std::lock_guard<std::mutex> lock(request_mutex);
        hold_pending = true;
        target_pending = false;
    }

    void drain_command_socket()
    {
        uint8_t buffer[512];
        for (;;) {
            const ssize_t received = recv(command_fd, buffer, sizeof(buffer), MSG_DONTWAIT);
            if (received >= 0) continue;
            if (errno == EINTR) continue;
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                set_errno_error("recv(ACK)");
            return;
        }
    }

    void drain_telemetry_socket()
    {
        uint8_t buffer[2048];
        for (;;) {
            sockaddr_in sender{};
            socklen_t sender_length = sizeof(sender);
            const ssize_t received = recvfrom(
                telemetry_fd, buffer, sizeof(buffer), MSG_DONTWAIT,
                reinterpret_cast<sockaddr*>(&sender), &sender_length);
            if (received < 0) {
                if (errno == EINTR) continue;
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                    set_errno_error("recvfrom(telemetry)");
                return;
            }
            if (sender.sin_family != AF_INET ||
                sender.sin_addr.s_addr != stm32_ip.s_addr ||
                received != static_cast<ssize_t>(sizeof(TelemetryPacketWire))) {
                continue;
            }

            TelemetryPacketWire packet{};
            std::memcpy(&packet, buffer, sizeof(packet));
            if (packet.header.magic != kMagic ||
                packet.header.version != kVersion ||
                packet.header.message_type != kMessageTelemetry ||
                packet.header.payload_size != sizeof(packet) - sizeof(packet.header)) {
                continue;
            }

            std::lock_guard<std::mutex> lock(telemetry_mutex);
            latest.valid = true;
            latest.remote_active =
                (packet.system_flags & kSystemRemoteActive) != 0;
            latest.watchdog_expired =
                (packet.system_flags & kSystemWatchdogExpired) != 0;
            latest.m1_position_deg = packet.motor1.motor_position_mdeg / 1000.0;
            latest.m2_position_deg = packet.motor2.motor_position_mdeg / 1000.0;
            latest.m1_target_deg = packet.motor1.target_position_mdeg / 1000.0;
            latest.m2_target_deg = packet.motor2.target_position_mdeg / 1000.0;
            ++latest.received_count;
            last_telemetry_ms = monotonic_ms();
        }
    }

    void worker_loop()
    {
        uint64_t next_heartbeat_ms = monotonic_ms() + kHeartbeatPeriodMs;
        uint64_t last_target_ms = 0;

        while (is_running.load(std::memory_order_acquire)) {
            pollfd descriptors[2]{};
            descriptors[0].fd = command_fd;
            descriptors[0].events = POLLIN;
            descriptors[1].fd = telemetry_fd;
            descriptors[1].events = POLLIN;
            const int poll_result = poll(descriptors, 2, 10);
            if (poll_result > 0) {
                if ((descriptors[0].revents & POLLIN) != 0)
                    drain_command_socket();
                if ((descriptors[1].revents & POLLIN) != 0)
                    drain_telemetry_socket();
            } else if (poll_result < 0 && errno != EINTR) {
                set_errno_error("poll(gimbal)");
            }

            const uint64_t now_ms = monotonic_ms();
            if (now_ms >= next_heartbeat_ms) {
                send_command(kCommandHeartbeat, 0, 0, 0);
                next_heartbeat_ms = now_ms + kHeartbeatPeriodMs;
            }

            bool send_hold = false;
            bool send_target = false;
            double m1_deg = 0.0;
            double m2_deg = 0.0;
            {
                std::lock_guard<std::mutex> lock(request_mutex);
                if (hold_pending) {
                    send_hold = true;
                    hold_pending = false;
                } else if (target_pending && now_ms - last_target_ms >= kTargetPeriodMs) {
                    send_target = true;
                    m1_deg = requested_m1_deg;
                    m2_deg = requested_m2_deg;
                    target_pending = false;
                }
            }
            if (send_hold) {
                send_command(kCommandHoldCurrent, kFlagEnableWatchdog, 0, 0);
            } else if (send_target) {
                send_command(kCommandSetTargets,
                             kFlagM1Valid | kFlagM2Valid | kFlagEnableWatchdog,
                             degrees_to_mdeg(m1_deg), degrees_to_mdeg(m2_deg));
                last_target_ms = now_ms;
            }
        }

        // Safe shutdown: stop motion at the measured current position.
        send_command(kCommandHoldCurrent, kFlagEnableWatchdog, 0, 0);
    }

    GimbalTelemetrySnapshot telemetry() const
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        GimbalTelemetrySnapshot result = latest;
        if (result.valid) result.age_ms = monotonic_ms() - last_telemetry_ms;
        return result;
    }

    void close_sockets()
    {
        if (telemetry_fd >= 0) close(telemetry_fd);
        if (command_fd >= 0) close(command_fd);
        telemetry_fd = -1;
        command_fd = -1;
    }

    void set_error(const std::string& message)
    {
        std::lock_guard<std::mutex> lock(error_mutex);
        error = message;
    }

    void set_errno_error(const std::string& operation)
    {
        set_error(operation + ": " + std::strerror(errno));
    }

    std::string last_error() const
    {
        std::lock_guard<std::mutex> lock(error_mutex);
        return error;
    }

    std::string bind_ip;
    std::string stm32_ip_text;
    uint16_t command_port;
    uint16_t telemetry_port;
    in_addr stm32_ip{};
    sockaddr_in stm32_address{};
    int command_fd = -1;
    int telemetry_fd = -1;
    uint32_t next_sequence = 0;
    std::atomic<bool> is_running{false};
    std::thread worker;

    mutable std::mutex request_mutex;
    double requested_m1_deg = 0.0;
    double requested_m2_deg = 0.0;
    bool target_pending = false;
    bool hold_pending = false;

    mutable std::mutex telemetry_mutex;
    GimbalTelemetrySnapshot latest;
    uint64_t last_telemetry_ms = 0;

    mutable std::mutex error_mutex;
    std::string error;
};

GimbalUdpController::GimbalUdpController(const std::string& bind_ip,
                                         const std::string& stm32_ip,
                                         uint16_t command_port,
                                         uint16_t telemetry_port)
    : impl_(new Impl(bind_ip, stm32_ip, command_port, telemetry_port))
{
}

GimbalUdpController::~GimbalUdpController() = default;

bool GimbalUdpController::start()
{
    return impl_->start();
}

void GimbalUdpController::stop()
{
    impl_->stop();
}

bool GimbalUdpController::running() const
{
    return impl_->is_running.load(std::memory_order_acquire);
}

void GimbalUdpController::set_targets(double m1_deg, double m2_deg)
{
    impl_->set_targets(m1_deg, m2_deg);
}

void GimbalUdpController::hold()
{
    impl_->hold();
}

GimbalTelemetrySnapshot GimbalUdpController::telemetry() const
{
    return impl_->telemetry();
}

std::string GimbalUdpController::last_error() const
{
    return impl_->last_error();
}
