#include "gimbal_udp_controller.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>

namespace {

constexpr uint32_t kMagic = UINT32_C(0x31424D47);
constexpr uint8_t kVersion = 1;
constexpr uint8_t kMessageCommand = 1;
constexpr uint8_t kMessageAck = 2;
constexpr uint8_t kMessageTelemetry = 3;
constexpr uint16_t kCommandHeartbeat = 1;
constexpr uint16_t kCommandSetTargets = 2;
constexpr uint16_t kCommandHoldCurrent = 3;
constexpr uint16_t kCommandDisarm = 5;
constexpr uint16_t kCommandSubscribe = 6;
constexpr uint16_t kFlagM1Valid = 1U << 0;
constexpr uint16_t kFlagM2Valid = 1U << 1;
constexpr uint16_t kFlagEnableWatchdog = 1U << 2;
constexpr uint32_t kSystemRemoteActive = 1U << 5;
constexpr uint32_t kSystemWatchdogExpired = 1U << 6;
constexpr uint64_t kHeartbeatPeriodMs = 100;
constexpr uint64_t kTargetPeriodMs = 33;
constexpr uint64_t kTelemetryFreshMs = 300;
constexpr uint64_t kAckFreshMs = 300;
constexpr uint64_t kSequenceResetMs = 1000;

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

struct PACKED AckPacket {
    PacketHeader header;
    int32_t result;
    uint32_t command_sequence;
    uint32_t client_timestamp_ms;
    int32_t applied_m1_target_mdeg;
    int32_t applied_m2_target_mdeg;
    uint32_t watchdog_remaining_ms;
};

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
    uint32_t current_start_status;
    uint32_t command_rx_packets;
    uint32_t valid_commands;
    uint32_t invalid_commands;
    uint32_t stale_commands;
    uint32_t command_tx_errors;
    uint32_t telemetry_tx_packets;
    uint32_t telemetry_tx_errors;
    uint32_t last_command_age_ms;
    uint32_t watchdog_trip_count;
    int32_t last_socket_result;
    uint32_t spi_poll_transfers;
    uint32_t spi_dma_transfers;
    uint32_t spi_dma_errors;
    uint32_t spi_dma_timeouts;
    uint8_t phase_current[48];
    uint8_t encoder1[40];
    uint8_t encoder2[40];
    AxisTelemetryWire motor1;
    AxisTelemetryWire motor2;
};

static_assert(sizeof(PacketHeader) == 16, "PacketHeader layout mismatch");
static_assert(sizeof(CommandPacket) == 28, "CommandPacket layout mismatch");
static_assert(sizeof(AckPacket) == 40, "AckPacket layout mismatch");
static_assert(sizeof(AxisTelemetryWire) == 208, "AxisTelemetryWire layout mismatch");
static_assert(offsetof(TelemetryPacketWire, motor1) == 208, "M1 telemetry offset mismatch");
static_assert(offsetof(TelemetryPacketWire, motor2) == 416, "M2 telemetry offset mismatch");
static_assert(sizeof(TelemetryPacketWire) == 624, "TelemetryPacketWire layout mismatch");

uint64_t monotonic_ms()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint32_t monotonic_ms32() { return static_cast<uint32_t>(monotonic_ms()); }

bool sequence_is_newer(uint32_t candidate, uint32_t reference)
{
    return static_cast<int32_t>(candidate - reference) > 0;
}

int32_t degrees_to_mdeg(double degrees)
{
    const double scaled = degrees * 1000.0;
    const double bounded = std::clamp(scaled,
        static_cast<double>(std::numeric_limits<int32_t>::min()),
        static_cast<double>(std::numeric_limits<int32_t>::max()));
    return static_cast<int32_t>(bounded >= 0.0 ? bounded + 0.5 : bounded - 0.5);
}

int set_nonblocking(int fd)
{
    const int flags = fcntl(fd, F_GETFL, 0);
    return flags < 0 ? -1 : fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

bool valid_header(const PacketHeader& header, uint8_t type, size_t packet_size)
{
    return header.magic == kMagic && header.version == kVersion &&
           header.message_type == type &&
           header.payload_size == packet_size - sizeof(PacketHeader);
}

enum class PendingAction : uint8_t { None, Targets, Hold, Disarm };

}  // namespace

struct GimbalUdpController::Impl {
    Impl(std::string local_ip, std::string remote_ip, uint16_t remote_port, uint16_t local_port)
        : bind_ip(std::move(local_ip)), stm32_ip_text(std::move(remote_ip)),
          command_port(remote_port), telemetry_port(local_port) {}

    ~Impl() { stop(); }

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
        telemetry_fd = create_socket(telemetry_port);
        wake_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
        if (command_fd < 0 || telemetry_fd < 0 || wake_fd < 0) {
            if (wake_fd < 0) set_errno_error("eventfd");
            close_descriptors();
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(status_mutex);
            latest = {};
            have_telemetry_sequence = false;
            last_telemetry_ms = 0;
            last_ack_ms = 0;
            last_control_ack_ms = 0;
        }
        next_sequence = monotonic_ms32();
        is_running.store(true, std::memory_order_release);
        worker = std::thread(&Impl::worker_loop, this);
        return true;
    }

    void stop()
    {
        if (!is_running.exchange(false, std::memory_order_acq_rel)) {
            if (worker.joinable()) worker.join();
            close_descriptors();
            return;
        }
        wake_worker();
        if (worker.joinable()) worker.join();
        close_descriptors();
    }

    int create_socket(uint16_t port)
    {
        const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (fd < 0) { set_errno_error("socket"); return -1; }
        const int enabled = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)) != 0) {
            set_errno_error("setsockopt(SO_REUSEADDR)");
            close(fd);
            return -1;
        }
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(port);
        if (inet_pton(AF_INET, bind_ip.c_str(), &local.sin_addr) != 1 ||
            bind(fd, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0 ||
            set_nonblocking(fd) != 0) {
            set_errno_error("bind/fcntl(" + bind_ip + ":" + std::to_string(port) + ")");
            close(fd);
            return -1;
        }
        return fd;
    }

    uint32_t send_command(uint16_t command_id, uint16_t flags, int32_t m1_mdeg, int32_t m2_mdeg)
    {
        if (command_fd < 0) return 0;
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
        const ssize_t sent = sendto(command_fd, &packet, sizeof(packet), 0,
            reinterpret_cast<const sockaddr*>(&stm32_address), sizeof(stm32_address));
        if (sent != static_cast<ssize_t>(sizeof(packet))) {
            set_errno_error("sendto(command)");
            return 0;
        }
        if (command_id == kCommandSetTargets || command_id == kCommandHoldCurrent ||
            command_id == kCommandDisarm) {
            std::lock_guard<std::mutex> lock(status_mutex);
            latest.last_control_sequence = packet.header.sequence;
            latest.last_control_ack_result = GimbalAckResult::Unknown;
            last_control_ack_ms = 0;
        }
        return packet.header.sequence;
    }

    bool set_targets(double m1_deg, double m2_deg)
    {
        if (!std::isfinite(m1_deg) || !std::isfinite(m2_deg)) {
            set_error("target angle is NaN or infinity");
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(request_mutex);
            requested_m1_deg = m1_deg;
            requested_m2_deg = m2_deg;
            pending_action = PendingAction::Targets;
        }
        wake_worker();
        return true;
    }

    void request(PendingAction action)
    {
        {
            std::lock_guard<std::mutex> lock(request_mutex);
            pending_action = action;
        }
        wake_worker();
    }

    void wake_worker()
    {
        if (wake_fd < 0) return;
        const uint64_t one = 1;
        const ssize_t ignored = write(wake_fd, &one, sizeof(one));
        (void)ignored;
    }

    void drain_wake_fd()
    {
        uint64_t value;
        while (read(wake_fd, &value, sizeof(value)) == static_cast<ssize_t>(sizeof(value))) {}
    }

    bool valid_sender(const sockaddr_in& sender, uint16_t expected_port) const
    {
        return sender.sin_family == AF_INET && sender.sin_addr.s_addr == stm32_ip.s_addr &&
               ntohs(sender.sin_port) == expected_port;
    }

    void drain_ack_socket()
    {
        for (;;) {
            AckPacket packet{};
            sockaddr_in sender{};
            socklen_t sender_length = sizeof(sender);
            const ssize_t received = recvfrom(command_fd, &packet, sizeof(packet), MSG_DONTWAIT,
                reinterpret_cast<sockaddr*>(&sender), &sender_length);
            if (received < 0) {
                if (errno == EINTR) continue;
                if (errno != EAGAIN && errno != EWOULDBLOCK) set_errno_error("recvfrom(ACK)");
                return;
            }
            const uint64_t now = monotonic_ms();
            std::lock_guard<std::mutex> lock(status_mutex);
            if (!valid_sender(sender, command_port) || received != static_cast<ssize_t>(sizeof(packet)) ||
                !valid_header(packet.header, kMessageAck, sizeof(packet))) {
                ++latest.ack_invalid;
                continue;
            }
            ++latest.ack_received;
            latest.last_ack_result = static_cast<GimbalAckResult>(packet.result);
            latest.last_acked_command_sequence = packet.command_sequence;
            last_ack_ms = now;
            if (packet.command_sequence == latest.last_control_sequence) {
                latest.last_control_ack_result = static_cast<GimbalAckResult>(packet.result);
                last_control_ack_ms = now;
            }
        }
    }

    void drain_telemetry_socket()
    {
        for (;;) {
            TelemetryPacketWire packet{};
            sockaddr_in sender{};
            socklen_t sender_length = sizeof(sender);
            const ssize_t received = recvfrom(telemetry_fd, &packet, sizeof(packet), MSG_DONTWAIT,
                reinterpret_cast<sockaddr*>(&sender), &sender_length);
            if (received < 0) {
                if (errno == EINTR) continue;
                if (errno != EAGAIN && errno != EWOULDBLOCK) set_errno_error("recvfrom(telemetry)");
                return;
            }

            const uint64_t now = monotonic_ms();
            std::lock_guard<std::mutex> lock(status_mutex);
            if (!valid_sender(sender, telemetry_port) || received != static_cast<ssize_t>(sizeof(packet)) ||
                !valid_header(packet.header, kMessageTelemetry, sizeof(packet))) {
                ++latest.telemetry_invalid;
                continue;
            }

            if (have_telemetry_sequence && last_telemetry_ms != 0 &&
                now - last_telemetry_ms > kSequenceResetMs) have_telemetry_sequence = false;
            if (have_telemetry_sequence && !sequence_is_newer(packet.header.sequence, last_telemetry_sequence)) {
                ++latest.telemetry_stale;
                continue;
            }
            if (have_telemetry_sequence) {
                const uint32_t delta = packet.header.sequence - last_telemetry_sequence;
                if (delta > 1U) latest.telemetry_lost += static_cast<uint64_t>(delta - 1U);
            }
            have_telemetry_sequence = true;
            last_telemetry_sequence = packet.header.sequence;
            last_telemetry_ms = now;

            latest.valid = true;
            latest.system_flags = packet.system_flags;
            latest.remote_active = (packet.system_flags & kSystemRemoteActive) != 0;
            latest.watchdog_expired = (packet.system_flags & kSystemWatchdogExpired) != 0;
            latest.telemetry_sequence = packet.header.sequence;
            latest.last_command_age_ms = packet.last_command_age_ms;
            latest.watchdog_trip_count = packet.watchdog_trip_count;
            latest.m1_position_deg = packet.motor1.motor_position_mdeg / 1000.0;
            latest.m2_position_deg = packet.motor2.motor_position_mdeg / 1000.0;
            latest.m1_target_deg = packet.motor1.target_position_mdeg / 1000.0;
            latest.m2_target_deg = packet.motor2.target_position_mdeg / 1000.0;
            ++latest.telemetry_received;
        }
    }

    void process_pending(uint64_t now_ms, uint64_t& last_target_ms)
    {
        PendingAction action = PendingAction::None;
        double m1_deg = 0.0;
        double m2_deg = 0.0;
        {
            std::lock_guard<std::mutex> lock(request_mutex);
            if (pending_action == PendingAction::Targets && now_ms - last_target_ms < kTargetPeriodMs) return;
            action = pending_action;
            m1_deg = requested_m1_deg;
            m2_deg = requested_m2_deg;
            pending_action = PendingAction::None;
        }

        switch (action) {
        case PendingAction::Targets:
            send_command(kCommandSetTargets, kFlagM1Valid | kFlagM2Valid | kFlagEnableWatchdog,
                         degrees_to_mdeg(m1_deg), degrees_to_mdeg(m2_deg));
            last_target_ms = now_ms;
            break;
        case PendingAction::Hold:
            send_command(kCommandHoldCurrent, kFlagEnableWatchdog, 0, 0);
            break;
        case PendingAction::Disarm:
            send_command(kCommandDisarm, 0, 0, 0);
            break;
        default:
            break;
        }
    }

    void worker_loop()
    {
        send_command(kCommandSubscribe, 0, 0, 0);
        uint64_t next_heartbeat_ms = monotonic_ms() + kHeartbeatPeriodMs;
        uint64_t last_target_ms = 0;

        while (is_running.load(std::memory_order_acquire)) {
            pollfd descriptors[3]{};
            descriptors[0] = {command_fd, POLLIN, 0};
            descriptors[1] = {telemetry_fd, POLLIN, 0};
            descriptors[2] = {wake_fd, POLLIN, 0};
            const int result = poll(descriptors, 3, 20);
            if (result > 0) {
                if ((descriptors[0].revents & POLLIN) != 0) drain_ack_socket();
                if ((descriptors[1].revents & POLLIN) != 0) drain_telemetry_socket();
                if ((descriptors[2].revents & POLLIN) != 0) drain_wake_fd();
            } else if (result < 0 && errno != EINTR) {
                set_errno_error("poll(gimbal)");
            }

            const uint64_t now_ms = monotonic_ms();
            if (now_ms >= next_heartbeat_ms) {
                send_command(kCommandHeartbeat, 0, 0, 0);
                next_heartbeat_ms = now_ms + kHeartbeatPeriodMs;
            }
            process_pending(now_ms, last_target_ms);
        }

        // Best-effort safe shutdown. The MCU watchdog remains the final fallback.
        send_command(kCommandDisarm, 0, 0, 0);
    }

    GimbalTelemetrySnapshot telemetry() const
    {
        std::lock_guard<std::mutex> lock(status_mutex);
        GimbalTelemetrySnapshot result = latest;
        const uint64_t now = monotonic_ms();
        if (result.valid && last_telemetry_ms != 0) result.telemetry_age_ms = now - last_telemetry_ms;
        if (last_ack_ms != 0) result.ack_age_ms = now - last_ack_ms;
        if (last_control_ack_ms != 0) result.last_control_ack_age_ms = now - last_control_ack_ms;
        result.telemetry_fresh = result.valid && result.telemetry_age_ms <= kTelemetryFreshMs;
        result.command_link_ok = result.ack_age_ms <= kAckFreshMs &&
                                 result.last_ack_result == GimbalAckResult::Ok;
        return result;
    }

    void close_descriptors()
    {
        if (wake_fd >= 0) close(wake_fd);
        if (telemetry_fd >= 0) close(telemetry_fd);
        if (command_fd >= 0) close(command_fd);
        wake_fd = -1;
        telemetry_fd = -1;
        command_fd = -1;
    }

    void set_error(const std::string& message)
    {
        std::lock_guard<std::mutex> lock(error_mutex);
        error = message;
    }

    void set_errno_error(const std::string& operation) { set_error(operation + ": " + std::strerror(errno)); }

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
    int wake_fd = -1;
    uint32_t next_sequence = 0;
    std::atomic<bool> is_running{false};
    std::thread worker;

    mutable std::mutex request_mutex;
    PendingAction pending_action = PendingAction::None;
    double requested_m1_deg = 0.0;
    double requested_m2_deg = 0.0;

    mutable std::mutex status_mutex;
    GimbalTelemetrySnapshot latest;
    bool have_telemetry_sequence = false;
    uint32_t last_telemetry_sequence = 0;
    uint64_t last_telemetry_ms = 0;
    uint64_t last_ack_ms = 0;
    uint64_t last_control_ack_ms = 0;

    mutable std::mutex error_mutex;
    std::string error;
};

GimbalUdpController::GimbalUdpController(const std::string& bind_ip,
                                         const std::string& stm32_ip,
                                         uint16_t command_port,
                                         uint16_t telemetry_port)
    : impl_(new Impl(bind_ip, stm32_ip, command_port, telemetry_port)) {}

GimbalUdpController::~GimbalUdpController() = default;
bool GimbalUdpController::start() { return impl_->start(); }
void GimbalUdpController::stop() { impl_->stop(); }
bool GimbalUdpController::running() const { return impl_->is_running.load(std::memory_order_acquire); }
bool GimbalUdpController::set_targets(double m1_deg, double m2_deg) { return impl_->set_targets(m1_deg, m2_deg); }
void GimbalUdpController::hold() { impl_->request(PendingAction::Hold); }
void GimbalUdpController::disarm() { impl_->request(PendingAction::Disarm); }
GimbalTelemetrySnapshot GimbalUdpController::telemetry() const { return impl_->telemetry(); }
std::string GimbalUdpController::last_error() const { return impl_->last_error(); }
