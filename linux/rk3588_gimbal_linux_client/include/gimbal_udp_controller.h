#pragma once

#include <cstdint>
#include <memory>
#include <string>

enum class GimbalAckResult : int32_t {
    Ok = 0,
    BadLength = -1,
    BadMagic = -2,
    BadVersion = -3,
    BadType = -4,
    BadCommand = -5,
    StaleSequence = -6,
    Unknown = INT32_MIN
};

struct GimbalTelemetrySnapshot {
    bool valid = false;
    bool telemetry_fresh = false;
    bool command_link_ok = false;
    bool remote_active = false;
    bool watchdog_expired = false;
    uint32_t system_flags = 0;
    uint32_t telemetry_sequence = 0;
    uint32_t last_command_age_ms = UINT32_MAX;
    uint32_t watchdog_trip_count = 0;
    double m1_position_deg = 0.0;
    double m2_position_deg = 0.0;
    double m1_target_deg = 0.0;
    double m2_target_deg = 0.0;
    uint64_t telemetry_received = 0;
    uint64_t telemetry_lost = 0;
    uint64_t telemetry_stale = 0;
    uint64_t telemetry_invalid = 0;
    uint64_t telemetry_age_ms = UINT64_MAX;
    uint64_t ack_received = 0;
    uint64_t ack_invalid = 0;
    uint64_t ack_age_ms = UINT64_MAX;
    GimbalAckResult last_ack_result = GimbalAckResult::Unknown;
    uint32_t last_acked_command_sequence = 0;
    GimbalAckResult last_control_ack_result = GimbalAckResult::Unknown;
    uint32_t last_control_sequence = 0;
    uint64_t last_control_ack_age_ms = UINT64_MAX;
};

class GimbalUdpController {
public:
    GimbalUdpController(const std::string& bind_ip = "192.168.10.1",
                        const std::string& stm32_ip = "192.168.10.2",
                        uint16_t command_port = 5000,
                        uint16_t telemetry_port = 5001);
    ~GimbalUdpController();

    GimbalUdpController(const GimbalUdpController&) = delete;
    GimbalUdpController& operator=(const GimbalUdpController&) = delete;

    bool start();
    void stop();
    bool running() const;

    // These functions only update a latest-value mailbox. The worker thread is
    // the sole owner of both UDP sockets and performs all actual send/receive.
    bool set_targets(double m1_deg, double m2_deg);
    void hold();
    void disarm();

    GimbalTelemetrySnapshot telemetry() const;
    std::string last_error() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
