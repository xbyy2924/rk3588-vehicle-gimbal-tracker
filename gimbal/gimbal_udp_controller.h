#pragma once

#include <cstdint>
#include <memory>
#include <string>

struct GimbalTelemetrySnapshot {
    bool valid = false;
    bool remote_active = false;
    bool watchdog_expired = false;
    double m1_position_deg = 0.0;
    double m2_position_deg = 0.0;
    double m1_target_deg = 0.0;
    double m2_target_deg = 0.0;
    uint64_t received_count = 0;
    uint64_t age_ms = UINT64_MAX;
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

    void set_targets(double m1_deg, double m2_deg);
    void hold();
    GimbalTelemetrySnapshot telemetry() const;
    std::string last_error() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
