#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "gimbal_control_state.h"
#include "gimbal_udp_controller.h"

class GimbalCommandConsole {
public:
    GimbalCommandConsole(GimbalControlState& control,
                         GimbalUdpController& udp,
                         std::atomic<bool>& application_running);
    ~GimbalCommandConsole();

    GimbalCommandConsole(const GimbalCommandConsole&) = delete;
    GimbalCommandConsole& operator=(const GimbalCommandConsole&) = delete;

    void start();
    void stop();
    static void print_help();

private:
    void loop();
    void execute(const std::string& line);
    void print_status() const;
    static void print_classes();
    static void print_prompt();

    GimbalControlState& control_;
    GimbalUdpController& udp_;
    std::atomic<bool>& application_running_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};
