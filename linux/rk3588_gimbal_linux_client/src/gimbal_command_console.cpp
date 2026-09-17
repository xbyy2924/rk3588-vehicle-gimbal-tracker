#include "gimbal_command_console.h"
#include "coco_classes.h"

#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

bool axis_from_text(const std::string& text, unsigned& axis)
{
    if (text == "m1" || text == "M1" || text == "1") { axis = 1U; return true; }
    if (text == "m2" || text == "M2" || text == "2") { axis = 2U; return true; }
    return false;
}

}  // namespace

GimbalCommandConsole::GimbalCommandConsole(GimbalControlState& control,
                                           GimbalUdpController& udp,
                                           std::atomic<bool>& application_running)
    : control_(control), udp_(udp), application_running_(application_running) {}

GimbalCommandConsole::~GimbalCommandConsole() { stop(); }

void GimbalCommandConsole::start()
{
    if (running_.exchange(true, std::memory_order_acq_rel)) return;
    thread_ = std::thread(&GimbalCommandConsole::loop, this);
}

void GimbalCommandConsole::stop()
{
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
}

void GimbalCommandConsole::print_help()
{
    std::fprintf(stderr,
        "\nCommands:\n"
        "  mode track CLASS          track a COCO class, e.g. car or person\n"
        "  mode track                track the currently selected class\n"
        "  mode hold                 hold current position\n"
        "  mode manual M1 M2         enter manual angle mode\n"
        "  track on                  track the currently selected class\n"
        "  track CLASS               select a class and start tracking\n"
        "  track off                leave tracking and hold current position\n"
        "  speed DEG_S              set both tracking speed limits\n"
        "  speed m1 DEG_S           set M1 tracking speed limit\n"
        "  speed m2 DEG_S           set M2 tracking speed limit\n"
        "  range m1 MIN MAX         set Linux-side M1 angle range\n"
        "  range m2 MIN MAX         set Linux-side M2 angle range\n"
        "  manual M1_DEG M2_DEG     enter manual angle mode\n"
        "  m1 DEG                   manual M1, keep current M2 target\n"
        "  m2 DEG                   manual M2, keep current M1 target\n"
        "  hold                      hold current position\n"
        "  disarm                    hold and disable remote-control state\n"
        "  status                    show mode, limits and UDP health\n"
        "  classes                   list all supported COCO classes\n"
        "  help                      show this help\n"
        "  exit                      stop all threads and exit\n\n");
}

void GimbalCommandConsole::print_classes()
{
    std::fprintf(stderr, "\nCOCO classes (ID:name):\n");
    for (int i = 0; i < kCocoClassCount; ++i) {
        std::fprintf(stderr, "%2d:%-18s%s", i, coco_class_name(i), (i + 1) % 4 == 0 ? "\n" : "  ");
    }
    if (kCocoClassCount % 4 != 0) std::fprintf(stderr, "\n");
    std::fprintf(stderr, "Use spaces, '_' or '-', for example: mode track traffic_light\n\n");
}

void GimbalCommandConsole::print_prompt()
{
    std::fprintf(stderr, "gimbal> ");
    std::fflush(stderr);
}

void GimbalCommandConsole::loop()
{
    print_help();
    print_prompt();
    std::string pending;
    while (running_.load(std::memory_order_acquire) &&
           application_running_.load(std::memory_order_acquire)) {
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const int result = poll(&descriptor, 1, 100);
        if (result < 0) {
            if (errno == EINTR) continue;
            std::fprintf(stderr, "console poll failed\n");
            return;
        }
        if (result == 0 || (descriptor.revents & POLLIN) == 0) continue;
        std::string line;
        if (!std::getline(std::cin, line)) return;
        execute(line);
        if (running_.load(std::memory_order_acquire) &&
            application_running_.load(std::memory_order_acquire)) {
            print_prompt();
        }
    }
}

void GimbalCommandConsole::execute(const std::string& line)
{
    std::istringstream input(line);
    std::string command;
    input >> command;
    if (command.empty()) return;
    command = normalize_coco_class(command);

    if (command == "help") {
        print_help();
        return;
    }
    if (command == "status") {
        print_status();
        return;
    }
    if (command == "classes") {
        print_classes();
        return;
    }
    if (command == "mode") {
        std::string mode;
        input >> mode;
        mode = normalize_coco_class(mode);
        if (mode == "track") {
            std::string class_text;
            std::getline(input, class_text);
            class_text = normalize_coco_class(class_text);
            if (!class_text.empty()) {
                const int class_id = coco_class_id_from_text(class_text);
                if (class_id < 0 || !control_.track_class(class_id)) {
                    std::fprintf(stderr, "unknown COCO class: %s (use 'classes')\n", class_text.c_str());
                    return;
                }
            } else {
                control_.track();
            }
            const GimbalControlSnapshot state = control_.snapshot();
            std::fprintf(stderr, "[GIMBAL] mode=TRACK target=%s(%d), searching for a new target\n",
                         coco_class_name(state.target_class_id), state.target_class_id);
        } else if (mode == "hold") {
            control_.hold();
            std::fprintf(stderr, "[GIMBAL] mode=HOLD\n");
        } else if (mode == "manual") {
            double m1 = 0.0;
            double m2 = 0.0;
            if (!(input >> m1 >> m2) || !control_.manual(m1, m2)) {
                std::fprintf(stderr, "usage: mode manual M1_DEG M2_DEG\n");
                return;
            }
            const GimbalControlSnapshot state = control_.snapshot();
            std::fprintf(stderr, "[GIMBAL] mode=MANUAL M1=%.2f M2=%.2f\n",
                         state.manual_m1_deg, state.manual_m2_deg);
        } else {
            std::fprintf(stderr, "usage: mode track [CLASS] | mode hold | mode manual M1 M2\n");
        }
        return;
    }
    if (command == "track") {
        std::string action;
        input >> action;
        action = normalize_coco_class(action);
        if (action == "on" || action.empty()) {
            control_.track();
            const GimbalControlSnapshot state = control_.snapshot();
            std::fprintf(stderr, "[GIMBAL] mode=TRACK target=%s(%d)\n",
                         coco_class_name(state.target_class_id), state.target_class_id);
        } else if (action == "off") {
            control_.hold();
            std::fprintf(stderr, "[GIMBAL] mode=HOLD\n");
        } else {
            std::string remainder;
            std::getline(input, remainder);
            const std::string class_text = normalize_coco_class(action + " " + remainder);
            const int class_id = coco_class_id_from_text(class_text);
            if (class_id < 0 || !control_.track_class(class_id)) {
                std::fprintf(stderr, "unknown COCO class: %s (use 'classes')\n", class_text.c_str());
                return;
            }
            std::fprintf(stderr, "[GIMBAL] mode=TRACK target=%s(%d), searching for a new target\n",
                         coco_class_name(class_id), class_id);
        }
        return;
    }
    if (command == "hold") {
        control_.hold();
        std::fprintf(stderr, "[GIMBAL] mode=HOLD\n");
        return;
    }
    if (command == "disarm") {
        control_.hold();
        udp_.disarm();
        std::fprintf(stderr, "[GIMBAL] mode=HOLD, remote disarmed\n");
        return;
    }
    if (command == "speed") {
        std::string first;
        input >> first;
        if (first.empty()) { std::fprintf(stderr, "usage: speed [m1|m2] DEG_S\n"); return; }
        unsigned axis = 0;
        if (axis_from_text(first, axis)) {
            double value;
            if (!(input >> value) || !control_.set_tracking_speed(axis, value)) {
                std::fprintf(stderr, "invalid speed\n");
                return;
            }
        } else {
            try {
                size_t used = 0;
                const double value = std::stod(first, &used);
                if (used != first.size() || !control_.set_tracking_speed(value)) throw std::invalid_argument("speed");
            } catch (...) {
                std::fprintf(stderr, "invalid speed\n");
                return;
            }
        }
        print_status();
        return;
    }
    if (command == "range") {
        std::string axis_text;
        unsigned axis = 0;
        double minimum = 0.0;
        double maximum = 0.0;
        if (!(input >> axis_text >> minimum >> maximum) || !axis_from_text(axis_text, axis) ||
            !control_.set_range(axis, minimum, maximum)) {
            std::fprintf(stderr, "usage: range m1|m2 MIN MAX, MIN must be less than MAX\n");
            return;
        }
        print_status();
        return;
    }
    if (command == "manual") {
        double m1 = 0.0;
        double m2 = 0.0;
        if (!(input >> m1 >> m2) || !control_.manual(m1, m2)) {
            std::fprintf(stderr, "usage: manual M1_DEG M2_DEG\n");
            return;
        }
        const GimbalControlSnapshot state = control_.snapshot();
        std::fprintf(stderr, "[GIMBAL] mode=MANUAL M1=%.2f M2=%.2f\n",
                     state.manual_m1_deg, state.manual_m2_deg);
        return;
    }
    if (command == "m1" || command == "m2") {
        double angle = 0.0;
        if (!(input >> angle)) { std::fprintf(stderr, "usage: %s DEG\n", command.c_str()); return; }
        const GimbalTelemetrySnapshot telemetry = udp_.telemetry();
        const GimbalControlSnapshot state = control_.snapshot();
        if (state.mode != GimbalMode::Manual && !telemetry.telemetry_fresh) {
            std::fprintf(stderr,
                         "telemetry is unavailable; use 'manual M1 M2' to set both axes explicitly\n");
            return;
        }
        const double m1 = command == "m1" ? angle :
            (state.mode == GimbalMode::Manual ? state.manual_m1_deg : telemetry.m1_target_deg);
        const double m2 = command == "m2" ? angle :
            (state.mode == GimbalMode::Manual ? state.manual_m2_deg : telemetry.m2_target_deg);
        if (!control_.manual(m1, m2)) {
            std::fprintf(stderr, "invalid manual angle\n");
        } else {
            const GimbalControlSnapshot applied = control_.snapshot();
            std::fprintf(stderr, "[GIMBAL] mode=MANUAL M1=%.2f M2=%.2f\n",
                         applied.manual_m1_deg, applied.manual_m2_deg);
        }
        return;
    }
    if (command == "exit" || command == "quit") {
        control_.hold();
        udp_.disarm();
        application_running_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        return;
    }

    std::fprintf(stderr, "unknown command: %s (type 'help')\n", command.c_str());
}

void GimbalCommandConsole::print_status() const
{
    const GimbalControlSnapshot control = control_.snapshot();
    const GimbalTelemetrySnapshot udp = udp_.telemetry();
    std::fprintf(stderr,
        "[STATUS] mode=%s target=%s(%d) speed=(%.1f,%.1f)dps range=M1[%.1f,%.1f] M2[%.1f,%.1f]\n"
        "         telemetry=%s age=%llums seq=%u rx=%llu lost=%llu stale=%llu invalid=%llu\n"
        "         command=%s ack_age=%llums ack_result=%d remote=%u watchdog=%u trips=%u\n"
        "         M1 pos=%.3f target=%.3f | M2 pos=%.3f target=%.3f\n",
        gimbal_mode_name(control.mode),
        coco_class_name(control.target_class_id), control.target_class_id,
        control.config.m1_tracking_speed_dps, control.config.m2_tracking_speed_dps,
        control.config.m1_min_deg, control.config.m1_max_deg,
        control.config.m2_min_deg, control.config.m2_max_deg,
        udp.telemetry_fresh ? "OK" : "STALE",
        static_cast<unsigned long long>(udp.telemetry_age_ms), udp.telemetry_sequence,
        static_cast<unsigned long long>(udp.telemetry_received),
        static_cast<unsigned long long>(udp.telemetry_lost),
        static_cast<unsigned long long>(udp.telemetry_stale),
        static_cast<unsigned long long>(udp.telemetry_invalid),
        udp.command_link_ok ? "OK" : "STALE",
        static_cast<unsigned long long>(udp.ack_age_ms), static_cast<int>(udp.last_ack_result),
        udp.remote_active ? 1U : 0U, udp.watchdog_expired ? 1U : 0U,
        udp.watchdog_trip_count,
        udp.m1_position_deg, udp.m1_target_deg,
        udp.m2_position_deg, udp.m2_target_deg);
}
