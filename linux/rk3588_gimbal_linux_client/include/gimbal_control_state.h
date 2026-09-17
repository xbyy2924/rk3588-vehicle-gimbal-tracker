#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include "coco_classes.h"

enum class GimbalMode : uint8_t { Hold, Tracking, Manual };

inline const char* gimbal_mode_name(GimbalMode mode)
{
    switch (mode) {
    case GimbalMode::Tracking: return "TRACK";
    case GimbalMode::Manual: return "MANUAL";
    default: return "HOLD";
    }
}

struct GimbalControlConfig {
    double m1_min_deg = -70.0;
    double m1_max_deg = 55.0;
    double m2_min_deg = -110.0;
    double m2_max_deg = 110.0;
    double m1_tracking_speed_dps = 40.0;
    double m2_tracking_speed_dps = 50.0;
};

struct GimbalControlSnapshot {
    GimbalMode mode = GimbalMode::Hold;
    GimbalControlConfig config;
    int target_class_id = 2;
    double manual_m1_deg = 0.0;
    double manual_m2_deg = 0.0;
    uint64_t revision = 0;
};

class GimbalControlState {
public:
    GimbalControlSnapshot snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    void track()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.mode = GimbalMode::Tracking;
        ++state_.revision;
    }

    bool track_class(int class_id)
    {
        if (class_id < 0 || class_id >= kCocoClassCount) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        state_.target_class_id = class_id;
        state_.mode = GimbalMode::Tracking;
        ++state_.revision;
        return true;
    }

    bool select_target_class(int class_id)
    {
        if (class_id < 0 || class_id >= kCocoClassCount) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        state_.target_class_id = class_id;
        ++state_.revision;
        return true;
    }

    void hold()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.mode = GimbalMode::Hold;
        ++state_.revision;
    }

    bool manual(double m1_deg, double m2_deg)
    {
        if (!std::isfinite(m1_deg) || !std::isfinite(m2_deg)) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        state_.manual_m1_deg = std::clamp(m1_deg, state_.config.m1_min_deg, state_.config.m1_max_deg);
        state_.manual_m2_deg = std::clamp(m2_deg, state_.config.m2_min_deg, state_.config.m2_max_deg);
        state_.mode = GimbalMode::Manual;
        ++state_.revision;
        return true;
    }

    bool set_manual_axis(unsigned axis, double angle_deg, double other_axis_deg)
    {
        return axis == 1U ? manual(angle_deg, other_axis_deg) :
               axis == 2U ? manual(other_axis_deg, angle_deg) : false;
    }

    bool set_tracking_speed(unsigned axis, double speed_dps)
    {
        if ((axis != 1U && axis != 2U) || !std::isfinite(speed_dps) || speed_dps <= 0.0) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (axis == 1U) state_.config.m1_tracking_speed_dps = speed_dps;
        else state_.config.m2_tracking_speed_dps = speed_dps;
        ++state_.revision;
        return true;
    }

    bool set_tracking_speed(double speed_dps)
    {
        if (!std::isfinite(speed_dps) || speed_dps <= 0.0) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        state_.config.m1_tracking_speed_dps = speed_dps;
        state_.config.m2_tracking_speed_dps = speed_dps;
        ++state_.revision;
        return true;
    }

    bool set_range(unsigned axis, double minimum_deg, double maximum_deg)
    {
        if ((axis != 1U && axis != 2U) || !std::isfinite(minimum_deg) ||
            !std::isfinite(maximum_deg) || minimum_deg >= maximum_deg) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (axis == 1U) {
            state_.config.m1_min_deg = minimum_deg;
            state_.config.m1_max_deg = maximum_deg;
            state_.manual_m1_deg = std::clamp(state_.manual_m1_deg, minimum_deg, maximum_deg);
        } else {
            state_.config.m2_min_deg = minimum_deg;
            state_.config.m2_max_deg = maximum_deg;
            state_.manual_m2_deg = std::clamp(state_.manual_m2_deg, minimum_deg, maximum_deg);
        }
        ++state_.revision;
        return true;
    }

private:
    mutable std::mutex mutex_;
    GimbalControlSnapshot state_;
};
