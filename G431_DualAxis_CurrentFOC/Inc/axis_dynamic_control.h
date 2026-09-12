#ifndef AXIS_DYNAMIC_CONTROL_H
#define AXIS_DYNAMIC_CONTROL_H

#include <stdbool.h>
#include <math.h>

/*
 * Shared mechanical controller for both axes.
 *
 * Plant model:
 *     speed_dot = iq / q + disturbance
 *     q = J / Kt  [A/(rad/s^2)]
 *
 * Control path:
 *     position P -> speed PI -> iq command -> verified current PI
 *
 * q is identified only from correlated motion data and continuously sets the
 * speed-PI gains. The ESO is diagnostic only: its disturbance estimate is
 * reported but is never added to iq. The speed-PI integrator is therefore the
 * controller's only DC load-memory state.
 */

typedef struct
{
    float position_gain_per_s;
    float speed_reference_limit_rad_s;
    float speed_bandwidth_hz;
    float speed_damping_ratio;
    float speed_integral_limit_a;
    float speed_antiwindup_gain_per_s;
    float iq_command_limit_a;

    float observer_bandwidth_hz;
    float observer_disturbance_current_limit_a;

    float inverse_plant_initial;
    float inverse_plant_minimum;
    float inverse_plant_maximum;
    float gain_estimator_lowpass_hz;
    float gain_window_minimum_s;
    float gain_window_maximum_s;
    float gain_return_delay_s;
    float gain_cooldown_s;
    float gain_minimum_speed_rad_s;
    float gain_continue_minimum_speed_rad_s;
    float gain_minimum_rms_acceleration_rad_s2;
    float gain_minimum_rms_current_a;
    float gain_minimum_correlation;
    float gain_maximum_step_fraction;
    float gain_minimum_position_error_rad;
    float gain_maximum_innovation_rad_s;
} AxisDynamicConfig;

typedef struct
{
    bool initialized;
    bool observer_limited;
    bool gain_adaptation_active;
    bool gain_limited;
    bool gain_window_active;

    float eso_speed_rad_s;
    float disturbance_acceleration_rad_s2;
    float speed_integral_current_a;
    float inverse_plant_a_per_rad_s2;
    float model_current_lowpass_a;
    float model_acceleration_lowpass_rad_s2;

    float gain_sum_acceleration_squared;
    float gain_sum_acceleration_current;
    float gain_sum_current_squared;
    float gain_window_time_s;
    float gain_reliable_time_s;
    float gain_cooldown_remaining_s;
    float gain_update_latch_remaining_s;
    float gain_candidate_a_per_rad_s2;
    float gain_correlation;
} AxisDynamicState;

typedef struct
{
    float dt_s;
    bool trajectory_active;
    bool saturated_previous;
    float position_error_rad;
    float trajectory_speed_rad_s;
    float trajectory_acceleration_rad_s2;
    float measured_speed_rad_s;
    float measured_acceleration_rad_s2;
    float measured_iq_a;
} AxisDynamicInput;

typedef struct
{
    bool observer_limited;
    bool gain_adaptation_active;
    bool gain_limited;
    bool gain_window_active;

    float position_gain_per_s;
    float speed_bandwidth_hz;
    float observer_bandwidth_hz;
    float speed_reference_rad_s;
    float speed_error_rad_s;
    float speed_kp_a_per_rad_s;
    float speed_ki_a_per_rad;
    float speed_proportional_current_a;
    float speed_integral_current_a;
    float eso_speed_rad_s;
    float observer_innovation_rad_s;
    float disturbance_acceleration_rad_s2;
    float raw_disturbance_current_a;
    float iq_unsaturated_a;
    float iq_command_a;

    float inverse_plant_a_per_rad_s2;
    float plant_gain_rad_s2_per_a;
    float gain_excitation_rad_s2;
    float gain_residual_a;
    float gain_window_time_s;
    float gain_candidate_a_per_rad_s2;
    float gain_correlation;
} AxisDynamicOutput;

static inline float AxisDynamic_Clamp(float value,
                                      float minimum,
                                      float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static inline float AxisDynamic_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static inline void AxisDynamic_ClearGainWindow(AxisDynamicState *state)
{
    state->gain_window_active = false;
    state->gain_sum_acceleration_squared = 0.0f;
    state->gain_sum_acceleration_current = 0.0f;
    state->gain_sum_current_squared = 0.0f;
    state->gain_window_time_s = 0.0f;
}

static inline void AxisDynamic_Reset(AxisDynamicState *state,
                                     const AxisDynamicConfig *config,
                                     float measured_speed_rad_s,
                                     float measured_iq_a,
                                     float measured_acceleration_rad_s2)
{
    state->initialized = true;
    state->observer_limited = false;
    state->gain_adaptation_active = false;
    state->gain_limited = false;
    state->gain_window_active = false;
    state->eso_speed_rad_s = measured_speed_rad_s;
    state->disturbance_acceleration_rad_s2 = 0.0f;
    state->speed_integral_current_a = 0.0f;
    state->inverse_plant_a_per_rad_s2 = AxisDynamic_Clamp(
        config->inverse_plant_initial,
        config->inverse_plant_minimum,
        config->inverse_plant_maximum);
    state->model_current_lowpass_a = measured_iq_a;
    state->model_acceleration_lowpass_rad_s2 =
        measured_acceleration_rad_s2;
    state->gain_sum_acceleration_squared = 0.0f;
    state->gain_sum_acceleration_current = 0.0f;
    state->gain_sum_current_squared = 0.0f;
    state->gain_window_time_s = 0.0f;
    state->gain_reliable_time_s = 0.0f;
    state->gain_cooldown_remaining_s = 0.0f;
    state->gain_update_latch_remaining_s = 0.0f;
    state->gain_candidate_a_per_rad_s2 =
        state->inverse_plant_a_per_rad_s2;
    state->gain_correlation = 0.0f;
}

static inline void AxisDynamic_Update(AxisDynamicState *state,
                                      const AxisDynamicConfig *config,
                                      const AxisDynamicInput *input,
                                      AxisDynamicOutput *output)
{
    const float two_pi = 6.28318530718f;
    const float dt = AxisDynamic_Clamp(input->dt_s, 0.0002f, 0.0020f);

    if (!state->initialized)
    {
        AxisDynamic_Reset(state,
                          config,
                          input->measured_speed_rad_s,
                          input->measured_iq_a,
                          input->measured_acceleration_rad_s2);
    }

    if (state->gain_cooldown_remaining_s > 0.0f)
    {
        state->gain_cooldown_remaining_s -= dt;
        if (state->gain_cooldown_remaining_s < 0.0f)
            state->gain_cooldown_remaining_s = 0.0f;
    }
    if (state->gain_update_latch_remaining_s > 0.0f)
    {
        state->gain_update_latch_remaining_s -= dt;
        if (state->gain_update_latch_remaining_s < 0.0f)
            state->gain_update_latch_remaining_s = 0.0f;
    }

    /* Diagnostic ESO. Its output is deliberately excluded from iq. */
    const float observer_omega = two_pi * config->observer_bandwidth_hz;
    const float beta1 = 2.0f * observer_omega;
    const float beta2 = observer_omega * observer_omega;
    const float innovation =
        state->eso_speed_rad_s - input->measured_speed_rad_s;
    const float q_before_update = state->inverse_plant_a_per_rad_s2;

    state->eso_speed_rad_s += dt *
        (state->disturbance_acceleration_rad_s2 +
         input->measured_iq_a / q_before_update - beta1 * innovation);
    state->disturbance_acceleration_rad_s2 +=
        dt * (-beta2 * innovation);

    float raw_disturbance_current = -q_before_update *
        state->disturbance_acceleration_rad_s2;
    const float raw_limited = AxisDynamic_Clamp(
        raw_disturbance_current,
        -config->observer_disturbance_current_limit_a,
        config->observer_disturbance_current_limit_a);
    state->observer_limited = raw_limited != raw_disturbance_current;
    raw_disturbance_current = raw_limited;
    state->disturbance_acceleration_rad_s2 =
        -raw_disturbance_current / q_before_update;

    /* q=J/Kt is fitted from high-pass current and acceleration over a complete
     * correlated return-motion window. Static load current is filtered out. */
    const float estimator_x = two_pi *
        config->gain_estimator_lowpass_hz * dt;
    const float estimator_alpha = estimator_x / (1.0f + estimator_x);
    state->model_current_lowpass_a += estimator_alpha *
        (input->measured_iq_a - state->model_current_lowpass_a);
    state->model_acceleration_lowpass_rad_s2 += estimator_alpha *
        (input->measured_acceleration_rad_s2 -
         state->model_acceleration_lowpass_rad_s2);

    const float excitation = input->measured_acceleration_rad_s2 -
        state->model_acceleration_lowpass_rad_s2;
    const float dynamic_current = input->measured_iq_a -
        state->model_current_lowpass_a;
    const float gain_residual = dynamic_current -
        state->inverse_plant_a_per_rad_s2 * excitation;
    const bool returning_to_target =
        input->position_error_rad * input->measured_speed_rad_s > 0.0f;
    const bool start_motion =
        (input->trajectory_active || returning_to_target) &&
        AxisDynamic_Abs(input->position_error_rad) >=
            config->gain_minimum_position_error_rad &&
        AxisDynamic_Abs(input->measured_speed_rad_s) >=
            config->gain_minimum_speed_rad_s;
    const bool continue_motion =
        (input->trajectory_active || returning_to_target) &&
        AxisDynamic_Abs(input->measured_speed_rad_s) >=
            config->gain_continue_minimum_speed_rad_s;
    const bool quality_sample =
        !input->saturated_previous &&
        AxisDynamic_Abs(innovation) <=
            config->gain_maximum_innovation_rad_s &&
        state->gain_cooldown_remaining_s <= 0.0f;

    if (!state->gain_window_active)
    {
        if (start_motion && quality_sample)
            state->gain_reliable_time_s += dt;
        else
            state->gain_reliable_time_s = 0.0f;

        if (state->gain_reliable_time_s >= config->gain_return_delay_s)
        {
            AxisDynamic_ClearGainWindow(state);
            state->gain_window_active = true;
        }
    }

    if (state->gain_window_active && continue_motion && quality_sample)
    {
        state->gain_sum_acceleration_squared +=
            excitation * excitation * dt;
        state->gain_sum_acceleration_current +=
            excitation * dynamic_current * dt;
        state->gain_sum_current_squared +=
            dynamic_current * dynamic_current * dt;
        state->gain_window_time_s += dt;
    }

    const bool close_gain_window = state->gain_window_active &&
        (!continue_motion || !quality_sample ||
         state->gain_window_time_s >= config->gain_window_maximum_s);

    state->gain_limited = false;
    if (close_gain_window)
    {
        const float window_time = state->gain_window_time_s;
        const float acceleration_energy =
            state->gain_sum_acceleration_squared;
        const float current_energy = state->gain_sum_current_squared;
        const float cross = state->gain_sum_acceleration_current;
        float correlation = 0.0f;
        if (acceleration_energy > 0.0f && current_energy > 0.0f)
        {
            correlation = AxisDynamic_Abs(cross) /
                sqrtf(acceleration_energy * current_energy);
        }
        correlation = AxisDynamic_Clamp(correlation, 0.0f, 1.0f);
        state->gain_correlation = correlation;

        const bool enough_time =
            window_time >= config->gain_window_minimum_s;
        const bool enough_acceleration = acceleration_energy >=
            config->gain_minimum_rms_acceleration_rad_s2 *
            config->gain_minimum_rms_acceleration_rad_s2 * window_time;
        const bool enough_current = current_energy >=
            config->gain_minimum_rms_current_a *
            config->gain_minimum_rms_current_a * window_time;
        const bool coherent = cross > 0.0f &&
            correlation >= config->gain_minimum_correlation;

        if (enough_time && enough_acceleration &&
            enough_current && coherent)
        {
            const float raw_candidate = cross / acceleration_energy;
            const float current_q = state->inverse_plant_a_per_rad_s2;
            const float maximum_step =
                config->gain_maximum_step_fraction * current_q;
            float new_q = AxisDynamic_Clamp(raw_candidate,
                                             current_q - maximum_step,
                                             current_q + maximum_step);
            new_q = AxisDynamic_Clamp(new_q,
                                      config->inverse_plant_minimum,
                                      config->inverse_plant_maximum);
            state->gain_candidate_a_per_rad_s2 = raw_candidate;
            state->gain_limited = new_q != raw_candidate;
            if (new_q != current_q)
            {
                state->inverse_plant_a_per_rad_s2 = new_q;
                state->disturbance_acceleration_rad_s2 =
                    -raw_disturbance_current / new_q;
                state->gain_update_latch_remaining_s = 0.10f;
            }
        }
        state->gain_cooldown_remaining_s = config->gain_cooldown_s;
        state->gain_reliable_time_s = 0.0f;
        AxisDynamic_ClearGainWindow(state);
    }
    state->gain_adaptation_active =
        state->gain_update_latch_remaining_s > 0.0f;

    const float speed_omega = two_pi * config->speed_bandwidth_hz;
    const float q = state->inverse_plant_a_per_rad_s2;
    const float speed_kp =
        2.0f * config->speed_damping_ratio * speed_omega * q;
    const float speed_ki = speed_omega * speed_omega * q;
    float speed_reference = input->trajectory_speed_rad_s +
        config->position_gain_per_s * input->position_error_rad;
    speed_reference = AxisDynamic_Clamp(
        speed_reference,
        -config->speed_reference_limit_rad_s,
        config->speed_reference_limit_rad_s);
    const float speed_error =
        speed_reference - input->measured_speed_rad_s;
    const float proportional_current = speed_kp * speed_error;

    float iq_unsaturated = proportional_current +
        state->speed_integral_current_a;
    float iq_command = AxisDynamic_Clamp(
        iq_unsaturated,
        -config->iq_command_limit_a,
        config->iq_command_limit_a);
    state->speed_integral_current_a += dt *
        (speed_ki * speed_error +
         config->speed_antiwindup_gain_per_s *
            (iq_command - iq_unsaturated));
    state->speed_integral_current_a = AxisDynamic_Clamp(
        state->speed_integral_current_a,
        -config->speed_integral_limit_a,
        config->speed_integral_limit_a);
    iq_unsaturated = proportional_current +
        state->speed_integral_current_a;
    iq_command = AxisDynamic_Clamp(
        iq_unsaturated,
        -config->iq_command_limit_a,
        config->iq_command_limit_a);

    output->observer_limited = state->observer_limited;
    output->gain_adaptation_active = state->gain_adaptation_active;
    output->gain_limited = state->gain_limited;
    output->gain_window_active = state->gain_window_active;
    output->position_gain_per_s = config->position_gain_per_s;
    output->speed_bandwidth_hz = config->speed_bandwidth_hz;
    output->observer_bandwidth_hz = config->observer_bandwidth_hz;
    output->speed_reference_rad_s = speed_reference;
    output->speed_error_rad_s = speed_error;
    output->speed_kp_a_per_rad_s = speed_kp;
    output->speed_ki_a_per_rad = speed_ki;
    output->speed_proportional_current_a = proportional_current;
    output->speed_integral_current_a =
        state->speed_integral_current_a;
    output->eso_speed_rad_s = state->eso_speed_rad_s;
    output->observer_innovation_rad_s = innovation;
    output->disturbance_acceleration_rad_s2 =
        state->disturbance_acceleration_rad_s2;
    output->raw_disturbance_current_a = raw_disturbance_current;
    output->iq_unsaturated_a = iq_unsaturated;
    output->iq_command_a = iq_command;
    output->inverse_plant_a_per_rad_s2 =
        state->inverse_plant_a_per_rad_s2;
    output->plant_gain_rad_s2_per_a =
        1.0f / state->inverse_plant_a_per_rad_s2;
    output->gain_excitation_rad_s2 = excitation;
    output->gain_residual_a = gain_residual;
    output->gain_window_time_s = state->gain_window_time_s;
    output->gain_candidate_a_per_rad_s2 =
        state->gain_candidate_a_per_rad_s2;
    output->gain_correlation = state->gain_correlation;
}

#endif
