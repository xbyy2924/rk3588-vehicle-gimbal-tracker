#include "current_foc.h"

#include "as5600.h"
#include "axis_dynamic_control.h"
#include "cordic.h"
#include "motor2_calibration.h"
#include "phase_current.h"
#include "tim.h"

#include <math.h>
#include <string.h>

/* Fixed motor and encoder calibration verified by ECAL2/CLID3. */
#define MOTOR_POLE_PAIRS                         7
#define ENCODER_DIRECTION                       -1
#define MECHANICAL_ZERO_RAW                   3916
#define ELECTRICAL_OFFSET_DEG               215.410f

#define ENCODER_TIMEOUT_MS                       10U
#define PWM_BUS_VOLTAGE_V                     12.0f
#define PWM_DUTY_MIN                           0.02f
#define PWM_DUTY_MAX                           0.98f
#define VOLTAGE_VECTOR_LIMIT_V                  4.0f

/* Verified 20 kHz current loop. */
#define CURRENT_LOOP_DT_S                    0.00005f
#define CURRENT_KP_V_PER_A                      5.50f
#define CURRENT_KI_V_PER_A_S                 18000.0f
#define CURRENT_AW_GAIN_PER_S                 1000.0f

/* Verified adaptive M/T velocity estimate for the 12-bit AS5600. */
#define SPEED_HISTORY_LENGTH                      17U
#define SPEED_MIN_TRAVEL_COUNTS                    2
#define SPEED_FILTER_2MS_HZ                     120.0f
#define SPEED_FILTER_4MS_HZ                      80.0f
#define SPEED_FILTER_8MS_HZ                      50.0f
#define SPEED_FILTER_16MS_HZ                     30.0f
#define ACCELERATION_FILTER_HZ                    8.0f

/* DUAL_CASCADE2 M1: firmer position response with extra release damping. */
#define POSITION_GAIN_PER_S                       11.0f
#define SPEED_REFERENCE_LIMIT_DPS               260.0f
#define SPEED_BANDWIDTH_HZ                         7.0f
#define SPEED_DAMPING_RATIO                        1.70f
#define SPEED_INTEGRAL_LIMIT_A                     0.30f
#define SPEED_ANTIWINDUP_GAIN_PER_S               30.0f
#define IQ_COMMAND_LIMIT_A                        0.70f
#define ESO_DIAGNOSTIC_BANDWIDTH_HZ                8.0f
#define OBSERVER_DISTURBANCE_CURRENT_LIMIT_A       0.50f

/* M1 q=J/Kt starts from CLID3.  Only a correlated motion window may change it. */
#define INVERSE_PLANT_INITIAL_A_PER_RAD_S2      0.00030f
#define INVERSE_PLANT_MINIMUM_A_PER_RAD_S2      0.00020f
#define INVERSE_PLANT_MAXIMUM_A_PER_RAD_S2      0.00060f
#define GAIN_ESTIMATOR_LOWPASS_HZ                  2.0f
#define GAIN_WINDOW_MINIMUM_S                       0.20f
#define GAIN_WINDOW_MAXIMUM_S                       0.70f
#define GAIN_RETURN_DELAY_S                         0.05f
#define GAIN_COOLDOWN_S                             2.00f
#define GAIN_MINIMUM_SPEED_DPS                     15.0f
#define GAIN_CONTINUE_MINIMUM_SPEED_DPS              5.0f
#define GAIN_MINIMUM_RMS_ACCEL_RAD_S2              15.0f
#define GAIN_MINIMUM_RMS_CURRENT_A                  0.010f
#define GAIN_MINIMUM_CORRELATION                    0.80f
#define GAIN_MAXIMUM_STEP_FRACTION                  0.05f
#define GAIN_MINIMUM_POSITION_ERROR_DEG              5.0f
#define GAIN_MAXIMUM_INNOVATION_DPS                 80.0f

/* Jerk-limited position command, relative to the captured start angle. */
#define TRAJECTORY_POSITION_RATE_PER_S            4.0f
#define TRAJECTORY_SPEED_RATE_PER_S              16.0f
#define TRAJECTORY_MAX_SPEED_DPS                 90.0f
#define TRAJECTORY_MAX_ACCEL_DPS2               600.0f
#define TRAJECTORY_MAX_JERK_DPS3               6000.0f
#define TRAJECTORY_SNAP_POSITION_DEG              0.01f
#define TRAJECTORY_SNAP_SPEED_DPS                 0.10f
#define TRAJECTORY_SNAP_ACCEL_DPS2                1.0f

#define USB_CONNECT_WAIT_MS                    5000U
#define SETTLED_CONFIRM_MS                      300U
#define SETTLED_POSITION_DEG                    0.25f
#define SETTLED_SPEED_DPS                       2.00f

/* Independent hardware and feedback faults still stop the motor. */
#define PHASE_CURRENT_LIMIT_A                    0.78f
#define ADC_RAIL_LOW_COUNT                        64U
#define ADC_RAIL_HIGH_COUNT                     4031U

#define PI_F                                  3.14159265359f
#define TWO_PI_F                              6.28318530718f
#define DEG_TO_RAD                            0.0174532925199f
#define RAD_TO_DEG                           57.2957795131f
#define COUNT_TO_RAD                         (TWO_PI_F / 4096.0f)
#define INV_SQRT3                             0.57735026919f
#define SQRT3_OVER_2                          0.86602540378f
#define Q31_SCALE                            2147483648.0f
#define Q31_TO_FLOAT                         (1.0f / Q31_SCALE)

static CurrentFOCData s_data;

/* Shared between the 1 kHz task and the 20 kHz ADC ISR. */
static volatile bool s_running;
static volatile float s_sin_e;
static volatile float s_cos_e;
static volatile float s_id_reference_a;
static volatile float s_iq_reference_a;
static volatile float s_target_position_deg;
static volatile uint32_t s_last_encoder_ms;

/* Current-loop integrators are updated only by the ADC ISR. */
static float s_id_integral_v;
static float s_iq_integral_v;

/* Motion-loop and observer states are updated on each encoder sample. */
static int32_t s_test_center_count;
static int32_t s_previous_encoder_count;
static uint32_t s_previous_encoder_ms;
static uint32_t s_processed_encoder_sample;
static int32_t s_speed_count_history[SPEED_HISTORY_LENGTH];
static uint32_t s_speed_time_history[SPEED_HISTORY_LENGTH];
static uint8_t s_speed_history_index;
static uint8_t s_speed_history_valid;
static bool s_speed_estimator_ready;
static float s_filtered_speed_rad_s;
static float s_previous_filtered_speed_rad_s;
static float s_filtered_acceleration_rad_s2;
static AxisDynamicState s_dynamic_state;
static float s_trajectory_position_deg;
static float s_trajectory_speed_dps;
static float s_trajectory_acceleration_dps2;
static uint32_t s_state_start_ms;
static uint32_t s_servo_start_ms;
static uint32_t s_settled_samples;

static const AxisDynamicConfig s_dynamic_config =
{
    .position_gain_per_s = POSITION_GAIN_PER_S,
    .speed_reference_limit_rad_s =
        SPEED_REFERENCE_LIMIT_DPS * DEG_TO_RAD,
    .speed_bandwidth_hz = SPEED_BANDWIDTH_HZ,
    .speed_damping_ratio = SPEED_DAMPING_RATIO,
    .speed_integral_limit_a = SPEED_INTEGRAL_LIMIT_A,
    .speed_antiwindup_gain_per_s = SPEED_ANTIWINDUP_GAIN_PER_S,
    .iq_command_limit_a = IQ_COMMAND_LIMIT_A,
    .observer_bandwidth_hz = ESO_DIAGNOSTIC_BANDWIDTH_HZ,
    .observer_disturbance_current_limit_a =
        OBSERVER_DISTURBANCE_CURRENT_LIMIT_A,
    .inverse_plant_initial = INVERSE_PLANT_INITIAL_A_PER_RAD_S2,
    .inverse_plant_minimum = INVERSE_PLANT_MINIMUM_A_PER_RAD_S2,
    .inverse_plant_maximum = INVERSE_PLANT_MAXIMUM_A_PER_RAD_S2,
    .gain_estimator_lowpass_hz = GAIN_ESTIMATOR_LOWPASS_HZ,
    .gain_window_minimum_s = GAIN_WINDOW_MINIMUM_S,
    .gain_window_maximum_s = GAIN_WINDOW_MAXIMUM_S,
    .gain_return_delay_s = GAIN_RETURN_DELAY_S,
    .gain_cooldown_s = GAIN_COOLDOWN_S,
    .gain_minimum_speed_rad_s = GAIN_MINIMUM_SPEED_DPS * DEG_TO_RAD,
    .gain_continue_minimum_speed_rad_s =
        GAIN_CONTINUE_MINIMUM_SPEED_DPS * DEG_TO_RAD,
    .gain_minimum_rms_acceleration_rad_s2 =
        GAIN_MINIMUM_RMS_ACCEL_RAD_S2,
    .gain_minimum_rms_current_a = GAIN_MINIMUM_RMS_CURRENT_A,
    .gain_minimum_correlation = GAIN_MINIMUM_CORRELATION,
    .gain_maximum_step_fraction = GAIN_MAXIMUM_STEP_FRACTION,
    .gain_minimum_position_error_rad =
        GAIN_MINIMUM_POSITION_ERROR_DEG * DEG_TO_RAD,
    .gain_maximum_innovation_rad_s =
        GAIN_MAXIMUM_INNOVATION_DPS * DEG_TO_RAD
};

static float clampf(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static float wrap_0_2pi(float angle)
{
    while (angle >= TWO_PI_F) angle -= TWO_PI_F;
    while (angle < 0.0f) angle += TWO_PI_F;
    return angle;
}

static float wrap_pm_pi(float angle)
{
    while (angle >= PI_F) angle -= TWO_PI_F;
    while (angle < -PI_F) angle += TWO_PI_F;
    return angle;
}

static uint32_t angle_to_mdeg(float angle)
{
    return (uint32_t)(wrap_0_2pi(angle) * (180000.0f / PI_F) + 0.5f);
}

static bool configure_cordic(void)
{
    CORDIC_ConfigTypeDef config = {0};
    config.Function = CORDIC_FUNCTION_COSINE;
    config.Precision = CORDIC_PRECISION_6CYCLES;
    config.Scale = CORDIC_SCALE_0;
    config.NbWrite = CORDIC_NBWRITE_1;
    config.NbRead = CORDIC_NBREAD_2;
    config.InSize = CORDIC_INSIZE_32BITS;
    config.OutSize = CORDIC_OUTSIZE_32BITS;
    return HAL_CORDIC_Configure(&hcordic, &config) == HAL_OK;
}

static void cordic_sin_cos(float angle, float *sine, float *cosine)
{
    const float normalized = wrap_pm_pi(angle) / PI_F;
    const int32_t input_q31 = (int32_t)(normalized * Q31_SCALE);
    CORDIC->WDATA = (uint32_t)input_q31;
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U)
    {
    }
    const int32_t cosine_q31 = (int32_t)CORDIC->RDATA;
    const int32_t sine_q31 = (int32_t)CORDIC->RDATA;
    *cosine = (float)cosine_q31 * Q31_TO_FLOAT;
    *sine = (float)sine_q31 * Q31_TO_FLOAT;
}

static void driver_enable(bool enable)
{
    HAL_GPIO_WritePin(M1_EN_GPIO_Port,
                      M1_EN_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t duty_to_ccr(float duty)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    duty = clampf(duty, PWM_DUTY_MIN, PWM_DUTY_MAX);
    uint32_t compare = (uint32_t)(duty * (float)(arr + 1U));
    if (compare > arr) compare = arr;
    return compare;
}

static void pwm_set(float duty_a, float duty_b, float duty_c)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_to_ccr(duty_a));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_to_ccr(duty_b));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_to_ccr(duty_c));
}

static void pwm_neutral(void)
{
    pwm_set(0.5f, 0.5f, 0.5f);
}

static void apply_alpha_beta(float alpha_voltage, float beta_voltage)
{
    float phase_a = alpha_voltage;
    float phase_b = -0.5f * alpha_voltage + SQRT3_OVER_2 * beta_voltage;
    float phase_c = -0.5f * alpha_voltage - SQRT3_OVER_2 * beta_voltage;
    float maximum = phase_a;
    float minimum = phase_a;
    if (phase_b > maximum) maximum = phase_b;
    if (phase_c > maximum) maximum = phase_c;
    if (phase_b < minimum) minimum = phase_b;
    if (phase_c < minimum) minimum = phase_c;
    const float common_mode = -0.5f * (maximum + minimum);
    phase_a += common_mode;
    phase_b += common_mode;
    phase_c += common_mode;
    pwm_set(0.5f + phase_a / PWM_BUS_VOLTAGE_V,
            0.5f + phase_b / PWM_BUS_VOLTAGE_V,
            0.5f + phase_c / PWM_BUS_VOLTAGE_V);
}

static bool encoder_valid(const AS5600Data *encoder)
{
    const uint32_t age_ms = HAL_GetTick() - encoder->last_update_ms;
    return encoder->device_ready &&
           encoder->sample_valid &&
           encoder->magnet_detected &&
           !encoder->magnet_too_strong &&
           age_ms <= ENCODER_TIMEOUT_MS;
}

static float motor_position_rad(const AS5600Data *encoder)
{
    const int32_t relative_count =
        encoder->total_count - s_test_center_count;
    return (float)(ENCODER_DIRECTION * relative_count) * COUNT_TO_RAD;
}

static float electrical_angle_from_encoder(const AS5600Data *encoder)
{
    return wrap_0_2pi(
        (float)(ENCODER_DIRECTION * MOTOR_POLE_PAIRS) *
            (float)encoder->total_count * COUNT_TO_RAD +
        ELECTRICAL_OFFSET_DEG * DEG_TO_RAD);
}

static void publish_electrical_angle(const AS5600Data *encoder)
{
    const float electrical_angle = electrical_angle_from_encoder(encoder);
    float sine;
    float cosine;
    cordic_sin_cos(electrical_angle, &sine, &cosine);
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_sin_e = sine;
    s_cos_e = cosine;
    s_data.electrical_mdeg = angle_to_mdeg(electrical_angle);
    if (primask == 0U) __enable_irq();
}

static void set_iq_reference(float iq_reference)
{
    iq_reference = clampf(iq_reference,
                          -IQ_COMMAND_LIMIT_A,
                          IQ_COMMAND_LIMIT_A);
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_running)
    {
        s_id_reference_a = 0.0f;
        s_iq_reference_a = iq_reference;
        s_data.iq_command_a = iq_reference;
    }
    if (primask == 0U) __enable_irq();
}

static void begin_state(uint8_t state)
{
    s_data.servo_state = state;
    s_data.state_elapsed_ms = 0U;
    s_state_start_ms = HAL_GetTick();
}

static void observer_reset(const AS5600Data *encoder)
{
    s_previous_encoder_count = encoder->total_count;
    s_previous_encoder_ms = encoder->last_update_ms;
    s_processed_encoder_sample = encoder->sample_count;
    s_last_encoder_ms = encoder->last_update_ms;
    for (uint32_t index = 0U; index < SPEED_HISTORY_LENGTH; ++index)
    {
        s_speed_count_history[index] = encoder->total_count;
        s_speed_time_history[index] = encoder->last_update_ms;
    }
    s_speed_history_index = 0U;
    s_speed_history_valid = 1U;
    s_speed_estimator_ready = false;
    s_filtered_speed_rad_s = 0.0f;
    s_previous_filtered_speed_rad_s = 0.0f;
    s_filtered_acceleration_rad_s2 = 0.0f;
    s_data.raw_speed_dps = 0.0f;
    s_data.window_speed_dps = 0.0f;
    s_data.speed_window_ms = 0U;
    s_data.speed_dps = 0.0f;
    s_data.acceleration_dps2 = 0.0f;
}

static void observer_update(const AS5600Data *encoder)
{
    uint32_t elapsed_ms =
        encoder->last_update_ms - s_previous_encoder_ms;
    if (elapsed_ms == 0U) elapsed_ms = 1U;
    const float dt = (float)elapsed_ms * 0.001f;
    const int32_t count_delta =
        encoder->total_count - s_previous_encoder_count;

    s_previous_encoder_count = encoder->total_count;
    s_previous_encoder_ms = encoder->last_update_ms;
    s_processed_encoder_sample = encoder->sample_count;
    s_last_encoder_ms = encoder->last_update_ms;

    const float position = motor_position_rad(encoder);
    const float raw_speed =
        (float)(ENCODER_DIRECTION * count_delta) * COUNT_TO_RAD / dt;

    s_speed_history_index++;
    if (s_speed_history_index >= SPEED_HISTORY_LENGTH)
        s_speed_history_index = 0U;
    s_speed_count_history[s_speed_history_index] = encoder->total_count;
    s_speed_time_history[s_speed_history_index] = encoder->last_update_ms;
    if (s_speed_history_valid < SPEED_HISTORY_LENGTH)
        s_speed_history_valid++;

    float window_speed = 0.0f;
    uint32_t selected_window_ms = 0U;
    if (s_speed_history_valid >= 3U)
    {
        static const uint8_t candidate_intervals[] = {2U, 4U, 8U, 16U};
        uint8_t selected_intervals = 0U;
        int32_t window_count_delta = 0;

        for (uint32_t candidate = 0U;
             candidate < sizeof(candidate_intervals);
             ++candidate)
        {
            const uint8_t intervals = candidate_intervals[candidate];
            if (s_speed_history_valid <= intervals) continue;
            const uint8_t old_index = (uint8_t)(
                (s_speed_history_index + SPEED_HISTORY_LENGTH - intervals) %
                SPEED_HISTORY_LENGTH);
            const int32_t candidate_delta = encoder->total_count -
                s_speed_count_history[old_index];
            const int32_t abs_delta = candidate_delta >= 0 ?
                candidate_delta : -candidate_delta;
            if (abs_delta >= SPEED_MIN_TRAVEL_COUNTS || intervals == 16U)
            {
                selected_intervals = intervals;
                window_count_delta = candidate_delta;
                selected_window_ms = encoder->last_update_ms -
                    s_speed_time_history[old_index];
                break;
            }
        }

        if (selected_intervals == 0U)
        {
            selected_intervals = (uint8_t)(s_speed_history_valid - 1U);
            if (selected_intervals > 16U) selected_intervals = 16U;
            const uint8_t old_index = (uint8_t)(
                (s_speed_history_index + SPEED_HISTORY_LENGTH -
                 selected_intervals) % SPEED_HISTORY_LENGTH);
            window_count_delta = encoder->total_count -
                s_speed_count_history[old_index];
            selected_window_ms = encoder->last_update_ms -
                s_speed_time_history[old_index];
        }

        if (selected_window_ms == 0U)
            selected_window_ms = selected_intervals;
        window_speed =
            (float)(ENCODER_DIRECTION * window_count_delta) * COUNT_TO_RAD /
            ((float)selected_window_ms * 0.001f);

        float speed_filter_hz = SPEED_FILTER_16MS_HZ;
        if (selected_intervals <= 2U)
            speed_filter_hz = SPEED_FILTER_2MS_HZ;
        else if (selected_intervals <= 4U)
            speed_filter_hz = SPEED_FILTER_4MS_HZ;
        else if (selected_intervals <= 8U)
            speed_filter_hz = SPEED_FILTER_8MS_HZ;

        if (!s_speed_estimator_ready)
        {
            s_filtered_speed_rad_s = window_speed;
            s_previous_filtered_speed_rad_s = window_speed;
            s_filtered_acceleration_rad_s2 = 0.0f;
            s_speed_estimator_ready = true;
        }
        else
        {
            const float speed_x = TWO_PI_F * speed_filter_hz * dt;
            const float speed_alpha = speed_x / (1.0f + speed_x);
            s_filtered_speed_rad_s += speed_alpha *
                (window_speed - s_filtered_speed_rad_s);

            const float raw_acceleration =
                (s_filtered_speed_rad_s - s_previous_filtered_speed_rad_s) /
                dt;
            s_previous_filtered_speed_rad_s = s_filtered_speed_rad_s;
            const float acceleration_x =
                TWO_PI_F * ACCELERATION_FILTER_HZ * dt;
            const float acceleration_alpha =
                acceleration_x / (1.0f + acceleration_x);
            s_filtered_acceleration_rad_s2 += acceleration_alpha *
                (raw_acceleration - s_filtered_acceleration_rad_s2);
        }
    }

    publish_electrical_angle(encoder);
    s_data.encoder_raw = encoder->raw_angle;
    s_data.encoder_total_count = encoder->total_count;
    s_data.motor_position_deg = position * RAD_TO_DEG;
    s_data.raw_speed_dps = raw_speed * RAD_TO_DEG;
    s_data.window_speed_dps = window_speed * RAD_TO_DEG;
    s_data.speed_window_ms = selected_window_ms > 255U ?
        255U : (uint8_t)selected_window_ms;
    s_data.speed_dps = s_filtered_speed_rad_s * RAD_TO_DEG;
    s_data.acceleration_dps2 =
        s_filtered_acceleration_rad_s2 * RAD_TO_DEG;

    const float abs_position = fabsf(s_data.motor_position_deg);
    const float abs_speed = fabsf(s_data.speed_dps);
    if (abs_position > s_data.maximum_abs_position_deg)
        s_data.maximum_abs_position_deg = abs_position;
    if (abs_speed > s_data.maximum_abs_speed_dps)
        s_data.maximum_abs_speed_dps = abs_speed;
}

static void trajectory_reset(void)
{
    s_trajectory_position_deg = 0.0f;
    s_trajectory_speed_dps = 0.0f;
    s_trajectory_acceleration_dps2 = 0.0f;
    s_data.trajectory_position_deg = 0.0f;
    s_data.trajectory_speed_dps = 0.0f;
    s_data.trajectory_acceleration_dps2 = 0.0f;
    s_data.trajectory_active = false;
}

static void trajectory_update(float dt)
{
    const float target = s_target_position_deg;
    const float position_error = target - s_trajectory_position_deg;
    const float desired_speed = clampf(
        TRAJECTORY_POSITION_RATE_PER_S * position_error,
        -TRAJECTORY_MAX_SPEED_DPS,
        TRAJECTORY_MAX_SPEED_DPS);
    const float desired_acceleration = clampf(
        TRAJECTORY_SPEED_RATE_PER_S *
            (desired_speed - s_trajectory_speed_dps),
        -TRAJECTORY_MAX_ACCEL_DPS2,
        TRAJECTORY_MAX_ACCEL_DPS2);
    const float maximum_acceleration_step =
        TRAJECTORY_MAX_JERK_DPS3 * dt;

    s_trajectory_acceleration_dps2 += clampf(
        desired_acceleration - s_trajectory_acceleration_dps2,
        -maximum_acceleration_step,
        maximum_acceleration_step);
    s_trajectory_acceleration_dps2 = clampf(
        s_trajectory_acceleration_dps2,
        -TRAJECTORY_MAX_ACCEL_DPS2,
        TRAJECTORY_MAX_ACCEL_DPS2);
    s_trajectory_speed_dps += s_trajectory_acceleration_dps2 * dt;
    s_trajectory_speed_dps = clampf(s_trajectory_speed_dps,
                                    -TRAJECTORY_MAX_SPEED_DPS,
                                    TRAJECTORY_MAX_SPEED_DPS);
    s_trajectory_position_deg += s_trajectory_speed_dps * dt;

    if (fabsf(target - s_trajectory_position_deg) <=
            TRAJECTORY_SNAP_POSITION_DEG &&
        fabsf(s_trajectory_speed_dps) <= TRAJECTORY_SNAP_SPEED_DPS &&
        fabsf(s_trajectory_acceleration_dps2) <=
            TRAJECTORY_SNAP_ACCEL_DPS2)
    {
        s_trajectory_position_deg = target;
        s_trajectory_speed_dps = 0.0f;
        s_trajectory_acceleration_dps2 = 0.0f;
    }

    s_data.target_position_deg = target;
    s_data.trajectory_position_deg = s_trajectory_position_deg;
    s_data.trajectory_speed_dps = s_trajectory_speed_dps;
    s_data.trajectory_acceleration_dps2 =
        s_trajectory_acceleration_dps2;
    s_data.trajectory_active =
        fabsf(target - s_trajectory_position_deg) >
            TRAJECTORY_SNAP_POSITION_DEG ||
        fabsf(s_trajectory_speed_dps) > TRAJECTORY_SNAP_SPEED_DPS ||
        fabsf(s_trajectory_acceleration_dps2) >
            TRAJECTORY_SNAP_ACCEL_DPS2;
}

static void motion_control_update(float dt, uint32_t sample_delta)
{
    const float position_error_deg =
        s_trajectory_position_deg - s_data.motor_position_deg;
    AxisDynamicInput input =
    {
        .dt_s = dt,
        .trajectory_active = s_data.trajectory_active,
        .saturated_previous = s_data.saturated,
        .position_error_rad = position_error_deg * DEG_TO_RAD,
        .trajectory_speed_rad_s = s_trajectory_speed_dps * DEG_TO_RAD,
        .trajectory_acceleration_rad_s2 =
            s_trajectory_acceleration_dps2 * DEG_TO_RAD,
        .measured_speed_rad_s = s_filtered_speed_rad_s,
        .measured_acceleration_rad_s2 = s_filtered_acceleration_rad_s2,
        .measured_iq_a = s_data.iq
    };
    AxisDynamicOutput output;
    AxisDynamic_Update(&s_dynamic_state,
                       &s_dynamic_config,
                       &input,
                       &output);

    s_data.position_error_deg = position_error_deg;
    s_data.speed_reference_dps =
        output.speed_reference_rad_s * RAD_TO_DEG;
    s_data.speed_error_dps = output.speed_error_rad_s * RAD_TO_DEG;
    s_data.iq_unsaturated_a = output.iq_unsaturated_a;
    s_data.speed_proportional_current_a =
        output.speed_proportional_current_a;
    s_data.speed_integral_current_a =
        output.speed_integral_current_a;
    s_data.speed_kp_a_per_rad_s = output.speed_kp_a_per_rad_s;
    s_data.speed_ki_a_per_rad = output.speed_ki_a_per_rad;
    s_data.raw_disturbance_current_a =
        output.raw_disturbance_current_a;
    s_data.eso_speed_dps = output.eso_speed_rad_s * RAD_TO_DEG;
    s_data.observer_innovation_dps =
        output.observer_innovation_rad_s * RAD_TO_DEG;
    s_data.disturbance_acceleration_dps2 =
        output.disturbance_acceleration_rad_s2 * RAD_TO_DEG;
    s_data.inverse_plant_a_per_rad_s2 =
        output.inverse_plant_a_per_rad_s2;
    s_data.plant_gain_rad_s2_per_a = output.plant_gain_rad_s2_per_a;
    s_data.gain_excitation_rad_s2 = output.gain_excitation_rad_s2;
    s_data.gain_residual_a = output.gain_residual_a;
    s_data.gain_window_time_s = output.gain_window_time_s;
    s_data.gain_candidate_a_per_rad_s2 =
        output.gain_candidate_a_per_rad_s2;
    s_data.gain_correlation = output.gain_correlation;
    s_data.position_gain_per_s = output.position_gain_per_s;
    s_data.speed_bandwidth_hz = output.speed_bandwidth_hz;
    s_data.observer_bandwidth_hz = output.observer_bandwidth_hz;
    s_data.observer_limited = output.observer_limited;
    s_data.gain_adaptation_active = output.gain_adaptation_active;
    s_data.gain_limited = output.gain_limited;
    s_data.gain_window_active = output.gain_window_active;
    s_data.saturated =
        output.iq_command_a != output.iq_unsaturated_a;
    if (s_data.saturated) s_data.saturation_ms += sample_delta;

    const float abs_error = fabsf(position_error_deg);
    const float abs_command = fabsf(output.iq_command_a);
    if (abs_error > s_data.maximum_abs_error_deg)
        s_data.maximum_abs_error_deg = abs_error;
    if (abs_command > s_data.maximum_abs_iq_command_a)
        s_data.maximum_abs_iq_command_a = abs_command;
    set_iq_reference(output.iq_command_a);
}

static void update_settled(uint32_t sample_delta)
{
    if (!s_data.trajectory_active &&
        fabsf(s_data.position_error_deg) <= SETTLED_POSITION_DEG &&
        fabsf(s_data.speed_dps) <= SETTLED_SPEED_DPS)
    {
        const uint32_t new_samples = s_settled_samples + sample_delta;
        s_settled_samples = new_samples > SETTLED_CONFIRM_MS ?
            SETTLED_CONFIRM_MS : new_samples;
    }
    else
    {
        s_settled_samples = 0U;
    }
    s_data.settled_ms = s_settled_samples;
    s_data.settled = s_settled_samples >= SETTLED_CONFIRM_MS;
}

static void stop_output(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_running = false;
    s_id_reference_a = 0.0f;
    s_iq_reference_a = 0.0f;
    s_id_integral_v = 0.0f;
    s_iq_integral_v = 0.0f;
    pwm_neutral();
    driver_enable(false);
    s_data.running = false;
    s_data.iq_command_a = 0.0f;
    s_data.id = 0.0f;
    s_data.iq = 0.0f;
    s_data.vd = 0.0f;
    s_data.vq = 0.0f;
    if (primask == 0U) __enable_irq();
}

bool CurrentFOC_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    s_running = false;
    s_sin_e = 0.0f;
    s_cos_e = 1.0f;
    s_id_reference_a = 0.0f;
    s_iq_reference_a = 0.0f;
    s_target_position_deg = 0.0f;
    s_id_integral_v = 0.0f;
    s_iq_integral_v = 0.0f;
    s_test_center_count = 0;
    memset(&s_dynamic_state, 0, sizeof(s_dynamic_state));
    s_settled_samples = 0U;
    trajectory_reset();

    s_data.encoder_direction = ENCODER_DIRECTION;
    s_data.pole_pairs = MOTOR_POLE_PAIRS;
    s_data.mechanical_zero_raw = MECHANICAL_ZERO_RAW;
    s_data.electrical_offset_deg = ELECTRICAL_OFFSET_DEG;
    s_data.servo_state = FOC_SERVO_IDLE;

    driver_enable(false);
    pwm_neutral();
    if (!configure_cordic()) return false;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) return false;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) return false;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) return false;
    pwm_neutral();
    s_data.initialized = true;
    return true;
}

void CurrentFOC_Task(void)
{
    AS5600Data encoder;
    PhaseCurrentData current;
    AS5600_GetData(&encoder);
    PhaseCurrent_GetData(&current);

    if (!s_data.initialized || s_data.servo_state == FOC_SERVO_FAULT)
        return;

    if (!encoder_valid(&encoder))
    {
        if (s_data.servo_state != FOC_SERVO_IDLE)
            CurrentFOC_Stop(FOC_FAULT_ENCODER);
        return;
    }
    if (!current.calibrated || !current.m1_offset_valid) return;
    if (HAL_GPIO_ReadPin(M1_nFAULT_GPIO_Port,
                         M1_nFAULT_Pin) == GPIO_PIN_RESET)
    {
        CurrentFOC_Stop(FOC_FAULT_NFAULT);
        return;
    }
    if (current.m1_ia_raw <= ADC_RAIL_LOW_COUNT ||
        current.m1_ia_raw >= ADC_RAIL_HIGH_COUNT ||
        current.m1_ib_raw <= ADC_RAIL_LOW_COUNT ||
        current.m1_ib_raw >= ADC_RAIL_HIGH_COUNT)
    {
        CurrentFOC_Stop(FOC_FAULT_ADC_RAIL);
        return;
    }

    if (s_data.servo_state == FOC_SERVO_IDLE)
    {
        s_data.encoder_raw = encoder.raw_angle;
        s_data.encoder_total_count = encoder.total_count;
        begin_state(FOC_SERVO_USB_WAIT);
        return;
    }

    const uint32_t now = HAL_GetTick();
    s_data.state_elapsed_ms = now - s_state_start_ms;
    if (s_data.servo_state == FOC_SERVO_ACTIVE)
        s_data.total_elapsed_ms = now - s_servo_start_ms;

    if (s_data.servo_state == FOC_SERVO_USB_WAIT)
    {
        s_data.encoder_raw = encoder.raw_angle;
        s_data.encoder_total_count = encoder.total_count;
        if (s_data.state_elapsed_ms < USB_CONNECT_WAIT_MS) return;

        s_test_center_count = encoder.total_count;
        s_data.test_center_raw = encoder.raw_angle;
        s_data.maximum_abs_position_deg = 0.0f;
        s_data.maximum_abs_error_deg = 0.0f;
        s_data.maximum_abs_speed_dps = 0.0f;
        s_data.maximum_abs_iq_command_a = 0.0f;
        s_data.peak_phase_current_a = 0.0f;
        s_data.saturation_ms = 0U;
        memset(&s_dynamic_state, 0, sizeof(s_dynamic_state));
        s_settled_samples = 0U;
        trajectory_reset();
        observer_reset(&encoder);
        publish_electrical_angle(&encoder);
        AxisDynamic_Reset(&s_dynamic_state,
                          &s_dynamic_config,
                          0.0f,
                          0.0f,
                          0.0f);
        s_data.iq_unsaturated_a = 0.0f;
        s_data.speed_proportional_current_a = 0.0f;
        s_data.speed_integral_current_a = 0.0f;
        s_data.speed_kp_a_per_rad_s = 2.0f * SPEED_DAMPING_RATIO *
            (TWO_PI_F * SPEED_BANDWIDTH_HZ) *
            s_dynamic_state.inverse_plant_a_per_rad_s2;
        s_data.speed_ki_a_per_rad =
            (TWO_PI_F * SPEED_BANDWIDTH_HZ) *
            (TWO_PI_F * SPEED_BANDWIDTH_HZ) *
            s_dynamic_state.inverse_plant_a_per_rad_s2;
        s_data.raw_disturbance_current_a = 0.0f;
        s_data.eso_speed_dps = 0.0f;
        s_data.observer_innovation_dps = 0.0f;
        s_data.disturbance_acceleration_dps2 = 0.0f;
        s_data.inverse_plant_a_per_rad_s2 =
            s_dynamic_state.inverse_plant_a_per_rad_s2;
        s_data.plant_gain_rad_s2_per_a =
            1.0f / s_dynamic_state.inverse_plant_a_per_rad_s2;
        s_data.gain_excitation_rad_s2 = 0.0f;
        s_data.gain_residual_a = 0.0f;
        s_data.gain_window_time_s = 0.0f;
        s_data.gain_candidate_a_per_rad_s2 =
            s_dynamic_state.inverse_plant_a_per_rad_s2;
        s_data.gain_correlation = 0.0f;
        s_data.position_gain_per_s = POSITION_GAIN_PER_S;
        s_data.speed_bandwidth_hz = SPEED_BANDWIDTH_HZ;
        s_data.observer_bandwidth_hz = ESO_DIAGNOSTIC_BANDWIDTH_HZ;
        s_data.observer_limited = false;
        s_data.gain_adaptation_active = false;
        s_data.gain_limited = false;
        s_data.gain_window_active = false;
        driver_enable(true);
        s_running = true;
        s_data.running = true;
        s_servo_start_ms = now;
        set_iq_reference(0.0f);
        begin_state(FOC_SERVO_ACTIVE);
        return;
    }

    if (!s_running) return;
    if (encoder.sample_count == s_processed_encoder_sample) return;

    const uint32_t sample_delta =
        encoder.sample_count - s_processed_encoder_sample;
    const uint32_t bounded_sample_delta =
        sample_delta > 2U ? 2U : sample_delta;
    const float control_dt =
        (float)bounded_sample_delta * 0.001f;
    observer_update(&encoder);

    trajectory_update(control_dt);
    motion_control_update(control_dt, sample_delta);
    update_settled(sample_delta);
}

void CurrentFOC_FastLoop(float ia, float ib, float ic)
{
    if (!s_running) return;
    if ((HAL_GetTick() - s_last_encoder_ms) > ENCODER_TIMEOUT_MS)
    {
        CurrentFOC_Stop(FOC_FAULT_ENCODER);
        return;
    }

    float peak = fabsf(ia);
    if (fabsf(ib) > peak) peak = fabsf(ib);
    if (fabsf(ic) > peak) peak = fabsf(ic);
    if (peak > s_data.peak_phase_current_a)
        s_data.peak_phase_current_a = peak;
    if (peak > PHASE_CURRENT_LIMIT_A)
    {
        CurrentFOC_Stop(FOC_FAULT_OVERCURRENT);
        return;
    }

    const float sine = s_sin_e;
    const float cosine = s_cos_e;
    const float id_reference = s_id_reference_a;
    const float iq_reference = s_iq_reference_a;
    const float i_alpha = ia;
    const float i_beta = (ia + 2.0f * ib) * INV_SQRT3;
    const float id = i_alpha * cosine + i_beta * sine;
    const float iq = -i_alpha * sine + i_beta * cosine;
    const float id_error = id_reference - id;
    const float iq_error = iq_reference - iq;
    const float vd_unsaturated =
        CURRENT_KP_V_PER_A * id_error + s_id_integral_v;
    const float vq_unsaturated =
        CURRENT_KP_V_PER_A * iq_error + s_iq_integral_v;
    const float magnitude_squared =
        vd_unsaturated * vd_unsaturated + vq_unsaturated * vq_unsaturated;
    float voltage_scale = 1.0f;
    if (magnitude_squared > VOLTAGE_VECTOR_LIMIT_V * VOLTAGE_VECTOR_LIMIT_V)
        voltage_scale = VOLTAGE_VECTOR_LIMIT_V / sqrtf(magnitude_squared);
    const float vd = vd_unsaturated * voltage_scale;
    const float vq = vq_unsaturated * voltage_scale;

    s_id_integral_v +=
        (CURRENT_KI_V_PER_A_S * id_error +
         CURRENT_AW_GAIN_PER_S * (vd - vd_unsaturated)) * CURRENT_LOOP_DT_S;
    s_iq_integral_v +=
        (CURRENT_KI_V_PER_A_S * iq_error +
         CURRENT_AW_GAIN_PER_S * (vq - vq_unsaturated)) * CURRENT_LOOP_DT_S;
    s_id_integral_v = clampf(s_id_integral_v,
                             -VOLTAGE_VECTOR_LIMIT_V,
                             VOLTAGE_VECTOR_LIMIT_V);
    s_iq_integral_v = clampf(s_iq_integral_v,
                             -VOLTAGE_VECTOR_LIMIT_V,
                             VOLTAGE_VECTOR_LIMIT_V);

    const float alpha_voltage = vd * cosine - vq * sine;
    const float beta_voltage = vd * sine + vq * cosine;
    apply_alpha_beta(alpha_voltage, beta_voltage);

    s_data.id = id;
    s_data.iq = iq;
    s_data.vd = vd;
    s_data.vq = vq;
}

void CurrentFOC_SetTargetDeg(float target_position_deg)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_target_position_deg = target_position_deg;
    s_data.target_position_deg = target_position_deg;
    if (primask == 0U) __enable_irq();
}

void CurrentFOC_Stop(uint8_t fault)
{
    if (fault == FOC_FAULT_NONE) return;
    s_data.fault = fault;
    s_data.servo_state = FOC_SERVO_FAULT;
    stop_output();
}

void CurrentFOC_GetData(CurrentFOCData *data)
{
    if (data == NULL) return;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data = s_data;
    if (primask == 0U) __enable_irq();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == M1_nFAULT_Pin)
        CurrentFOC_Stop(FOC_FAULT_NFAULT);
    if (GPIO_Pin == M2_nFAULT_Pin)
        Motor2Calibration_Stop(M2_CAL_FAULT_NFAULT);
}
