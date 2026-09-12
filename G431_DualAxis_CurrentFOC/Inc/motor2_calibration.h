#ifndef MOTOR2_CALIBRATION_H
#define MOTOR2_CALIBRATION_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define M2_FOC_FAULT_NONE          0U
#define M2_FOC_FAULT_NFAULT        1U
#define M2_FOC_FAULT_ENCODER       3U
#define M2_FOC_FAULT_OVERCURRENT   4U
#define M2_FOC_FAULT_ADC_RAIL      5U

/* Compatibility name used by the frozen M1 SERVO8 fault callback. */
#define M2_CAL_FAULT_NFAULT M2_FOC_FAULT_NFAULT

#define M2_FOC_SERVO_IDLE          0U
#define M2_FOC_SERVO_USB_WAIT      1U
#define M2_FOC_SERVO_ACTIVE        2U
#define M2_FOC_SERVO_FAULT         3U

typedef struct
{
    bool initialized;
    bool running;
    bool settled;
    bool trajectory_active;
    bool saturated;
    bool observer_limited;
    bool gain_adaptation_active;
    bool gain_limited;
    bool gain_window_active;

    uint8_t fault;
    uint8_t servo_state;
    uint32_t state_elapsed_ms;
    uint32_t total_elapsed_ms;
    uint32_t settled_ms;
    uint32_t saturation_ms;

    uint16_t encoder_raw;
    int32_t encoder_total_count;
    uint16_t mechanical_zero_raw;
    uint16_t test_center_raw;
    int8_t encoder_direction;
    uint8_t pole_pairs;
    float electrical_offset_deg;
    uint32_t electrical_mdeg;

    float target_position_deg;
    float trajectory_position_deg;
    float trajectory_speed_dps;
    float trajectory_acceleration_dps2;
    float motor_position_deg;
    float position_error_deg;
    float raw_speed_dps;
    float window_speed_dps;
    uint8_t speed_window_ms;
    float speed_dps;
    float acceleration_dps2;
    float speed_reference_dps;
    float speed_error_dps;

    float iq_command_a;
    float iq_unsaturated_a;
    float speed_proportional_current_a;
    float speed_integral_current_a;
    float speed_kp_a_per_rad_s;
    float speed_ki_a_per_rad;
    float raw_disturbance_current_a;
    float eso_speed_dps;
    float observer_innovation_dps;
    float disturbance_acceleration_dps2;
    float inverse_plant_a_per_rad_s2;
    float plant_gain_rad_s2_per_a;
    float gain_excitation_rad_s2;
    float gain_residual_a;
    float gain_window_time_s;
    float gain_candidate_a_per_rad_s2;
    float gain_correlation;
    float position_gain_per_s;
    float speed_bandwidth_hz;
    float observer_bandwidth_hz;
    float id;
    float iq;
    float vd;
    float vq;

    float maximum_abs_position_deg;
    float maximum_abs_error_deg;
    float maximum_abs_speed_dps;
    float maximum_abs_iq_command_a;
    float peak_phase_current_a;
} Motor2CalibrationData;

bool Motor2Calibration_Init(void);
void Motor2Calibration_Task(void);
void Motor2Calibration_FastLoop(float ia, float ib, float ic);
void Motor2Calibration_SetTargetDeg(float target_position_deg);
void Motor2Calibration_Stop(uint8_t fault);
void Motor2Calibration_GetData(Motor2CalibrationData *data);

#endif
