#include "usb_debug.h"

#include "as5600.h"
#include "current_foc.h"
#include "motor2_calibration.h"
#include "phase_current.h"

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <stdint.h>
#include <stdio.h>

#define FIREWATER_PERIOD_MS    10U
#define TEXT_DEBUG_PERIOD_MS   50U
#define USB_TX_BUFFER_SIZE   2200U

extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint32_t current_start_status;
extern volatile bool as5600_m1_init_ok;
extern volatile bool as5600_m2_init_ok;

static uint32_t s_last_firewater_ms;
static uint32_t s_last_text_ms;
static char s_usb_tx_buffer[USB_TX_BUFFER_SIZE];

static int32_t scaled_i32(float value, float scale)
{
    const float scaled = value * scale;
    return scaled >= 0.0f ?
        (int32_t)(scaled + 0.5f) :
        (int32_t)(scaled - 0.5f);
}

static int usb_cdc_ready(void)
{
    if (hUsbDeviceFS.pClassData == NULL) return 0;
    const USBD_CDC_HandleTypeDef *cdc =
        (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return cdc->TxState == 0U;
}

void UsbDebug_Init(void)
{
    const uint32_t now = HAL_GetTick();
    s_last_firewater_ms = now;
    s_last_text_ms = now;
}

void UsbDebug_Task(void)
{
    const uint32_t now = HAL_GetTick();
    if ((now - s_last_firewater_ms) < FIREWATER_PERIOD_MS) return;
    if (!usb_cdc_ready()) return;
    s_last_firewater_ms = now;

    const int include_text =
        (now - s_last_text_ms) >= TEXT_DEBUG_PERIOD_MS;
    if (include_text) s_last_text_ms = now;

    PhaseCurrentData current;
    AS5600Data encoder1;
    AS5600Data encoder2;
    CurrentFOCData m1;
    Motor2CalibrationData m2;
    PhaseCurrent_GetData(&current);
    AS5600_GetDataAxis(AS5600_MOTOR1, &encoder1);
    AS5600_GetDataAxis(AS5600_MOTOR2, &encoder2);
    CurrentFOC_GetData(&m1);
    Motor2Calibration_GetData(&m2);

    /* FireWater, 100 Hz:
     * CH0..8  M1: PERR, SPDR, SPD, ESPD, IQCMD, IQ, IQP, IQI, QINV
     * CH9..17 M2: PERR, SPDR, SPD, ESPD, IQCMD, IQ, IQP, IQI, QINV
     * CH18..23: M1_ID, M2_ID, M1_SPDE, M2_SPDE, M1_SAT, M2_SAT
     * CH24..27: M1_KSP, M1_KSI, M2_KSP, M2_KSI
     * CH28..33: M1_IDRAW, M2_IDRAW, M1_QCAND, M2_QCAND,
     *           M1_QADAPT, M2_QADAPT
     * Angles/speeds use milli-degrees; currents use mA; Q uses
     * 1e6*A/(rad/s^2); KSP uses mA/(rad/s); KSI uses mA/rad. */
    int length = snprintf(
        s_usb_tx_buffer,
        sizeof(s_usb_tx_buffer),
        "FW:%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,%u,%u,%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,%u,%u\r\n",
        (long)scaled_i32(m1.position_error_deg, 1000.0f),
        (long)scaled_i32(m1.speed_reference_dps, 1000.0f),
        (long)scaled_i32(m1.speed_dps, 1000.0f),
        (long)scaled_i32(m1.eso_speed_dps, 1000.0f),
        (long)scaled_i32(m1.iq_command_a, 1000.0f),
        (long)scaled_i32(m1.iq, 1000.0f),
        (long)scaled_i32(m1.speed_proportional_current_a, 1000.0f),
        (long)scaled_i32(m1.speed_integral_current_a, 1000.0f),
        (long)scaled_i32(m1.inverse_plant_a_per_rad_s2, 1000000.0f),
        (long)scaled_i32(m2.position_error_deg, 1000.0f),
        (long)scaled_i32(m2.speed_reference_dps, 1000.0f),
        (long)scaled_i32(m2.speed_dps, 1000.0f),
        (long)scaled_i32(m2.eso_speed_dps, 1000.0f),
        (long)scaled_i32(m2.iq_command_a, 1000.0f),
        (long)scaled_i32(m2.iq, 1000.0f),
        (long)scaled_i32(m2.speed_proportional_current_a, 1000.0f),
        (long)scaled_i32(m2.speed_integral_current_a, 1000.0f),
        (long)scaled_i32(m2.inverse_plant_a_per_rad_s2, 1000000.0f),
        (long)scaled_i32(m1.id, 1000.0f),
        (long)scaled_i32(m2.id, 1000.0f),
        (long)scaled_i32(m1.speed_error_dps, 1000.0f),
        (long)scaled_i32(m2.speed_error_dps, 1000.0f),
        m1.saturated ? 1U : 0U,
        m2.saturated ? 1U : 0U,
        (long)scaled_i32(m1.speed_kp_a_per_rad_s, 1000.0f),
        (long)scaled_i32(m1.speed_ki_a_per_rad, 1000.0f),
        (long)scaled_i32(m2.speed_kp_a_per_rad_s, 1000.0f),
        (long)scaled_i32(m2.speed_ki_a_per_rad, 1000.0f),
        (long)scaled_i32(m1.raw_disturbance_current_a, 1000.0f),
        (long)scaled_i32(m2.raw_disturbance_current_a, 1000.0f),
        (long)scaled_i32(m1.gain_candidate_a_per_rad_s2, 1000000.0f),
        (long)scaled_i32(m2.gain_candidate_a_per_rad_s2, 1000000.0f),
        m1.gain_adaptation_active ? 1U : 0U,
        m2.gain_adaptation_active ? 1U : 0U);
    if (length <= 0 || length >= (int)sizeof(s_usb_tx_buffer)) return;

    if (include_text)
    {
        const int remaining = (int)sizeof(s_usb_tx_buffer) - length;
        const int text_length = snprintf(
            &s_usb_tx_buffer[length],
            (size_t)remaining,
            "MODE=DUAL_CASCADE2,START=%lu,ADC_CAL=%u,"
            "M1_ADC_OK=%u,M2_ADC_OK=%u,"
            "E1=%u,M1_MAG=%u,E1ERR=%lu,E2=%u,M2_MAG=%u,E2ERR=%lu,"
            "M1_SST=%u,M1_RUN=%u,M1_FLT=%u,M1_SET=%u,M1_TMS=%lums,"
            "M1_POS=%ldmdeg,M1_PERR=%ldmdeg,M1_SPDR=%ldmdps,"
            "M1_SPD=%ldmdps,M1_SPDE=%ldmdps,M1_ESPD=%ldmdps,"
            "M1_IQCMD=%ldmA,M1_IQUN=%ldmA,M1_IQP=%ldmA,M1_IQI=%ldmA,"
            "M1_KSP=%ldmAprs,M1_KSI=%ldmApr,M1_IDRAW=%ldmA,"
            "M1_QINV=%lduAps2,M1_QCAND=%lduAps2,M1_GCORR=%ldpermil,"
            "M1_GWIN=%ldms,M1_QADAPT=%u,M1_QWIN=%u,M1_QLIM=%u,"
            "M1_OLIM=%u,M1_SAT=%u,M1_SATMS=%lums,M1_ID=%ldmA,M1_IQ=%ldmA,"
            "M2_SST=%u,M2_RUN=%u,M2_FLT=%u,M2_SET=%u,M2_TMS=%lums,"
            "M2_POS=%ldmdeg,M2_PERR=%ldmdeg,M2_SPDR=%ldmdps,"
            "M2_SPD=%ldmdps,M2_SPDE=%ldmdps,M2_ESPD=%ldmdps,"
            "M2_IQCMD=%ldmA,M2_IQUN=%ldmA,M2_IQP=%ldmA,M2_IQI=%ldmA,"
            "M2_KSP=%ldmAprs,M2_KSI=%ldmApr,M2_IDRAW=%ldmA,"
            "M2_QINV=%lduAps2,M2_QCAND=%lduAps2,M2_GCORR=%ldpermil,"
            "M2_GWIN=%ldms,M2_QADAPT=%u,M2_QWIN=%u,M2_QLIM=%u,"
            "M2_OLIM=%u,M2_SAT=%u,M2_SATMS=%lums,M2_ID=%ldmA,M2_IQ=%ldmA\r\n",
            (unsigned long)current_start_status,
            current.calibrated ? 1U : 0U,
            current.m1_offset_valid ? 1U : 0U,
            current.m2_offset_valid ? 1U : 0U,
            as5600_m1_init_ok ? 1U : 0U,
            encoder1.magnet_detected ? 1U : 0U,
            (unsigned long)encoder1.error_count,
            as5600_m2_init_ok ? 1U : 0U,
            encoder2.magnet_detected ? 1U : 0U,
            (unsigned long)encoder2.error_count,
            m1.servo_state, m1.running ? 1U : 0U, m1.fault,
            m1.settled ? 1U : 0U, (unsigned long)m1.total_elapsed_ms,
            (long)scaled_i32(m1.motor_position_deg, 1000.0f),
            (long)scaled_i32(m1.position_error_deg, 1000.0f),
            (long)scaled_i32(m1.speed_reference_dps, 1000.0f),
            (long)scaled_i32(m1.speed_dps, 1000.0f),
            (long)scaled_i32(m1.speed_error_dps, 1000.0f),
            (long)scaled_i32(m1.eso_speed_dps, 1000.0f),
            (long)scaled_i32(m1.iq_command_a, 1000.0f),
            (long)scaled_i32(m1.iq_unsaturated_a, 1000.0f),
            (long)scaled_i32(m1.speed_proportional_current_a, 1000.0f),
            (long)scaled_i32(m1.speed_integral_current_a, 1000.0f),
            (long)scaled_i32(m1.speed_kp_a_per_rad_s, 1000.0f),
            (long)scaled_i32(m1.speed_ki_a_per_rad, 1000.0f),
            (long)scaled_i32(m1.raw_disturbance_current_a, 1000.0f),
            (long)scaled_i32(m1.inverse_plant_a_per_rad_s2, 1000000.0f),
            (long)scaled_i32(m1.gain_candidate_a_per_rad_s2, 1000000.0f),
            (long)scaled_i32(m1.gain_correlation, 1000.0f),
            (long)scaled_i32(m1.gain_window_time_s, 1000.0f),
            m1.gain_adaptation_active ? 1U : 0U,
            m1.gain_window_active ? 1U : 0U,
            m1.gain_limited ? 1U : 0U,
            m1.observer_limited ? 1U : 0U,
            m1.saturated ? 1U : 0U,
            (unsigned long)m1.saturation_ms,
            (long)scaled_i32(m1.id, 1000.0f),
            (long)scaled_i32(m1.iq, 1000.0f),
            m2.servo_state, m2.running ? 1U : 0U, m2.fault,
            m2.settled ? 1U : 0U, (unsigned long)m2.total_elapsed_ms,
            (long)scaled_i32(m2.motor_position_deg, 1000.0f),
            (long)scaled_i32(m2.position_error_deg, 1000.0f),
            (long)scaled_i32(m2.speed_reference_dps, 1000.0f),
            (long)scaled_i32(m2.speed_dps, 1000.0f),
            (long)scaled_i32(m2.speed_error_dps, 1000.0f),
            (long)scaled_i32(m2.eso_speed_dps, 1000.0f),
            (long)scaled_i32(m2.iq_command_a, 1000.0f),
            (long)scaled_i32(m2.iq_unsaturated_a, 1000.0f),
            (long)scaled_i32(m2.speed_proportional_current_a, 1000.0f),
            (long)scaled_i32(m2.speed_integral_current_a, 1000.0f),
            (long)scaled_i32(m2.speed_kp_a_per_rad_s, 1000.0f),
            (long)scaled_i32(m2.speed_ki_a_per_rad, 1000.0f),
            (long)scaled_i32(m2.raw_disturbance_current_a, 1000.0f),
            (long)scaled_i32(m2.inverse_plant_a_per_rad_s2, 1000000.0f),
            (long)scaled_i32(m2.gain_candidate_a_per_rad_s2, 1000000.0f),
            (long)scaled_i32(m2.gain_correlation, 1000.0f),
            (long)scaled_i32(m2.gain_window_time_s, 1000.0f),
            m2.gain_adaptation_active ? 1U : 0U,
            m2.gain_window_active ? 1U : 0U,
            m2.gain_limited ? 1U : 0U,
            m2.observer_limited ? 1U : 0U,
            m2.saturated ? 1U : 0U,
            (unsigned long)m2.saturation_ms,
            (long)scaled_i32(m2.id, 1000.0f),
            (long)scaled_i32(m2.iq, 1000.0f));
        if (text_length < 0) return;
        if (text_length >= remaining)
            length = (int)sizeof(s_usb_tx_buffer) - 1;
        else
            length += text_length;
    }

    (void)CDC_Transmit_FS((uint8_t *)s_usb_tx_buffer, (uint16_t)length);
}
