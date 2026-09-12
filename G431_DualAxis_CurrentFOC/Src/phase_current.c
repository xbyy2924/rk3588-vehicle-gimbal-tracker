#include "phase_current.h"

#include "adc.h"
#include "current_foc.h"
#include "motor2_calibration.h"

#include <string.h>

#define ADC_REFERENCE_VOLTAGE       3.3f
#define ADC_FULL_SCALE              4095.0f
#define INA240_SHUNT_OHM            0.1f
#define INA240_GAIN                 20.0f
#define CURRENT_PER_ADC_COUNT       \
    (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE / \
     (INA240_SHUNT_OHM * INA240_GAIN))
#define CURRENT_OFFSET_SAMPLES      4096U
#define CURRENT_DISCARD_SAMPLES     256U
#define CURRENT_OFFSET_MIN          1700U
#define CURRENT_OFFSET_MAX          2400U

/* The four INA240 channels use the same PCB current direction. */
#define M1_IA_POLARITY              1.0f
#define M1_IB_POLARITY              1.0f
#define M2_IA_POLARITY              1.0f
#define M2_IB_POLARITY              1.0f

typedef struct
{
    volatile uint16_t m1_ia_raw;
    volatile uint16_t m1_ib_raw;
    volatile uint16_t m2_ia_raw;
    volatile uint16_t m2_ib_raw;
    volatile uint16_t m1_ia_offset;
    volatile uint16_t m1_ib_offset;
    volatile uint16_t m2_ia_offset;
    volatile uint16_t m2_ib_offset;
    volatile float m1_ia;
    volatile float m1_ib;
    volatile float m1_ic;
    volatile float m2_ia;
    volatile float m2_ib;
    volatile float m2_ic;
    volatile uint32_t sample_count;
    volatile uint32_t calibration_count;
    volatile bool calibrated;
    volatile bool m1_offset_valid;
    volatile bool m2_offset_valid;
    uint32_t m1_ia_sum;
    uint32_t m1_ib_sum;
    uint32_t m2_ia_sum;
    uint32_t m2_ib_sum;
} PhaseCurrentState;

static PhaseCurrentState s_current;

static bool offset_valid(uint16_t offset)
{
    return offset >= CURRENT_OFFSET_MIN && offset <= CURRENT_OFFSET_MAX;
}

HAL_StatusTypeDef PhaseCurrent_Init(void)
{
    memset(&s_current, 0, sizeof(s_current));
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    const uint16_t m1_ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(
        &hadc1, ADC_INJECTED_RANK_1);
    const uint16_t m1_ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(
        &hadc2, ADC_INJECTED_RANK_1);
    const uint16_t m2_ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(
        &hadc1, ADC_INJECTED_RANK_2);
    const uint16_t m2_ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(
        &hadc2, ADC_INJECTED_RANK_2);

    s_current.m1_ia_raw = m1_ia_raw;
    s_current.m1_ib_raw = m1_ib_raw;
    s_current.m2_ia_raw = m2_ia_raw;
    s_current.m2_ib_raw = m2_ib_raw;
    s_current.sample_count++;

    if (!s_current.calibrated)
    {
        if (s_current.sample_count <= CURRENT_DISCARD_SAMPLES) return;

        s_current.m1_ia_sum += m1_ia_raw;
        s_current.m1_ib_sum += m1_ib_raw;
        s_current.m2_ia_sum += m2_ia_raw;
        s_current.m2_ib_sum += m2_ib_raw;
        s_current.calibration_count++;

        if (s_current.calibration_count >= CURRENT_OFFSET_SAMPLES)
        {
            const uint32_t rounding = CURRENT_OFFSET_SAMPLES / 2U;
            s_current.m1_ia_offset = (uint16_t)(
                (s_current.m1_ia_sum + rounding) / CURRENT_OFFSET_SAMPLES);
            s_current.m1_ib_offset = (uint16_t)(
                (s_current.m1_ib_sum + rounding) / CURRENT_OFFSET_SAMPLES);
            s_current.m2_ia_offset = (uint16_t)(
                (s_current.m2_ia_sum + rounding) / CURRENT_OFFSET_SAMPLES);
            s_current.m2_ib_offset = (uint16_t)(
                (s_current.m2_ib_sum + rounding) / CURRENT_OFFSET_SAMPLES);
            s_current.m1_offset_valid =
                offset_valid(s_current.m1_ia_offset) &&
                offset_valid(s_current.m1_ib_offset);
            s_current.m2_offset_valid =
                offset_valid(s_current.m2_ia_offset) &&
                offset_valid(s_current.m2_ib_offset);
            s_current.calibrated = true;
        }
        return;
    }

    s_current.m1_ia = ((float)m1_ia_raw - (float)s_current.m1_ia_offset) *
                      CURRENT_PER_ADC_COUNT * M1_IA_POLARITY;
    s_current.m1_ib = ((float)m1_ib_raw - (float)s_current.m1_ib_offset) *
                      CURRENT_PER_ADC_COUNT * M1_IB_POLARITY;
    s_current.m1_ic = -(s_current.m1_ia + s_current.m1_ib);
    s_current.m2_ia = ((float)m2_ia_raw - (float)s_current.m2_ia_offset) *
                      CURRENT_PER_ADC_COUNT * M2_IA_POLARITY;
    s_current.m2_ib = ((float)m2_ib_raw - (float)s_current.m2_ib_offset) *
                      CURRENT_PER_ADC_COUNT * M2_IB_POLARITY;
    s_current.m2_ic = -(s_current.m2_ia + s_current.m2_ib);

    CurrentFOC_FastLoop(s_current.m1_ia,
                        s_current.m1_ib,
                        s_current.m1_ic);
    Motor2Calibration_FastLoop(s_current.m2_ia,
                               s_current.m2_ib,
                               s_current.m2_ic);
}

bool PhaseCurrent_IsCalibrated(void)
{
    return s_current.calibrated;
}

bool PhaseCurrent_M1OffsetValid(void)
{
    return s_current.m1_offset_valid;
}

bool PhaseCurrent_M2OffsetValid(void)
{
    return s_current.m2_offset_valid;
}

void PhaseCurrent_GetData(PhaseCurrentData *data)
{
    if (data == NULL) return;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    data->m1_ia_raw = s_current.m1_ia_raw;
    data->m1_ib_raw = s_current.m1_ib_raw;
    data->m2_ia_raw = s_current.m2_ia_raw;
    data->m2_ib_raw = s_current.m2_ib_raw;
    data->m1_ia_offset = s_current.m1_ia_offset;
    data->m1_ib_offset = s_current.m1_ib_offset;
    data->m2_ia_offset = s_current.m2_ia_offset;
    data->m2_ib_offset = s_current.m2_ib_offset;
    data->m1_ia = s_current.m1_ia;
    data->m1_ib = s_current.m1_ib;
    data->m1_ic = s_current.m1_ic;
    data->m2_ia = s_current.m2_ia;
    data->m2_ib = s_current.m2_ib;
    data->m2_ic = s_current.m2_ic;
    data->sample_count = s_current.sample_count;
    data->calibrated = s_current.calibrated;
    data->m1_offset_valid = s_current.m1_offset_valid;
    data->m2_offset_valid = s_current.m2_offset_valid;
    if (primask == 0U) __enable_irq();
}
