#ifndef PHASE_CURRENT_H
#define PHASE_CURRENT_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t m1_ia_raw;
    uint16_t m1_ib_raw;
    uint16_t m2_ia_raw;
    uint16_t m2_ib_raw;
    uint16_t m1_ia_offset;
    uint16_t m1_ib_offset;
    uint16_t m2_ia_offset;
    uint16_t m2_ib_offset;
    float m1_ia;
    float m1_ib;
    float m1_ic;
    float m2_ia;
    float m2_ib;
    float m2_ic;
    uint32_t sample_count;
    bool calibrated;
    bool m1_offset_valid;
    bool m2_offset_valid;
} PhaseCurrentData;

HAL_StatusTypeDef PhaseCurrent_Init(void);
bool PhaseCurrent_IsCalibrated(void);
bool PhaseCurrent_M1OffsetValid(void);
bool PhaseCurrent_M2OffsetValid(void);
void PhaseCurrent_GetData(PhaseCurrentData *data);

#endif
