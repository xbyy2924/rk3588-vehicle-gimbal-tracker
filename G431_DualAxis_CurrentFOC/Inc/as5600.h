#ifndef AS5600_H
#define AS5600_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    AS5600_MOTOR1 = 0,
    AS5600_MOTOR2 = 1,
    AS5600_AXIS_COUNT = 2
} AS5600Axis;

typedef struct
{
    uint16_t raw_angle;
    float mechanical_angle;
    float multi_turn_angle;
    int32_t total_count;
    uint32_t sample_count;
    uint32_t error_count;
    uint32_t busy_skip_count;
    uint32_t last_update_ms;
    bool device_ready;
    bool sample_valid;
    bool magnet_detected;
    bool magnet_too_weak;
    bool magnet_too_strong;
    bool busy;
} AS5600Data;

bool AS5600_InitAxis(AS5600Axis axis, I2C_HandleTypeDef *hi2c);
void AS5600_RequestReadAxis(AS5600Axis axis);
void AS5600_RequestReadAll(void);
void AS5600_GetDataAxis(AS5600Axis axis, AS5600Data *data);
bool AS5600_AxisAngleValid(AS5600Axis axis);

/* Compatibility wrappers: the frozen SERVO8 code continues to address M1. */
bool AS5600_Init(I2C_HandleTypeDef *hi2c);
void AS5600_RequestRead(void);
void AS5600_GetData(AS5600Data *data);
float AS5600_GetMechanicalAngle(void);
bool AS5600_AngleValid(void);

#endif
