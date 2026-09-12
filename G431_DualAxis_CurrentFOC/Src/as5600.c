#include "as5600.h"

#include <string.h>

#define AS5600_ADDRESS_HAL             (0x36U << 1U)
#define AS5600_REG_STATUS              0x0BU
#define AS5600_STATUS_MAGNET_HIGH      (1U << 3)
#define AS5600_STATUS_MAGNET_LOW       (1U << 4)
#define AS5600_STATUS_MAGNET_DETECTED  (1U << 5)
#define AS5600_COUNTS_PER_REVOLUTION   4096
#define AS5600_HALF_COUNT              2048
#define AS5600_TWO_PI                  6.28318530718f
#define AS5600_MAX_ANGLE_AGE_MS        10U

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    volatile uint16_t raw_angle;
    volatile uint16_t previous_raw_angle;
    volatile int32_t total_count;
    volatile float mechanical_angle;
    volatile float multi_turn_angle;
    volatile uint32_t sample_count;
    volatile uint32_t error_count;
    volatile uint32_t busy_skip_count;
    volatile uint32_t last_update_ms;
    volatile bool device_ready;
    volatile bool sample_valid;
    volatile bool magnet_detected;
    volatile bool magnet_too_weak;
    volatile bool magnet_too_strong;
    volatile bool busy;
    uint8_t receive_buffer[3];
} AS5600State;

static AS5600State s_encoder[AS5600_AXIS_COUNT];

static bool axis_valid(AS5600Axis axis)
{
    return axis >= AS5600_MOTOR1 && axis < AS5600_AXIS_COUNT;
}

static AS5600State *state_from_i2c(I2C_HandleTypeDef *hi2c)
{
    for (uint32_t i = 0U; i < (uint32_t)AS5600_AXIS_COUNT; ++i)
    {
        if (s_encoder[i].hi2c == hi2c) return &s_encoder[i];
    }
    return NULL;
}

static void update_sample(AS5600State *encoder, const uint8_t buffer[3])
{
    const uint8_t status = buffer[0];
    const uint16_t raw = (uint16_t)((((uint16_t)buffer[1] & 0x0FU) << 8U) |
                                    (uint16_t)buffer[2]);

    encoder->magnet_detected =
        (status & AS5600_STATUS_MAGNET_DETECTED) != 0U;
    encoder->magnet_too_weak =
        (status & AS5600_STATUS_MAGNET_LOW) != 0U;
    encoder->magnet_too_strong =
        (status & AS5600_STATUS_MAGNET_HIGH) != 0U;

    if (!encoder->sample_valid)
    {
        encoder->previous_raw_angle = raw;
        encoder->total_count = raw;
        encoder->sample_valid = true;
    }
    else
    {
        int32_t difference =
            (int32_t)raw - (int32_t)encoder->previous_raw_angle;
        if (difference > AS5600_HALF_COUNT)
            difference -= AS5600_COUNTS_PER_REVOLUTION;
        else if (difference < -AS5600_HALF_COUNT)
            difference += AS5600_COUNTS_PER_REVOLUTION;
        encoder->total_count += difference;
        encoder->previous_raw_angle = raw;
    }

    encoder->raw_angle = raw;
    encoder->mechanical_angle =
        (float)raw * (AS5600_TWO_PI / (float)AS5600_COUNTS_PER_REVOLUTION);
    encoder->multi_turn_angle =
        (float)encoder->total_count *
        (AS5600_TWO_PI / (float)AS5600_COUNTS_PER_REVOLUTION);
    encoder->sample_count++;
    encoder->last_update_ms = HAL_GetTick();
}

bool AS5600_InitAxis(AS5600Axis axis, I2C_HandleTypeDef *hi2c)
{
    if (!axis_valid(axis) || hi2c == NULL) return false;

    AS5600State *encoder = &s_encoder[axis];
    uint8_t initial_data[3];
    memset(encoder, 0, sizeof(*encoder));
    encoder->hi2c = hi2c;

    if (HAL_I2C_IsDeviceReady(hi2c, AS5600_ADDRESS_HAL, 3U, 20U) != HAL_OK)
    {
        encoder->error_count++;
        return false;
    }
    encoder->device_ready = true;

    if (HAL_I2C_Mem_Read(hi2c,
                         AS5600_ADDRESS_HAL,
                         AS5600_REG_STATUS,
                         I2C_MEMADD_SIZE_8BIT,
                         initial_data,
                         sizeof(initial_data),
                         20U) != HAL_OK)
    {
        encoder->error_count++;
        encoder->device_ready = false;
        return false;
    }
    update_sample(encoder, initial_data);
    return true;
}

void AS5600_RequestReadAxis(AS5600Axis axis)
{
    if (!axis_valid(axis)) return;
    AS5600State *encoder = &s_encoder[axis];
    if (!encoder->device_ready || encoder->hi2c == NULL) return;
    if (encoder->busy)
    {
        encoder->busy_skip_count++;
        return;
    }

    encoder->busy = true;
    if (HAL_I2C_Mem_Read_DMA(encoder->hi2c,
                             AS5600_ADDRESS_HAL,
                             AS5600_REG_STATUS,
                             I2C_MEMADD_SIZE_8BIT,
                             encoder->receive_buffer,
                             sizeof(encoder->receive_buffer)) != HAL_OK)
    {
        encoder->busy = false;
        encoder->error_count++;
    }
}

void AS5600_RequestReadAll(void)
{
    AS5600_RequestReadAxis(AS5600_MOTOR1);
    AS5600_RequestReadAxis(AS5600_MOTOR2);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600State *encoder = state_from_i2c(hi2c);
    if (encoder == NULL) return;
    update_sample(encoder, encoder->receive_buffer);
    encoder->busy = false;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600State *encoder = state_from_i2c(hi2c);
    if (encoder == NULL) return;
    encoder->busy = false;
    encoder->error_count++;
}

void AS5600_GetDataAxis(AS5600Axis axis, AS5600Data *data)
{
    if (!axis_valid(axis) || data == NULL) return;
    AS5600State *encoder = &s_encoder[axis];
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    data->raw_angle = encoder->raw_angle;
    data->mechanical_angle = encoder->mechanical_angle;
    data->multi_turn_angle = encoder->multi_turn_angle;
    data->total_count = encoder->total_count;
    data->sample_count = encoder->sample_count;
    data->error_count = encoder->error_count;
    data->busy_skip_count = encoder->busy_skip_count;
    data->last_update_ms = encoder->last_update_ms;
    data->device_ready = encoder->device_ready;
    data->sample_valid = encoder->sample_valid;
    data->magnet_detected = encoder->magnet_detected;
    data->magnet_too_weak = encoder->magnet_too_weak;
    data->magnet_too_strong = encoder->magnet_too_strong;
    data->busy = encoder->busy;
    if (primask == 0U) __enable_irq();
}

bool AS5600_AxisAngleValid(AS5600Axis axis)
{
    if (!axis_valid(axis)) return false;
    const AS5600State *encoder = &s_encoder[axis];
    const uint32_t age = HAL_GetTick() - encoder->last_update_ms;
    return encoder->device_ready && encoder->sample_valid &&
           encoder->magnet_detected && !encoder->magnet_too_weak &&
           !encoder->magnet_too_strong && age <= AS5600_MAX_ANGLE_AGE_MS;
}

bool AS5600_Init(I2C_HandleTypeDef *hi2c)
{
    return AS5600_InitAxis(AS5600_MOTOR1, hi2c);
}

void AS5600_RequestRead(void)
{
    AS5600_RequestReadAxis(AS5600_MOTOR1);
}

void AS5600_GetData(AS5600Data *data)
{
    AS5600_GetDataAxis(AS5600_MOTOR1, data);
}

float AS5600_GetMechanicalAngle(void)
{
    return s_encoder[AS5600_MOTOR1].mechanical_angle;
}

bool AS5600_AngleValid(void)
{
    return AS5600_AxisAngleValid(AS5600_MOTOR1);
}
