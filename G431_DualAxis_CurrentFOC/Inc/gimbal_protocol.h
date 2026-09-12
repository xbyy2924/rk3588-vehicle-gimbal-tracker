#ifndef GIMBAL_PROTOCOL_H
#define GIMBAL_PROTOCOL_H

#include <stdint.h>

/*
 * Gimbal UDP wire protocol, version 1.
 *
 * All multi-byte fields are little-endian. STM32G431 and RK3588 are both
 * little-endian, so the firmware can serialize the packed structures directly.
 * Angles use millidegrees, speeds use millidegrees/second, currents use mA,
 * voltages use mV and accelerations use millidegrees/second^2 unless noted.
 */

#define GIMBAL_PROTOCOL_MAGIC             0x31424D47UL /* bytes: "GMB1" */
#define GIMBAL_PROTOCOL_VERSION           1U

#define GIMBAL_COMMAND_PORT               5000U
#define GIMBAL_TELEMETRY_LOCAL_PORT       5001U
#define GIMBAL_TELEMETRY_DESTINATION_PORT 5001U

#define GIMBAL_TELEMETRY_PERIOD_MS        10U
#define GIMBAL_COMMAND_TIMEOUT_MS         500U
#define GIMBAL_SUBSCRIPTION_TIMEOUT_MS    2000U

typedef enum {
    GIMBAL_MESSAGE_COMMAND = 1,
    GIMBAL_MESSAGE_ACK = 2,
    GIMBAL_MESSAGE_TELEMETRY = 3
} GimbalMessageType;

typedef enum {
    GIMBAL_COMMAND_HEARTBEAT = 1,
    GIMBAL_COMMAND_SET_TARGETS = 2,
    GIMBAL_COMMAND_HOLD_CURRENT = 3,
    GIMBAL_COMMAND_ZERO = 4,
    GIMBAL_COMMAND_DISARM = 5,
    GIMBAL_COMMAND_SUBSCRIBE = 6
} GimbalCommandId;

enum {
    GIMBAL_COMMAND_FLAG_M1_VALID = 1U << 0,
    GIMBAL_COMMAND_FLAG_M2_VALID = 1U << 1,
    GIMBAL_COMMAND_FLAG_ENABLE_WATCHDOG = 1U << 2
};

typedef enum {
    GIMBAL_RESULT_OK = 0,
    GIMBAL_RESULT_BAD_LENGTH = -1,
    GIMBAL_RESULT_BAD_MAGIC = -2,
    GIMBAL_RESULT_BAD_VERSION = -3,
    GIMBAL_RESULT_BAD_TYPE = -4,
    GIMBAL_RESULT_BAD_COMMAND = -5,
    GIMBAL_RESULT_STALE_SEQUENCE = -6
} GimbalCommandResult;

enum {
    GIMBAL_SYSTEM_FLAG_INITIALIZED = 1UL << 0,
    GIMBAL_SYSTEM_FLAG_LINK_UP = 1UL << 1,
    GIMBAL_SYSTEM_FLAG_COMMAND_SOCKET_OPEN = 1UL << 2,
    GIMBAL_SYSTEM_FLAG_TELEMETRY_SOCKET_OPEN = 1UL << 3,
    GIMBAL_SYSTEM_FLAG_TELEMETRY_ENABLED = 1UL << 4,
    GIMBAL_SYSTEM_FLAG_REMOTE_CONTROL_ACTIVE = 1UL << 5,
    GIMBAL_SYSTEM_FLAG_WATCHDOG_EXPIRED = 1UL << 6,
    GIMBAL_SYSTEM_FLAG_SPI_ERROR = 1UL << 7
};

enum {
    GIMBAL_AXIS_FLAG_INITIALIZED = 1UL << 0,
    GIMBAL_AXIS_FLAG_RUNNING = 1UL << 1,
    GIMBAL_AXIS_FLAG_SETTLED = 1UL << 2,
    GIMBAL_AXIS_FLAG_TRAJECTORY_ACTIVE = 1UL << 3,
    GIMBAL_AXIS_FLAG_SATURATED = 1UL << 4,
    GIMBAL_AXIS_FLAG_OBSERVER_LIMITED = 1UL << 5,
    GIMBAL_AXIS_FLAG_GAIN_ADAPTATION_ACTIVE = 1UL << 6,
    GIMBAL_AXIS_FLAG_GAIN_LIMITED = 1UL << 7,
    GIMBAL_AXIS_FLAG_GAIN_WINDOW_ACTIVE = 1UL << 8
};

enum {
    GIMBAL_ENCODER_FLAG_DEVICE_READY = 1UL << 0,
    GIMBAL_ENCODER_FLAG_SAMPLE_VALID = 1UL << 1,
    GIMBAL_ENCODER_FLAG_MAGNET_DETECTED = 1UL << 2,
    GIMBAL_ENCODER_FLAG_MAGNET_TOO_WEAK = 1UL << 3,
    GIMBAL_ENCODER_FLAG_MAGNET_TOO_STRONG = 1UL << 4,
    GIMBAL_ENCODER_FLAG_BUSY = 1UL << 5
};

enum {
    GIMBAL_CURRENT_FLAG_CALIBRATED = 1UL << 0,
    GIMBAL_CURRENT_FLAG_M1_OFFSET_VALID = 1UL << 1,
    GIMBAL_CURRENT_FLAG_M2_OFFSET_VALID = 1UL << 2
};

#if defined(__GNUC__)
#define GIMBAL_PACKED __attribute__((packed))
#else
#define GIMBAL_PACKED
#pragma pack(push, 1)
#endif

typedef struct GIMBAL_PACKED {
    uint32_t magic;
    uint8_t version;
    uint8_t message_type;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t timestamp_ms;
} GimbalPacketHeader;

typedef struct GIMBAL_PACKED {
    GimbalPacketHeader header;
    uint16_t command_id;
    uint16_t flags;
    int32_t m1_target_mdeg;
    int32_t m2_target_mdeg;
} GimbalCommandPacket;

typedef struct GIMBAL_PACKED {
    GimbalPacketHeader header;
    int32_t result;
    uint32_t command_sequence;
    uint32_t client_timestamp_ms;
    int32_t applied_m1_target_mdeg;
    int32_t applied_m2_target_mdeg;
    uint32_t watchdog_remaining_ms;
} GimbalAckPacket;

typedef struct GIMBAL_PACKED {
    uint32_t flags;
    uint32_t sample_count;
    uint16_t m1_ia_raw;
    uint16_t m1_ib_raw;
    uint16_t m2_ia_raw;
    uint16_t m2_ib_raw;
    uint16_t m1_ia_offset;
    uint16_t m1_ib_offset;
    uint16_t m2_ia_offset;
    uint16_t m2_ib_offset;
    int32_t m1_ia_ma;
    int32_t m1_ib_ma;
    int32_t m1_ic_ma;
    int32_t m2_ia_ma;
    int32_t m2_ib_ma;
    int32_t m2_ic_ma;
} GimbalPhaseCurrentTelemetry;

typedef struct GIMBAL_PACKED {
    uint32_t flags;
    uint16_t raw_angle;
    uint16_t reserved;
    int32_t mechanical_angle_mdeg;
    int32_t multi_turn_angle_mdeg;
    int32_t total_count;
    uint32_t sample_count;
    uint32_t error_count;
    uint32_t busy_skip_count;
    uint32_t last_update_ms;
    uint32_t sample_age_ms;
} GimbalEncoderTelemetry;

typedef struct GIMBAL_PACKED {
    uint32_t flags;
    uint8_t fault;
    uint8_t servo_state;
    uint8_t speed_window_ms;
    int8_t encoder_direction;
    uint8_t pole_pairs;
    uint8_t reserved0;
    uint16_t reserved1;

    uint16_t encoder_raw;
    uint16_t mechanical_zero_raw;
    uint16_t test_center_raw;
    uint16_t reserved2;
    int32_t encoder_total_count;
    uint32_t electrical_mdeg;

    uint32_t state_elapsed_ms;
    uint32_t total_elapsed_ms;
    uint32_t settled_ms;
    uint32_t saturation_ms;

    int32_t target_position_mdeg;
    int32_t trajectory_position_mdeg;
    int32_t trajectory_speed_mdps;
    int32_t trajectory_acceleration_mdps2;
    int32_t motor_position_mdeg;
    int32_t position_error_mdeg;
    int32_t raw_speed_mdps;
    int32_t window_speed_mdps;
    int32_t speed_mdps;
    int32_t acceleration_mdps2;
    int32_t speed_reference_mdps;
    int32_t speed_error_mdps;

    int32_t iq_command_ma;
    int32_t iq_unsaturated_ma;
    int32_t speed_proportional_current_ma;
    int32_t speed_integral_current_ma;
    int32_t speed_kp_maprs;
    int32_t speed_ki_mapr;
    int32_t raw_disturbance_current_ma;
    int32_t eso_speed_mdps;
    int32_t observer_innovation_mdps;
    int32_t disturbance_acceleration_mdps2;
    int32_t inverse_plant_uaps2;
    int32_t plant_gain_mrads2pa;
    int32_t gain_excitation_mrads2;
    int32_t gain_residual_ma;
    int32_t gain_window_time_ms;
    int32_t gain_candidate_uaps2;
    int32_t gain_correlation_permil;
    int32_t position_gain_mpers;
    int32_t speed_bandwidth_mhz;
    int32_t observer_bandwidth_mhz;

    int32_t id_ma;
    int32_t iq_ma;
    int32_t vd_mv;
    int32_t vq_mv;

    int32_t maximum_abs_position_mdeg;
    int32_t maximum_abs_error_mdeg;
    int32_t maximum_abs_speed_mdps;
    int32_t maximum_abs_iq_command_ma;
    int32_t peak_phase_current_ma;
} GimbalAxisTelemetry;

typedef struct GIMBAL_PACKED {
    GimbalPacketHeader header;
    uint32_t system_flags;
    uint32_t current_start_status;
    uint32_t command_rx_packets;
    uint32_t valid_commands;
    uint32_t invalid_commands;
    uint32_t stale_commands;
    uint32_t command_tx_errors;
    uint32_t telemetry_tx_packets;
    uint32_t telemetry_tx_errors;
    uint32_t last_command_age_ms;
    uint32_t watchdog_trip_count;
    int32_t last_socket_result;
    uint32_t spi_poll_transfers;
    uint32_t spi_dma_transfers;
    uint32_t spi_dma_errors;
    uint32_t spi_dma_timeouts;
    GimbalPhaseCurrentTelemetry phase_current;
    GimbalEncoderTelemetry encoder1;
    GimbalEncoderTelemetry encoder2;
    GimbalAxisTelemetry motor1;
    GimbalAxisTelemetry motor2;
} GimbalTelemetryPacket;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

_Static_assert(sizeof(GimbalPacketHeader) == 16U,
               "GimbalPacketHeader wire size changed");
_Static_assert(sizeof(GimbalCommandPacket) == 28U,
               "GimbalCommandPacket wire size changed");
_Static_assert(sizeof(GimbalAckPacket) == 40U,
               "GimbalAckPacket wire size changed");
_Static_assert(sizeof(GimbalPhaseCurrentTelemetry) == 48U,
               "GimbalPhaseCurrentTelemetry wire size changed");
_Static_assert(sizeof(GimbalEncoderTelemetry) == 40U,
               "GimbalEncoderTelemetry wire size changed");
_Static_assert(sizeof(GimbalAxisTelemetry) == 208U,
               "GimbalAxisTelemetry wire size changed");
_Static_assert(sizeof(GimbalTelemetryPacket) == 624U,
               "GimbalTelemetryPacket wire size changed");

#endif /* GIMBAL_PROTOCOL_H */
