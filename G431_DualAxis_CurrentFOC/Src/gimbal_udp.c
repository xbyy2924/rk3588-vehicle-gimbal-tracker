#include "gimbal_udp.h"

#include "as5600.h"
#include "current_foc.h"
#include "main.h"
#include "motor2_calibration.h"
#include "phase_current.h"
#include "socket.h"
#include "w5500_port.h"
#include "wizchip_conf.h"

#include <ctype.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define COMMAND_SOCKET       0U
#define TELEMETRY_SOCKET     1U
#define RX_BUFFER_SIZE     256U
#define TEXT_BUFFER_SIZE   160U
#define LINK_POLL_MS       250U
#define SOCKET_RETRY_MS    500U

/* Direct cable: Linux eth1=192.168.10.1, W5500=192.168.10.2. */
static wiz_NetInfo s_network = {
    .mac = {0x02U, 0x47U, 0x34U, 0x31U, 0x00U, 0x01U},
    .ip = {192U, 168U, 10U, 2U},
    .sn = {255U, 255U, 255U, 0U},
    .gw = {192U, 168U, 10U, 1U},
    .dns = {0U, 0U, 0U, 0U},
    .dhcp = NETINFO_STATIC
};

static GimbalUdpStatus s_status;
static uint8_t s_rx_buffer[RX_BUFFER_SIZE];
static uint8_t s_text_buffer[TEXT_BUFFER_SIZE];
static GimbalAckPacket s_ack __attribute__((aligned(4)));
static GimbalTelemetryPacket s_telemetry __attribute__((aligned(4)));
static PhaseCurrentData s_phase_snapshot;
static AS5600Data s_encoder1_snapshot;
static AS5600Data s_encoder2_snapshot;
static CurrentFOCData s_m1_snapshot;
static Motor2CalibrationData s_m2_snapshot;
static W5500PortStats s_port_snapshot;
static uint32_t s_last_link_ms;
static uint32_t s_last_socket_retry_ms;
static uint32_t s_last_telemetry_ms;
static uint32_t s_ack_sequence;
static uint32_t s_telemetry_sequence;
static bool s_chip_configured;
static bool s_have_command_sequence;

extern volatile uint32_t current_start_status;

static int32_t scaled_i32(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled >= (float)INT32_MAX) return INT32_MAX;
    if (scaled <= (float)INT32_MIN) return INT32_MIN;
    return scaled >= 0.0f ? (int32_t)(scaled + 0.5f) :
                            (int32_t)(scaled - 0.5f);
}

static const char *skip_spaces(const char *text)
{
    while ((*text == ' ') || (*text == '\t')) ++text;
    return text;
}

static bool at_end(const char *text)
{
    text = skip_spaces(text);
    while ((*text == '\r') || (*text == '\n')) ++text;
    return *text == '\0';
}

static bool parse_degrees_mdeg(const char **cursor, int32_t *value_mdeg)
{
    const char *p = skip_spaces(*cursor);
    bool negative = false;
    int64_t integer = 0;
    int32_t fraction = 0;
    int32_t fraction_scale = 100;
    bool has_digit = false;

    if ((*p == '+') || (*p == '-')) {
        negative = *p == '-';
        ++p;
    }
    while (isdigit((unsigned char)*p)) {
        has_digit = true;
        integer = integer * 10 + (*p - '0');
        if (integer > (INT32_MAX / 1000)) return false;
        ++p;
    }
    if (*p == '.') {
        ++p;
        while (isdigit((unsigned char)*p)) {
            has_digit = true;
            if (fraction_scale > 0) {
                fraction += (*p - '0') * fraction_scale;
                fraction_scale /= 10;
            }
            ++p;
        }
    }
    if (!has_digit) return false;

    int64_t result = integer * 1000 + fraction;
    if (negative) result = -result;
    if ((result < INT32_MIN) || (result > INT32_MAX)) return false;
    *cursor = p;
    *value_mdeg = (int32_t)result;
    return true;
}

static bool command_word(const char *text, const char *word,
                         const char **arguments)
{
    const size_t length = strlen(word);
    if (strncmp(text, word, length) != 0) return false;
    if ((text[length] != '\0') && (text[length] != ' ') &&
        (text[length] != '\t') && (text[length] != '\r') &&
        (text[length] != '\n')) return false;
    *arguments = text + length;
    return true;
}

static void apply_m1_target(int32_t target_mdeg)
{
    s_status.m1_target_mdeg = target_mdeg;
    CurrentFOC_SetTargetDeg((float)target_mdeg * 0.001f);
}

static void apply_m2_target(int32_t target_mdeg)
{
    s_status.m2_target_mdeg = target_mdeg;
    Motor2Calibration_SetTargetDeg((float)target_mdeg * 0.001f);
}

static void hold_current_position(void)
{
    CurrentFOC_GetData(&s_m1_snapshot);
    Motor2Calibration_GetData(&s_m2_snapshot);
    apply_m1_target(scaled_i32(s_m1_snapshot.motor_position_deg, 1000.0f));
    apply_m2_target(scaled_i32(s_m2_snapshot.motor_position_deg, 1000.0f));
}

static bool process_text_command(char *command)
{
    const char *arguments = NULL;
    const char *cursor = skip_spaces(command);
    int32_t first = 0;
    int32_t second = 0;

    if (command_word(cursor, "PING", &arguments) && at_end(arguments)) {
        (void)snprintf((char *)s_text_buffer, sizeof(s_text_buffer),
                       "PONG V=%02X LINK=%u TELSEQ=%lu\n",
                       (unsigned)s_status.version,
                       s_status.link_up ? 1U : 0U,
                       (unsigned long)s_telemetry_sequence);
        return true;
    }
    if (command_word(cursor, "STATUS", &arguments) && at_end(arguments)) {
        (void)snprintf((char *)s_text_buffer, sizeof(s_text_buffer),
                       "STATUS LINK=%u REMOTE=%u WDOG=%u RX=%lu TEL=%lu\n",
                       s_status.link_up ? 1U : 0U,
                       s_status.remote_control_active ? 1U : 0U,
                       s_status.watchdog_expired ? 1U : 0U,
                       (unsigned long)s_status.rx_packets,
                       (unsigned long)s_status.telemetry_packets);
        return true;
    }

    /* Commissioning-only text commands do not arm the production watchdog. */
    if (command_word(cursor, "ZERO", &arguments) && at_end(arguments)) {
        apply_m1_target(0);
        apply_m2_target(0);
    } else if (command_word(cursor, "SET", &arguments) &&
               parse_degrees_mdeg(&arguments, &first) &&
               parse_degrees_mdeg(&arguments, &second) && at_end(arguments)) {
        apply_m1_target(first);
        apply_m2_target(second);
    } else if (command_word(cursor, "M1", &arguments) &&
               parse_degrees_mdeg(&arguments, &first) && at_end(arguments)) {
        apply_m1_target(first);
    } else if (command_word(cursor, "M2", &arguments) &&
               parse_degrees_mdeg(&arguments, &first) && at_end(arguments)) {
        apply_m2_target(first);
    } else {
        (void)snprintf((char *)s_text_buffer, sizeof(s_text_buffer),
                       "ERR FORMAT\n");
        return false;
    }

    (void)snprintf((char *)s_text_buffer, sizeof(s_text_buffer),
                   "OK M1=%ld M2=%ld mdeg\n",
                   (long)s_status.m1_target_mdeg,
                   (long)s_status.m2_target_mdeg);
    return true;
}

static bool sequence_is_newer(uint32_t candidate, uint32_t reference)
{
    return (int32_t)(candidate - reference) > 0;
}

static void prepare_ack(const GimbalCommandPacket *command,
                        GimbalCommandResult result, uint32_t now)
{
    uint32_t remaining_ms = 0U;
    if (s_status.remote_control_active) {
        const uint32_t age = now - s_status.last_valid_command_ms;
        if (age < GIMBAL_COMMAND_TIMEOUT_MS)
            remaining_ms = GIMBAL_COMMAND_TIMEOUT_MS - age;
    }

    memset(&s_ack, 0, sizeof(s_ack));
    s_ack.header.magic = GIMBAL_PROTOCOL_MAGIC;
    s_ack.header.version = GIMBAL_PROTOCOL_VERSION;
    s_ack.header.message_type = GIMBAL_MESSAGE_ACK;
    s_ack.header.payload_size =
        (uint16_t)(sizeof(s_ack) - sizeof(s_ack.header));
    s_ack.header.sequence = ++s_ack_sequence;
    s_ack.header.timestamp_ms = now;
    s_ack.result = result;
    s_ack.command_sequence = command->header.sequence;
    s_ack.client_timestamp_ms = command->header.timestamp_ms;
    s_ack.applied_m1_target_mdeg = s_status.m1_target_mdeg;
    s_ack.applied_m2_target_mdeg = s_status.m2_target_mdeg;
    s_ack.watchdog_remaining_ms = remaining_ms;
}

static GimbalCommandResult process_binary_command(
    const GimbalCommandPacket *command, uint32_t now,
    const uint8_t peer_ip[4])
{
    if (command->header.magic != GIMBAL_PROTOCOL_MAGIC)
        return GIMBAL_RESULT_BAD_MAGIC;
    if (command->header.version != GIMBAL_PROTOCOL_VERSION)
        return GIMBAL_RESULT_BAD_VERSION;
    if (command->header.message_type != GIMBAL_MESSAGE_COMMAND)
        return GIMBAL_RESULT_BAD_TYPE;
    if (command->header.payload_size !=
        (uint16_t)(sizeof(*command) - sizeof(command->header)))
        return GIMBAL_RESULT_BAD_LENGTH;

    if (s_have_command_sequence &&
        !sequence_is_newer(command->header.sequence,
                           s_status.last_command_sequence) &&
        ((uint32_t)(now - s_status.last_valid_command_ms) <
         GIMBAL_SUBSCRIPTION_TIMEOUT_MS)) {
        ++s_status.stale_commands;
        return GIMBAL_RESULT_STALE_SEQUENCE;
    }

    switch (command->command_id) {
    case GIMBAL_COMMAND_HEARTBEAT:
    case GIMBAL_COMMAND_SUBSCRIBE:
        break;
    case GIMBAL_COMMAND_SET_TARGETS:
        if ((command->flags & (GIMBAL_COMMAND_FLAG_M1_VALID |
                               GIMBAL_COMMAND_FLAG_M2_VALID)) == 0U)
            return GIMBAL_RESULT_BAD_COMMAND;
        if ((command->flags & GIMBAL_COMMAND_FLAG_M1_VALID) != 0U)
            apply_m1_target(command->m1_target_mdeg);
        if ((command->flags & GIMBAL_COMMAND_FLAG_M2_VALID) != 0U)
            apply_m2_target(command->m2_target_mdeg);
        s_status.remote_control_active =
            (command->flags & GIMBAL_COMMAND_FLAG_ENABLE_WATCHDOG) != 0U;
        break;
    case GIMBAL_COMMAND_HOLD_CURRENT:
        hold_current_position();
        s_status.remote_control_active =
            (command->flags & GIMBAL_COMMAND_FLAG_ENABLE_WATCHDOG) != 0U;
        break;
    case GIMBAL_COMMAND_ZERO:
        apply_m1_target(0);
        apply_m2_target(0);
        s_status.remote_control_active =
            (command->flags & GIMBAL_COMMAND_FLAG_ENABLE_WATCHDOG) != 0U;
        break;
    case GIMBAL_COMMAND_DISARM:
        hold_current_position();
        s_status.remote_control_active = false;
        break;
    default:
        return GIMBAL_RESULT_BAD_COMMAND;
    }

    s_have_command_sequence = true;
    s_status.last_command_sequence = command->header.sequence;
    s_status.last_valid_command_ms = now;
    s_status.last_command_age_ms = 0U;
    s_status.telemetry_enabled = true;
    s_status.watchdog_expired = false;
    memcpy(s_status.telemetry_peer_ip, peer_ip,
           sizeof(s_status.telemetry_peer_ip));
    return GIMBAL_RESULT_OK;
}

static bool open_udp_socket(uint8_t sn, uint16_t port)
{
    if (getSn_SR(sn) != SOCK_CLOSED) (void)close(sn);
    const int8_t result = socket(sn, Sn_MR_UDP, port, 0U);
    s_status.last_socket_result = result;
    if (result != (int8_t)sn) {
        s_status.state = GIMBAL_UDP_STATE_SOCKET_ERROR;
        return false;
    }
    return true;
}

static void update_link_state(void)
{
    uint8_t link = PHY_LINK_OFF;
    if (ctlwizchip(CW_GET_PHYLINK, &link) != 0) {
        s_status.link_up = false;
        s_status.state = GIMBAL_UDP_STATE_PORT_ERROR;
        return;
    }
    s_status.link_up = link == PHY_LINK_ON;
    s_status.state = s_status.link_up ? GIMBAL_UDP_STATE_READY :
                                       GIMBAL_UDP_STATE_LINK_DOWN;
}

static uint32_t axis_flags(bool initialized, bool running, bool settled,
                           bool trajectory_active, bool saturated,
                           bool observer_limited, bool gain_adaptation_active,
                           bool gain_limited, bool gain_window_active)
{
    uint32_t flags = 0U;
    if (initialized) flags |= GIMBAL_AXIS_FLAG_INITIALIZED;
    if (running) flags |= GIMBAL_AXIS_FLAG_RUNNING;
    if (settled) flags |= GIMBAL_AXIS_FLAG_SETTLED;
    if (trajectory_active) flags |= GIMBAL_AXIS_FLAG_TRAJECTORY_ACTIVE;
    if (saturated) flags |= GIMBAL_AXIS_FLAG_SATURATED;
    if (observer_limited) flags |= GIMBAL_AXIS_FLAG_OBSERVER_LIMITED;
    if (gain_adaptation_active)
        flags |= GIMBAL_AXIS_FLAG_GAIN_ADAPTATION_ACTIVE;
    if (gain_limited) flags |= GIMBAL_AXIS_FLAG_GAIN_LIMITED;
    if (gain_window_active) flags |= GIMBAL_AXIS_FLAG_GAIN_WINDOW_ACTIVE;
    return flags;
}

#define FILL_AXIS(dst_, src_) do {                                           \
    (dst_).flags = axis_flags((src_).initialized, (src_).running,            \
        (src_).settled, (src_).trajectory_active, (src_).saturated,          \
        (src_).observer_limited, (src_).gain_adaptation_active,              \
        (src_).gain_limited, (src_).gain_window_active);                     \
    (dst_).fault = (src_).fault;                                             \
    (dst_).servo_state = (src_).servo_state;                                 \
    (dst_).speed_window_ms = (src_).speed_window_ms;                         \
    (dst_).encoder_direction = (src_).encoder_direction;                     \
    (dst_).pole_pairs = (src_).pole_pairs;                                   \
    (dst_).encoder_raw = (src_).encoder_raw;                                 \
    (dst_).mechanical_zero_raw = (src_).mechanical_zero_raw;                 \
    (dst_).test_center_raw = (src_).test_center_raw;                         \
    (dst_).encoder_total_count = (src_).encoder_total_count;                 \
    (dst_).electrical_mdeg = (src_).electrical_mdeg;                         \
    (dst_).state_elapsed_ms = (src_).state_elapsed_ms;                       \
    (dst_).total_elapsed_ms = (src_).total_elapsed_ms;                       \
    (dst_).settled_ms = (src_).settled_ms;                                   \
    (dst_).saturation_ms = (src_).saturation_ms;                             \
    (dst_).target_position_mdeg = scaled_i32((src_).target_position_deg, 1000.0f); \
    (dst_).trajectory_position_mdeg = scaled_i32((src_).trajectory_position_deg, 1000.0f); \
    (dst_).trajectory_speed_mdps = scaled_i32((src_).trajectory_speed_dps, 1000.0f); \
    (dst_).trajectory_acceleration_mdps2 = scaled_i32((src_).trajectory_acceleration_dps2, 1000.0f); \
    (dst_).motor_position_mdeg = scaled_i32((src_).motor_position_deg, 1000.0f); \
    (dst_).position_error_mdeg = scaled_i32((src_).position_error_deg, 1000.0f); \
    (dst_).raw_speed_mdps = scaled_i32((src_).raw_speed_dps, 1000.0f);        \
    (dst_).window_speed_mdps = scaled_i32((src_).window_speed_dps, 1000.0f); \
    (dst_).speed_mdps = scaled_i32((src_).speed_dps, 1000.0f);               \
    (dst_).acceleration_mdps2 = scaled_i32((src_).acceleration_dps2, 1000.0f); \
    (dst_).speed_reference_mdps = scaled_i32((src_).speed_reference_dps, 1000.0f); \
    (dst_).speed_error_mdps = scaled_i32((src_).speed_error_dps, 1000.0f);   \
    (dst_).iq_command_ma = scaled_i32((src_).iq_command_a, 1000.0f);         \
    (dst_).iq_unsaturated_ma = scaled_i32((src_).iq_unsaturated_a, 1000.0f); \
    (dst_).speed_proportional_current_ma = scaled_i32((src_).speed_proportional_current_a, 1000.0f); \
    (dst_).speed_integral_current_ma = scaled_i32((src_).speed_integral_current_a, 1000.0f); \
    (dst_).speed_kp_maprs = scaled_i32((src_).speed_kp_a_per_rad_s, 1000.0f); \
    (dst_).speed_ki_mapr = scaled_i32((src_).speed_ki_a_per_rad, 1000.0f);   \
    (dst_).raw_disturbance_current_ma = scaled_i32((src_).raw_disturbance_current_a, 1000.0f); \
    (dst_).eso_speed_mdps = scaled_i32((src_).eso_speed_dps, 1000.0f);       \
    (dst_).observer_innovation_mdps = scaled_i32((src_).observer_innovation_dps, 1000.0f); \
    (dst_).disturbance_acceleration_mdps2 = scaled_i32((src_).disturbance_acceleration_dps2, 1000.0f); \
    (dst_).inverse_plant_uaps2 = scaled_i32((src_).inverse_plant_a_per_rad_s2, 1000000.0f); \
    (dst_).plant_gain_mrads2pa = scaled_i32((src_).plant_gain_rad_s2_per_a, 1000.0f); \
    (dst_).gain_excitation_mrads2 = scaled_i32((src_).gain_excitation_rad_s2, 1000.0f); \
    (dst_).gain_residual_ma = scaled_i32((src_).gain_residual_a, 1000.0f);   \
    (dst_).gain_window_time_ms = scaled_i32((src_).gain_window_time_s, 1000.0f); \
    (dst_).gain_candidate_uaps2 = scaled_i32((src_).gain_candidate_a_per_rad_s2, 1000000.0f); \
    (dst_).gain_correlation_permil = scaled_i32((src_).gain_correlation, 1000.0f); \
    (dst_).position_gain_mpers = scaled_i32((src_).position_gain_per_s, 1000.0f); \
    (dst_).speed_bandwidth_mhz = scaled_i32((src_).speed_bandwidth_hz, 1000.0f); \
    (dst_).observer_bandwidth_mhz = scaled_i32((src_).observer_bandwidth_hz, 1000.0f); \
    (dst_).id_ma = scaled_i32((src_).id, 1000.0f);                           \
    (dst_).iq_ma = scaled_i32((src_).iq, 1000.0f);                           \
    (dst_).vd_mv = scaled_i32((src_).vd, 1000.0f);                           \
    (dst_).vq_mv = scaled_i32((src_).vq, 1000.0f);                           \
    (dst_).maximum_abs_position_mdeg = scaled_i32((src_).maximum_abs_position_deg, 1000.0f); \
    (dst_).maximum_abs_error_mdeg = scaled_i32((src_).maximum_abs_error_deg, 1000.0f); \
    (dst_).maximum_abs_speed_mdps = scaled_i32((src_).maximum_abs_speed_dps, 1000.0f); \
    (dst_).maximum_abs_iq_command_ma = scaled_i32((src_).maximum_abs_iq_command_a, 1000.0f); \
    (dst_).peak_phase_current_ma = scaled_i32((src_).peak_phase_current_a, 1000.0f); \
} while (0)

static void fill_encoder(GimbalEncoderTelemetry *dst,
                         const AS5600Data *src, uint32_t now)
{
    uint32_t flags = 0U;
    if (src->device_ready) flags |= GIMBAL_ENCODER_FLAG_DEVICE_READY;
    if (src->sample_valid) flags |= GIMBAL_ENCODER_FLAG_SAMPLE_VALID;
    if (src->magnet_detected) flags |= GIMBAL_ENCODER_FLAG_MAGNET_DETECTED;
    if (src->magnet_too_weak) flags |= GIMBAL_ENCODER_FLAG_MAGNET_TOO_WEAK;
    if (src->magnet_too_strong) flags |= GIMBAL_ENCODER_FLAG_MAGNET_TOO_STRONG;
    if (src->busy) flags |= GIMBAL_ENCODER_FLAG_BUSY;
    dst->flags = flags;
    dst->raw_angle = src->raw_angle;
    dst->mechanical_angle_mdeg = scaled_i32(src->mechanical_angle, 1000.0f);
    dst->multi_turn_angle_mdeg = scaled_i32(src->multi_turn_angle, 1000.0f);
    dst->total_count = src->total_count;
    dst->sample_count = src->sample_count;
    dst->error_count = src->error_count;
    dst->busy_skip_count = src->busy_skip_count;
    dst->last_update_ms = src->last_update_ms;
    dst->sample_age_ms = now - src->last_update_ms;
}

static void prepare_telemetry(uint32_t now)
{
    uint32_t flags = 0U;

    PhaseCurrent_GetData(&s_phase_snapshot);
    AS5600_GetDataAxis(AS5600_MOTOR1, &s_encoder1_snapshot);
    AS5600_GetDataAxis(AS5600_MOTOR2, &s_encoder2_snapshot);
    CurrentFOC_GetData(&s_m1_snapshot);
    Motor2Calibration_GetData(&s_m2_snapshot);
    W5500_Port_GetStats(&s_port_snapshot);
    memset(&s_telemetry, 0, sizeof(s_telemetry));

    s_telemetry.header.magic = GIMBAL_PROTOCOL_MAGIC;
    s_telemetry.header.version = GIMBAL_PROTOCOL_VERSION;
    s_telemetry.header.message_type = GIMBAL_MESSAGE_TELEMETRY;
    s_telemetry.header.payload_size =
        (uint16_t)(sizeof(s_telemetry) - sizeof(s_telemetry.header));
    s_telemetry.header.sequence = ++s_telemetry_sequence;
    s_telemetry.header.timestamp_ms = now;

    if (s_chip_configured) flags |= GIMBAL_SYSTEM_FLAG_INITIALIZED;
    if (s_status.link_up) flags |= GIMBAL_SYSTEM_FLAG_LINK_UP;
    if (s_status.command_socket_open) flags |= GIMBAL_SYSTEM_FLAG_COMMAND_SOCKET_OPEN;
    if (s_status.telemetry_socket_open) flags |= GIMBAL_SYSTEM_FLAG_TELEMETRY_SOCKET_OPEN;
    if (s_status.telemetry_enabled) flags |= GIMBAL_SYSTEM_FLAG_TELEMETRY_ENABLED;
    if (s_status.remote_control_active) flags |= GIMBAL_SYSTEM_FLAG_REMOTE_CONTROL_ACTIVE;
    if (s_status.watchdog_expired) flags |= GIMBAL_SYSTEM_FLAG_WATCHDOG_EXPIRED;
    if (W5500_Port_HadSpiError()) flags |= GIMBAL_SYSTEM_FLAG_SPI_ERROR;

    s_telemetry.system_flags = flags;
    s_telemetry.current_start_status = current_start_status;
    s_telemetry.command_rx_packets = s_status.rx_packets;
    s_telemetry.valid_commands = s_status.valid_commands;
    s_telemetry.invalid_commands = s_status.invalid_commands;
    s_telemetry.stale_commands = s_status.stale_commands;
    s_telemetry.command_tx_errors = s_status.tx_errors;
    s_telemetry.telemetry_tx_packets = s_status.telemetry_packets;
    s_telemetry.telemetry_tx_errors = s_status.telemetry_errors;
    s_telemetry.last_command_age_ms = s_status.last_command_age_ms;
    s_telemetry.watchdog_trip_count = s_status.watchdog_trip_count;
    s_telemetry.last_socket_result = s_status.last_socket_result;
    s_telemetry.spi_poll_transfers = s_port_snapshot.polling_transfers;
    s_telemetry.spi_dma_transfers = s_port_snapshot.dma_transfers;
    s_telemetry.spi_dma_errors = s_port_snapshot.dma_errors;
    s_telemetry.spi_dma_timeouts = s_port_snapshot.dma_timeouts;

    s_telemetry.phase_current.flags =
        (s_phase_snapshot.calibrated ? GIMBAL_CURRENT_FLAG_CALIBRATED : 0U) |
        (s_phase_snapshot.m1_offset_valid ? GIMBAL_CURRENT_FLAG_M1_OFFSET_VALID : 0U) |
        (s_phase_snapshot.m2_offset_valid ? GIMBAL_CURRENT_FLAG_M2_OFFSET_VALID : 0U);
    s_telemetry.phase_current.sample_count = s_phase_snapshot.sample_count;
    s_telemetry.phase_current.m1_ia_raw = s_phase_snapshot.m1_ia_raw;
    s_telemetry.phase_current.m1_ib_raw = s_phase_snapshot.m1_ib_raw;
    s_telemetry.phase_current.m2_ia_raw = s_phase_snapshot.m2_ia_raw;
    s_telemetry.phase_current.m2_ib_raw = s_phase_snapshot.m2_ib_raw;
    s_telemetry.phase_current.m1_ia_offset = s_phase_snapshot.m1_ia_offset;
    s_telemetry.phase_current.m1_ib_offset = s_phase_snapshot.m1_ib_offset;
    s_telemetry.phase_current.m2_ia_offset = s_phase_snapshot.m2_ia_offset;
    s_telemetry.phase_current.m2_ib_offset = s_phase_snapshot.m2_ib_offset;
    s_telemetry.phase_current.m1_ia_ma = scaled_i32(s_phase_snapshot.m1_ia, 1000.0f);
    s_telemetry.phase_current.m1_ib_ma = scaled_i32(s_phase_snapshot.m1_ib, 1000.0f);
    s_telemetry.phase_current.m1_ic_ma = scaled_i32(s_phase_snapshot.m1_ic, 1000.0f);
    s_telemetry.phase_current.m2_ia_ma = scaled_i32(s_phase_snapshot.m2_ia, 1000.0f);
    s_telemetry.phase_current.m2_ib_ma = scaled_i32(s_phase_snapshot.m2_ib, 1000.0f);
    s_telemetry.phase_current.m2_ic_ma = scaled_i32(s_phase_snapshot.m2_ic, 1000.0f);
    fill_encoder(&s_telemetry.encoder1, &s_encoder1_snapshot, now);
    fill_encoder(&s_telemetry.encoder2, &s_encoder2_snapshot, now);
    FILL_AXIS(s_telemetry.motor1, s_m1_snapshot);
    FILL_AXIS(s_telemetry.motor2, s_m2_snapshot);
}

bool GimbalUdp_Init(void)
{
    uint8_t memory[16] = {2U,2U,2U,2U,2U,2U,2U,2U,
                          2U,2U,2U,2U,2U,2U,2U,2U};
    const wiz_NetTimeout timeout = {.retry_cnt = 2U, .time_100us = 200U};

    memset(&s_status, 0, sizeof(s_status));
    s_status.state = GIMBAL_UDP_STATE_OFF;
    s_status.last_command_age_ms = UINT32_MAX;
    s_chip_configured = false;
    s_have_command_sequence = false;
    s_ack_sequence = 0U;
    s_telemetry_sequence = 0U;

    if (!W5500_Port_Init()) {
        s_status.version = W5500_Port_GetVersion();
        s_status.state = GIMBAL_UDP_STATE_PORT_ERROR;
        return false;
    }
    s_status.version = W5500_Port_GetVersion();
    if ((ctlwizchip(CW_INIT_WIZCHIP, memory) != 0) ||
        (ctlnetwork(CN_SET_NETINFO, &s_network) != 0) ||
        (ctlnetwork(CN_SET_TIMEOUT, (void *)&timeout) != 0)) {
        s_status.state = GIMBAL_UDP_STATE_CONFIG_ERROR;
        return false;
    }

    s_chip_configured = true;
    s_status.command_socket_open = open_udp_socket(COMMAND_SOCKET,
                                                   GIMBAL_COMMAND_PORT);
    s_status.telemetry_socket_open = open_udp_socket(
        TELEMETRY_SOCKET, GIMBAL_TELEMETRY_LOCAL_PORT);
    if (!s_status.command_socket_open || !s_status.telemetry_socket_open)
        return false;

    s_last_link_ms = HAL_GetTick();
    s_last_socket_retry_ms = s_last_link_ms;
    s_last_telemetry_ms = s_last_link_ms;
    update_link_state();
    return true;
}

static void service_command_socket(uint32_t now)
{
    uint8_t peer_ip[4];
    uint16_t peer_port = 0U;
    uint32_t received_magic = 0U;
    int32_t received, sent;

    if (!s_status.command_socket_open ||
        (getSn_RX_RSR(COMMAND_SOCKET) == 0U)) return;
    received = recvfrom(COMMAND_SOCKET, s_rx_buffer, RX_BUFFER_SIZE - 1U,
                        peer_ip, &peer_port);
    s_status.last_socket_result = received;
    if (received <= 0) return;

    ++s_status.rx_packets;
    memcpy(s_status.last_peer_ip, peer_ip, sizeof(peer_ip));
    s_status.last_peer_port = peer_port;

    if (received >= (int32_t)sizeof(received_magic))
        memcpy(&received_magic, s_rx_buffer, sizeof(received_magic));

    if ((received == (int32_t)sizeof(GimbalCommandPacket)) &&
        (received_magic == GIMBAL_PROTOCOL_MAGIC)) {
        GimbalCommandPacket command;
        GimbalCommandResult result;
        memcpy(&command, s_rx_buffer, sizeof(command));
        result = process_binary_command(&command, now, peer_ip);
        if (result == GIMBAL_RESULT_OK) ++s_status.valid_commands;
        else ++s_status.invalid_commands;
        prepare_ack(&command, result, now);
        sent = sendto(COMMAND_SOCKET, (uint8_t *)(void *)&s_ack,
                      (uint16_t)sizeof(s_ack), peer_ip, peer_port);
    } else {
        s_rx_buffer[received] = '\0';
        if (process_text_command((char *)s_rx_buffer)) ++s_status.valid_commands;
        else ++s_status.invalid_commands;
        sent = sendto(COMMAND_SOCKET, s_text_buffer,
                      (uint16_t)strnlen((char *)s_text_buffer,
                                        sizeof(s_text_buffer)),
                      peer_ip, peer_port);
    }
    s_status.last_socket_result = sent;
    if (sent < 0) ++s_status.tx_errors;
}

static void service_watchdog(uint32_t now)
{
    if (!s_have_command_sequence) {
        s_status.last_command_age_ms = UINT32_MAX;
        return;
    }
    s_status.last_command_age_ms = now - s_status.last_valid_command_ms;
    if (s_status.remote_control_active &&
        (s_status.last_command_age_ms >= GIMBAL_COMMAND_TIMEOUT_MS)) {
        hold_current_position();
        s_status.remote_control_active = false;
        s_status.watchdog_expired = true;
        ++s_status.watchdog_trip_count;
    }
    if (s_status.telemetry_enabled &&
        (s_status.last_command_age_ms >= GIMBAL_SUBSCRIPTION_TIMEOUT_MS))
        s_status.telemetry_enabled = false;
}

static void service_telemetry(uint32_t now)
{
    if (!s_status.telemetry_enabled || !s_status.telemetry_socket_open ||
        ((uint32_t)(now - s_last_telemetry_ms) < GIMBAL_TELEMETRY_PERIOD_MS))
        return;

    s_last_telemetry_ms = now;
    prepare_telemetry(now);
    const int32_t sent = sendto(
        TELEMETRY_SOCKET, (uint8_t *)(void *)&s_telemetry,
        (uint16_t)sizeof(s_telemetry), s_status.telemetry_peer_ip,
        GIMBAL_TELEMETRY_DESTINATION_PORT);
    s_status.last_socket_result = sent;
    if (sent == (int32_t)sizeof(s_telemetry)) ++s_status.telemetry_packets;
    else ++s_status.telemetry_errors;
}

void GimbalUdp_Task(void)
{
    const uint32_t now = HAL_GetTick();
    if (!s_chip_configured) return;

    if ((uint32_t)(now - s_last_link_ms) >= LINK_POLL_MS) {
        s_last_link_ms = now;
        update_link_state();
    }
    if ((uint32_t)(now - s_last_socket_retry_ms) >= SOCKET_RETRY_MS) {
        s_last_socket_retry_ms = now;
        s_status.command_socket_open = getSn_SR(COMMAND_SOCKET) == SOCK_UDP;
        if (!s_status.command_socket_open)
            s_status.command_socket_open = open_udp_socket(
                COMMAND_SOCKET, GIMBAL_COMMAND_PORT);
        s_status.telemetry_socket_open =
            getSn_SR(TELEMETRY_SOCKET) == SOCK_UDP;
        if (!s_status.telemetry_socket_open)
            s_status.telemetry_socket_open = open_udp_socket(
                TELEMETRY_SOCKET, GIMBAL_TELEMETRY_LOCAL_PORT);
    }

    service_watchdog(now);
    if (!s_status.link_up) return;
    service_command_socket(now);
    service_telemetry(now);
}

void GimbalUdp_GetStatus(GimbalUdpStatus *status)
{
    if (status != NULL) *status = s_status;
}
