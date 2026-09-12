#ifndef GIMBAL_UDP_H
#define GIMBAL_UDP_H

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_protocol.h"

#define GIMBAL_UDP_LOCAL_PORT GIMBAL_COMMAND_PORT

typedef enum {
    GIMBAL_UDP_STATE_OFF = 0,
    GIMBAL_UDP_STATE_PORT_ERROR,
    GIMBAL_UDP_STATE_CONFIG_ERROR,
    GIMBAL_UDP_STATE_SOCKET_ERROR,
    GIMBAL_UDP_STATE_LINK_DOWN,
    GIMBAL_UDP_STATE_READY
} GimbalUdpState;

typedef struct {
    GimbalUdpState state;
    uint8_t version;
    bool link_up;
    bool command_socket_open;
    bool telemetry_socket_open;
    bool telemetry_enabled;
    bool remote_control_active;
    bool watchdog_expired;
    int32_t m1_target_mdeg;
    int32_t m2_target_mdeg;
    uint32_t rx_packets;
    uint32_t valid_commands;
    uint32_t invalid_commands;
    uint32_t stale_commands;
    uint32_t tx_errors;
    uint32_t telemetry_packets;
    uint32_t telemetry_errors;
    uint32_t watchdog_trip_count;
    uint32_t last_valid_command_ms;
    uint32_t last_command_age_ms;
    uint32_t last_command_sequence;
    int32_t last_socket_result;
    uint8_t last_peer_ip[4];
    uint16_t last_peer_port;
    uint8_t telemetry_peer_ip[4];
} GimbalUdpStatus;

/* Returns true when SPI, W5500 configuration and UDP socket all initialize. */
bool GimbalUdp_Init(void);

/* Non-blocking foreground service. Call continuously from while (1). */
void GimbalUdp_Task(void);

void GimbalUdp_GetStatus(GimbalUdpStatus *status);

#endif /* GIMBAL_UDP_H */
