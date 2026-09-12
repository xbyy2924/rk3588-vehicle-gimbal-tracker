#ifndef W5500_PORT_H
#define W5500_PORT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * STM32G431 + W5500 hardware adaptation layer.
 * CubeMX must generate hspi2 and these GPIO labels:
 *   W5500_CS, W5500_RST, W5500_INT
 */

bool W5500_Port_Init(void);
uint8_t W5500_Port_GetVersion(void);
bool W5500_Port_HadSpiError(void);
void W5500_Port_ClearSpiError(void);

typedef struct {
    uint32_t polling_transfers;
    uint32_t dma_transfers;
    uint32_t dma_errors;
    uint32_t dma_timeouts;
} W5500PortStats;

void W5500_Port_GetStats(W5500PortStats *stats);

#endif /* W5500_PORT_H */
