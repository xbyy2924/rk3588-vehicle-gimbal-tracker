#include "w5500_port.h"

#include "main.h"
#include "spi.h"
#include "wizchip_conf.h"

#include <string.h>

#define W5500_SPI_TIMEOUT_MS        10U
#define W5500_DMA_TIMEOUT_MS        10U
#define W5500_DMA_MIN_LENGTH        16U
#define W5500_RESET_LOW_MS           2U
#define W5500_RESET_RECOVERY_MS     50U
#define W5500_DUMMY_BLOCK_SIZE     256U

static volatile bool s_spi_error = false;
static volatile bool s_dma_complete = false;
static volatile bool s_dma_error = false;
static uint8_t s_version = 0U;
static uint8_t s_dummy_tx[W5500_DUMMY_BLOCK_SIZE]
    __attribute__((aligned(4)));
static W5500PortStats s_stats;

static void w5500_cs_select(void)
{
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

static void w5500_cs_deselect(void)
{
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

/*
 * ioLibrary calls the critical-section callbacks around one complete W5500
 * SPI transaction. They intentionally do not mask interrupts here: the 20 kHz
 * ADC current-loop ISR must remain able to pre-empt Ethernet transfers.
 * W5500 access is restricted to the foreground GimbalUdp_Task().
 */
static void w5500_critical_enter(void)
{
}

static void w5500_critical_exit(void)
{
}

static bool wait_for_dma(void)
{
    const uint32_t started_ms = HAL_GetTick();

    while (!s_dma_complete && !s_dma_error) {
        if ((uint32_t)(HAL_GetTick() - started_ms) >=
            W5500_DMA_TIMEOUT_MS) {
            ++s_stats.dma_timeouts;
            s_spi_error = true;
            (void)HAL_SPI_Abort(&hspi2);
            return false;
        }
    }

    if (s_dma_error) {
        ++s_stats.dma_errors;
        s_spi_error = true;
        return false;
    }

    return true;
}

static bool start_dma_receive(uint8_t *buffer, uint16_t length)
{
    s_dma_complete = false;
    s_dma_error = false;

    if (HAL_SPI_TransmitReceive_DMA(&hspi2, s_dummy_tx, buffer,
                                    length) != HAL_OK) {
        ++s_stats.dma_errors;
        s_spi_error = true;
        return false;
    }

    ++s_stats.dma_transfers;
    return wait_for_dma();
}

static bool start_dma_transmit(uint8_t *buffer, uint16_t length)
{
    s_dma_complete = false;
    s_dma_error = false;

    if (HAL_SPI_Transmit_DMA(&hspi2, buffer, length) != HAL_OK) {
        ++s_stats.dma_errors;
        s_spi_error = true;
        return false;
    }

    ++s_stats.dma_transfers;
    return wait_for_dma();
}

static uint8_t w5500_spi_read_byte(void)
{
    uint8_t tx = 0xFFU;
    uint8_t rx = 0U;

    if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1U,
                                W5500_SPI_TIMEOUT_MS) != HAL_OK) {
        s_spi_error = true;
    }
    ++s_stats.polling_transfers;
    return rx;
}

static void w5500_spi_write_byte(uint8_t value)
{
    if (HAL_SPI_Transmit(&hspi2, &value, 1U,
                         W5500_SPI_TIMEOUT_MS) != HAL_OK) {
        s_spi_error = true;
    }
    ++s_stats.polling_transfers;
}

static void w5500_spi_read_burst(uint8_t *buffer, uint16_t length)
{
    while (length > 0U) {
        const uint16_t chunk =
            (length > W5500_DUMMY_BLOCK_SIZE) ? W5500_DUMMY_BLOCK_SIZE : length;

        bool ok;

        if (chunk >= W5500_DMA_MIN_LENGTH) {
            ok = start_dma_receive(buffer, chunk);
        } else {
            ok = HAL_SPI_TransmitReceive(&hspi2, s_dummy_tx, buffer, chunk,
                                         W5500_SPI_TIMEOUT_MS) == HAL_OK;
            ++s_stats.polling_transfers;
            if (!ok) s_spi_error = true;
        }

        if (!ok) memset(buffer, 0, chunk);

        buffer += chunk;
        length = (uint16_t)(length - chunk);
    }
}

static void w5500_spi_write_burst(uint8_t *buffer, uint16_t length)
{
    if (length == 0U) return;

    if (length >= W5500_DMA_MIN_LENGTH) {
        (void)start_dma_transmit(buffer, length);
        return;
    }

    if (HAL_SPI_Transmit(&hspi2, buffer, length,
                         W5500_SPI_TIMEOUT_MS) != HAL_OK) {
        s_spi_error = true;
    }
    ++s_stats.polling_transfers;
}

bool W5500_Port_Init(void)
{
    memset(s_dummy_tx, 0xFF, sizeof(s_dummy_tx));
    memset(&s_stats, 0, sizeof(s_stats));
    s_spi_error = false;
    s_dma_complete = false;
    s_dma_error = false;
    s_version = 0U;

    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);

    reg_wizchip_cris_cbfunc(w5500_critical_enter, w5500_critical_exit);
    reg_wizchip_cs_cbfunc(w5500_cs_select, w5500_cs_deselect);
    reg_wizchip_spi_cbfunc(w5500_spi_read_byte, w5500_spi_write_byte);
    reg_wizchip_spiburst_cbfunc(w5500_spi_read_burst,
                               w5500_spi_write_burst);

    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(W5500_RESET_LOW_MS);
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(W5500_RESET_RECOVERY_MS);

    s_version = getVERSIONR();
    return (!s_spi_error && (s_version == 0x04U));
}

uint8_t W5500_Port_GetVersion(void)
{
    return s_version;
}

bool W5500_Port_HadSpiError(void)
{
    return s_spi_error;
}

void W5500_Port_ClearSpiError(void)
{
    s_spi_error = false;
}

void W5500_Port_GetStats(W5500PortStats *stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
        s_dma_complete = true;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
        s_dma_complete = true;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
        s_dma_error = true;
    }
}
