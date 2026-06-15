/**
 * @file    mt6835.c
 * @brief   STM32 HAL/LL driver for the MagnTek MT6835 21-bit AMR magnetic rotary encoder
 *
 * SPI framing (24-bit, MSB first):
 *   Byte 0: [7:4] cmd nibble, [3:0] addr[11:8]
 *   Byte 1: addr[7:0]
 *   Byte 2: TX = write data / 0x00 for reads; RX = register value
 *
 * Burst RX buffer layout (6 bytes, cmd 0x0A addr 0x003):
 *   [0..1] echoed header (ignored)
 *   [2]    ANGLE[20:13]
 *   [3]    ANGLE[12:5]
 *   [4]    ANGLE[4:0] | STATUS[2:0]
 *   [5]    CRC-8
 *
 * Timestamp contract (all three read paths):
 *   timestamp is captured immediately after cs_low() in every path.
 *   For blocking paths it flows as a local variable directly into parse_burst.
 *   For the DMA path it is stored in dev->pending_timestamp (volatile) and
 *   consumed by MT6835_DMA_IRQHandler; dma_busy acts as the write barrier so
 *   StartDMA will never be called again before the IRQ has consumed it.
 */

#include "mt6835.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_dma.h"
#include "cmsis_os2.h"

/* ── Module-level state (single encoder) ───────────────────────────────── */

static MT6835_Handle *s_dev = NULL;
static MT6835_AngleResult *s_result = NULL;
static DMA_TypeDef *s_dma = NULL;
static uint32_t s_ch_tx = 0;
static uint32_t s_ch_rx = 0;

/* ── Private helpers ────────────────────────────────────────────────────── */

static inline void cs_low(void) { s_dev->cs_port->BSRR = ((uint32_t)s_dev->cs_pin << 16U); }
static inline void cs_high(void) { s_dev->cs_port->BSRR = (uint32_t)s_dev->cs_pin; }

static void build_header(uint8_t cmd, uint16_t addr, uint8_t header[2])
{
    header[0] = (uint8_t)(((cmd & 0x0F) << 4) | ((addr >> 8) & 0x0F));
    header[1] = (uint8_t)(addr & 0xFF);
}

/**
 * Parse a 6-byte burst buffer into result.
 *
 * @param result    Destination struct — timestamp written here too.
 * @param r         Pointer to the 6-byte buffer (dma_rx or a local stack buffer).
 *                  r[0..1] are the echoed header and are ignored.
 * @param timestamp Timer count captured at the falling CSN edge.
 */
static inline void parse_burst(MT6835_AngleResult *result,
                               const uint8_t *r,
                               uint16_t timestamp)
{
    result->timestamp = timestamp;
    result->raw = ((uint32_t)r[2] << 13) |
                  ((uint32_t)r[3] << 5) |
                  ((uint32_t)r[4] >> 3);
    result->status = r[4] & 0x07;
    result->crc_received = r[5];
}

/* ── CRC-8 (poly 0x07, init 0x00, no final XOR, MSB-first) ─────────────── */

uint8_t MT6835_ComputeCRC(const uint8_t *data)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < 3; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ── Init ───────────────────────────────────────────────────────────────── */

void MT6835_Init(MT6835_Handle *dev,
                 SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin,
                 TIM_HandleTypeDef *timestamp_timer)
{
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->dma_busy = 0;
    dev->pending_timestamp = 0;
    dev->timestamp_timer = timestamp_timer;

    /* Burst read header — constant after init, written once */
    dev->dma_tx[0] = 0xA0;
    dev->dma_tx[1] = 0x03;
    dev->dma_tx[2] = dev->dma_tx[3] = dev->dma_tx[4] = dev->dma_tx[5] = 0x00;

    s_dev = dev;
    s_dev->cs_port->BSRR = (uint32_t)s_dev->cs_pin; /* CS high */

    /* BW=x128: fastest response, highest noise.
     * HYST=0°: no hysteresis, maximum angle responsiveness. */
    MT6835_ConfigureField(dev, MT6835_REG_BW, MT6835_BW_MASK, MT6835_BW_SHIFT, MT6835_BW_X128);
    MT6835_ConfigureField(dev, MT6835_REG_HYST, MT6835_HYST_MASK, MT6835_HYST_SHIFT, MT6835_HYST_0DEG);
}

/**
 * Wire DMA channels to handle buffers, enable SPI DMA requests, store
 * all state needed by StartDMA and the IRQ handler.
 * NVIC priority/enable must be set by the caller after this returns.
 */
void MT6835_LowLevelInit(MT6835_Handle *dev,
                         MT6835_AngleResult *result,
                         DMA_TypeDef *dma,
                         uint32_t ch_tx,
                         uint32_t ch_rx)
{
    s_result = result;
    s_dma = dma;
    s_ch_tx = ch_tx;
    s_ch_rx = ch_rx;

    SPI_TypeDef *spi = dev->hspi->Instance;

    /* TX: dma_tx → SPI DR */
    LL_DMA_SetMemoryAddress(dma, ch_tx, (uint32_t)dev->dma_tx);
    LL_DMA_SetPeriphAddress(dma, ch_tx, (uint32_t)&spi->DR);
    LL_DMA_SetDataLength(dma, ch_tx, 6);

    /* RX: SPI DR → dma_rx */
    LL_DMA_SetMemoryAddress(dma, ch_rx, (uint32_t)dev->dma_rx);
    LL_DMA_SetPeriphAddress(dma, ch_rx, (uint32_t)&spi->DR);
    LL_DMA_SetDataLength(dma, ch_rx, 6);

    /* TC/TE on RX only — TX always finishes first */
    LL_DMA_EnableIT_TC(dma, ch_rx);
    LL_DMA_EnableIT_TE(dma, ch_rx);

    LL_SPI_EnableDMAReq_TX(spi);
    LL_SPI_EnableDMAReq_RX(spi);
}

/* ── Register access ────────────────────────────────────────────────────── */

MT6835_Status MT6835_ReadRegister(MT6835_Handle *dev, uint16_t addr, uint8_t *data)
{
    uint8_t tx[3] = {0}, rx[3] = {0};
    build_header(MT6835_CMD_READ, addr, tx);

    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    *data = rx[2];
    return MT6835_OK;
}

MT6835_Status MT6835_WriteRegister(MT6835_Handle *dev, uint16_t addr, uint8_t data)
{
    uint8_t tx[3] = {0}, rx[3] = {0};
    build_header(MT6835_CMD_WRITE, addr, tx);
    tx[2] = data;

    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    return MT6835_OK;
}

MT6835_Status MT6835_ConfigureField(MT6835_Handle *dev,
                                    uint16_t addr,
                                    uint8_t mask, uint8_t shift,
                                    uint8_t value)
{
    uint8_t reg = 0;
    MT6835_Status rc = MT6835_ReadRegister(dev, addr, &reg);
    if (rc != MT6835_OK)
        return rc;

    reg &= (uint8_t)~mask;
    reg |= (uint8_t)((value << shift) & mask);

    return MT6835_WriteRegister(dev, addr, reg);
}

/* ── EEPROM / zero ──────────────────────────────────────────────────────── */

MT6835_Status MT6835_ProgramEEPROM(MT6835_Handle *dev)
{
    /* cmd 0x0C, addr 0x000 — sensor returns 0x55 on success.
     * Caller must hold power for ≥6 s after this returns. */
    uint8_t tx[3] = {0}, rx[3] = {0};
    build_header(MT6835_CMD_PROG, 0x000, tx);

    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    HAL_Delay(25);
    return MT6835_OK;
}

MT6835_Status MT6835_SetZeroPosition(MT6835_Handle *dev)
{
    /* cmd 0x05, addr 0x000 — volatile; call ProgramEEPROM to persist. */
    uint8_t tx[3] = {0}, rx[3] = {0};
    build_header(MT6835_CMD_ZERO, 0x000, tx);

    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    return MT6835_OK;
}
#define MT6835_AUTOCAL_SPEED_MASK (0x07 << 4)

MT6835_Status MT6835_WriteAutoCalSpeed(MT6835_Handle *dev, MT6835_CalSpeed speed)
{
    /* ── 1. read current register value ────────────────────────────── */
    uint8_t data[3] = {0};
    build_header(MT6835_CMD_READ, MT6835_REG_AUTOCAL, data);

    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, data, data, 3, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    /* ── 2. modify bits [6:4] only ──────────────────────────────────── */
    uint8_t reg = data[2];
    reg &= ~MT6835_AUTOCAL_SPEED_MASK;     /* clear bits [6:4] */
    reg |= (((uint8_t)speed & 0x07) << 4); /* insert new speed */

    /* ── 3. write back ──────────────────────────────────────────────── */
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    build_header(MT6835_CMD_WRITE, MT6835_REG_AUTOCAL, data);
    data[2] = reg;

    cs_low();
    rc = HAL_SPI_TransmitReceive(dev->hspi, data, data, 3, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    return MT6835_OK;
}

MT6835_Status MT6835_ReadAutoCalStatus(MT6835_Handle *dev, MT6835_CalState *state)
{
    uint8_t data[3] = {0};
    build_header(MT6835_CMD_READ, MT6835_REG_AUTOCAL_STAT, data);

    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, data, data, 3, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    *state = (MT6835_CalState)((data[2] >> 6) & 0x03);

    return MT6835_OK;
}

/* write speed continuously until cal starts (manually assert AUTOCAL pin to VDD) */
void MT6835_BLOCKING_CALIBRATE(MT6835_Handle *dev)
{
    MT6835_CalState cal_state = MT6835_CAL_NONE;
    while (cal_state != MT6835_CAL_RUNNING)
    {
        MT6835_WriteAutoCalSpeed(dev, MT6835_CAL_SPEED_200);
        MT6835_ReadAutoCalStatus(dev, &cal_state);
        HAL_Delay(50);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    /* cal has started, poll until finished */
    do
    {
        HAL_Delay(50);
        MT6835_ReadAutoCalStatus(dev, &cal_state);
    } while (cal_state == MT6835_CAL_RUNNING);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    if (cal_state != MT6835_CAL_SUCCESS)
    { /* handle failure */
    }
}

/* write speed continuously until cal starts (manually assert AUTOCAL pin to VDD) */
void MT6835_NONBLOCKING_CALIBRATE(MT6835_Handle *dev)
{
    MT6835_CalState cal_state = MT6835_CAL_NONE;
    while (cal_state != MT6835_CAL_RUNNING)
    {
        MT6835_WriteAutoCalSpeed(dev, MT6835_CAL_SPEED_200);
        MT6835_ReadAutoCalStatus(dev, &cal_state);
        osDelay(50);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    /* cal has started, poll until finished */
    do
    {
        osDelay(50);
        MT6835_ReadAutoCalStatus(dev, &cal_state);
    } while (cal_state == MT6835_CAL_RUNNING);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    if (cal_state != MT6835_CAL_SUCCESS)
    { /* handle failure */
    }
}

// ---------------------------------------------------------
// Send the zero-set command (latches current position as zero in RAM)
// The MT6835_CMD_ZERO command tells the chip to sample its current
// angle and store it as the zero reference — no manual reg packing needed
// ---------------------------------------------------------
MT6835_Status MT6835_SendZeroCommand(MT6835_Handle *dev)
{
    uint8_t tx[2] = {0}, rx[2] = {0};
    build_header(MT6835_CMD_ZERO, 0x000, tx);
    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, MT6835_SPI_TIMEOUT_MS);
    cs_high();
    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;
    return MT6835_OK;
}

// ---------------------------------------------------------
// Write zero position explicitly by value (21-bit raw angle)
// Use this if you want to set an arbitrary zero rather than
// zeroing at the current rotor position
// ---------------------------------------------------------
MT6835_Status MT6835_WriteZeroPosition(MT6835_Handle *dev, uint32_t angle_raw)
{
    uint8_t reg_h = (angle_raw >> 13) & 0xFF; // bits [20:13]
    uint8_t reg_m = (angle_raw >> 5) & 0xFF;  // bits [12:5]
    uint8_t reg_l = (angle_raw << 3) & 0xF8;  // bits [4:0] -> [7:3]

    MT6835_Status s;
    s = MT6835_WriteRegister(dev, MT6835_REG_ZERO_H, reg_h);
    if (s != MT6835_OK)
        return s;
    s = MT6835_WriteRegister(dev, MT6835_REG_ZERO_L, reg_m);
    if (s != MT6835_OK)
        return s;
    s = MT6835_WriteRegister(dev, MT6835_REG_UVW, reg_l);
    if (s != MT6835_OK)
        return s;
    return MT6835_OK;
}

// ---------------------------------------------------------
// Burn zero position (and any other register changes) to EEPROM
// WARNING: limited write cycles (~10k). Only call when you intend
// to make the zero permanent across power cycles.
// ---------------------------------------------------------
MT6835_Status MT6835_BurnEEPROM(MT6835_Handle *dev)
{
    uint8_t tx[2] = {0}, rx[2] = {0};
    build_header(MT6835_CMD_PROG, 0x000, tx);
    cs_low();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, MT6835_SPI_TIMEOUT_MS);
    cs_high();
    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;
    // Datasheet specifies ~20ms programming time — do not talk to device during this
    HAL_Delay(20);
    return MT6835_OK;
}
/* ── Angle reads ────────────────────────────────────────────────────────── */

MT6835_Status MT6835_ReadAngle(MT6835_Handle *dev, MT6835_AngleResult *result)
{
    uint8_t data[6] = {0};
    build_header(MT6835_CMD_BURST, MT6835_REG_ANGLE_H, data);

    cs_low();
    uint16_t timestamp = __HAL_TIM_GET_COUNTER(dev->timestamp_timer);
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(dev->hspi, data, data, 6, MT6835_SPI_TIMEOUT_MS);
    cs_high();

    if (rc == HAL_TIMEOUT)
        return MT6835_ERR_TIMEOUT;
    if (rc != HAL_OK)
        return MT6835_ERR_SPI;

    parse_burst(result, data, timestamp);

    result->crc_computed = MT6835_ComputeCRC(&data[2]);
    if (result->crc_computed != result->crc_received)
        return MT6835_ERR_CRC;
    if (result->status & MT6835_STATUS_MAG_WEAK)
        return MT6835_ERR_MAG_WEAK;
    if (result->status & MT6835_STATUS_UNDERVOLT)
        return MT6835_ERR_MAG_OVER;

    return MT6835_OK;
}

/**
 * Blocking burst read via LL tight-loop (~10 µs at 10 MHz).  No CRC.
 *
 * The STM32G4 SPI peripheral has a 32-bit (4-byte) FIFO. The 4-byte dummy TX
 * loop runs to completion before the RX loop drains it — this is safe because
 * the FIFO can absorb all 4 bytes without overrun.  Do not reorder the loops.
 *
 * Bytes land in dma_rx[2..5] to match the burst buffer layout expected by
 * parse_burst, with [0..1] left as don't-care (echoed header, not used).
 */
MT6835_Status MT6835_ReadAngleFast(MT6835_Handle *dev, MT6835_AngleResult *result)
{
    SPI_TypeDef *spi = dev->hspi->Instance;

    cs_low();
    uint16_t timestamp = __HAL_TIM_GET_COUNTER(dev->timestamp_timer);

    /* Send 2-byte burst command */
    while (!LL_SPI_IsActiveFlag_TXE(spi))
        ;
    LL_SPI_TransmitData8(spi, 0xA0);
    while (!LL_SPI_IsActiveFlag_TXE(spi))
        ;
    LL_SPI_TransmitData8(spi, 0x03);

    /* Flush RX echoes from command phase */
    while (!LL_SPI_IsActiveFlag_RXNE(spi))
        ;
    LL_SPI_ReceiveData8(spi);
    while (!LL_SPI_IsActiveFlag_RXNE(spi))
        ;
    LL_SPI_ReceiveData8(spi);

    /* Clock out 4 dummy bytes then drain RX — FIFO absorbs TX burst */
    for (int i = 0; i < 4; i++)
    {
        while (!LL_SPI_IsActiveFlag_TXE(spi))
            ;
        LL_SPI_TransmitData8(spi, 0x00);
    }
    for (int i = 0; i < 4; i++)
    {
        while (!LL_SPI_IsActiveFlag_RXNE(spi))
            ;
        dev->dma_rx[2 + i] = LL_SPI_ReceiveData8(spi);
    }

    while (LL_SPI_IsActiveFlag_BSY(spi))
        ;
    cs_high();

    parse_burst(result, dev->dma_rx, timestamp);

    if (result->status & MT6835_STATUS_MAG_WEAK)
        return MT6835_ERR_MAG_WEAK;
    if (result->status & MT6835_STATUS_UNDERVOLT)
        return MT6835_ERR_MAG_OVER;

    return MT6835_OK;
}
#define ANGLE_AVG_SAMPLES 256
uint32_t MT6835_read_angle_avg(MT6835_Handle *dev)
{
    MT6835_AngleResult result;

    // Get reference sample
    while (MT6835_ReadAngle(dev, &result) != MT6835_OK)
        ;
    uint32_t ref = result.raw;

    int32_t sum = 0;
    int count = 1;

    for (int i = 1; i < ANGLE_AVG_SAMPLES; i++)
    {
        if (MT6835_ReadAngle(dev, &result) != MT6835_OK)
            continue;

        // Sign-extend 21-bit delta to handle wraparound
        int32_t delta = (int32_t)((result.raw - ref) << 11) >> 11;
        sum += delta;
        count++;
    }

    return (uint32_t)(ref + (sum / count)) & 0x1FFFFF;
}
/* ── DMA non-blocking path ──────────────────────────────────────────────── */

/**
 * Kick off a non-blocking DMA burst read.
 *
 * Timestamp is captured immediately after cs_low() and stored in
 * dev->pending_timestamp.  The IRQ handler consumes it via parse_burst.
 * dma_busy is the gate: StartDMA must not be called while dma_busy is set,
 * which also ensures pending_timestamp is not overwritten before it is read.
 */
void MT6835_StartDMA(void)
{
    s_dev->dma_busy = 1;
    cs_low();
    s_dev->pending_timestamp = __HAL_TIM_GET_COUNTER(s_dev->timestamp_timer);

    /* Rearm transfer counts — addresses stay set from LowLevelInit */
    LL_DMA_DisableChannel(s_dma, s_ch_tx);
    LL_DMA_DisableChannel(s_dma, s_ch_rx);
    LL_DMA_SetDataLength(s_dma, s_ch_tx, 6);
    LL_DMA_SetDataLength(s_dma, s_ch_rx, 6);
    LL_DMA_EnableChannel(s_dma, s_ch_rx); /* RX first to avoid overrun */
    LL_DMA_EnableChannel(s_dma, s_ch_tx);

    // // Equivalent to DisableChannel + SetDataLength + EnableChannel
    // // but direct register writes — saves ~few cycles per call
    // DMA2_Channel1->CCR &= ~DMA_CCR_EN;
    // DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    // DMA2_Channel1->CNDTR = 6;
    // DMA2_Channel2->CNDTR = 6;
    // // cs_low();
    // DMA2_Channel2->CCR |= DMA_CCR_EN;
    // DMA2_Channel1->CCR |= DMA_CCR_EN;
}

/**
 * Call from the DMA RX transfer-complete ISR:
 *
 *   void DMA2_Channel2_IRQHandler(void) { MT6835_DMA_IRQHandler(); }
 *
 * To inline: copy the body directly into the ISR.
 *
 * dma_rx layout:
 *   [0..1] echoed header (ignored)
 *   [2]    ANGLE[20:13]
 *   [3]    ANGLE[12:5]
 *   [4]    ANGLE[4:0] | STATUS[2:0]
 *   [5]    CRC
 */
// #include "profiling.h"
void MT6835_DMA_IRQHandler(void)
{
    LL_DMA_ClearFlag_TC2(s_dma);
    LL_DMA_ClearFlag_GI2(s_dma);

    parse_burst(s_result, (const uint8_t *)s_dev->dma_rx, s_dev->pending_timestamp);

    cs_high();
    s_dev->dma_busy = 0;
}

bool MT6835_DMA_Ready(void)
{
    return !s_dev->dma_busy;
}