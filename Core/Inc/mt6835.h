/**
 * @file    mt6835.h
 * @brief   STM32 HAL/LL driver for the MagnTek MT6835 21-bit AMR magnetic rotary encoder
 *
 * Interface:  4-wire SPI, Mode 3 (CPOL=1, CPHA=1), up to 16 MHz
 * Resolution: 21-bit absolute angle (2,097,152 counts/rev)
 *
 * SPI frame format (24-bit, MSB first):
 *   [23:20] Command nibble  [19:8] 12-bit address  [7:0] Data byte
 *
 * Burst read registers (latched atomically on falling CSN):
 *   0x003 – ANGLE[20:13]
 *   0x004 – ANGLE[12:5]
 *   0x005 – ANGLE[4:0] | STATUS[2:0]
 *   0x006 – CRC-8 (poly 0x07, MSB-first, over 0x003..0x005)
 */

#ifndef MT6835_H
#define MT6835_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_dma.h"
#include <stdint.h>
#include <stdbool.h>

    /* ── Register addresses ─────────────────────────────────────────────────── */

#define MT6835_REG_ANGLE_H 0x003
#define MT6835_REG_ANGLE_M 0x004
#define MT6835_REG_ANGLE_L 0x005
#define MT6835_REG_CRC 0x006
#define MT6835_REG_ABZ_RES_H 0x007
#define MT6835_REG_ABZ_RES_L 0x008
#define MT6835_REG_ZERO_H 0x009
#define MT6835_REG_ZERO_L 0x00A
#define MT6835_REG_UVW 0x00B
#define MT6835_REG_PWM 0x00C
#define MT6835_REG_HYST 0x00D
#define MT6835_REG_BW 0x011

#define MT6835_REG_AUTOCAL 0x00E
#define MT6835_REG_AUTOCAL_STAT 0x113

    typedef enum
    {
        MT6835_CAL_NONE = 0x00,    /* 0b00 – no calibration       */
        MT6835_CAL_RUNNING = 0x01, /* 0b01 – running              */
        MT6835_CAL_FAILED = 0x02,  /* 0b10 – failed               */
        MT6835_CAL_SUCCESS = 0x03, /* 0b11 – successful           */
    } MT6835_CalState;

    typedef enum
    {
        MT6835_CAL_SPEED_3200 = 0x0, /* 3200 ≤ speed < 6400 RPM */
        MT6835_CAL_SPEED_1600 = 0x1, /* 1600 ≤ speed < 3200 RPM */
        MT6835_CAL_SPEED_800 = 0x2,  /*  800 ≤ speed < 1600 RPM */
        MT6835_CAL_SPEED_400 = 0x3,  /*  400 ≤ speed <  800 RPM */
        MT6835_CAL_SPEED_200 = 0x4,  /*  200 ≤ speed <  400 RPM */
        MT6835_CAL_SPEED_100 = 0x5,  /*  100 ≤ speed <  200 RPM */
        MT6835_CAL_SPEED_50 = 0x6,   /*   50 ≤ speed <  100 RPM */
        MT6835_CAL_SPEED_25 = 0x7,   /*   25 ≤ speed <   50 RPM */
    } MT6835_CalSpeed;

    /* ── Bit masks / shifts (always read-modify-write to preserve adjacent bits) */

#define MT6835_HYST_MASK 0x07
#define MT6835_HYST_SHIFT 0
#define MT6835_ROT_DIR_MASK 0x08
#define MT6835_ROT_DIR_SHIFT 3
#define MT6835_BW_MASK 0x07
#define MT6835_BW_SHIFT 0
#define MT6835_AUTOCAL_FREQ_MASK 0x70
#define MT6835_AUTOCAL_FREQ_SHIFT 4
#define MT6835_GPIO_DS_MASK 0x80
#define MT6835_GPIO_DS_SHIFT 7
#define MT6835_AB_SWAP_MASK 0x01
#define MT6835_ABZ_OFF_MASK 0x02
#define MT6835_Z_PUL_WID_MASK 0x07
#define MT6835_Z_EDGE_MASK 0x08

    /* ── HYST values (reg 0x00D bits [2:0]) ────────────────────────────────── */

#define MT6835_HYST_0_022DEG 0x00
#define MT6835_HYST_0_044DEG 0x01
#define MT6835_HYST_0_088DEG 0x02
#define MT6835_HYST_0_176DEG 0x03
#define MT6835_HYST_0DEG 0x04
#define MT6835_HYST_0_003DEG 0x05
#define MT6835_HYST_0_006DEG 0x06
#define MT6835_HYST_0_011DEG 0x07 /* factory default */

    /* ── BW values (reg 0x011 bits [2:0]) ──────────────────────────────────── */

#define MT6835_BW_BASELINE 0x00 /* slowest, best noise  */
#define MT6835_BW_X2 0x01
#define MT6835_BW_X4 0x02
#define MT6835_BW_X8 0x03
#define MT6835_BW_X16 0x04
#define MT6835_BW_X32 0x05 /* factory default      */
#define MT6835_BW_X64 0x06
#define MT6835_BW_X128 0x07 /* fastest, most noise  */

    /* ── SPI command nibbles [23:20] ────────────────────────────────────────── */

#define MT6835_CMD_READ 0x03
#define MT6835_CMD_WRITE 0x06
#define MT6835_CMD_PROG 0x0C
#define MT6835_CMD_ZERO 0x05
#define MT6835_CMD_BURST 0x0A

    /* ── STATUS bits (reg 0x005 bits [2:0]) ─────────────────────────────────── */

#define MT6835_STATUS_OVERSPEED (1U << 0)
#define MT6835_STATUS_MAG_WEAK (1U << 1)
#define MT6835_STATUS_UNDERVOLT (1U << 2)

    /* ── Conversion helpers ─────────────────────────────────────────────────── */

#define MT6835_MAX_COUNTS (1UL << 21)
#define MT6835_COUNTS_TO_DEG(c) ((c) * 360.0f / MT6835_MAX_COUNTS)
#define MT6835_COUNTS_TO_RAD(c) ((c) * 6.28318530718f / MT6835_MAX_COUNTS)
#define MT6835_SPI_TIMEOUT_MS 10

    /* ── Types ──────────────────────────────────────────────────────────────── */

    typedef enum
    {
        MT6835_OK = 0,
        MT6835_ERR_SPI = -1,
        MT6835_ERR_CRC = -2,
        MT6835_ERR_MAG_WEAK = -3,
        MT6835_ERR_MAG_OVER = -4,
        MT6835_ERR_TIMEOUT = -5,
    } MT6835_Status;

    typedef struct
    {
        SPI_HandleTypeDef *hspi;
        GPIO_TypeDef *cs_port;
        uint16_t cs_pin;
        TIM_HandleTypeDef *timestamp_timer;
        /* Timestamp captured at cs_low; written by StartDMA, consumed by IRQ.
         * volatile because it is written in task context and read in ISR context. */
        volatile uint32_t pending_timestamp;
        /* DMA buffers — dma_tx is constant after Init; dma_rx written by DMA/LL. */
        volatile uint8_t dma_busy;
        uint8_t dma_tx[6];
        uint8_t dma_rx[6];
    } MT6835_Handle;

    typedef struct
    {
        uint32_t timestamp; /* Timer count captured at falling CSN edge     */
        uint32_t raw;       /* 21-bit count [0, 2097151]                    */
        uint8_t status;     /* MT6835_STATUS_* flags                        */
        uint8_t crc_received;
        uint8_t crc_computed;
    } MT6835_AngleResult;

    /* ── API ────────────────────────────────────────────────────────────────── */

    /* Initialise handle, deassert CS, apply default BW/HYST. */
    void MT6835_Init(MT6835_Handle *dev,
                     SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                     TIM_HandleTypeDef *timestamp_timer);

    /**
     * Wire DMA channels to the handle's buffers and enable SPI DMA requests.
     * Stores dma/channel args internally — StartDMA and the IRQ handler need
     * no further arguments.  Call after MT6835_Init.
     *
     * Also registers the result struct that MT6835_DMA_IRQHandler will write to.
     */
    void MT6835_LowLevelInit(MT6835_Handle *dev,
                             MT6835_AngleResult *result,
                             DMA_TypeDef *dma,
                             uint32_t ch_tx,
                             uint32_t ch_rx);

    /* Register R/W */
    MT6835_Status MT6835_ReadRegister(MT6835_Handle *dev, uint16_t addr, uint8_t *data);
    MT6835_Status MT6835_WriteRegister(MT6835_Handle *dev, uint16_t addr, uint8_t data);

    /* Read-modify-write a sub-byte field without disturbing adjacent bits. */
    MT6835_Status MT6835_ConfigureField(MT6835_Handle *dev,
                                        uint16_t addr,
                                        uint8_t mask, uint8_t shift,
                                        uint8_t value);

    /* Burn shadow registers to EEPROM.  Max 1000 cycles; hold power ≥6 s after. */
    MT6835_Status MT6835_ProgramEEPROM(MT6835_Handle *dev);

    /* Latch current angle as zero (volatile).  Call ProgramEEPROM to persist. */
    MT6835_Status MT6835_SetZeroPosition(MT6835_Handle *dev);

    MT6835_Status MT6835_WriteAutoCalSpeed(MT6835_Handle *dev, MT6835_CalSpeed speed);
    MT6835_Status MT6835_ReadAutoCalStatus(MT6835_Handle *dev, MT6835_CalState *state);
    void MT6835_BLOCKING_CALIBRATE(MT6835_Handle *dev);
    void MT6835_NONBLOCKING_CALIBRATE(MT6835_Handle *dev);

    MT6835_Status MT6835_SendZeroCommand(MT6835_Handle *dev);
    MT6835_Status MT6835_WriteZeroPosition(MT6835_Handle *dev, uint32_t angle_raw);
    MT6835_Status MT6835_BurnEEPROM(MT6835_Handle *dev);

    /* Blocking burst read with CRC check. */
    MT6835_Status MT6835_ReadAngle(MT6835_Handle *dev, MT6835_AngleResult *result);

    /**
     * Blocking burst read via LL tight-loop (~10 µs at 10 MHz).  No CRC.
     *
     * The STM32G4 SPI FIFO is 32-bit deep; the 4-byte dummy TX / RX loops below
     * rely on it to avoid overrun — do not reorder them.
     */
    MT6835_Status MT6835_ReadAngleFast(MT6835_Handle *dev, MT6835_AngleResult *result);
    uint32_t MT6835_read_angle_avg(MT6835_Handle *dev);

    /* Kick off a non-blocking DMA burst read. */
    void MT6835_StartDMA(void);

    /**
     * Call directly from the DMA RX transfer-complete ISR — no externs needed:
     *
     *   void DMA2_Channel2_IRQHandler(void) { MT6835_DMA_IRQHandler(); }
     *
     * Clears DMA flags, parses dma_rx, deasserts CS, clears dma_busy.
     * To inline: copy the function body into the ISR.
     */
    void MT6835_DMA_IRQHandler(void);

    bool MT6835_DMA_Ready(void);

    /* CRC-8 (poly 0x07, init 0x00, MSB-first) over 3 bytes [0x003..0x005]. */
    uint8_t MT6835_ComputeCRC(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MT6835_H */