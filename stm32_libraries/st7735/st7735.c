/* vim: set ai et ts=4 sw=4: */
/**
 * ST7735 framebuffer driver for STM32G4
 *
 * Architecture
 * ------------
 * All draw calls (FillRectangle, WriteString, DrawImage, …) write into a
 * 16-bit RGB565 framebuffer in SRAM.  Nothing touches the SPI bus during
 * rendering.  When the frame is complete, ST7735_Flush() sets the full-screen
 * address window once and fires a single DMA transfer of the entire
 * framebuffer (~25 KB for 160×80).  The DMA TC callback clears a flag so the
 * application can detect completion via ST7735_FlushWait().
 *
 * Frame rate
 * ----------
 * FRMCTR1 is set to RTNA=0x06, FPA=0x01, BPA=0x01 which drives the display's
 * internal scan at ~100 Hz — the practical maximum for the ST7735/ST7735S
 * without overclocking.  At 20 MHz SPI the full-screen DMA flush takes:
 *   160×80×2 bytes × 8 bits / 20 MHz ≈ 10.2 ms  (~98 Hz achievable)
 *
 * Usage pattern in main loop
 * --------------------------
 *   while (dma_busy);  // wait for in-flight DMA          // wait for previous flush to finish
 *   // … render into st7735_fb …
 *   ST7735_Flush();              // fire next DMA transfer, returns immediately
 */

#include "st7735.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Framebuffer definition
// ---------------------------------------------------------------------------
// Override ST7735_FB_ATTR in st7735.h (or before the #include) to relocate
// this to a DMA-accessible SRAM section if your linker script defines one.
ST7735_FB_ATTR uint16_t st7735_fb[ST7735_FB_SIZE];

// ---------------------------------------------------------------------------
// DMA state
// ---------------------------------------------------------------------------
static volatile uint8_t dma_busy = 0; // 1 while DMA is transferring

// ---------------------------------------------------------------------------
// Internal helpers – forward declarations
// ---------------------------------------------------------------------------
static void ST7735_Select(void);
static void ST7735_Reset(void);
static void ST7735_WriteCommand(uint8_t cmd);
static void ST7735_WriteData(const uint8_t *buff, size_t len);
static void ST7735_ExecuteCommandList(const uint8_t *addr);
static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0,
                                    uint8_t x1, uint8_t y1);

// ---------------------------------------------------------------------------
// Init command tables
// ---------------------------------------------------------------------------
#define DELAY 0x80

static const uint8_t init_cmds1[] = {
    15,
    ST7735_SWRESET,
    DELAY,
    150,
    ST7735_SLPOUT,
    DELAY,
    255,
    // FRMCTR1: RTNA=0x06, FPA=0x01, BPA=0x01 → ~100 Hz (max practical rate)
    // Default was 0x01,0x2C,0x2D which gives ~60 Hz.
    ST7735_FRMCTR1,
    3,
    0x06,
    0x01,
    0x01,
    ST7735_FRMCTR2,
    3,
    0x06,
    0x01,
    0x01,
    ST7735_FRMCTR3,
    6,
    0x06,
    0x01,
    0x01,
    0x06,
    0x01,
    0x01,
    ST7735_INVCTR,
    1,
    0x07,
    ST7735_PWCTR1,
    3,
    0xA2,
    0x02,
    0x84,
    ST7735_PWCTR2,
    1,
    0xC5,
    ST7735_PWCTR3,
    2,
    0x0A,
    0x00,
    ST7735_PWCTR4,
    2,
    0x8A,
    0x2A,
    ST7735_PWCTR5,
    2,
    0x8A,
    0xEE,
    ST7735_VMCTR1,
    1,
    0x0E,
    ST7735_INVOFF,
    0,
    ST7735_MADCTL,
    1,
    ST7735_ROTATION,
    ST7735_COLMOD,
    1,
    0x05, // 16-bit/pixel RGB565
};

#if defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128)
static const uint8_t init_cmds2[] = {
    2,
    ST7735_CASET,
    4,
    0x00,
    0x00,
    0x00,
    0x7F,
    ST7735_RASET,
    4,
    0x00,
    0x00,
    0x00,
    0x7F,
};
#elif defined(ST7735_IS_160X80)
static const uint8_t init_cmds2[] = {
    3,
    ST7735_CASET,
    4,
    0x00,
    0x00,
    0x00,
    0x4F,
    ST7735_RASET,
    4,
    0x00,
    0x00,
    0x00,
    0x9F,
    ST7735_INVON,
    0,
};
#else
static const uint8_t init_cmds2[] = {
    2,
    ST7735_CASET,
    4,
    0x00,
    0x00,
    0x00,
    0x9F,
    ST7735_RASET,
    4,
    0x00,
    0x00,
    0x00,
    0x7F,
};
#endif

static const uint8_t init_cmds3[] = {
    4,
    ST7735_GMCTRP1,
    16,
    0x02,
    0x1c,
    0x07,
    0x12,
    0x37,
    0x32,
    0x29,
    0x2d,
    0x29,
    0x25,
    0x2B,
    0x39,
    0x00,
    0x01,
    0x03,
    0x10,
    ST7735_GMCTRN1,
    16,
    0x03,
    0x1d,
    0x07,
    0x06,
    0x2E,
    0x2C,
    0x29,
    0x2D,
    0x2E,
    0x2E,
    0x37,
    0x3F,
    0x00,
    0x00,
    0x02,
    0x10,
    ST7735_NORON,
    DELAY,
    10,
    ST7735_DISPON,
    DELAY,
    100,
};

// ---------------------------------------------------------------------------
// GPIO / SPI primitives
// ---------------------------------------------------------------------------

static void ST7735_Select(void)
{
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}

void ST7735_Unselect(void)
{
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_Reset(void)
{
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_SET);
}

static void ST7735_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
}

static void ST7735_WriteData(const uint8_t *buff, size_t len)
{
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, (uint8_t *)buff, len, HAL_MAX_DELAY);
}

static void ST7735_ExecuteCommandList(const uint8_t *addr)
{
    uint8_t numCommands = *addr++;
    while (numCommands--)
    {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(cmd);
        uint8_t numArgs = *addr++;
        uint16_t hasDelay = numArgs & DELAY;
        numArgs &= ~DELAY;
        if (numArgs)
        {
            ST7735_WriteData(addr, numArgs);
            addr += numArgs;
        }
        if (hasDelay)
        {
            uint16_t ms = *addr++;
            if (ms == 255)
                ms = 500;
            HAL_Delay(ms);
        }
    }
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0,
                                    uint8_t x1, uint8_t y1)
{
    ST7735_WriteCommand(ST7735_CASET);
    uint8_t col[] = {0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART};
    ST7735_WriteData(col, sizeof(col));

    ST7735_WriteCommand(ST7735_RASET);
    uint8_t row[] = {0x00, y0 + ST7735_YSTART, 0x00, y1 + ST7735_YSTART};
    ST7735_WriteData(row, sizeof(row));

    ST7735_WriteCommand(ST7735_RAMWR);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void ST7735_Init(void)
{
    dma_busy = 0;
    memset(st7735_fb, 0, ST7735_FB_BYTES);

    ST7735_Select();
    ST7735_Reset();
    ST7735_ExecuteCommandList(init_cmds1);
    ST7735_ExecuteCommandList(init_cmds2);
    ST7735_ExecuteCommandList(init_cmds3);
    ST7735_Unselect();
}

// ---------------------------------------------------------------------------
// DMA callback – called from HAL_SPI_TxCpltCallback in your application:
//
//   void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//       ST7735_DMA_TxCpltCallback(hspi);
//   }
// ---------------------------------------------------------------------------

void ST7735_DMA_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &ST7735_SPI_PORT)
    {
        ST7735_Unselect();
        dma_busy = 0;
    }
}

// ---------------------------------------------------------------------------
// Flush – blast the entire framebuffer to the display in one DMA transfer.
//
// The framebuffer is stored as native-endian uint16_t (RGB565).  The ST7735
// expects big-endian (MSB first) over SPI.  On a little-endian Cortex-M this
// means each pixel must be byte-swapped before transmission.
//
// Rather than maintaining a separate byte-swapped shadow buffer (doubling
// RAM usage), we exploit the fact that HAL_SPI_Transmit_DMA on the G4 uses
// byte-width transfers.  The ST7735 treats the two bytes of each pixel as
// [D15:D8] then [D7:D0], so we simply swap each pixel in-place before the
// DMA and swap back in the TC callback — but that adds CPU time before and
// after every flush.
//
// The cleaner solution (used here) is to store the framebuffer in
// byte-swapped order from the start.  All draw primitives write
// __REV16(color) into the buffer so the bytes are already in wire order.
// ST7735_PutPixel and the helpers all do this swap.  The raw fb[] is then
// sent byte-by-byte over DMA with no additional processing.
// ---------------------------------------------------------------------------

// Internal helper that actually arms the DMA – caller must ensure the bus
// is idle before calling.
static void ST7735_StartDMA(void)
{
    ST7735_Select();
    ST7735_SetAddressWindow(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    dma_busy = 1;
    HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT,
                         (uint8_t *)st7735_fb,
                         ST7735_FB_BYTES);
}

ST7735_StatusTypeDef ST7735_FlushReady(void)
{
    return dma_busy ? ST7735_BUSY : ST7735_OK;
}

ST7735_StatusTypeDef ST7735_Flush(void)
{
    if (dma_busy)
        return ST7735_BUSY;
    ST7735_StartDMA();
    return ST7735_OK;
}

ST7735_StatusTypeDef ST7735_FlushBlocking(void)
{
    while (dma_busy)
        ; // swap for __WFE()/__SEV() if you want to sleep the core
    ST7735_StartDMA();
    return ST7735_OK;
}

// ---------------------------------------------------------------------------
// Draw primitives – all write to st7735_fb[], NOT to SPI.
//
// Pixels are stored byte-swapped (big-endian wire order) so the DMA can send
// the buffer directly without any additional processing.
// ---------------------------------------------------------------------------

// Inline helper: swap bytes of a 16-bit colour for wire-order storage.
static inline uint16_t swap16(uint16_t c)
{
    return (c >> 8) | (c << 8);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT)
        return;
    st7735_fb[y * ST7735_WIDTH + x] = swap16(color);
}

void ST7735_FillRectangle(uint16_t x, uint16_t y,
                          uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT)
        return;
    if (x + w > ST7735_WIDTH)
        w = ST7735_WIDTH - x;
    if (y + h > ST7735_HEIGHT)
        h = ST7735_HEIGHT - y;
    if (w == 0 || h == 0)
        return;

    uint16_t c = swap16(color);
    for (uint16_t row = y; row < y + h; row++)
    {
        uint16_t *dst = &st7735_fb[row * ST7735_WIDTH + x];
        for (uint16_t col = 0; col < w; col++)
            dst[col] = c;
    }
}

// Fast alias – same implementation now that it's all in RAM.
void ST7735_FillRectangleFast(uint16_t x, uint16_t y,
                              uint16_t w, uint16_t h, uint16_t color)
{
    ST7735_FillRectangle(x, y, w, h, color);
}

void ST7735_FillScreen(uint16_t color)
{
    // Whole buffer – use memset-style word fill for speed.
    uint16_t c = swap16(color);
    for (uint32_t i = 0; i < ST7735_FB_SIZE; i++)
        st7735_fb[i] = c;
}

void ST7735_FillScreenFast(uint16_t color)
{
    ST7735_FillScreen(color);
}

void ST7735_DrawImage(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h, const uint16_t *data)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT)
        return;
    if (x + w > ST7735_WIDTH || y + h > ST7735_HEIGHT)
        return;
    if (w == 0 || h == 0)
        return;

    for (uint16_t row = 0; row < h; row++)
    {
        uint16_t *dst = &st7735_fb[(y + row) * ST7735_WIDTH + x];
        const uint16_t *src = &data[row * w];
        for (uint16_t col = 0; col < w; col++)
            dst[col] = swap16(src[col]);
    }
}

void ST7735_WriteString(uint16_t x, uint16_t y, const char *str,
                        FontDef font, uint16_t color, uint16_t bgcolor)
{
    uint16_t cx = swap16(color);
    uint16_t bx = swap16(bgcolor);

    while (*str)
    {
        // wrap to next line if this character won't fit
        if (x + font.width > ST7735_WIDTH) // was >=, now > (off-by-one fix)
        {
            x = 0;
            y += font.height;
            if (y + font.height > ST7735_HEIGHT)
                break;
            if (*str == ' ')
            {
                str++;
                continue;
            }
        }

        const uint32_t glyph_offset = (uint32_t)(*str - 32) * font.height;

        for (uint32_t row = 0; row < font.height; row++)
        {
            if (y + row >= ST7735_HEIGHT)
                break;

            // load as same type as font.data to avoid width assumptions
            uint16_t bits = font.data[glyph_offset + row];
            uint16_t *dst = &st7735_fb[(y + row) * ST7735_WIDTH + x];

            for (uint32_t col = 0; col < font.width; col++)
            {
                if (x + col >= ST7735_WIDTH)
                    break;
                // shift MSB into position for each column
                dst[col] = (bits & (0x8000u >> col)) ? cx : bx;
            }
        }

        x += font.width;
        str++;
    }
}
// ---------------------------------------------------------------------------
// Display control – these go directly to SPI (not framebuffer).
// Wait for any flush to finish first so we don't corrupt a DMA transfer.
// ---------------------------------------------------------------------------

void ST7735_InvertColors(bool invert)
{
    while (dma_busy)
        ; // wait for in-flight DMA
    ST7735_Select();
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect();
}

void ST7735_SetGamma(GammaDef gamma)
{
    while (dma_busy)
        ; // wait for in-flight DMA
    ST7735_Select();
    ST7735_WriteCommand(ST7735_GAMSET);
    ST7735_WriteData((uint8_t *)&gamma, sizeof(gamma));
    ST7735_Unselect();
}