/* vim: set ai et ts=4 sw=4: */
#ifndef __ST7735_H__
#define __ST7735_H__

#include "fonts.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"

#define ST7735_MADCTL_MY 0x80
#define ST7735_MADCTL_MX 0x40
#define ST7735_MADCTL_MV 0x20
#define ST7735_MADCTL_ML 0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

/*** Redefine if necessary ***/
#define ST7735_SPI_PORT hspi2
extern SPI_HandleTypeDef ST7735_SPI_PORT;

#define ST7735_RES_Pin GPIO_PIN_0
#define ST7735_RES_GPIO_Port GPIOC
#define ST7735_CS_Pin GPIO_PIN_0
#define ST7735_CS_GPIO_Port GPIOB
#define ST7735_DC_Pin GPIO_PIN_1
#define ST7735_DC_GPIO_Port GPIOC

// AliExpress/eBay 1.8" display, default orientation
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY)
*/

// AliExpress/eBay 1.8" display, rotate right
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 128
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (ST7735_MADCTL_MY | ST7735_MADCTL_MV)
*/

// AliExpress/eBay 1.8" display, rotate left
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 128
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MV)
*/

// AliExpress/eBay 1.8" display, upside down
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (0)
*/

// WaveShare ST7735S-based 1.8" display, default orientation
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
#define ST7735_XSTART 2
#define ST7735_YSTART 1
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB)
*/

// WaveShare ST7735S-based 1.8" display, rotate right
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 128
#define ST7735_XSTART 1
#define ST7735_YSTART 2
#define ST7735_ROTATION (ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB)
*/

// WaveShare ST7735S-based 1.8" display, rotate left
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 128
#define ST7735_XSTART 1
#define ST7735_YSTART 2
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB)
*/

// WaveShare ST7735S-based 1.8" display, upside down
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
#define ST7735_XSTART 2
#define ST7735_YSTART 1
#define ST7735_ROTATION (ST7735_MADCTL_RGB)
*/

// 1.44" display, default orientation
// #define ST7735_IS_128X128 1
// #define ST7735_WIDTH  128
// #define ST7735_HEIGHT 128
// #define ST7735_XSTART 2
// #define ST7735_YSTART 3
// #define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR)

// 1.44" display, rotate right
/*
#define ST7735_IS_128X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 128
#define ST7735_XSTART 3
#define ST7735_YSTART 2
#define ST7735_ROTATION (ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR)
*/

// 1.44" display, rotate left
/*
#define ST7735_IS_128X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 128
#define ST7735_XSTART 1
#define ST7735_YSTART 2
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR)
*/

// 1.44" display, upside down
/*
#define ST7735_IS_128X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 128
#define ST7735_XSTART 2
#define ST7735_YSTART 1
#define ST7735_ROTATION (ST7735_MADCTL_BGR)
*/

// mini 160x80 display (it's unlikely you want the default orientation)
/*
#define ST7735_IS_160X80 1
#define ST7735_XSTART 26
#define ST7735_YSTART 1
#define ST7735_WIDTH  80
#define ST7735_HEIGHT 160
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR)
*/

// mini 160x80, rotate left
// /*
#define ST7735_IS_160X80 1
#define ST7735_XSTART 1
#define ST7735_YSTART 26
#define ST7735_WIDTH 160
#define ST7735_HEIGHT 80
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR)
// */

// mini 160x80, rotate right
/*
#define ST7735_IS_160X80 1
#define ST7735_XSTART 1
#define ST7735_YSTART 26
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 80
#define ST7735_ROTATION (ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR)
*/

/****************************/

// ---------------------------------------------------------------------------
// Framebuffer
//
// The full screen is rendered into fb[] in RAM, then flushed to the display
// in a single DMA transfer — no per-pixel or per-rectangle SPI activity
// during rendering, and no inter-scanline gaps during the flush.
//
// Size: ST7735_WIDTH * ST7735_HEIGHT * 2 bytes (RGB565, 16-bit/pixel)
//   160×80  = 25,600 bytes  (25 KB)
//   128×128 = 32,768 bytes  (32 KB)
//   160×128 = 40,960 bytes  (40 KB)
//
// If your linker script defines a .fb_ram section in DMA-accessible SRAM,
// override the placement macro before including this header:
//   #define ST7735_FB_ATTR __attribute__((section(".fb_ram"), aligned(4)))
// ---------------------------------------------------------------------------
#ifndef ST7735_FB_ATTR
#define ST7735_FB_ATTR __attribute__((aligned(4)))
#endif

#define ST7735_FB_SIZE (ST7735_WIDTH * ST7735_HEIGHT) /* pixels */
#define ST7735_FB_BYTES (ST7735_FB_SIZE * 2u)         /* bytes  */

/****************************/

#define ST7735_NOP 0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID 0x04
#define ST7735_RDDST 0x09

#define ST7735_SLPIN 0x10
#define ST7735_SLPOUT 0x11
#define ST7735_PTLON 0x12
#define ST7735_NORON 0x13

#define ST7735_INVOFF 0x20
#define ST7735_INVON 0x21
#define ST7735_GAMSET 0x26
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON 0x29
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_RAMRD 0x2E

#define ST7735_PTLAR 0x30
#define ST7735_COLMOD 0x3A
#define ST7735_MADCTL 0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_RDID1 0xDA
#define ST7735_RDID2 0xDB
#define ST7735_RDID3 0xDC
#define ST7735_RDID4 0xDD

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions (RGB565)
#define ST7735_BLACK 0x0000
#define ST7735_BLUE 0x001F
#define ST7735_RED 0xF800
#define ST7735_GREEN 0x07E0
#define ST7735_CYAN 0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW 0xFFE0
#define ST7735_WHITE 0xFFFF
#define ST7735_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

typedef enum
{
    GAMMA_10 = 0x01,
    GAMMA_25 = 0x02,
    GAMMA_22 = 0x04,
    GAMMA_18 = 0x08
} GammaDef;

#ifdef __cplusplus
extern "C"
{
#endif

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    // Call before initializing any other SPI device to deassert CS.
    void ST7735_Unselect(void);

    // Hardware reset + register init.  Blocks until complete (~700 ms).
    void ST7735_Init(void);

    // Must be forwarded from HAL_SPI_TxCpltCallback() in your application:
    //
    //   void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    //       ST7735_DMA_TxCpltCallback(hspi);
    //   }
    void ST7735_DMA_TxCpltCallback(SPI_HandleTypeDef *hspi);

    // -----------------------------------------------------------------------
    // Framebuffer – draw into RAM, then flush once
    // -----------------------------------------------------------------------

    // Direct access to the framebuffer.  Write RGB565 pixels here, then call
    // ST7735_Flush().  Pixel at (x, y) is fb[y * ST7735_WIDTH + x].
    extern uint16_t st7735_fb[ST7735_FB_SIZE];

// Convenience macro – set one pixel in the framebuffer.
#define ST7735_PutPixel(x, y, color) \
    (st7735_fb[(y) * ST7735_WIDTH + (x)] = (color))

    // Return type for flush operations – mirrors HAL_StatusTypeDef style.
    typedef enum
    {
        ST7735_OK = 0x00,      // operation succeeded / DMA is idle
        ST7735_BUSY = 0x01,    // DMA transfer still in progress
        ST7735_ERROR = 0x02,   // SPI/DMA error (future use)
        ST7735_TIMEOUT = 0x03, // blocking wait timed out (future use)
    } ST7735_StatusTypeDef;

    // Non-blocking poll – returns ST7735_OK when the display is idle and
    // ready to accept a new flush, ST7735_BUSY while DMA is running.
    //
    //   if (ST7735_FlushReady() == ST7735_OK) {
    //       render_frame();
    //       ST7735_Flush();
    //   }
    ST7735_StatusTypeDef ST7735_FlushReady(void);

    // Non-blocking flush – arms DMA and returns immediately.
    // Returns ST7735_OK if DMA was successfully started.
    // Returns ST7735_BUSY if a previous flush is still running (no-op, try again).
    //
    //   if (ST7735_Flush() == ST7735_BUSY) { /* come back next loop tick */ }
    ST7735_StatusTypeDef ST7735_Flush(void);

    // Blocking flush – spins until any in-progress transfer finishes,
    // then fires a new one.  Use when a guaranteed flush is required.
    // Returns ST7735_OK once DMA is armed.
    ST7735_StatusTypeDef ST7735_FlushBlocking(void);

    // -----------------------------------------------------------------------
    // High-level draw calls – all write to the framebuffer, NOT to SPI.
    // Call ST7735_Flush() when the frame is complete.
    // -----------------------------------------------------------------------

    void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
    void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void ST7735_FillRectangleFast(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void ST7735_FillScreen(uint16_t color);
    void ST7735_FillScreenFast(uint16_t color);
    void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
    void ST7735_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor);

    // -----------------------------------------------------------------------
    // Display control (go directly to SPI, not framebuffer)
    // -----------------------------------------------------------------------
    void ST7735_InvertColors(bool invert);
    void ST7735_SetGamma(GammaDef gamma);

#ifdef __cplusplus
}
#endif

#endif // __ST7735_H__