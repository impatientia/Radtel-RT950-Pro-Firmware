/*
 * display.h - Display / UI abstraction for the RT-950 Pro
 *
 * Higher-level drawing primitives on top of the LCD 8080 driver.
 * Uses a built-in 5x7 bitmap font for text rendering and flash fonts
 * (via font.h) for the main screen layout.
 * All coordinates are in pixels; (0,0) = top-left corner.
 *
 * Boot splash (V0.27): RGB565 240x320, stored at SPI 0x090000.
 *   Loaded in 10 blocks x 15360 bytes, big-endian byte-swapped on read.
 *   Splash disabled when calibration validity flag at 0xF0E0 == 0xFF.
 *   OEM LCD_Init @ fw 0x08026954, LCD_BulkTransfer @ fw 0x080266C4.
 *
 * See font.h for glyph flash addresses and CJK index formula.
 */

#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <stdint.h>

/* Font metrics (5x7 bitmap, 1-pixel inter-character gap) */
#define DISPLAY_FONT_W  5
#define DISPLAY_FONT_H  7
#define DISPLAY_CHAR_GAP 1

/* RGB565 color palette ----------------------------------------------- */
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_DARK_GRAY 0x4208
#define COLOR_GRAY      0x8410
#define COLOR_ORANGE    0xFD20
#define COLOR_CYAN      0x07FF

/* Screen layout geometry --------------------------------------------- */
#define LAYOUT_STATUS_Y     0
#define LAYOUT_STATUS_H     20
#define LAYOUT_VFO_A_Y      20
#define LAYOUT_VFO_A_H      140
#define LAYOUT_VFO_B_Y      160
#define LAYOUT_VFO_B_H      80
#define LAYOUT_SMETER_Y     240
#define LAYOUT_SMETER_H     30
#define LAYOUT_INFO_Y       270
#define LAYOUT_INFO_H       50

/*
 * display_init - Initialize the display subsystem.
 * Calls lcd_init() and prepares the screen.
 */
void display_init(void);

/*
 * display_clear - Fill the entire screen with a solid RGB565 color.
 */
void display_clear(uint16_t color);

/*
 * display_draw_char - Draw a single ASCII character at pixel position (x, y).
 * Characters outside the printable range (0x20-0x7E) are rendered as '?'.
 */
void display_draw_char(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg);

/*
 * display_draw_text - Draw a null-terminated string starting at (x, y).
 * Characters advance by (DISPLAY_FONT_W + DISPLAY_CHAR_GAP) pixels horizontally.
 */
void display_draw_text(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);

/*
 * display_draw_hline - Draw a horizontal line of 1-pixel thickness.
 */
void display_draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color);

/*
 * display_draw_vline - Draw a vertical line of 1-pixel thickness.
 */
void display_draw_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color);

/*
 * display_printf - Formatted text output at pixel position (x, y).
 * Supports standard printf format specifiers (limited to 128-char buffer).
 */
void display_printf(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg,
                    const char *fmt, ...);

/*
 * display_update - Main display refresh, called at ~30 fps from the main loop.
 * Routes to main screen, menu, or freq-entry screen as appropriate.
 */
void display_update(void);

/* Main screen layout rendering --------------------------------------- */

/* Draw the complete main screen (all sections) */
void display_draw_main_screen(void);

/* Draw individual sections (for partial updates) */
void display_draw_status_bar(void);
void display_draw_vfo_a(void);
void display_draw_vfo_b(void);
void display_draw_smeter(void);
void display_draw_info_bar(void);

#endif /* APP_DISPLAY_H */
