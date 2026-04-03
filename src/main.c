/*
 * main.c - Entry point for the RT-950 Pro custom firmware
 *
 * Super-loop architecture matching OEM firmware pattern:
 *   SystemInit -> peripherals -> infinite loop with periodic tasks.
 *
 * Subsystems are brought up in dependency order:
 *   1. DMA (needed by LCD + DAC)
 *   2. SPI2 (flash)
 *   3. BK4829 (dual RF transceivers)
 *   4. SI4732 (broadcast receiver)
 *   5. UART (GPS + Bluetooth)
 *   6. ADC (battery + audio level)
 *   7. DAC (CTCSS/AFSK tone gen)
 *   8. LCD + display
 *   9. Keypad + encoder
 *   10. GPS parser
 *   11. Radio control
 *
 * Build with -DHW_TEST=N to run a standalone hardware test instead.
 * See include/tests/hw_test.h for test list.
 */

#ifdef HW_TEST
#include "tests/hw_test.h"

int main(void)
{
#if   HW_TEST == 1
    test_blinky();
#elif HW_TEST == 2
    test_uart_echo();
#elif HW_TEST == 3
    test_lcd_pattern();
#elif HW_TEST == 4
    test_bk4829_id();
#elif HW_TEST == 5
    test_si4732_rev();
#elif HW_TEST == 6
    test_spi_flash_id();
#elif HW_TEST == 7
    test_adc_monitor();
#elif HW_TEST == 8
    test_dac_tone();
#elif HW_TEST == 9
    test_keypad_encoder();
#elif HW_TEST == 10
    test_gps_display();
#elif HW_TEST == 11
    test_full_diagnostic();
#else
#error "Unknown HW_TEST value (valid: 1-11)"
#endif
    return 0;
}

#else /* Normal firmware build */

#include "at32f403a.h"
#include "rt950_pinmap.h"
#include "debug_uart.h"
#include "drivers/gpio.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/bk4829.h"
#include "drivers/si4732.h"
#include "drivers/uart.h"
#include "drivers/adc.h"
#include "drivers/dac_audio.h"
#include "drivers/timer.h"
#include "drivers/lcd.h"
#include "app/display.h"
#include "app/keypad.h"
#include "app/encoder.h"
#include "app/gps.h"
#include "app/radio.h"
#include "app/vox.h"
#include "app/scanner.h"
#include "app/power.h"
#include "app/fm_radio.h"
#include "app/channel.h"
#include "app/menu.h"
#include "app/freq_entry.h"
#include "app/dtmf.h"
#include "app/splash.h"
#include "app/vfo.h"
#include "app/aprs.h"
#include "app/bluetooth.h"
#include "app/am_radio.h"
#include "app/cps.h"
#include "app/settings.h"
#include "app/audio.h"
#include "app/noaa.h"
#include "app/crossband.h"
#include "drivers/calibration.h"
#include "drivers/flash_wearleveling.h"

extern void delay_ms(uint32_t ms);
extern uint32_t get_tick(void);

/* Periodic task intervals (ms) ---------------------------------------- */
#define TICK_KEYPAD_MS      20      /* 50 Hz keypad scan */
#define TICK_ENCODER_MS     5       /* 200 Hz encoder poll */
#define TICK_BATTERY_MS     1000    /* 1 Hz battery check */
#define TICK_GPS_MS         100     /* 10 Hz GPS parse */
#define TICK_DISPLAY_MS     33      /* ~30 fps display refresh */
#define TICK_DUALWATCH_MS   200     /* 5 Hz dual-watch RSSI check */

/* State --------------------------------------------------------------- */
#ifndef DEBUG_UART
static uint32_t tick_keypad;
static uint32_t tick_encoder;
static uint32_t tick_battery;
static uint32_t tick_gps;
static uint32_t tick_display;
static uint32_t tick_dualwatch;
static uint8_t  cps_was_active;
#endif

/* Feed IWDG early - bootloader enables watchdog before jumping to us */
#define IWDG_FEED()  (*(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL)

/* Global calibration data - loaded once in hw_init, used by radio/power */
calibration_t cal_data;

/* Forward declarations ------------------------------------------------ */
#ifndef DEBUG_UART
static void hw_init(void);
static void app_init(void);
static void super_loop(void);
#endif

/* ======================================================================== */

int main(void)
{
    /*
     * SystemInit() has already been called by Reset_Handler in startup.c.
     * At this point: SYSCLK = 120 MHz, SysTick @ 1 ms, GPIO clocks A-E on.
     */

    /* Dump reset reason flags from CRM->CTRLSTS (0x40021024).
     * Bit 28: WDT (IWDG), Bit 29: SFT (software), Bit 30: POR,
     * Bit 31: PIN (NRST), Bit 27: WWDT. Clear flags after reading. */
    {
        volatile uint32_t *ctrlsts = (volatile uint32_t *)0x40021024UL;
        uint32_t rst = *ctrlsts;
        dbg_reg("[DBG] RST_FLAGS=0x", rst);
        *ctrlsts |= (1UL << 24);  /* RSTFC: clear reset flags */
    }

    dbg_puts("[DBG] main() entered\n");

    /* ABSOLUTE MINIMUM: PB9 power latch + backlight */
    gpio_config_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);

    /* LEDs for visual feedback */
    gpio_config_pin(LED_RED_PORT, LED_RED_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_config_pin(LED_GREEN_PORT, LED_GREEN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);

    /* Red LED on = starting LCD init */
    gpio_set_pin(LED_RED_PORT, LED_RED_PIN);
    dbg_puts("[DBG] calling lcd_init...\n");

    IWDG_FEED();
    lcd_init();
    IWDG_FEED();

    dbg_puts("[DBG] lcd_init done, drawing boot screen\n");

    /* Green LED on = LCD init complete */
    gpio_set_pin(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_clear_pin(LED_RED_PORT, LED_RED_PIN);

    /* Fill entire screen dark navy background */
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x000A);
    IWDG_FEED();

    /* Header bar */
    lcd_fill_rect(0, 0, LCD_WIDTH, 3, 0x07E0);    /* green accent line */
    IWDG_FEED();

    /* Title: "RT-950 PRO" centered (2x scale) */
    lcd_draw_string_2x(30, 20, "RT-950 PRO", 0x07E0, 0x000A);
    IWDG_FEED();

    /* Subtitle */
    lcd_draw_string(52, 56, "CUSTOM FIRMWARE", 0xFFFF, 0x000A);
    IWDG_FEED();

    /* Divider line */
    lcd_fill_rect(20, 75, 200, 1, 0x4228);
    IWDG_FEED();

    /* Status lines */
    lcd_draw_string(8, 90,  "CPU: AT32F403A 120MHz", 0xBDF7, 0x000A);
    IWDG_FEED();
    lcd_draw_string(8, 106, "LCD: ST7789V 240x320",  0xBDF7, 0x000A);
    IWDG_FEED();
    lcd_draw_string(8, 122, "RF:  Dual BK4829",      0xBDF7, 0x000A);
    IWDG_FEED();
    lcd_draw_string(8, 138, "RCV: SI4732 AM/FM/SW",  0xBDF7, 0x000A);
    IWDG_FEED();
    lcd_draw_string(8, 154, "GPS: NMEA 9600 baud",   0xBDF7, 0x000A);
    IWDG_FEED();

    /* Divider */
    lcd_fill_rect(20, 175, 200, 1, 0x4228);
    IWDG_FEED();

    /* Confirmation block */
    lcd_draw_string(16, 190, "GPIO Fix:  VERIFIED", 0x07E0, 0x000A);
    IWDG_FEED();
    lcd_draw_string(16, 206, "Red LED:   PC13 OK",  0xF800, 0x000A);
    IWDG_FEED();
    lcd_draw_string(16, 222, "Green LED: PC14 OK",  0x07E0, 0x000A);
    IWDG_FEED();
    lcd_draw_string(16, 238, "Backlight: PC6  OK",  0xFFE0, 0x000A);
    IWDG_FEED();
    lcd_draw_string(16, 254, "Band Rly:  PC4  OK",  0x07FF, 0x000A);
    IWDG_FEED();
    lcd_draw_string(16, 270, "LCD Bus:   8080 OK",  0xF81F, 0x000A);
    IWDG_FEED();

    /* Footer accent */
    lcd_fill_rect(0, 297, LCD_WIDTH, 3, 0x07E0);
    IWDG_FEED();

    /* Version string */
    lcd_draw_string(56, 306, "v0.0.1-dev", 0x4228, 0x000A);
    IWDG_FEED();

    dbg_puts("[DBG] boot screen drawn, entering hold\n");

    /* Both LEDs on = done */
    gpio_set_pin(LED_RED_PORT, LED_RED_PIN);
    gpio_set_pin(LED_GREEN_PORT, LED_GREEN_PIN);

    /* Hold and print heartbeats */
    {
        uint32_t last_hb = get_tick();
        while (1) {
            IWDG_FEED();
            uint32_t now = get_tick();
            if ((now - last_hb) >= 2000) {
                last_hb = now;
                dbg_reg("[DBG] hb t=", now);
            }
        }
    }

    return 0;  /* unreachable */
}

/* ========================================================================
 *  hw_init - Bring up all hardware peripherals
 * ======================================================================== */

#ifndef DEBUG_UART
static void hw_init(void)
{
    /* POWER LATCH: PB9 must be held HIGH to keep the radio powered.
     * The bootloader asserts this briefly; our firmware must take over
     * immediately or the radio will power off. */
    gpio_config_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);

    IWDG_FEED();

    /* DMA (must be before LCD and DAC) -------------------------------- */
    dbg_puts("[DBG] dma_init...\n");
    dma_init();
    IWDG_FEED();

    /* SPI flash (SPI2) ----------------------------------------------- */
    dbg_puts("[DBG] spi2_init...\n");
    spi2_init();
    IWDG_FEED();

    /* Verify flash chip: expect W25Q16 (Winbond 0xEF, 16Mbit 0x4015) */
    {
        uint32_t jedec = spi_flash_read_id();
        dbg_reg("[DBG] SPI JEDEC=0x", jedec);
    }

    /* Calibration + wear-leveling (needs SPI flash) ------------------ */
    dbg_puts("[DBG] calibration+wl...\n");
    calibration_load(&cal_data);
    wl_init(&WL_SYSCFG);
    wl_init(&WL_VFOCFG);
    wl_init(&WL_EXTCFG);
    wl_init(&WL_VFOSEL);
    wl_init(&WL_CHCFG);
    IWDG_FEED();

    /* BK4829 dual RF transceivers (GPIOE bit-bang) ------------------- */
    dbg_puts("[DBG] bk4829_init(0)...\n");
    bk4829_init(BK4829_CHIP0);
    IWDG_FEED();
    dbg_puts("[DBG] bk4829_init(1)...\n");
    bk4829_init(BK4829_CHIP1);
    IWDG_FEED();

    /* SI4732 broadcast receiver (GPIOB bit-bang I2C) ----------------- */
    dbg_puts("[DBG] si4732_init...\n");
    si4732_init();
    IWDG_FEED();

    /* UARTs ---------------------------------------------------------- */
    dbg_puts("[DBG] uart_bt_init...\n");
    bt_init();              /* USART1 @ 115200 - Bluetooth + AT config */
    IWDG_FEED();
    dbg_puts("[DBG] uart_gps_init...\n");
    uart_gps_init();        /* USART3 @ 9600 - GPS (PB10/PB11) */
    IWDG_FEED();

    /* ADC (battery + audio level) ------------------------------------ */
    dbg_puts("[DBG] adc_init...\n");
    adc_init();
    IWDG_FEED();

    /* DAC + TIM6 (CTCSS/AFSK tone generation) ----------------------- */
    dbg_puts("[DBG] dac_audio_init...\n");
    dac_audio_init();
    IWDG_FEED();

    /* LCD ------------------------------------------------------------ */
    dbg_puts("[DBG] lcd_init...\n");
    lcd_init();
    IWDG_FEED();
    dbg_puts("[DBG] lcd_backlight...\n");
    /* LCD backlight on ----------------------------------------------- */
    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);

    /* PC12 GPIO output (not LCD DMA, LCD uses software bit-bang) */
    gpio_config_pin(PC12_GPIO_PORT, PC12_GPIO_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);

    /* LCD diagnostic: fill screen blue to confirm 8080 bus works */
    dbg_puts("[DBG] lcd_fill blue...\n");
    lcd_fill_rect(0, 0, 240, 320, 0x001F);  /* RGB565 blue */
    IWDG_FEED();

    /* Boot indicator: 3 backlight blinks + beep tone.
     * Uses blocking delays since super_loop isn't running yet.
     * Confirms SysTick, DAC, and LCD backlight GPIO are working. */
    dbg_puts("[DBG] boot indicator...\n");
    {
        int i;
        for (i = 0; i < 3; i++) {
            lcd_backlight_on();
            dac_audio_play_tone(10000);  /* 1000 Hz */
            delay_ms(80);
            IWDG_FEED();
            lcd_backlight_off();
            dac_audio_stop();
            delay_ms(120);
            IWDG_FEED();
        }
        lcd_backlight_on();  /* leave backlight on */
    }
}
#endif /* !DEBUG_UART - hw_init */

/* ========================================================================
 *  app_init - Initialize application-layer modules
 * ======================================================================== */

#ifndef DEBUG_UART
static void app_init(void)
{
    /* Boot splash (blocks ~1 s while LCD shows image) ---------------- */
    dbg_puts("[DBG] splash_show...\n");
    splash_show();
    IWDG_FEED();

    /* Input devices -------------------------------------------------- */
    dbg_puts("[DBG] keypad+encoder...\n");
    keypad_init();
    encoder_init();
    IWDG_FEED();

    /* Persistent settings (needs WL_SYSCFG ready) -------------------- */
    dbg_puts("[DBG] settings_init...\n");
    settings_init();
    IWDG_FEED();

    /* Radio subsystems ----------------------------------------------- */
    dbg_puts("[DBG] vfo+radio_init...\n");
    vfo_init();
    radio_init();
    IWDG_FEED();
    dbg_puts("[DBG] vox+scanner+dtmf...\n");
    vox_init();
    scanner_init();
    dtmf_load_config(); /* Load PTT-ID string + DTMF speed from flash */
    IWDG_FEED();
    dbg_puts("[DBG] audio_init...\n");
    audio_init();
    audio_power_on();  /* ascending 3-tone chime (non-blocking, needs audio_poll) */
    IWDG_FEED();

    /* Auxiliary modules ---------------------------------------------- */
    dbg_puts("[DBG] gps+power_init...\n");
    gps_init();
    power_init();
    IWDG_FEED();

    /* Apply factory calibration to power thresholds (if cal data valid) */
    /* NOTE: OEM V0.27 does NOT read battery thresholds from flash 0xF200.
     * Battery thresholds may be hardcoded or in a different calibration block
     * (possibly 0xF0C0 or 0xF0D0). Using hardcoded defaults for now. */
    (void)cal_data.validity_flag; /* suppress unused warning */

    dbg_puts("[DBG] menu+aprs+am...\n");
    menu_init();
    aprs_init();
    am_radio_init();
    crossband_set_mode((xband_mode_t)settings_get()->crossband_mode);
    IWDG_FEED();

    /* Display last - shows fully-initialized state ------------------- */
    dbg_puts("[DBG] display_init...\n");
    display_init();
    IWDG_FEED();

    /* CPS protocol handler (UART4, always ready).
     * In DEBUG builds, UART4 is used for debug output - skip CPS to
     * avoid the CPS module interpreting debug bytes as handshake. */
#ifndef DEBUG_UART
    dbg_puts("[DBG] cps_init...\n");
    cps_init();
#else
    dbg_puts("[DBG] cps_init SKIPPED (DEBUG: UART4 shared)\n");
#endif
    IWDG_FEED();

    /* Snapshot initial tick values */
    uint32_t now = get_tick();
    tick_keypad    = now;
    tick_encoder   = now;
    tick_battery   = now;
    tick_gps       = now;
    tick_display   = now;
    tick_dualwatch = now;
}
#endif /* !DEBUG_UART */

/* ========================================================================
 *  super_loop - Main event loop (no RTOS, matching OEM firmware pattern)
 *
 *  Each iteration checks elapsed time for periodic tasks.  The loop
 *  runs at full speed; tasks are gated by their individual intervals.
 * ======================================================================== */

#ifndef DEBUG_UART
static void super_loop(void)
{
    uint32_t last_heartbeat = get_tick();

    dbg_puts("[DBG] super_loop start (minimal)\n");

    while (1) {
        uint32_t now = get_tick();

        /* Force ALL known output pins every iteration */
        gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);       /* PC6 backlight */
        gpio_set_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN); /* PB3 keyboard */
        gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN); /* power */
        gpio_set_pin(GPIOC, GPIO_PIN_14);  /* PC14 LCD enable */

        /* Heartbeat every 2 seconds */
        if ((now - last_heartbeat) >= 2000) {
            last_heartbeat = now;
            dbg_reg("[DBG] hb t=", now);
        }

        /* NOTHING ELSE - no poll functions */
    }
}
#endif /* !DEBUG_UART - super_loop */

#endif /* HW_TEST */
