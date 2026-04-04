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
#include "kernel/scheduler.h"
#include "kernel/event.h"

extern void delay_ms(uint32_t ms);
extern uint32_t get_tick(void);


/* Periodic task intervals (ms) - used by sched_register in app_init */
#define TICK_KEYPAD_MS      20      /* 50 Hz keypad scan */
#define TICK_ENCODER_MS     5       /* 200 Hz encoder poll */
#define TICK_BATTERY_MS     1000    /* 1 Hz battery check */
#define TICK_GPS_MS         100     /* 10 Hz GPS parse */
#define TICK_DISPLAY_MS     33      /* ~30 fps display refresh */
#define TICK_DUALWATCH_MS   200     /* 5 Hz dual-watch RSSI check */
#define TICK_POWER_BTN_MS   20      /* 50 Hz power button monitor */
#define TICK_BUTTONS_MS     20      /* 50 Hz PTT + side button monitor */
#define TICK_AUDIO_MS       5       /* 200 Hz audio tone management */
#define TICK_CPS_MS         50      /* 20 Hz CPS programming poll */

/* Feed IWDG early - bootloader enables watchdog before jumping to us */
#define IWDG_FEED()  (*(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL)

/* Global calibration data - loaded once in hw_init, used by radio/power */
calibration_t cal_data;

/* Forward declarations ------------------------------------------------ */
static void hw_init(void);
static void app_init(void);

/* Scheduler task wrappers - adapt module APIs to void(*)(void) ---------- */

/* One-shot diagnostic counters */
static uint8_t keypad_diag_count = 0;
static uint8_t encoder_diag_count = 0;

static const char * const key_names[] = {
    "1","2","3","A/VFO", "4","5","6","B/SCAN",
    "7","8","9","C/MENU", "*","0","#","D/BAND",
    "C4R0","C4R1","C4R2","C4R3",
    "SIDE1","SIDE4"
};

/* Unique tone per key (freq x10): each key gets a distinct pitch */
static const uint16_t key_tones[] = {
    /* 1=700Hz  2=800Hz  3=900Hz  A=1900Hz */
    7000, 8000, 9000, 19000,
    /* 4=1000Hz 5=1100Hz 6=1200Hz B=2000Hz */
    10000, 11000, 12000, 20000,
    /* 7=1300Hz 8=1400Hz 9=1500Hz C=2100Hz */
    13000, 14000, 15000, 21000,
    /* *=600Hz  0=1600Hz #=1700Hz D=2200Hz */
    6000, 16000, 17000, 22000,
    /* C4R0=500Hz C4R1=2400Hz C4R2=2600Hz C4R3=2800Hz */
    5000, 24000, 26000, 28000,
    /* SIDE1=3000Hz SIDE4=3200Hz */
    30000, 32000,
};

static void task_keypad(void)
{
    /* Dump raw GPIO state every ~2 seconds for first 10 samples */
    if (keypad_diag_count < 10) {
        static uint16_t diag_ticks = 0;
        diag_ticks++;
        if (diag_ticks >= 100) {  /* 100 * 20ms = 2s */
            diag_ticks = 0;
            keypad_diag_count++;
            /* Side buttons (PE5/PA12), cols (PC0-3), rows (PD4-7) */
            volatile uint32_t *gpioc_idr = (volatile uint32_t *)0x40011008UL;
            volatile uint32_t *gpiod_idr = (volatile uint32_t *)0x40011408UL;
            volatile uint32_t *gpioe_idr = (volatile uint32_t *)0x40011808UL;
            volatile uint32_t *gpioa_idr = (volatile uint32_t *)0x40010808UL;
            uint32_t pc = *gpioc_idr;
            uint32_t pd = *gpiod_idr;
            uint32_t pe = *gpioe_idr;
            uint32_t pa = *gpioa_idr;
            dbg_puts("[KBD_DIAG] PC=");
            dbg_reg("", pc);
            dbg_puts(" PD=");
            dbg_reg("", pd);
            uint8_t side1 = (pe >> 5) & 1;  /* PE5 TOP_PROG */
            uint8_t side4 = (pa >> 12) & 1; /* PA12 BOT_PROG */
            dbg_puts(" SIDE1=");
            dbg_reg("", side1);
            dbg_puts(" SIDE4=");
            dbg_reg("", side4);
        }
    }

    key_event_t evt;
    if (keypad_get_event(&evt)) {
        const char *kn = (evt.key < KEY_COUNT) ? key_names[evt.key] : "??";
        switch (evt.type) {
        case KEY_EVT_PRESS:
            dbg_puts("[KEY] press: ");
            dbg_puts(kn);
            dbg_puts("\n");
            if (evt.key < KEY_COUNT)
                audio_beep_freq(key_tones[evt.key], 60);
            event_post(EVT_KEY_PRESS, evt.key);
            power_reset_idle_timer();
            break;
        case KEY_EVT_RELEASE:
            dbg_puts("[KEY] release: ");
            dbg_puts(kn);
            dbg_puts("\n");
            event_post(EVT_KEY_RELEASE, evt.key);
            break;
        case KEY_EVT_REPEAT:
            dbg_puts("[KEY] repeat: ");
            dbg_puts(kn);
            dbg_puts("\n");
            event_post(EVT_KEY_PRESS, evt.key);
            break;
        default:
            break;
        }
    }
}

static void task_encoder(void)
{
    /* Dump raw PB4/PB5 state every ~2s for first 10 samples */
    if (encoder_diag_count < 10) {
        static uint16_t enc_diag_ticks = 0;
        enc_diag_ticks++;
        if (enc_diag_ticks >= 400) {  /* 400 * 5ms = 2s */
            enc_diag_ticks = 0;
            encoder_diag_count++;
            volatile uint32_t *gpiob_idr = (volatile uint32_t *)0x40010C08UL;
            uint32_t pb = *gpiob_idr;
            uint8_t enc_a = (pb >> 4) & 1;
            uint8_t enc_b = (pb >> 5) & 1;
            dbg_puts("[ENC_DIAG] PB=");
            dbg_reg("", pb);
            dbg_puts(" A=");
            dbg_reg("", enc_a);
            dbg_puts(" B=");
            dbg_reg("", enc_b);
        }
    }

    int8_t dir = encoder_poll();
    if (dir > 0) {
        dbg_puts("[ENC] CW\n");
        audio_beep_freq(18000, 30);  /* 1800 Hz, 30 ms - rising feel */
        event_post(EVT_ENCODER_CW, 1);
        power_reset_idle_timer();
    } else if (dir < 0) {
        dbg_puts("[ENC] CCW\n");
        audio_beep_freq(12000, 30);  /* 1200 Hz, 30 ms - falling feel */
        event_post(EVT_ENCODER_CCW, 1);
        power_reset_idle_timer();
    }
}

/* PTT and side-port button monitor (Phase 12.3/12.4).
 * Reads standalone GPIO buttons NOT on the keypad matrix.
 * OEM references:
 *   PE3 = PTT primary   (gpio_modes_init @ 0x0801391C, BINARY_VERIFIED)
 *   PE2 = PTT2 secondary (gpio_modes_init @ 0x0801391C, BINARY_VERIFIED)
 *   PE6 = EXT_PTT        (spk_mute_on_ptt @ 0x08019254, BINARY_VERIFIED)
 * Reports state changes via debug UART. Does NOT activate TX relays. */
static uint8_t prev_ptt   = 1;   /* idle HIGH (active-low) */
static uint8_t prev_ptt2  = 1;
static uint8_t prev_extptt = 1;

static void task_buttons(void)
{
    uint8_t ptt    = gpio_read_pin(PTT_PORT,     PTT_PIN)     ? 1 : 0;
    uint8_t ptt2   = gpio_read_pin(PTT2_PORT,    PTT2_PIN)    ? 1 : 0;
    uint8_t extptt = gpio_read_pin(EXT_PTT_PORT, EXT_PTT_PIN) ? 1 : 0;

    if (ptt != prev_ptt) {
        prev_ptt = ptt;
        dbg_puts(ptt ? "[BTN] PTT released\n" : "[BTN] PTT pressed\n");
        event_post(ptt ? EVT_KEY_RELEASE : EVT_KEY_PRESS, 0xE3);
        if (!ptt) power_reset_idle_timer();
    }
    if (ptt2 != prev_ptt2) {
        prev_ptt2 = ptt2;
        dbg_puts(ptt2 ? "[BTN] PTT2 released\n" : "[BTN] PTT2 pressed\n");
    }
    if (extptt != prev_extptt) {
        prev_extptt = extptt;
        dbg_puts(extptt ? "[BTN] EXT_PTT released\n" : "[BTN] EXT_PTT pressed\n");
    }
}

static void task_gps(void)
{
    gps_process();
}

static void task_cps(void)
{
    cps_poll();
}

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

    /* Hex table self-test: verify .rodata integrity after BTF upload.
     * If corrupted, you'll see garbled chars instead of 0123456789ABCDEF. */
    dbg_puts("[DBG] HEX_TEST=");
    for (uint32_t i = 0; i < 16; i++)
        dbg_hex8((unsigned char)i);
    dbg_puts(" (expect 000102...0F)\n");

    /* ABSOLUTE MINIMUM: PB9 power latch must be asserted immediately
     * or the radio will power off. */
    gpio_config_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);
    IWDG_FEED();

    /* Initialize event queue */
    event_init();

    /* Bring up hardware peripherals */
    hw_init();

    /* Initialize application-layer modules and register scheduler tasks */
    app_init();

    /* Main loop - never returns */
    dbg_puts("[DBG] entering scheduler\n");
    sched_run();

    return 0;  /* unreachable */
}

/* ========================================================================
 *  hw_init - Bring up all hardware peripherals
 * ======================================================================== */

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

    /* Verify flash chip: expect W25Q16 (Winbond 0xEF, 16Mbit 0x4015)
     * OEM: periph_init_flash_adc @ 0x08013820 reads JEDEC after SPI2 init.
     * Valid IDs: 0xEF4015 (W25Q16BV), 0xEF4016 (W25Q32). */
    {
        uint32_t jedec = spi_flash_read_id();
        dbg_reg("[DBG] SPI JEDEC=0x", jedec);
        if ((jedec & 0xFFFF00) == 0xEF4000)
            dbg_puts("[DBG] Flash: Winbond W25Q detected OK\n");
        else if (jedec == 0x000000 || jedec == 0xFFFFFF)
            dbg_puts("[ERR] Flash: no response (check SPI2 wiring)\n");
        else
            dbg_puts("[WARN] Flash: unexpected JEDEC ID\n");
    }

    /* Flash read sanity: dump first 16B of channel memory (0x0000)
     * and calibration validity flag (0xF0E0).
     * OEM: flash_data_load @ 0x08007358 reads these sectors. */
    {
        uint8_t sample[16];
        uint8_t all_ff = 1;
        spi_flash_read(0x000000, sample, 16);
        dbg_puts("[DBG] Flash[0x0000]: ");
        for (int i = 0; i < 16; i++) {
            dbg_hex8(sample[i]);
            dbg_puts(" ");
            if (sample[i] != 0xFF) all_ff = 0;
        }
        dbg_puts("\n");
        if (all_ff)
            dbg_puts("[WARN] Flash channel sector appears blank\n");

        uint8_t cal_flag;
        spi_flash_read(0x00F0E0, &cal_flag, 1);
        dbg_reg("[DBG] Cal flag @ 0xF0E0 = 0x", cal_flag);
        if (cal_flag == 0xFF)
            dbg_puts("[WARN] No calibration data programmed\n");
        else
            dbg_puts("[DBG] Calibration data present\n");
    }

    /* Calibration + wear-leveling (needs SPI flash) ------------------
     * OEM: hw_init_main calls calibration_load then initializes all 5
     * WL sectors. Bitmap scan @ 0x0800EF68, SYSCFG write @ 0x0800F918. */
    dbg_puts("[DBG] calibration+wl...\n");
    calibration_load(&cal_data);

    wl_init(&WL_SYSCFG);
    wl_init(&WL_VFOCFG);
    wl_init(&WL_EXTCFG);
    wl_init(&WL_VFOSEL);
    wl_init(&WL_CHCFG);

    /* Probe each WL sector: attempt a read to check valid/empty/corrupt.
     * OEM validates sectors on boot before dispatching main task loop.
     * Buffer must be large enough for the LARGEST record (EXTCFG = 160B). */
    {
        uint8_t probe[160];
        const char *names[] = {"SYSCFG","VFOCFG","EXTCFG","VFOSEL","CHCFG "};
        const wl_sector_t *secs[] = {&WL_SYSCFG,&WL_VFOCFG,&WL_EXTCFG,&WL_VFOSEL,&WL_CHCFG};
        for (int i = 0; i < 5; i++) {
            int rc = wl_read(secs[i], probe);
            dbg_puts("[DBG] WL ");
            dbg_puts(names[i]);
            dbg_puts(rc == 0 ? ": valid\n" : ": empty/uninitialized\n");
        }
    }
    IWDG_FEED();

    /* BK4829 dual RF transceivers (GPIOE bit-bang) ------------------- */
    /* RADIO DISABLED - skip RF transceiver init for peripheral testing */
    dbg_puts("[DBG] bk4829_init SKIPPED (radio disabled)\n");
    IWDG_FEED();

    /* SI4732 broadcast receiver (GPIOB bit-bang I2C) ----------------- */
    dbg_puts("[DBG] si4732_init SKIPPED (radio disabled)\n");
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

    /* Speaker audio path enable (must be before any tone playback) ---
     * PE1 = SPK_MUTE: LOW = unmuted. OEM asserts HIGH at power-off.
     * PC12 = BEEP_SW: HIGH = beep-to-speaker path enabled.
     *   OEM evidence: gpio_bits_set(GPIOC, 0x1000) @ fw 0x080038EA */
    dbg_puts("[DBG] spk_unmute+beep_sw...\n");
    gpio_config_pin(SPK_MUTE_PORT, SPK_MUTE_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(SPK_MUTE_PORT, SPK_MUTE_PIN);   /* unmute speaker */
    gpio_config_pin(BEEP_SW_PORT, BEEP_SW_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(BEEP_SW_PORT, BEEP_SW_PIN);        /* enable beep path */
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

    /* LCD diagnostic: fill screen blue to confirm 8080 bus works */
    dbg_puts("[DBG] lcd_fill blue...\n");
    lcd_fill_rect(0, 0, 240, 320, 0x001F);  /* RGB565 blue */
    IWDG_FEED();

    /* -- Audio Amplifier Test -----------------------------------------
     * OEM firmware analysis reveals PB8 is the audio amplifier enable:
     *   - OEM function at 0x08006574 sets PB8 HIGH before beep
     *   - Clears PB8 LOW after beep completes
     *   - Full audio path: DAC -> PE1(unmute) -> PC12(beep_sw) -> PB8(amp)
     * ------------------------------------------------------------------- */
    dbg_puts("[AMP] === AUDIO AMP TEST ===\n");
    {
        /* Confirm audio path state */
        dbg_puts("[AMP] PE1(SPK_MUTE)=");
        dbg_puts((GPIOE->ODR & 0x0002) ? "MUTED!" : "unmuted");
        dbg_puts(" PC12(BEEP_SW)=");
        dbg_puts((GPIOC->ODR & 0x1000) ? "on" : "OFF!");
        dbg_puts(" PB8=");
        dbg_puts((GPIOB->ODR & 0x0100) ? "HIGH" : "LOW");
        dbg_puts("\n");

        /* Enable amp for all tests */
        gpio_config_pin(GPIOB, GPIO_PIN_8, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
        gpio_set_pin(GPIOB, GPIO_PIN_8);

        /* ---- Test A: DIRECT DAC write (no DMA, no TIM6) ----
         * Write sine values directly to DAC data register in a
         * software loop.  If this produces sound, PA4→amp→speaker works
         * and the DMA path has a subtle problem.  If silent, the
         * analog PCB trace from PA4 doesn't reach the amplifier. */
        dbg_puts("[AMP] Test A: DIRECT DAC write (no DMA) 3s\n");
        {
            /* Stop any DMA/TIM6 that might be running */
            *(volatile uint32_t *)0x40001000UL &= ~1UL; /* TIM6 CR1: CEN=0 */
            *(volatile uint32_t *)0x40020430UL = 0;      /* DMA2_CH3 CCR=0 */

            /* Enable DAC CH1 + CH2 (no trigger, no DMA) - software writes */
            DAC->CR = DAC_CR_EN2 | DAC_CR_EN1 | DAC_CR_BOFF1;

            /* 32-sample sine at ~1kHz: each sample held ~31µs
             * 3 seconds ≈ 3000 cycles × 32 samples = 96000 writes */
            static const uint16_t sw_sine[32] = {
                2048,2448,2831,3185,3496,3750,3939,4056,
                4095,4056,3939,3750,3496,3185,2831,2448,
                2048,1648,1265, 911, 600, 346, 157,  40,
                   1,  40, 157, 346, 600, 911,1265,1648
            };
            int cyc;
            for (cyc = 0; cyc < 3000; cyc++) {
                int s;
                for (s = 0; s < 32; s++) {
                    DAC->DHR12R1 = sw_sine[s];
                    /* ~31µs busy wait (120MHz × 31µs ≈ 3720 cycles) */
                    volatile int d;
                    for (d = 0; d < 620; d++) ;
                }
                if ((cyc & 0xFF) == 0) IWDG_FEED();
            }
            dbg_reg("[AMP] DAC_DOR1=0x", DAC->DOR1);
        }

        /* ---- Test B: DMA-based tone (original test) ---- */
        dbg_puts("[AMP] Test B: DMA tone 3s\n");
        dac_audio_play_tone(10000);

        /* Diagnostic: dump DAC/TIM6/DMA registers to verify playback */
        dbg_reg("[AMP] DAC_CR =0x", *(volatile uint32_t *)0x40007400UL);
        dbg_reg("[AMP] DAC_SR =0x", *(volatile uint32_t *)0x40007434UL);
        dbg_reg("[AMP] DAC_DOR1=0x", *(volatile uint32_t *)0x4000742CUL);
        dbg_reg("[AMP] DMA2C3_CCR=0x",
                *(volatile uint32_t *)0x40020430UL);
        dbg_reg("[AMP] DMA2C3_CMAR=0x",
                *(volatile uint32_t *)0x40020438UL);
        dbg_reg("[AMP] DMA2C3_CPAR=0x",
                *(volatile uint32_t *)0x4002043CUL);

        { int i; for (i = 0; i < 6; i++) { delay_ms(500); IWDG_FEED(); } }

        /* ---- Test C: Try with BOFF1=0 (output buffer enabled) ---- */
        dbg_puts("[AMP] Test C: DAC buffer ON + DMA 3s\n");
        dac_audio_stop();
        /* Re-enable with output buffer (BOFF1=0) for lower impedance */
        DAC->CR = DAC_CR_EN2 | DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_TSEL1_TIM6;
        dac_audio_play_tone(10000);
        /* Now set DMAEN1 */
        DAC->CR |= DAC_CR_DMAEN1;
        { int i; for (i = 0; i < 6; i++) { delay_ms(500); IWDG_FEED(); } }

        dac_audio_stop();
        gpio_clear_pin(GPIOB, GPIO_PIN_8);
        dbg_puts("[AMP] === TEST COMPLETE ===\n");
    }


    /* Configure PTT and side-button inputs with internal pull-ups.
     * OEM gpio_modes_init leaves PE0-6 as default (floating input).
     * We explicitly configure with pull-ups for reliable reads.
     * OEM @ 0x0801391C: PE2-3 listed as PTT, PE5 as SIDE_KEY. */
    gpio_config_pin(GPIOE, GPIO_PIN_3, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_3);    /* PE3 pull-UP (PTT1, active LOW) */
    gpio_config_pin(GPIOE, GPIO_PIN_2, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_2);    /* PE2 pull-UP (PTT2, active LOW) */
    gpio_config_pin(GPIOE, GPIO_PIN_5, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_5);    /* PE5 pull-UP (TopProg, active LOW) */
    gpio_config_pin(GPIOE, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_0);    /* PE0 pull-UP (PWR_DET) */
    gpio_config_pin(GPIOA, GPIO_PIN_12, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOA, GPIO_PIN_12);   /* PA12 pull-UP (BotProg, active LOW) */
    dbg_puts("[DBG] PTT+direct inputs configured\n");


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

/* ========================================================================
 *  app_init - Initialize application-layer modules and register
 *  scheduler tasks.
 *
 *  Module init functions set up internal state. Scheduler tasks are
 *  registered for periodic polling. Tasks can be enabled/disabled at
 *  runtime via sched_enable() as peripherals are brought online.
 * ======================================================================== */

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

    /* Radio subsystems (software init only - RF hardware skipped) ------ */
    dbg_puts("[DBG] vfo+radio_init...\n");
    vfo_init();
    radio_init();
    IWDG_FEED();
    dbg_puts("[DBG] vox+scanner+dtmf...\n");
    vox_init();
    scanner_init();
    dtmf_load_config();
    IWDG_FEED();
    dbg_puts("[DBG] audio_init...\n");
    audio_init();
    audio_power_on();
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

    /* ================================================================
     * Register scheduler tasks.
     * Tasks are dispatched in registration order by sched_run().
     * Critical high-frequency tasks are registered first.
     * ================================================================ */
    dbg_puts("[DBG] registering scheduler tasks...\n");

    sched_register("encoder",   task_encoder,          TICK_ENCODER_MS);
    sched_register("keypad",    task_keypad,           TICK_KEYPAD_MS);
    sched_register("buttons",   task_buttons,          TICK_BUTTONS_MS);
    sched_register("pwr_btn",   power_button_poll,     TICK_POWER_BTN_MS);
    sched_register("display",   display_update,        TICK_DISPLAY_MS);
    sched_register("gps",       task_gps,              TICK_GPS_MS);
    sched_register("dualwatch", radio_dual_watch_poll, TICK_DUALWATCH_MS);
    sched_register("battery",   power_poll,            TICK_BATTERY_MS);
    sched_register("audio",     audio_poll,            TICK_AUDIO_MS);
    sched_register("cps",       task_cps,              TICK_CPS_MS);

    dbg_puts("[DBG] app_init complete\n");
}

#endif /* HW_TEST */
