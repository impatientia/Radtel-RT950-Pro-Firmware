/*
 * keypad.c - Keypad scanner for the RT-950 Pro
 *
 * 4x4+1 matrix scan matching OEM firmware (V0.27 @ fw 0x08012FF8).
 *
 * Column scan sequence (5 iterations, active-low):
 *   col 0: clear PC0, set PC1-PC3
 *   col 1: clear PC1, set PC0,PC2,PC3
 *   col 2: clear PC2, set PC0,PC1,PC3
 *   col 3: clear PC3, set PC0,PC1,PC2
 *   col 4: clear all PC0-PC3 (side buttons - active when all low)
 *
 * Row read: PD4-PD7, active-low.
 *   0x70 on bits 4-7 -> row 0 pressed (PD7=0)
 *   0xB0 -> row 1 (PD6=0)
 *   0xD0 -> row 2 (PD5=0)
 *   0xE0 -> row 3 (PD4=0)
 */

#include "app/keypad.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"

/* ---- Column pin lookup tables ----------------------------------------- */

/* Pin to drive LOW for each column scan iteration.
 * Column 4 drives all low (handled separately). */
static const uint16_t col_clear[] = {
    GPIO_PIN_0,                         /* col 0 */
    GPIO_PIN_1,                         /* col 1 */
    GPIO_PIN_2,                         /* col 2 */
    GPIO_PIN_3,                         /* col 3 */
    GPIO_PIN_0 | GPIO_PIN_1 |           /* col 4 - all low */
        GPIO_PIN_2 | GPIO_PIN_3
};

/* Pins to drive HIGH for each column (complement within PC0-PC3). */
static const uint16_t col_set[] = {
    GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,   /* col 0 */
    GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3,   /* col 1 */
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3,   /* col 2 */
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2,   /* col 3 */
    0                                        /* col 4 - none high */
};

/* Row patterns read from GPIOD IDR bits 4-7 (active-low, masked 0xF0). */
#define ROW0_PATTERN    0x70    /* 0111 - PD7 low */
#define ROW1_PATTERN    0xB0    /* 1011 - PD6 low */
#define ROW2_PATTERN    0xD0    /* 1101 - PD5 low */
#define ROW3_PATTERN    0xE0    /* 1110 - PD4 low */

/* ---- Internal helpers -------------------------------------------------- */

/* Short software delay (~10 loop iterations at 120 MHz).
 * The firmware uses a similar trivial busy loop for column settle time. */
static void scan_delay(void)
{
    volatile uint32_t n = 10;
    while (n--)
        ;
}

/* Read row lines (PD4-PD7) and return masked value (bits 4-7). */
static inline uint8_t read_rows(void)
{
    return (uint8_t)(KBD_ROW_PORT->IDR & KBD_ROW_MASK);
}

/* Decode a row pattern to row index (0-3) or 0xFF if none/multiple. */
static uint8_t decode_row(uint8_t pattern)
{
    switch (pattern) {
    case ROW0_PATTERN: return 0;
    case ROW1_PATTERN: return 1;
    case ROW2_PATTERN: return 2;
    case ROW3_PATTERN: return 3;
    default:           return 0xFF;
    }
}

/* ---- Public API -------------------------------------------------------- */

void keypad_init(void)
{
    /* Enable clocks for all ports involved */
    gpio_enable_clock(KBD_COL_PORT);
    gpio_enable_clock(KBD_ROW_PORT);
    gpio_enable_clock(KBD_SCAN_EN_PORT);
    gpio_enable_clock(KBD_LATCH_PORT);

    /* Columns PC0-PC3: push-pull output, 50 MHz */
    gpio_config_pin(KBD_COL_PORT, GPIO_PIN_0, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_config_pin(KBD_COL_PORT, GPIO_PIN_1, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_config_pin(KBD_COL_PORT, GPIO_PIN_2, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_config_pin(KBD_COL_PORT, GPIO_PIN_3, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);

    /* Rows PD4-PD7: input with pull-up */
    gpio_config_pin(KBD_ROW_PORT, GPIO_PIN_4, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_config_pin(KBD_ROW_PORT, GPIO_PIN_5, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_config_pin(KBD_ROW_PORT, GPIO_PIN_6, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_config_pin(KBD_ROW_PORT, GPIO_PIN_7, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    /* Activate pull-ups by setting ODR bits high */
    gpio_set_pin(KBD_ROW_PORT, GPIO_PIN_4 | GPIO_PIN_5 |
                                GPIO_PIN_6 | GPIO_PIN_7);

    /* Scan enable (PC5 - BINARY VERIFIED): input, floating */
    gpio_config_pin(KBD_SCAN_EN_PORT, KBD_SCAN_EN_PIN,
                    GPIO_MODE_INPUT, GPIO_CNF_FLOATING);

    /* Latch (PA7 - BINARY VERIFIED): input, floating */
    gpio_config_pin(KBD_LATCH_PORT, KBD_LATCH_PIN,
                    GPIO_MODE_INPUT, GPIO_CNF_FLOATING);

    /* Idle state: all columns high */
    gpio_set_pin(KBD_COL_PORT, KBD_COL_MASK);
}

uint8_t keypad_scan(void)
{
    /* Check scan enable - PC5 must be HIGH to proceed */
    if (!gpio_read_pin(KBD_SCAN_EN_PORT, KBD_SCAN_EN_PIN))
        return KEY_NONE;

    /* Check latch - PA7 must be LOW (not latched by another subsystem) */
    if (gpio_read_pin(KBD_LATCH_PORT, KBD_LATCH_PIN))
        return KEY_NONE;

    /* Drive all columns high, then wait for settle */
    gpio_set_pin(KBD_COL_PORT, KBD_COL_MASK);
    scan_delay();

    /* Quick check: if all rows are high with all columns high, nobody
     * is pressing anything - bail out early. */
    if (read_rows() == KBD_ROW_MASK)
        return KEY_NONE;

    /* Scan each of 5 columns */
    for (uint8_t col = 0; col < 5; col++) {
        /* Drive the selected column low, others high */
        gpio_clear_pin(KBD_COL_PORT, col_clear[col]);
        if (col_set[col])
            gpio_set_pin(KBD_COL_PORT, col_set[col]);
        scan_delay();

        uint8_t rows = read_rows();
        if (rows == KBD_ROW_MASK)
            continue;   /* no key in this column */

        uint8_t row = decode_row(rows);
        if (row == 0xFF)
            continue;   /* multiple keys or noise */

        /* Restore idle state before returning */
        gpio_set_pin(KBD_COL_PORT, KBD_COL_MASK);
        return (uint8_t)(col * 4 + row);
    }

    /* No key found - restore idle */
    gpio_set_pin(KBD_COL_PORT, KBD_COL_MASK);
    return KEY_NONE;
}

/* ---- Debounced key-event state machine --------------------------------- */

static uint8_t  prev_key   = KEY_NONE;  /* last accepted key */
static uint8_t  db_count   = 0;         /* debounce counter */
static uint16_t hold_count = 0;         /* repeat timer */

uint8_t keypad_get_event(key_event_t *evt)
{
    uint8_t raw = keypad_scan();
    evt->type = KEY_EVT_NONE;
    evt->key  = KEY_NONE;

    if (raw != KEY_NONE && raw == prev_key) {
        /* Same key still held - handle repeat */
        hold_count++;
        if (hold_count == KEY_REPEAT_DELAY) {
            evt->type = KEY_EVT_REPEAT;
            evt->key  = prev_key;
            return 1;
        }
        if (hold_count > KEY_REPEAT_DELAY &&
            ((hold_count - KEY_REPEAT_DELAY) % KEY_REPEAT_RATE) == 0) {
            evt->type = KEY_EVT_REPEAT;
            evt->key  = prev_key;
            return 1;
        }
        return 0;
    }

    if (raw != KEY_NONE && raw != prev_key) {
        /* New key candidate - debounce */
        db_count++;
        if (db_count >= KEY_DEBOUNCE_COUNT) {
            /* Generate release for previous key if one was held */
            if (prev_key != KEY_NONE) {
                evt->type = KEY_EVT_RELEASE;
                evt->key  = prev_key;
                prev_key  = raw;
                db_count  = 0;
                hold_count = 0;
                return 1;
            }
            /* Accept new press */
            prev_key   = raw;
            db_count   = 0;
            hold_count = 0;
            evt->type  = KEY_EVT_PRESS;
            evt->key   = raw;
            return 1;
        }
        return 0;
    }

    /* raw == KEY_NONE */
    if (prev_key != KEY_NONE) {
        db_count++;
        if (db_count >= KEY_DEBOUNCE_COUNT) {
            evt->type  = KEY_EVT_RELEASE;
            evt->key   = prev_key;
            prev_key   = KEY_NONE;
            db_count   = 0;
            hold_count = 0;
            return 1;
        }
        return 0;
    }

    db_count = 0;
    return 0;
}
