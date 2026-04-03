/*
 * power.c - Power management for the RT-950 Pro
 *
 * Battery: ADC2 channel 0 (PA0), 12-bit result shifted to 8-bit for
 * threshold comparison.  Voltage: mV = adc_8bit x 482 / 10.
 *
 * Seven ascending thresholds define 8 battery states.  Threshold[6] is
 * forced to 0xBB (~9.01V) per RE analysis of calibration loader.
 *
 * Auto power-off: idle counter incremented by power_poll() (~1Hz).
 * Shutdown writes SYSRESETREQ to SCB->AIRCR.
 *
 * Backlight: direct GPIO on LCD_BL_PORT / LCD_BL_PIN (PC6).
 */

#include "app/power.h"
#include "app/audio.h"
#include "app/settings.h"
#include "drivers/adc.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"
#include "cortex_m4.h"
#include <stddef.h>

/*
 * Default thresholds (ascending, 8-bit ADC scale).
 * These are overwritten at runtime by power_set_thresholds() when
 * calibration data is loaded from flash 0xF200.
 */
static uint8_t batt_thresholds[7] = {
    0x86, 0x8C, 0x92, 0x98, 0x9E, 0xA4, 0xBB
};

static uint16_t auto_off_minutes;
static uint32_t idle_seconds;

/* Low-battery alert: beep every 30s when battery <= BATT_LOW */
#define LOW_BATT_BEEP_INTERVAL  30   /* seconds between warning beeps */
static uint8_t  low_batt_counter;     /* seconds since last beep */
static uint8_t  low_batt_alert;       /* 1 = in low-battery state */

/* Auto keypad lock: lock after 60s of inactivity */
#define AUTO_KEYLOCK_SECONDS  60
static uint32_t keylock_idle_seconds;

/* ========================================================================
 *  power_init - Set up ADC and reset auto-off state.
 * ======================================================================== */

void power_init(void)
{
    auto_off_minutes = 0;
    idle_seconds = 0;

    /* Force highest threshold to 0xBB per RE analysis */
    batt_thresholds[6] = 0xBB;
}

/* ========================================================================
 *  power_read_battery_raw - 8-bit battery ADC reading.
 *
 *  adc_read_battery() returns 8-bit directly (matching OEM UBFX).
 * ======================================================================== */

uint8_t power_read_battery_raw(void)
{
    return adc_read_battery();
}

/* ========================================================================
 *  power_get_battery_level - Map ADC reading to battery_level_t.
 *
 *  Compares 8-bit raw value against 7 ascending thresholds.
 *  Returns BATT_CRITICAL (0) if below threshold[0], up to
 *  BATT_FULL (7) if at or above threshold[6].
 * ======================================================================== */

battery_level_t power_get_battery_level(void)
{
    uint8_t raw = power_read_battery_raw();

    for (uint8_t i = 0; i < 7; i++) {
        if (raw < batt_thresholds[i])
            return (battery_level_t)i;
    }
    return BATT_FULL;
}

/* ========================================================================
 *  power_get_battery_mv - Convert 8-bit ADC to millivolts.
 *
 *  Formula: mV = adc_8bit x 482 / 10
 *  Verified: 0xBB (187) x 482 / 10 = 9013 mV ~ 9.01V
 * ======================================================================== */

uint16_t power_get_battery_mv(void)
{
    uint8_t raw = power_read_battery_raw();
    return (uint16_t)((uint32_t)raw * 482U / 10U);
}

/* ========================================================================
 *  power_set_thresholds - Load thresholds from calibration data.
 *
 *  Accepts 7 ascending 8-bit values.  Threshold[6] is forced to 0xBB
 *  regardless of input, matching OEM calibration loader behavior.
 * ======================================================================== */

void power_set_thresholds(const uint8_t thresholds[7])
{
    for (uint8_t i = 0; i < 7; i++)
        batt_thresholds[i] = thresholds[i];

    /* OEM forces the top threshold */
    batt_thresholds[6] = 0xBB;
}

/* ========================================================================
 *  Auto power-off
 * ======================================================================== */

void power_set_auto_off(uint16_t minutes)
{
    auto_off_minutes = minutes;
    idle_seconds = 0;
}

void power_reset_idle_timer(void)
{
    idle_seconds = 0;
    keylock_idle_seconds = 0;
}

void power_poll(void)
{
    /* Auto power-off countdown */
    if (auto_off_minutes != 0) {
        idle_seconds++;
        if (idle_seconds >= (uint32_t)auto_off_minutes * 60U)
            power_shutdown();
    }

    /* Auto keypad lock after inactivity */
    {
        const settings_t *s = settings_get();
        if (s->auto_keypad_lock && !s->keypad_lock) {
            keylock_idle_seconds++;
            if (keylock_idle_seconds >= AUTO_KEYLOCK_SECONDS)
                settings_set_u8(offsetof(settings_t, keypad_lock), 1);
        }
    }

    /* Low-battery warning: periodic beep when <= BATT_LOW */
    battery_level_t level = power_get_battery_level();
    if (level <= BATT_LOW) {
        low_batt_alert = 1;
        low_batt_counter++;
        if (low_batt_counter >= LOW_BATT_BEEP_INTERVAL) {
            low_batt_counter = 0;
            audio_beep();  /* audible warning every 30s */
        }
    } else {
        low_batt_alert = 0;
        low_batt_counter = 0;
    }
}

uint8_t power_is_low_batt_alert(void)
{
    return low_batt_alert;
}

/* ========================================================================
 *  Backlight control - PC6 + PB3 (BINARY VERIFIED @ 0x08017C40)
 * ======================================================================== */

void power_backlight_on(void)
{
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    gpio_set_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
}

void power_backlight_off(void)
{
    gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);
    gpio_clear_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
}

void power_backlight_toggle(void)
{
    if (LCD_BL_PORT->ODR & LCD_BL_PIN) {
        gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);
        gpio_clear_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
    } else {
        gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
        gpio_set_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
    }
}

/* ========================================================================
 *  Breathing light - software PWM triangle wave on backlight GPIO.
 *
 *  Called at ~200 Hz.  PWM period = 20 ticks (100 Hz effective).
 *  Duty cycle ramps 0->20->0 over 400 ticks total (~2 s cycle).
 *  Only active when settings.breathing_light is enabled.
 * ======================================================================== */

#define BREATH_PWM_PERIOD  20   /* ticks per PWM cycle */
#define BREATH_RAMP_STEPS  20   /* duty steps from min to max */

static uint16_t breath_counter;  /* 0..399 */

void power_breathing_poll(void)
{
    if (!settings_get()->breathing_light) return;

    uint16_t phase = breath_counter % (BREATH_RAMP_STEPS * 2 * BREATH_PWM_PERIOD);
    uint16_t ramp_pos = phase / BREATH_PWM_PERIOD;  /* 0..39 */
    uint16_t duty;

    /* Triangle: ramp up 0->20, then down 20->0 */
    if (ramp_pos < BREATH_RAMP_STEPS)
        duty = ramp_pos;
    else
        duty = (BREATH_RAMP_STEPS * 2) - ramp_pos;

    uint16_t pwm_pos = phase % BREATH_PWM_PERIOD;
    if (pwm_pos < duty)
        gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    else
        gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);

    breath_counter++;
    if (breath_counter >= BREATH_RAMP_STEPS * 2 * BREATH_PWM_PERIOD)
        breath_counter = 0;
}

/* ========================================================================
 *  power_shutdown - System reset via NVIC SYSRESETREQ.
 *
 *  Writes 0x05FA0004 to SCB->AIRCR.  The DSB ensures the write
 *  completes before the core halts.  WFI loop is a safety net.
 * ======================================================================== */

void power_shutdown(void)
{
    __disable_irq();
    __DSB();
    SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    __DSB();
    for (;;)
        __WFI();
}
