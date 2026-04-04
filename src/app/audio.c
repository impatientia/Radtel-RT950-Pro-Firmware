/*
 * audio.c - System audio feedback for RT-950 Pro
 *
 * Non-blocking beep/tone system: caller starts a tone, audio_poll()
 * auto-stops it after the requested duration.  Uses the DAC sine-tone
 * generator (dac_audio_play_tone / dac_audio_stop).
 *
 * Audio routing GPIOs (OEM verified):
 *   PC12 (BEEP_SW)  - HIGH enables beep-to-speaker path  @ 0x080038EA
 *   PB8  (AMP_EN)   - HIGH enables audio amplifier       @ 0x08006574
 *   PE1  (SPK_MUTE) - HIGH mutes speaker (used during TX) @ 0x08019254
 *
 * OEM beep_play @ 0x08003808: SETs PC12+PB8 before tone, CLRs after.
 * OEM spk_mute_on_ptt @ 0x08019254: SETs PE1 during TX.
 * OEM spk_unmute @ 0x0801929C: CLRs PE1 when returning to RX.
 *
 * Multi-tone sequences are driven by a step table: each step has a
 * frequency (tenths-of-Hz) and duration (ms).  audio_poll() advances
 * through the steps automatically.
 */

#include "app/audio.h"
#include "drivers/dac_audio.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"
#include "app/settings.h"

extern uint32_t get_tick(void);

/* Tone state */

static uint32_t tone_end_ms;
static uint8_t  tone_active;

/* Multi-tone sequence state */

typedef struct {
    uint16_t freq_x10;
    uint16_t duration_ms;
} tone_step_t;

#define SEQ_MAX_STEPS  8

static const tone_step_t *seq_steps;
static uint8_t  seq_count;
static uint8_t  seq_index;
static uint8_t  seq_active;

/* Start a single tone */
static void audio_path_enable(void)
{
    gpio_set_pin(BEEP_SW_PORT, BEEP_SW_PIN);    /* PC12 HIGH - route beep to speaker */
    gpio_set_pin(AMP_EN_PORT,  AMP_EN_PIN);     /* PB8  HIGH - enable audio amplifier */
}

static void audio_path_disable(void)
{
    gpio_clear_pin(AMP_EN_PORT,  AMP_EN_PIN);   /* PB8  LOW - disable amplifier */
    gpio_clear_pin(BEEP_SW_PORT, BEEP_SW_PIN);  /* PC12 LOW - disconnect beep path */
}

static void tone_start(uint16_t freq_x10, uint16_t duration_ms)
{
    audio_path_enable();
    dac_audio_play_tone(freq_x10);
    tone_end_ms = get_tick() + duration_ms;
    tone_active = 1;
    seq_active = 0;
}

/* Start a multi-tone sequence */
static void seq_start(const tone_step_t *steps, uint8_t count)
{
    if (!settings_get()->beep || count == 0) return;
    seq_steps = steps;
    seq_count = count;
    seq_index = 0;
    seq_active = 1;
    if (steps[0].freq_x10 > 0) {
        audio_path_enable();
        dac_audio_play_tone(steps[0].freq_x10);
    } else {
        dac_audio_stop();
    }
    tone_end_ms = get_tick() + steps[0].duration_ms;
    tone_active = 1;
}

/* Public API */

void audio_init(void)
{
    tone_active = 0;
    tone_end_ms = 0;
    seq_active = 0;
}

void audio_beep(void)
{
    if (!settings_get()->beep) return;
    tone_start(10000, 50);   /* 1000.0 Hz x 50 ms */
}

void audio_error_beep(void)
{
    if (!settings_get()->beep) return;
    tone_start(5000, 100);   /* 500.0 Hz x 100 ms */
}

void audio_beep_freq(uint16_t freq_x10, uint16_t duration_ms)
{
    if (!settings_get()->beep) return;
    tone_start(freq_x10, duration_ms);
}

static const uint16_t roger_freq_x10[4] = { 10000, 14500, 17500, 21000 };

void audio_roger_beep(uint8_t freq_sel)
{
    if (freq_sel > 3) freq_sel = 0;
    tone_start(roger_freq_x10[freq_sel], 200);
}

/* Power-on chime: ascending 3-tone (800 -> 1200 -> 1600 Hz) */
static const tone_step_t power_on_seq[] = {
    { 8000,  80 },  /* 800 Hz, 80 ms */
    {    0,  30 },  /* silence gap */
    { 12000, 80 },  /* 1200 Hz, 80 ms */
    {    0,  30 },  /* silence gap */
    { 16000, 120 }, /* 1600 Hz, 120 ms */
};

void audio_power_on(void)
{
    seq_start(power_on_seq, 5);
}

/* Alert: rapid 2-tone alternation (1200/800 Hz, 3 cycles) */
static const tone_step_t alert_seq[] = {
    { 12000, 100 }, { 8000, 100 },
    { 12000, 100 }, { 8000, 100 },
    { 12000, 100 }, { 8000, 100 },
};

void audio_alert(void)
{
    seq_start(alert_seq, 6);
}

/* Scan hit: quick rising chirp (600 -> 1000 Hz) */
static const tone_step_t scan_hit_seq[] = {
    { 6000,  60 },
    { 8000,  60 },
    { 10000, 80 },
};

void audio_scan_hit(void)
{
    seq_start(scan_hit_seq, 3);
}

/* Poll (call from super_loop >=100 Hz) */

void audio_poll(void)
{
    if (!tone_active) return;

    if (get_tick() >= tone_end_ms) {
        if (seq_active && seq_index + 1 < seq_count) {
            seq_index++;
            const tone_step_t *s = &seq_steps[seq_index];
            if (s->freq_x10 > 0) {
                audio_path_enable();
                dac_audio_play_tone(s->freq_x10);
            } else {
                dac_audio_stop();
                /* Keep path enabled during silence gaps - OEM does not
                 * toggle PC12/PB8 between sequence steps. */
            }
            tone_end_ms = get_tick() + s->duration_ms;
        } else {
            dac_audio_stop();
            audio_path_disable();
            tone_active = 0;
            seq_active = 0;
        }
    }
}

/* ========================================================================
 *  Speaker mute / unmute - for TX path control.
 *
 *  OEM spk_mute_on_ptt @ 0x08019254: SETs PE1 (mute speaker) + PB8 (mic on)
 *  OEM spk_unmute       @ 0x0801929C: CLRs PE1 (unmute) + CLRs PB8 (mic off)
 *
 *  PE1 polarity: HIGH = muted, LOW = unmuted.
 *  Called by radio TX/RX transitions (not by beep/tone path).
 * ======================================================================== */

void audio_speaker_mute(void)
{
    gpio_set_pin(SPK_MUTE_PORT, SPK_MUTE_PIN);     /* PE1 HIGH = muted */
}

void audio_speaker_unmute(void)
{
    gpio_clear_pin(SPK_MUTE_PORT, SPK_MUTE_PIN);   /* PE1 LOW = unmuted */
}
