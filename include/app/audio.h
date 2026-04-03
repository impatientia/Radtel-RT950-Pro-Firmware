/*
 * audio.h - System audio feedback (beeps, tones, alert patterns) for RT-950 Pro
 *
 * Uses DAC audio (PA4 via TIM6/DMA) for tone generation.
 * All tones are non-blocking: start playback + set a duration,
 * then audio_poll() (called from super_loop) stops it when done.
 *
 * Multi-tone patterns (ascending/descending sequences) provide
 * audible feedback for events like power-on, scan hit, and alerts.
 */

#ifndef APP_AUDIO_H
#define APP_AUDIO_H

#include <stdint.h>

/* Initialize audio subsystem (call after dac_audio_init) */
void audio_init(void);

/* Key-press confirmation beep (1000 Hz, 50 ms) */
void audio_beep(void);

/* Error/reject beep (500 Hz, 100 ms) */
void audio_error_beep(void);

/*
 * Roger beep - sent at end of TX.
 * freq_sel: 0=1000 Hz, 1=1450 Hz, 2=1750 Hz, 3=2100 Hz
 */
void audio_roger_beep(uint8_t freq_sel);

/* Power-on chime: ascending 3-tone (800 -> 1200 -> 1600 Hz) */
void audio_power_on(void);

/* Alert tone: rapid 2-tone alternation (1200/800 Hz, 3 cycles) */
void audio_alert(void);

/* Scan hit: quick rising chirp (600 -> 1000 Hz) */
void audio_scan_hit(void);

/* Call from super_loop at >=100 Hz to manage tone duration and sequences */
void audio_poll(void);

#endif /* APP_AUDIO_H */
