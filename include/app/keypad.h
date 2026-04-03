/*
 * keypad.h - Keypad scanner for the RT-950 Pro
 *
 * 4x4+1 matrix: columns PC0-PC3, rows PD4-PD7.
 * Scan enable on PC5, latch on PA7 (BINARY VERIFIED @ 0x080136B0 V0.27).
 *
 * Scan algorithm reverse-engineered from V0.27 binary @ fw 0x08012FF8.
 */

#ifndef APP_KEYPAD_H
#define APP_KEYPAD_H

#include <stdint.h>

/* --- Key codes (column * 4 + row) --------------------------------------- */
#define KEY_1           0       /* C0 R0 */
#define KEY_2           1       /* C0 R1 */
#define KEY_3           2       /* C0 R2 */
#define KEY_A_VFO       3       /* C0 R3 */
#define KEY_4           4       /* C1 R0 */
#define KEY_5           5       /* C1 R1 */
#define KEY_6           6       /* C1 R2 */
#define KEY_B_SCAN      7       /* C1 R3 */
#define KEY_7           8       /* C2 R0 */
#define KEY_8           9       /* C2 R1 */
#define KEY_9           10      /* C2 R2 */
#define KEY_C_MENU      11      /* C2 R3 */
#define KEY_STAR        12      /* C3 R0 */
#define KEY_0           13      /* C3 R1 */
#define KEY_HASH        14      /* C3 R2 */
#define KEY_D_BAND      15      /* C3 R3 */
#define KEY_PTT         16      /* C4 R0 - side button */
#define KEY_SIDE1       17      /* C4 R1 */
#define KEY_SIDE2       18      /* C4 R2 */
#define KEY_SIDE3       19      /* C4 R3 */
#define KEY_NONE        0xFF

/* --- Key event types ---------------------------------------------------- */
typedef enum {
    KEY_EVT_NONE = 0,
    KEY_EVT_PRESS,
    KEY_EVT_REPEAT,
    KEY_EVT_RELEASE
} key_event_type_t;

typedef struct {
    key_event_type_t type;
    uint8_t          key;       /* KEY_* code */
} key_event_t;

/* --- Timing (in units of keypad_get_event() call intervals) ------------- */
#define KEY_DEBOUNCE_COUNT  3   /* calls before accepting a press */
#define KEY_REPEAT_DELAY   30   /* calls before first repeat */
#define KEY_REPEAT_RATE     6   /* calls between subsequent repeats */

/* --- API ---------------------------------------------------------------- */

/* Configure GPIO for keypad matrix. Call once at startup. */
void keypad_init(void);

/*
 * Perform a single matrix scan.
 * Returns KEY_* code (0-19) or KEY_NONE (0xFF) if no key pressed.
 * Checks scan-enable (PC5) and latch (PA7) before scanning.
 */
uint8_t keypad_scan(void);

/*
 * Debounced key event detector.
 * Call periodically (e.g. every 5-10 ms from a timer tick).
 * Fills *evt with press / repeat / release events.
 * Returns 1 if an event is available, 0 otherwise.
 */
uint8_t keypad_get_event(key_event_t *evt);

#endif /* APP_KEYPAD_H */
