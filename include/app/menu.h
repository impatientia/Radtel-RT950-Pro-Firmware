/*
 * menu.h - Hierarchical menu system for the RT-950 Pro
 *
 * 12-category menu matching OEM structure, with category -> item -> edit
 * navigation. Integrates with keypad, encoder, and display subsystems.
 *
 * V0.27 OEM menu structure (string table at fw 0x0805A440-0x0805BC00):
 *   12 top-level categories, each with sub-items.
 *   Format: 02 ID 20 "string" 00  (type=0x02 item, type=0x07 header)
 *   Menu dispatch at fw 0x080062EA (CMP chain on menu ID byte).
 *   ToneMenu_ShowEntry at fw 0x0800AF68.
 *
 * OEM top-level menu IDs (from string table at fw 0x0805B34C-0x0805B440):
 *   0x12 = VOX          0x13 = Zone          0x14 = VFO & CH
 *   0x15 = CTCSS DCS    0x16 = Radio Set     0x17 = APRS Set
 *   0x18 = User Key     0x19 = Bluetooth     0x1A = Signaling
 *   0x1B = Setting      0x1C = Reset         0x1D = About
 */

#ifndef APP_MENU_H
#define APP_MENU_H

#include <stdint.h>

/* OEM top-level menu category IDs (fw 0x0805B34C-0x0805B440) */
#define OEM_MENU_ID_VOX         0x12
#define OEM_MENU_ID_ZONE        0x13
#define OEM_MENU_ID_VFO_CH      0x14
#define OEM_MENU_ID_CTCSS_DCS   0x15
#define OEM_MENU_ID_RADIO_SET   0x16
#define OEM_MENU_ID_APRS_SET    0x17
#define OEM_MENU_ID_USER_KEY    0x18
#define OEM_MENU_ID_BLUETOOTH   0x19
#define OEM_MENU_ID_SIGNALING   0x1A
#define OEM_MENU_ID_SETTING     0x1B
#define OEM_MENU_ID_RESET       0x1C
#define OEM_MENU_ID_ABOUT       0x1D

/* Menu item IDs (flat index, used by format_value and get/set dispatch) */
typedef enum {
    /* VOX */
    MENU_VOX_SWITCH = 0,
    MENU_VOX_LEVEL,
    MENU_VOX_DELAY,
    /* VFO & CH */
    MENU_STEP,
    MENU_OFFSET_DIR,
    MENU_OFFSET_FREQ,
    MENU_MODULATION,
    MENU_CH_NAME,
    /* CTCSS DCS */
    MENU_CTCSS_TX,
    MENU_CTCSS_RX,
    MENU_DCS_TX,
    MENU_DCS_RX,
    MENU_SCRAMBLER,
    MENU_SCAN_ADD,
    /* Radio Set */
    MENU_SQUELCH,
    MENU_TX_POWER,
    MENU_BANDWIDTH,
    MENU_BCL,
    MENU_TOT,
    MENU_STE,
    MENU_ROGER,
    MENU_SCAN_MODE,
    MENU_RPSTE,
    MENU_RPT_RL,
    MENU_DUAL_WATCH,
    MENU_BATTERY_SAVE,
    MENU_CROSSBAND,
    /* APRS Set */
    MENU_APRS_ENABLE,
    MENU_APRS_SYMBOL,
    /* Bluetooth */
    MENU_BT_SWITCH,
    /* Signaling */
    MENU_PTT_ID,
    MENU_DTMF_ST,
    /* Setting */
    MENU_BEEP,
    MENU_VOICE_PROMPT,
    MENU_KEY_LOCK,
    MENU_MENU_TIMEOUT,
    MENU_POWER_MSG,
    MENU_BREATH_LED,
    MENU_NOAA_ALARM,
    MENU_BACKLIGHT,
    MENU_AUTO_POWER_OFF,
    /* Reset */
    MENU_FACTORY_RESET,
    /* About */
    MENU_ABOUT_VERSION,
    /* Count */
    MENU_ITEM_COUNT,
} menu_item_t;

#define MENU_CATEGORY_COUNT  12

/* Menu states */
typedef enum {
    MENU_STATE_CLOSED = 0,  /* Normal operation */
    MENU_STATE_CATEGORY,    /* Browsing top-level categories */
    MENU_STATE_LIST,        /* Browsing items within a category */
    MENU_STATE_EDIT,        /* Editing a value */
} menu_state_t;

/* Initialize the menu system */
void menu_init(void);

/* Open/close menu */
void menu_open(void);
void menu_close(void);

/* Get current menu state */
menu_state_t menu_get_state(void);

/* Handle encoder input (scroll categories, items, or adjust values) */
void menu_handle_encoder(int8_t direction);

/* Handle key press (confirm, back, digit entry) */
void menu_handle_key(uint8_t key);

/* Draw the current menu screen to LCD. Call from display refresh loop. */
void menu_draw(void);

#endif /* APP_MENU_H */
