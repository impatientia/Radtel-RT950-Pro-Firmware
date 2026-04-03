/*
 * aprs.c - APRS via BK4829 hardware AFSK for the RT-950 Pro
 *
 * The BK4829 transceiver handles ALL Bell 202 AFSK modulation and
 * demodulation internally. The MCU only needs to:
 *   1. Build AX.25 MIC-E frame in RAM
 *   2. Program BK4829 FSK registers (0x58, 0x59, 0x5A-5F, 0x70, 0x72)
 *   3. Write data to BK4829 FIFO register 0x5F (16-bit words)
 *   4. Poll register 0x0C bit 0 for TX completion
 *
 * Based on V0.27 binary analysis:
 *   APRS_SetPTT        @ fw 0x0800E920
 *   APRS_SendCommand11 @ fw 0x0800E94C
 *   APRS_CheckReady    @ fw 0x0800E9C4
 *   APRS_ReadStatus    @ fw 0x0800E9F0
 *   APRS_StartTX       @ fw 0x0800EA78
 *   APRS_TX_Trigger      @ 0x0800C97C
 *   APRS_MicE_EncodeDest @ 0x08028FE4
 *   APRS_MicE_BuildInfo  @ 0x0800AC2C
 */

#include "app/aprs.h"
#include "drivers/bk4829.h"
#include "drivers/spi.h"
#include "drivers/flash_layout.h"
#include "app/audio.h"
#include "app/gps.h"
#include "rt950_pinmap.h"

#include <string.h>

extern void delay_ms(uint32_t ms);

/* State ------------------------------------------------------------- */
static aprs_state_t  state;
static aprs_config_t config;
static aprs_packet_t last_rx_pkt;
static uint16_t      beacon_countdown;  /* seconds until next beacon */

/* Beacon interval table: index -> seconds */
static const uint16_t beacon_intervals[] = {
    30, 60, 120, 180, 300, 600, 900, 1800
};
#define BEACON_INTERVAL_COUNT  \
    (sizeof(beacon_intervals) / sizeof(beacon_intervals[0]))

/* ==========================================================================
 *  Flash configuration load / save
 * ========================================================================== */

static void trim_ff(char *s, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        if ((uint8_t)s[i] == 0xFF)
            s[i] = 0;
    }
}

void aprs_config_load(aprs_config_t *cfg)
{
    uint8_t blk[48];
    memset(cfg, 0, sizeof(*cfg));

    /* Defaults */
    cfg->lat_ns = 'N';
    cfg->lon_ew = 'W';
    cfg->symbol_table = '/';
    cfg->symbol_code  = APRS_SYM_DEFAULT;
    cfg->beacon_interval = 4;   /* 5 minutes */
    cfg->popup_time = 5;

    /* Block 1: basic + position (0x00FFFF, 16 bytes) */
    spi_flash_read(FLASH_ADDR_APRS, blk, 16);
    if (blk[0] != 0xFF) cfg->enable      = blk[0] & 1;
    if (blk[1] != 0xFF) cfg->gps_source  = blk[1];
    if (blk[2] != 0xFF) cfg->latlon_unit = (latlon_unit_t)(blk[2] & 1);
    if (blk[3] != 0xFF) cfg->speed_unit  = (speed_unit_t)(blk[3] % 3);
    if (blk[4] != 0xFF) cfg->dist_unit   = (distance_unit_t)(blk[4] & 1);
    if (blk[5] != 0xFF) cfg->alt_unit    = (altitude_unit_t)(blk[5] & 1);
    if (blk[6] != 0xFF) cfg->timezone    = (int8_t)blk[6];
    if (blk[7] != 0xFF) cfg->lat_ns      = blk[7];
    if (blk[8] != 0xFF) cfg->lat_deg     = blk[8];
    if (blk[9] != 0xFF) cfg->lat_min     = blk[9];
    if (blk[10] != 0xFF) cfg->lat_frac   = blk[10];
    if (blk[11] != 0xFF) cfg->lon_ew     = blk[11];
    if (blk[12] != 0xFF) cfg->lon_deg    = blk[12];
    if (blk[13] != 0xFF) cfg->lon_min    = blk[13];
    if (blk[14] != 0xFF) cfg->lon_frac   = blk[14];
    cfg->altitude = (uint16_t)blk[15] << 8;
    /* Second byte of altitude from block overlap - use 0 if unset */

    /* Block 2: identity & routing (0x01000F, 16 bytes) */
    spi_flash_read(FLASH_ADDR_APRS_EXT, blk, 16);
    memcpy(cfg->callsign, &blk[0], 6);
    cfg->callsign[6] = '\0';
    trim_ff(cfg->callsign, 6);
    cfg->ssid         = blk[6] & 0x0F;
    cfg->path_mode    = (aprs_path_mode_t)(blk[7] & 3);
    cfg->symbol_idx   = blk[8];
    if (blk[9] != 0xFF) cfg->symbol_table = (char)blk[9];
    if (blk[10] != 0xFF) cfg->symbol_code = (char)blk[10];
    cfg->tx_priority  = blk[11] & 1;

    /* Block 3: beacon & protocol (0x01001F, 16 bytes) */
    spi_flash_read(FLASH_ADDR_APRS_BEACON, blk, 16);
    cfg->beacon_mode    = (aprs_beacon_mode_t)(blk[0] & 1);
    cfg->beacon_interval = blk[1];
    if (cfg->beacon_interval >= BEACON_INTERVAL_COUNT)
        cfg->beacon_interval = 4;
    cfg->tx_delay        = blk[2];
    cfg->mic_e_type      = (mice_msg_type_t)(blk[3] & 7);
    cfg->tnc_data_type   = blk[4];
    cfg->tx_data_reporting = blk[5];
    cfg->decode_prompt   = blk[6];
    cfg->rx_popup        = blk[7];
    cfg->popup_time      = blk[8];
    if (cfg->popup_time == 0 || cfg->popup_time == 0xFF)
        cfg->popup_time = 5;
    cfg->fwd_channel     = blk[9];
    cfg->fwd_routing     = blk[10];
    cfg->wait_forward    = blk[11];

    /* Block 4: custom routing (0x01002F, 16 bytes) */
    spi_flash_read(FLASH_ADDR_APRS_ROUTE, blk, 16);
    memcpy(cfg->custom_path1, &blk[0], 6);
    cfg->custom_path1[6] = '\0';
    trim_ff(cfg->custom_path1, 6);
    cfg->digi1_ssid = blk[6] & 0x0F;
    memcpy(cfg->custom_path2, &blk[7], 6);
    cfg->custom_path2[6] = '\0';
    trim_ff(cfg->custom_path2, 6);
    cfg->digi2_ssid = blk[13] & 0x0F;

    /* Block 5: custom message (0x01003F, 48 bytes) */
    spi_flash_read(FLASH_ADDR_APRS_MSG, blk, 48);
    cfg->comment_enable = blk[0] & 1;
    memcpy(cfg->comment, &blk[1], 47);
    cfg->comment[47] = '\0';
    trim_ff(cfg->comment, 47);
}

void aprs_config_save(const aprs_config_t *cfg)
{
    /* Runtime config is stored back to RAM only - CPS owns the flash.
     * For now, just update the in-memory config copy. A future WL
     * persistence sector can be added if on-radio config editing is needed. */
    memcpy(&config, cfg, sizeof(config));
}

const aprs_config_t *aprs_config_get(void)
{
    return &config;
}

/* ==========================================================================
 *  Beacon interval helper
 * ========================================================================== */

uint16_t aprs_beacon_interval_sec(uint8_t idx)
{
    if (idx >= BEACON_INTERVAL_COUNT) idx = 4;
    return beacon_intervals[idx];
}

/* ==========================================================================
 *  BK4829 FSK register programming (from firmware @ 0x0801EB8C)
 * ========================================================================== */

static void bk_fsk_tx_init(void)
{
    bk4829_write_reg(0, 0x58, BK_FSK_REG58_TX);   /* TX baud divider */
    bk4829_write_reg(0, 0x70, BK_FSK_REG70_TX);   /* AF mod: AFSK */
    bk4829_write_reg(0, 0x72, BK_FSK_REG72_TX);   /* Mark=1200, Space=2200 */
    bk4829_write_reg(0, 0x5C, BK_FSK_REG5C_TX);   /* TX deviation */
    bk4829_write_reg(0, 0x5D, BK_FSK_REG5D_TX);   /* Preamble timing */
    bk4829_write_reg(0, 0x59, 0x4028);             /* FSK config 1 */
    bk4829_write_reg(0, 0x59, 0x1028);             /* Preamble setup */
    bk4829_write_reg(0, 0x3F, 0x2000);             /* FSK subsystem TX */
}

static void bk_fsk_rx_init(void)
{
    bk4829_write_reg(0, 0x58, BK_FSK_REG58_RX);   /* RX correlator */
    bk4829_write_reg(0, 0x72, BK_FSK_REG72_RX);   /* RX tone filters */
    bk4829_write_reg(0, 0x70, BK_FSK_REG70_RX);   /* AF demod: AFSK RX */
    bk4829_write_reg(0, 0x5D, BK_FSK_REG5D_RX);   /* RX timing */
    bk4829_write_reg(0, 0x59, 0x4028);             /* FSK RX config 1 */
    bk4829_write_reg(0, 0x59, 0x1028);             /* FSK RX config 2 */
    bk4829_write_reg(0, 0x5A, BK_FSK_SYNC_LO);    /* Sync word low */
    bk4829_write_reg(0, 0x5B, BK_FSK_SYNC_HI);    /* Sync word high */
    bk4829_write_reg(0, 0x5C, BK_FSK_REG5C_RX);   /* RX filter/length */
    bk4829_write_reg(0, 0x3F, 0x3000);             /* FSK subsystem RX */
}

static void bk_fsk_stop(void)
{
    bk4829_write_reg(0, 0x02, 0x0000);  /* Disable TX */
    bk4829_write_reg(0, 0x3F, 0x0000);  /* Disable FSK */
    bk4829_write_reg(0, 0x59, 0x0028);  /* Return to idle */
}

/* BK4829 FSK data transmission (from firmware @ 0x0801EEF4) --------- */

static int bk_fsk_send(const uint16_t *data, uint8_t word_count)
{
    /* FSK header from ROM (firmware @ 0x0802C260) */
    static const uint8_t fsk_header[8] = {
        0xFB, 0x72, 0x40, 0x99, 0xA7, 0x00, 0x00, 0x00
    };

    /* Reset FIFO */
    bk4829_write_reg(0, 0x3F, 0x8000);

    /* Configure TX + sync enable */
    bk4829_write_reg(0, 0x59, 0x8028);
    bk4829_write_reg(0, 0x59, 0x0028);

    /* Load sync word from header */
    bk4829_write_reg(0, 0x5A,
        (uint16_t)((uint16_t)fsk_header[0] << 8) | fsk_header[1]);
    bk4829_write_reg(0, 0x5B,
        (uint16_t)((uint16_t)fsk_header[2] << 8) | fsk_header[3]);

    /* Data length config */
    bk4829_write_reg(0, 0x5C,
        (uint16_t)((uint16_t)fsk_header[4] << 8) | 0x30);

    /* Load data into FSK FIFO */
    uint8_t n = word_count;
    if (n > APRS_FIFO_WORDS) n = APRS_FIFO_WORDS;
    for (uint8_t i = 0; i < n; i++) {
        bk4829_write_reg(0, 0x5F, data[i]);
    }

    /* Wait for preamble */
    delay_ms(APRS_PREAMBLE_MS);

    /* Trigger data transmission */
    bk4829_write_reg(0, 0x59, 0x0828);

    /* Poll for completion (reg 0x0C bit 0) */
    for (uint16_t retry = APRS_TX_TIMEOUT_MS / APRS_TX_POLL_MS;
         retry > 0; retry--) {
        delay_ms(APRS_TX_POLL_MS);
        uint16_t status = bk4829_read_reg(0, 0x0C);
        if (status & 1) {
            bk_fsk_stop();
            return 0;
        }
    }

    /* Timeout */
    bk_fsk_stop();
    return -1;
}

/* MIC-E digit encoding (from firmware @ 0x08028FE4) ----------------- */

static uint8_t mice_digit(uint8_t digit, uint8_t msg_bit)
{
    return digit + (msg_bit ? 0x50 : 0x30);
}

void mice_encode_dest(uint8_t *dest, const aprs_position_t *pos,
                      uint8_t msg_type)
{
    uint8_t msg_bits = 7 - (msg_type & 7);

    dest[0] = mice_digit((pos->lat_deg / 10) % 10, (msg_bits >> 2) & 1);
    dest[1] = mice_digit(pos->lat_deg % 10,         (msg_bits >> 1) & 1);
    dest[2] = mice_digit((pos->lat_min / 10) % 10,  (msg_bits >> 0) & 1);
    dest[3] = (pos->lat_frac % 10) + (pos->lat_ns == 'N' ? 0x50 : 0x30);
    uint8_t lon_off = (pos->lon_frac >= 10 && pos->lon_frac <= 99) ? 1 : 0;
    dest[4] = ((pos->lat_frac / 10) % 10) + (lon_off ? 0x30 : 0x50);
    dest[5] = (pos->lat_frac % 10) + (pos->lon_ew == 'W' ? 0x50 : 0x30);
}

/* MIC-E information field (from firmware @ 0x0800AC2C) -------------- */

uint16_t mice_build_info(uint8_t *buf, uint16_t max_len,
                         const aprs_position_t *pos,
                         const aprs_config_t *cfg)
{
    if (max_len < 16) return 0;
    uint16_t p = 0;

    /* Data type indicator: '`' = MIC-E current GPS data */
    buf[p++] = 0x60;

    /* Longitude degrees (MIC-E offset encoding) */
    uint8_t ld = pos->lon_deg;
    if (ld <= 9)
        buf[p++] = ld + 0x76;
    else if (ld <= 99)
        buf[p++] = ld + 0x1C;
    else if (ld <= 109)
        buf[p++] = ld + 0x08;
    else
        buf[p++] = ld - 0x48;

    /* Longitude minutes */
    if (pos->lon_min <= 9)
        buf[p++] = pos->lon_min + 0x58;
    else
        buf[p++] = pos->lon_min + 0x1C;

    /* Longitude fractional minutes */
    uint8_t lon_frac_mice = (uint8_t)((uint16_t)pos->lon_frac * 100 / 60);
    buf[p++] = lon_frac_mice + 0x1C;

    /* Speed / course */
    uint16_t spd = pos->speed / 10;     /* Convert to whole knots */
    uint16_t crs = pos->course;
    buf[p++] = (uint8_t)((spd / 10) * 10 + (crs / 100)) + 0x1C;
    buf[p++] = (uint8_t)((spd % 10) * 10 + ((crs / 10) % 10)) + 0x1C;
    buf[p++] = (uint8_t)(crs % 10) + 0x1C;

    /* Symbol */
    if (cfg->symbol_idx < 4) {
        static const uint8_t sym_tbl[4] = { '[', 'b', '>', 'R' };
        buf[p++] = sym_tbl[cfg->symbol_idx];
        buf[p++] = '/';
    } else {
        buf[p++] = (uint8_t)cfg->symbol_code;
        buf[p++] = '/';
    }

    /* Altitude extension (base-91, 3 chars + '}') */
    if (p + 4 <= max_len) {
        uint16_t alt = pos->altitude + 10000;
        buf[p++] = (uint8_t)(alt / 8281 + 33);     /* 91^2 = 8281 */
        buf[p++] = (uint8_t)((alt / 91) % 91 + 33);
        buf[p++] = (uint8_t)(alt % 91 + 33);
        buf[p++] = '}';
    }

    /* Optional comment */
    if (cfg->comment_enable && cfg->comment[0]) {
        uint16_t clen = (uint16_t)strlen(cfg->comment);
        if (p + clen > max_len) clen = max_len - p;
        memcpy(&buf[p], cfg->comment, clen);
        p += clen;
    }

    return p;
}

/* AX.25 frame builder (from firmware @ 0x0800C97C) ------------------ */

static uint16_t build_ax25_frame(uint8_t *frame, uint16_t max_len,
                                  const aprs_position_t *pos)
{
    if (max_len < 32) return 0;
    uint16_t fp = 0;

    /* Destination address - MIC-E encoded lat goes here */
    uint8_t dest[6];
    mice_encode_dest(dest, pos, (uint8_t)config.mic_e_type);
    for (int i = 0; i < 6; i++) {
        frame[fp++] = (uint8_t)(dest[i] << 1);
    }
    frame[fp++] = 0x60;    /* Dest SSID: command, not last */

    /* Source callsign (left-shifted, space-padded) */
    for (int i = 0; i < 6; i++) {
        if (i < (int)strlen(config.callsign))
            frame[fp++] = (uint8_t)((uint8_t)config.callsign[i] << 1);
        else
            frame[fp++] = (uint8_t)(' ' << 1);
    }

    /* Digipeater path */
    switch (config.path_mode) {
    case APRS_PATH_NONE:
        /* Last address - set bit 0 on SSID byte */
        frame[fp++] = (uint8_t)(0x61 | ((config.ssid & 0xF) << 1));
        break;

    case APRS_PATH_WIDE1_1:
        frame[fp++] = (uint8_t)(0x60 | ((config.ssid & 0xF) << 1));
        {
            const char *w1 = "WIDE1 ";
            for (int i = 0; i < 6; i++)
                frame[fp++] = (uint8_t)((uint8_t)w1[i] << 1);
            frame[fp++] = 0x63;     /* SSID 1, last address */
        }
        break;

    case APRS_PATH_WIDE1_WIDE2:
        frame[fp++] = (uint8_t)(0x60 | ((config.ssid & 0xF) << 1));
        {
            const char *w1 = "WIDE1 ";
            for (int i = 0; i < 6; i++)
                frame[fp++] = (uint8_t)((uint8_t)w1[i] << 1);
            frame[fp++] = 0x62;     /* SSID 1, not last */
            const char *w2 = "WIDE2 ";
            for (int i = 0; i < 6; i++)
                frame[fp++] = (uint8_t)((uint8_t)w2[i] << 1);
            frame[fp++] = 0x63;     /* SSID 1, last address */
        }
        break;

    case APRS_PATH_CUSTOM:
        frame[fp++] = (uint8_t)(0x60 | ((config.ssid & 0xF) << 1));
        if (config.custom_path1[0] && config.custom_path1[0] != (char)0xFF) {
            for (int i = 0; i < 6; i++) {
                if (i < (int)strlen(config.custom_path1))
                    frame[fp++] = (uint8_t)((uint8_t)config.custom_path1[i] << 1);
                else
                    frame[fp++] = (uint8_t)(' ' << 1);
            }
            frame[fp++] = (uint8_t)(0x61 | ((config.digi1_ssid & 0xF) << 1));
        }
        /* Second custom path */
        if (config.custom_path2[0] && config.custom_path2[0] != (char)0xFF) {
            /* Fix previous: mark digi1 as not-last */
            frame[fp - 1] &= 0xFE;
            for (int i = 0; i < 6; i++) {
                if (i < (int)strlen(config.custom_path2))
                    frame[fp++] = (uint8_t)((uint8_t)config.custom_path2[i] << 1);
                else
                    frame[fp++] = (uint8_t)(' ' << 1);
            }
            frame[fp++] = (uint8_t)(0x61 | ((config.digi2_ssid & 0xF) << 1));
        }
        break;
    }

    /* Control + PID */
    frame[fp++] = 0x03;    /* UI frame */
    frame[fp++] = 0xF0;    /* No L3 protocol */

    /* MIC-E information field */
    uint16_t info_len = mice_build_info(&frame[fp], max_len - fp, pos, &config);
    fp += info_len;

    return fp;
}

/* ==========================================================================
 *  AX.25 CRC-16 (FCS) - CCITT polynomial 0x8408 (bit-reversed 0x1021)
 * ========================================================================== */

static uint16_t ax25_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFF;
}

/* ==========================================================================
 *  AX.25 RX frame parser
 * ========================================================================== */

/* Extract a 7-byte AX.25 address field into a callsign-SSID string */
static uint8_t ax25_extract_call(const uint8_t *addr, char *out, uint8_t max)
{
    uint8_t p = 0;
    /* 6 shifted ASCII characters */
    for (uint8_t i = 0; i < 6 && p < max - 4; i++) {
        char c = (char)(addr[i] >> 1);
        if (c != ' ')
            out[p++] = c;
    }
    /* SSID from byte 6, bits [4:1] */
    uint8_t ssid = (addr[6] >> 1) & 0x0F;
    if (ssid > 0 && p < max - 3) {
        out[p++] = '-';
        if (ssid >= 10) {
            out[p++] = '1';
            out[p++] = (char)('0' + ssid - 10);
        } else {
            out[p++] = (char)('0' + ssid);
        }
    }
    out[p] = '\0';
    return p;
}

/* Decode MIC-E destination address into position */
static void mice_decode_dest(const uint8_t *dest_raw, aprs_packet_t *pkt)
{
    /* Dest address bytes are still left-shifted. Un-shift first. */
    uint8_t d[6];
    for (uint8_t i = 0; i < 6; i++)
        d[i] = dest_raw[i] >> 1;

    /* Extract MIC-E message type from bits A/B/C (bytes 0-2, bit 5) */
    uint8_t msg_abc = 0;
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t custom = (d[i] >= 'A' && d[i] <= 'K') ? 1 : 0;
        uint8_t std    = (d[i] >= 'P' && d[i] <= 'Z') ? 1 : 0;
        if (custom || std)
            msg_abc |= (1 << (2 - i));
    }
    pkt->mic_e_msg = (mice_msg_type_t)(7 - msg_abc);

    /* Extract latitude digits */
    uint8_t lat_d[6];
    for (uint8_t i = 0; i < 6; i++) {
        if (d[i] >= '0' && d[i] <= '9')
            lat_d[i] = (uint8_t)(d[i] - '0');
        else if (d[i] >= 'A' && d[i] <= 'J')
            lat_d[i] = (uint8_t)(d[i] - 'A');
        else if (d[i] >= 'P' && d[i] <= 'Y')
            lat_d[i] = (uint8_t)(d[i] - 'P');
        else
            lat_d[i] = 0;
    }

    uint8_t lat_deg = lat_d[0] * 10 + lat_d[1];
    uint8_t lat_min = lat_d[2] * 10 + lat_d[3];
    uint8_t lat_frac = lat_d[4] * 10 + lat_d[5];

    /* N/S from byte 3: >=0x50 = North */
    uint8_t is_north = (d[3] >= 'P') ? 1 : 0;

    /* Longitude offset flag from byte 4: >=0x50 = +100 degrees */
    uint8_t lon_100 = (d[4] >= 'P') ? 1 : 0;

    /* W/E from byte 5: >=0x50 = West (note: OEM uses inverted convention) */
    uint8_t is_west = (d[5] >= 'P') ? 1 : 0;

    /* Convert to signed deg x 100000 */
    int32_t lat = (int32_t)lat_deg * 100000 +
                  (int32_t)lat_min * 100000 / 60 +
                  (int32_t)lat_frac * 100000 / 6000;
    if (!is_north) lat = -lat;
    pkt->lat_deg1e5 = lat;

    /* Store lon_100 flag for info field decode */
    pkt->lon_deg1e5 = lon_100 ? 100L * 100000 : 0;
    (void)is_west;  /* Applied after info field lon decode */
}

/* Decode MIC-E information field (longitude, speed, course, symbol) */
static void mice_decode_info(const uint8_t *info, uint16_t len,
                             aprs_packet_t *pkt)
{
    if (len < 9) return;

    /* Byte 0: data type ('`' or '\'' ) - skip */
    uint8_t p = 1;

    /* Byte 1: longitude degrees */
    uint8_t d28 = info[p++] - 28;
    int32_t lon_deg = (int32_t)d28;
    /* Apply +100 offset from dest byte 4 */
    if (pkt->lon_deg1e5 >= 100L * 100000)
        lon_deg += 100;
    if (lon_deg >= 180 && lon_deg <= 189)
        lon_deg -= 80;
    else if (lon_deg >= 190 && lon_deg <= 199)
        lon_deg -= 190;

    /* Byte 2: longitude minutes */
    uint8_t m28 = info[p++] - 28;
    if (m28 >= 60) m28 -= 60;

    /* Byte 3: longitude hundredths of minutes */
    uint8_t h28 = info[p++] - 28;

    pkt->lon_deg1e5 = lon_deg * 100000 +
                      (int32_t)m28 * 100000 / 60 +
                      (int32_t)h28 * 100000 / 6000;

    /* Bytes 4-6: speed and course */
    uint16_t sp28 = (uint16_t)(info[p++] - 28);
    uint16_t dc28 = (uint16_t)(info[p++] - 28);
    uint16_t se28 = (uint16_t)(info[p++] - 28);

    uint16_t speed = sp28 * 10 + dc28 / 10;
    if (speed >= 800) speed -= 800;
    pkt->speed = speed * 10;    /* knots x 10 */

    uint16_t course = (dc28 % 10) * 100 + se28;
    if (course >= 400) course -= 400;
    pkt->course = course;

    /* Bytes 7-8: symbol code + table */
    if (p + 1 < len) {
        pkt->symbol_code  = (char)info[p++];
        pkt->symbol_table = (char)info[p++];
    }

    /* Altitude extension: look for /A=nnnnnn or base-91 }xxx */
    pkt->altitude = 0;
    while (p + 3 < len) {
        if (info[p] >= 33 && info[p] <= 124 &&
            info[p+1] >= 33 && info[p+1] <= 124 &&
            info[p+2] >= 33 && info[p+2] <= 124 &&
            p + 3 < len && info[p+3] == '}') {
            uint32_t alt91 = (uint32_t)(info[p] - 33) * 8281 +
                             (uint32_t)(info[p+1] - 33) * 91 +
                             (uint32_t)(info[p+2] - 33);
            if (alt91 > 10000)
                pkt->altitude = (uint16_t)(alt91 - 10000);
            p += 4;
            break;
        }
        p++;
    }

    /* Copy remaining as comment */
    uint16_t rem = len - p;
    if (rem > APRS_COMMENT_MAX - 1) rem = APRS_COMMENT_MAX - 1;
    if (rem > 0) memcpy(pkt->comment, &info[p], rem);
    pkt->comment[rem] = '\0';

    pkt->has_position = 1;
}

/* Parse a raw AX.25 UI frame into aprs_packet_t */
static int parse_ax25_frame(const uint8_t *frame, uint16_t len,
                            aprs_packet_t *pkt)
{
    memset(pkt, 0, sizeof(*pkt));

    /* Minimum: 14 addr + 2 ctrl/pid + 1 info + 2 FCS = 19 bytes */
    if (len < 19) return -1;

    /* Verify FCS (last 2 bytes, little-endian) */
    uint16_t fcs_calc = ax25_crc16(frame, len - 2);
    uint16_t fcs_recv = (uint16_t)frame[len - 2] |
                        ((uint16_t)frame[len - 1] << 8);
    if (fcs_calc != fcs_recv)
        return -1;

    uint16_t info_len = len - 2;    /* Exclude FCS */

    /* Destination address (bytes 0-6) */
    ax25_extract_call(&frame[0], pkt->dst_call, APRS_CALL_MAX);

    /* Source address (bytes 7-13) */
    ax25_extract_call(&frame[7], pkt->src_call, APRS_CALL_MAX);

    /* Digipeater path (variable length, 7 bytes each) */
    uint16_t addr_end = 14;
    uint8_t path_pos = 0;
    while (!(frame[addr_end - 1] & 1) && addr_end + 7 <= info_len) {
        char digi[APRS_CALL_MAX];
        ax25_extract_call(&frame[addr_end], digi, APRS_CALL_MAX);
        if (path_pos > 0 && path_pos < APRS_PATH_MAX - 2) {
            pkt->path[path_pos++] = ',';
        }
        uint8_t dlen = (uint8_t)strlen(digi);
        if (path_pos + dlen < APRS_PATH_MAX - 1) {
            memcpy(&pkt->path[path_pos], digi, dlen);
            path_pos += dlen;
        }
        addr_end += 7;
    }
    pkt->path[path_pos] = '\0';

    /* Control (0x03 = UI) + PID (0xF0 = no L3) */
    if (addr_end + 2 > info_len) return -1;
    if (frame[addr_end] != 0x03 || frame[addr_end + 1] != 0xF0)
        return -1;

    uint16_t info_start = addr_end + 2;
    const uint8_t *info = &frame[info_start];
    uint16_t info_bytes = info_len - info_start;

    /* Decode based on data type indicator */
    if (info_bytes > 0 && (info[0] == 0x60 || info[0] == 0x27)) {
        /* MIC-E format - position encoded in destination + info field */
        mice_decode_dest(frame, pkt);
        mice_decode_info(info, info_bytes, pkt);
    } else if (info_bytes > 0 && (info[0] == '!' || info[0] == '=')) {
        /* Uncompressed position report - simple extraction */
        pkt->has_position = 1;
        /* Minimal parse: skip type byte, extract lat/lon as text */
        if (info_bytes >= 20) {
            /* !DDMM.hhN/DDDMM.hhW... */
            /* Lat: bytes 1-8, Lon: bytes 10-18 */
            int32_t lat_d = (info[1] - '0') * 10 + (info[2] - '0');
            int32_t lat_m = (info[3] - '0') * 10 + (info[4] - '0');
            int32_t lat_f = (info[6] - '0') * 10 + (info[7] - '0');
            pkt->lat_deg1e5 = lat_d * 100000 +
                              lat_m * 100000 / 60 +
                              lat_f * 100000 / 6000;
            if (info[8] == 'S') pkt->lat_deg1e5 = -pkt->lat_deg1e5;

            pkt->symbol_table = (char)info[9];

            int32_t lon_d = (info[10] - '0') * 100 +
                            (info[11] - '0') * 10 + (info[12] - '0');
            int32_t lon_m = (info[13] - '0') * 10 + (info[14] - '0');
            int32_t lon_f = (info[16] - '0') * 10 + (info[17] - '0');
            pkt->lon_deg1e5 = lon_d * 100000 +
                              lon_m * 100000 / 60 +
                              lon_f * 100000 / 6000;
            if (info[18] == 'W') pkt->lon_deg1e5 = -pkt->lon_deg1e5;

            pkt->symbol_code = (char)info[19];
        }
    } else {
        /* Status or other - just copy as comment */
        uint16_t clen = info_bytes;
        if (clen > APRS_COMMENT_MAX - 1) clen = APRS_COMMENT_MAX - 1;
        memcpy(pkt->comment, info, clen);
        pkt->comment[clen] = '\0';
    }

    pkt->valid = 1;
    return 0;
}

/* ==========================================================================
 *  Public API
 * ========================================================================== */

void aprs_init(void)
{
    aprs_config_load(&config);
    state = APRS_IDLE;
    memset(&last_rx_pkt, 0, sizeof(last_rx_pkt));

    /* Initialize beacon countdown */
    if (config.beacon_mode == APRS_BEACON_TIMED)
        beacon_countdown = aprs_beacon_interval_sec(config.beacon_interval);
    else
        beacon_countdown = 0;
}

int aprs_is_ready(void)
{
    return state == APRS_IDLE;
}

aprs_state_t aprs_get_state(void)
{
    return state;
}

int aprs_send_position(const aprs_position_t *pos)
{
    if (state != APRS_IDLE) return -1;
    if (!config.enable) return -1;
    if (config.gps_source && !pos->valid) return -1;

    /* Build AX.25 frame */
    uint8_t frame[APRS_MAX_FRAME];
    uint16_t frame_len = build_ax25_frame(frame, sizeof(frame), pos);
    if (frame_len == 0) return -1;

    /* TX delay */
    uint16_t txd_ms = (uint16_t)config.tx_delay * 10;
    if (txd_ms > 0) {
        state = APRS_TX_KEYING;
        delay_ms(txd_ms);
    }

    /* Configure BK4829 for AFSK TX */
    state = APRS_TX_SENDING;
    bk_fsk_tx_init();

    /* Pack frame bytes into 16-bit words for BK4829 FIFO */
    uint16_t fifo_data[APRS_FIFO_WORDS];
    memset(fifo_data, 0, sizeof(fifo_data));
    for (uint8_t i = 0; i < APRS_FIFO_WORDS * 2 && i < frame_len; i += 2) {
        uint16_t w = (uint16_t)((uint16_t)frame[i] << 8);
        if (i + 1 < frame_len) w |= frame[i + 1];
        fifo_data[i / 2] = w;
    }

    /* Send via BK4829 FSK FIFO */
    int result = bk_fsk_send(fifo_data, APRS_FIFO_WORDS);

    state = APRS_TX_COMPLETE;
    bk_fsk_stop();
    state = APRS_IDLE;

    /* Reset beacon timer after manual TX */
    aprs_beacon_reset();

    return result;
}

void aprs_rx_start(void)
{
    bk_fsk_rx_init();
    state = APRS_RX_LISTENING;
}

void aprs_stop(void)
{
    bk_fsk_stop();
    state = APRS_IDLE;
}

int aprs_rx_poll_decode(aprs_packet_t *pkt)
{
    if (state != APRS_RX_LISTENING) return 0;

    /* Check BK4829 FSK interrupt (reg 0x0C bit 1 = RX data ready) */
    uint16_t status = bk4829_read_reg(0, 0x0C);
    if (!(status & 0x02)) return 0;

    /* Read raw bytes from FSK FIFO (reg 0x5F) */
    uint8_t raw[APRS_MAX_FRAME];
    uint16_t pos = 0;
    for (uint8_t i = 0; i < APRS_FIFO_WORDS && pos + 1 < (uint16_t)sizeof(raw); i++) {
        uint16_t word = bk4829_read_reg(0, 0x5F);
        raw[pos++] = (uint8_t)(word >> 8);
        raw[pos++] = (uint8_t)(word & 0xFF);
    }

    /* Clear the interrupt */
    bk4829_write_reg(0, 0x02, 0x0000);

    /* Parse AX.25 frame */
    if (parse_ax25_frame(raw, pos, pkt) == 0) {
        memcpy(&last_rx_pkt, pkt, sizeof(last_rx_pkt));

        /* Prompt beep on decode */
        if (config.decode_prompt)
            audio_beep();

        return 1;
    }

    return 0;
}

const aprs_packet_t *aprs_rx_last_packet(void)
{
    return last_rx_pkt.valid ? &last_rx_pkt : 0;
}

void aprs_beacon_reset(void)
{
    if (config.beacon_mode == APRS_BEACON_TIMED)
        beacon_countdown = aprs_beacon_interval_sec(config.beacon_interval);
}

void aprs_poll(void)
{
    switch (state) {
    case APRS_TX_COMPLETE:
        state = APRS_IDLE;
        break;
    default:
        break;
    }

    /* Timed beacon - called at 1 Hz from main loop */
    if (config.enable &&
        config.beacon_mode == APRS_BEACON_TIMED &&
        state == APRS_IDLE) {
        if (beacon_countdown > 0) {
            beacon_countdown--;
        } else {
            /* Fire beacon using GPS position if available */
            aprs_position_t pos;
            memset(&pos, 0, sizeof(pos));

            if (config.gps_source) {
                /* Convert GPS float coords to DDMM format */
                const gps_data_t *gps = gps_get_data();
                if (gps && gps->fix_quality > 0) {
                    float lat = gps->latitude;
                    float lon = gps->longitude;
                    pos.lat_ns = (lat >= 0) ? 'N' : 'S';
                    if (lat < 0) lat = -lat;
                    pos.lat_deg  = (uint8_t)lat;
                    float lat_min_f = (lat - pos.lat_deg) * 60.0f;
                    pos.lat_min  = (uint8_t)lat_min_f;
                    pos.lat_frac = (uint8_t)((lat_min_f - pos.lat_min) * 100.0f);

                    pos.lon_ew = (lon >= 0) ? 'E' : 'W';
                    if (lon < 0) lon = -lon;
                    pos.lon_deg  = (uint8_t)lon;
                    float lon_min_f = (lon - pos.lon_deg) * 60.0f;
                    pos.lon_min  = (uint8_t)lon_min_f;
                    pos.lon_frac = (uint8_t)((lon_min_f - pos.lon_min) * 100.0f);

                    pos.altitude = (uint16_t)(gps->altitude / 10.0f);
                    pos.valid    = 1;
                }
            } else {
                /* Use manual position from config */
                pos.lat_deg  = config.lat_deg;
                pos.lat_min  = config.lat_min;
                pos.lat_frac = config.lat_frac;
                pos.lat_ns   = config.lat_ns;
                pos.lon_deg  = config.lon_deg;
                pos.lon_min  = config.lon_min;
                pos.lon_frac = config.lon_frac;
                pos.lon_ew   = config.lon_ew;
                pos.altitude = config.altitude;
                pos.valid    = 1;
            }

            if (pos.valid)
                aprs_send_position(&pos);

            /* Reset countdown (send_position also calls beacon_reset) */
            beacon_countdown = aprs_beacon_interval_sec(config.beacon_interval);
        }
    }
}
