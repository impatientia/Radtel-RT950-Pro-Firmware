/*
 * hw_test.c - Hardware test suite for RT-950 Pro bring-up
 *
 * Each test is standalone and exercises one peripheral subsystem.
 * Output goes to USART1 (Bluetooth/debug port) at 115200 baud.
 *
 * Build with:  make test TEST=N
 *
 * Optional: output is also sent to the serial port for debugging.
 * 	Use minicom or other terminal program to monitor serial port output 
 * 	AFTER the flash process is complete
 *
 * 	Build with: make test TEST=N DEBUG=1
 *
 * Flash via:   tools/encrypt_btf.py to create flashable BTF
 */

#include "at32f403a.h"
#include "rt950_pinmap.h"
#include "tests/hw_test.h"
#include "drivers/gpio.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/bk4829.h"
#include "drivers/si4732.h"
#include "drivers/uart.h"
#include "drivers/adc.h"
#include "drivers/dac_audio.h"
#include "drivers/lcd.h"
#include "app/display.h"
#include "app/keypad.h"
#include "app/encoder.h"
#include "app/gps.h"

#ifdef DEBUG_UART
#include "debug_uart.h" 
#endif

			
extern void delay_ms(uint32_t ms);
extern uint32_t get_tick(void);

/* Debug output helpers ------------------------------------------------ */

static void hw_dbg_putc(char c){
#ifdef DEBUG_UART
    dbg_putc(c);
#endif
    uart_send_byte(USART1, c);
}

static void hw_dbg_puts(const char *s)
{
#ifdef DEBUG_UART
    dbg_puts(s);
#endif

    while (*s)
        uart_send_byte(USART1, (uint8_t)*s++);
}

static void hw_dbg_newline(void)
{
    uart_send_byte(USART1, '\r');
    uart_send_byte(USART1, '\n');

#ifdef DEBUG_UART
    dbg_newline();
#endif
}

static void hw_dbg_println(const char *s)
{
    hw_dbg_puts(s);
    hw_dbg_newline();
}

static void hw_dbg_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_send_byte(USART1, hex[v >> 4]);
    uart_send_byte(USART1, hex[v & 0xF]);

#ifdef DEBUG_UART
    dbg_hex8((unsigned char) v);
#endif
}

static void hw_dbg_hex16(uint16_t v)
{
    hw_dbg_hex8((uint8_t)(v >> 8));
    hw_dbg_hex8((uint8_t)(v & 0xFF));

#ifdef DEBUG_UART
    dbg_hex16((unsigned int) v);
#endif
}

static void hw_dbg_dec(uint32_t v)
{
    char buf[12];
    int i = 0;
    if (v == 0) {
	hw_dbg_putc('0');
	/*
        uart_send_byte(USART1, '0');
#ifdef DEBUG_UART
    	dbg_putc('0');
#endif
*/
        return;
    }
    while (v > 0) {
        buf[i++] = '0' + (char)(v % 10);
        v /= 10;
    }
    while (i > 0){
	    hw_dbg_putc((uint8_t)buf[--i]);
	    /*
        uart_send_byte(USART1, (uint8_t)buf[--i]);
#ifdef DEBUG_UART
    	dbg_putc(buf[i]);
#endif
*/
    }
}

/* ==========================================================================
 *  TEST 1: Blinky - Toggle LCD backlight at 1 Hz
 *  	    Alternate LED red and green
 *  Confirms: GPIO output, clock init, SysTick
 * ========================================================================== */

void test_blinky(void)
{
    uart_bt_init();
    hw_dbg_println("=== TEST 1: Blinky (LCD backlight PC6) ===");

    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);

    uint32_t count = 0;
    while (1) {
        gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
	if (count & 0x01)
        	gpio_set_pin(LED_RED_PORT, LED_RED_PIN);
	else
        	gpio_set_pin(LED_GREEN_PORT, LED_GREEN_PIN);

        hw_dbg_puts("ON  #");
        hw_dbg_dec(count);
	hw_dbg_puts((count&0x01) == 1 ? " Red":" Green");

        hw_dbg_newline();
        delay_ms(500);

        gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);
	if (count & 0x01)
		gpio_clear_pin(LED_RED_PORT, LED_RED_PIN);
	else
        	gpio_clear_pin(LED_GREEN_PORT, LED_GREEN_PIN);

        hw_dbg_puts("OFF #");
        hw_dbg_dec(count++);
        hw_dbg_newline();
        delay_ms(500);
    }
}

/* ==========================================================================
 *  TEST 2: UART Echo - Echo bytes on BT (USART1), monitor GPS (USART3)
 *  Confirms: USART1 TX/RX, USART3 RX
 * ========================================================================== */

void test_uart_echo(void)
{
    uart_bt_init();
    uart_gps_init();
    hw_dbg_println("=== TEST 2: UART Echo =!=");
    hw_dbg_println("BT(USART1): echo mode  |  GPS(USART3): passthrough to BT");

    while (1) {
        /* Echo BT input */
        if (uart_bt_rx_available()) {
            uint8_t c = uart_bt_rx_read();
	    hw_dbg_putc(c);
	    /*
            uart_send_byte(USART1, c);
#ifdef DEBUG_UART
	    dbg_putc((unsigned char) c);
#endif
*/
        }
        /* Forward GPS to BT */
        if (gps_rx_available()) {
            uint8_t c = gps_rx_read();
	    hw_dbg_putc(c);
	    /*
            uart_send_byte(USART1, c);
#ifdef DEBUG_UART
	    dbg_putc((unsigned char) c);
#endif
*/
        }
    }
}

/* ==========================================================================
 *  TEST 3: LCD Test Pattern - Fill screen with color bars
 *  Confirms: LCD 8080 bus, reset, init sequence, pixel write
 * ========================================================================== */

void test_lcd_pattern(void)
{
    uart_bt_init();
    hw_dbg_println("=== TEST 3: LCD Test Pattern ===");

    /* LCD backlight on */
    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);

    lcd_init();
    hw_dbg_println("LCD init done");

    /* RGB565 color bars: Red, Green, Blue, White, Black, Yellow, Magenta, Cyan */
    static const uint16_t colors[] = {
        0xF800, 0x07E0, 0x001F, 0xFFFF, 0x0000, 0xFFE0, 0xF81F, 0x07FF,
    };
    #define NUM_COLORS 8
    #define BAR_HEIGHT (LCD_HEIGHT / NUM_COLORS)

    uint32_t pass = 0;
    while (1) {
        hw_dbg_puts("Drawing pattern #");
        hw_dbg_dec(pass++);
        hw_dbg_newline();

        for (int bar = 0; bar < NUM_COLORS; bar++) {
            uint16_t y_start = (uint16_t)(bar * BAR_HEIGHT);
            lcd_fill_rect(0, y_start, LCD_WIDTH, BAR_HEIGHT, colors[bar]);
        }

        hw_dbg_println("Pattern drawn. Waiting 3s...");
        delay_ms(3000);
    }
    #undef NUM_COLORS
    #undef BAR_HEIGHT
}

/* ==========================================================================
 *  TEST 4: BK4829 Chip ID - Read register 0x00 from both transceivers
 *  Confirms: GPIOE bit-bang SPI, chip presence
 * ========================================================================== */

void test_bk4829_id(void)
{
    uart_bt_init();
    hw_dbg_println("=== TEST 4: BK4829 Chip ID ===");

    bk4829_init(BK4829_CHIP0);
    bk4829_init(BK4829_CHIP1);

    while (1) {
        uint16_t id0 = bk4829_read_reg(BK4829_CHIP0, 0x00);
        uint16_t id1 = bk4829_read_reg(BK4829_CHIP1, 0x00);

        hw_dbg_puts("Chip0 REG[0x00] = 0x");
        hw_dbg_hex16(id0);
        hw_dbg_puts("  Chip1 REG[0x00] = 0x");
        hw_dbg_hex16(id1);
        hw_dbg_newline();

        /* Also read RSSI (reg 0x67) */
        uint16_t rssi0 = bk4829_read_reg(BK4829_CHIP0, 0x67);
        uint16_t rssi1 = bk4829_read_reg(BK4829_CHIP1, 0x67);
        hw_dbg_puts("Chip0 RSSI=0x");
        hw_dbg_hex16(rssi0);
        hw_dbg_puts("  Chip1 RSSI=0x");
        hw_dbg_hex16(rssi1);
        hw_dbg_newline();
        hw_dbg_newline();

        delay_ms(1000);
    }
}

/* ==========================================================================
 *  TEST 5: SI4732 Chip Revision - Read GET_REV response
 *  Confirms: Bit-bang I2C on PB6/PB7, chip presence
 * ========================================================================== */

void test_si4732_rev(void)
{
    uart_bt_init();
    hw_dbg_println("=== TEST 5: SI4732 Revision ===");

    si4732_init();

    /* Power up in FM mode first */
    hw_dbg_println("Powering up SI4732 in FM mode...");
    si4732_power_up_fm();
    delay_ms(500);

    uint8_t part_number = 0;
    si4732_get_rev(&part_number);

    hw_dbg_puts("Part Number : 0x");
    hw_dbg_hex8(part_number);
    hw_dbg_newline();
    hw_dbg_puts(part_number == 0x20 ? "  -> SI4732 detected" :
             part_number == 0x00 ? "  -> FAIL (no response)" :
             "  -> Unexpected PN (check I2C)");
    hw_dbg_newline();

    /* Tune to a known FM station as additional test */
    hw_dbg_println("\nTuning to 100.0 MHz FM...");
    si4732_fm_tune(10000);
    delay_ms(500);

    struct si4732_tune_status status;
    si4732_fm_tune_status(&status);
    hw_dbg_puts("Freq: ");
    hw_dbg_dec(status.freq);
    hw_dbg_puts("0 kHz  RSSI: ");
    hw_dbg_dec(status.rssi);
    hw_dbg_puts("  SNR: ");
    hw_dbg_dec(status.snr);
    hw_dbg_newline();

    while (1) {
        delay_ms(2000);
        si4732_fm_tune_status(&status);
        hw_dbg_puts("RSSI=");
        hw_dbg_dec(status.rssi);
        hw_dbg_puts(" SNR=");
        hw_dbg_dec(status.snr);
        hw_dbg_newline();
    }
}

/* ==========================================================================
 *  TEST 6: SPI Flash JEDEC ID - Read 3-byte manufacturer/device ID
 *  Confirms: SPI2 HW, flash chip present
 * ========================================================================== */

void test_spi_flash_id(void)
{
    uart_bt_init();
    hw_dbg_println("=== TEST 6: SPI Flash JEDEC ID ===");

    spi2_init();

    while (1) {
        /* Use the built-in flash read ID function */
        uint32_t jedec = spi_flash_read_id();
        uint8_t mfr  = (uint8_t)(jedec >> 16);
        uint8_t type = (uint8_t)(jedec >> 8);
        uint8_t cap  = (uint8_t)(jedec);

        hw_dbg_puts("JEDEC ID: Mfr=0x");
        hw_dbg_hex8(mfr);
        hw_dbg_puts(" Type=0x");
        hw_dbg_hex8(type);
        hw_dbg_puts(" Cap=0x");
        hw_dbg_hex8(cap);

        /* Decode common manufacturers */
        hw_dbg_puts("  -> ");
        if (mfr == 0xEF) hw_dbg_puts("Winbond");
        else if (mfr == 0xC8) hw_dbg_puts("GigaDevice");
        else if (mfr == 0x20) hw_dbg_puts("Micron/Numonyx");
        else if (mfr == 0x1F) hw_dbg_puts("Adesto/Atmel");
        else if (mfr == 0x01) hw_dbg_puts("Spansion/Cypress");
        else if (mfr == 0xBF) hw_dbg_puts("SST");
        else if (mfr == 0x9D) hw_dbg_puts("ISSI");
        else if (mfr == 0x0B) hw_dbg_puts("XTX");
        else hw_dbg_puts("Unknown");

        hw_dbg_puts(" ");
        uint32_t size_kb = (1UL << cap) / 1024;
        hw_dbg_dec(size_kb);
        hw_dbg_puts("KB");
        hw_dbg_newline();

        if (mfr == 0xFF || mfr == 0x00) {
            hw_dbg_println("  !! No flash detected (check SPI2 wiring)");
        }

        delay_ms(2000);
    }
}

/* ==========================================================================
 *  TEST 7: ADC Monitor - Continuously read battery and audio level
 *  Confirms: ADC2, PA0 (battery), PA1 (audio)
 * ========================================================================== */

void test_adc_monitor(void)
{
    uart_bt_init();
    hw_dbg_println("=== TEST 7: ADC Monitor ===");

    adc_init();

    while (1) {
        uint8_t batt = adc_read_battery();
        uint8_t audio = adc_read_audio_level();

        hw_dbg_puts("Battery(PA0)=");
        hw_dbg_dec(batt);
        hw_dbg_puts("/255  Audio(PA1)=");
        hw_dbg_dec(audio);
        hw_dbg_puts("/255");

        /* Rough voltage estimate: assuming 1:2 divider, 3.3V ref */
        uint32_t mv = (uint32_t)batt * 3300 * 2 / 4095;
        hw_dbg_puts("  ~");
        hw_dbg_dec(mv);
        hw_dbg_puts("mV");
        hw_dbg_newline();

        delay_ms(500);
    }
}

/* ==========================================================================
 *  TEST 8: DAC Tone - Output 1 kHz test tone on PA4
 *  Confirms: DAC1, TIM6, DMA2 CH3
 * ========================================================================== */

void test_dac_tone(void)
{
    uart_bt_init();
    dma_init();
    hw_dbg_println("=== TEST 8: DAC 1kHz Tone on PA4 ===");

    dac_audio_init();

    /* 1000.0 Hz = 10000 in freqx10 format */
    hw_dbg_println("Playing 1000 Hz tone...");
    dac_audio_play_tone(10000);

    uint32_t sec = 0;
    while (1) {
        delay_ms(5000);
        sec += 5;
        hw_dbg_puts("Playing for ");
        hw_dbg_dec(sec);
        hw_dbg_puts("s  DMA active=");
        hw_dbg_dec((uint32_t)dac_audio_is_playing());
        hw_dbg_newline();

        /* Cycle through tones every 10 seconds */
        if ((sec % 20) == 10) {
            hw_dbg_println("Switching to 67.0 Hz (CTCSS)...");
            dac_audio_play_tone(670);
        } else if ((sec % 20) == 0) {
            hw_dbg_println("Switching to 1000 Hz...");
            dac_audio_play_tone(10000);
        }
    }
}

/* ==========================================================================
 *  TEST 9: Keypad + Encoder - Scan and report events
 *  Confirms: Matrix scan (PC0-3/PD4-7), encoder (PB4/PB5)
 * ========================================================================== */

void test_keypad_encoder(void)
{
    char cPower = !gpio_read_pin (PWR_SWITCH_PORT, PWR_SWITCH_PIN); //force initialization 
    uart_bt_init();
    hw_dbg_println("=== TEST 9: Keypad + Encoder ===");
    hw_dbg_println("Press keys or turn encoder. Output on USART1.");

    keypad_init();
    encoder_init();

    //works with pattern 4
    static const char * const key_names[] = {
        "OK", "ABC", "RET", "V/M",	//0x10-0x13 //moved side 1 and 4 out
        "3", "6", "9", "#", 	//4-7
        "2", "5", "8", "0",	//8-0x0B
        "UP", "DN", "<L", "R>",	//0x0C-0x0F
        "1", "4", "7", "*", 	//0-3
	"SIDE1", "SIDE4"		//0x14-0x15
    };

    while (1) {
        /* Keypad events */
        key_event_t evt;
	uint8_t bKeyEvent = 0;

	if (ptt_get_event(&evt))
		bKeyEvent=1;
	else 
		bKeyEvent = keypad_get_event(&evt);
		
        if (bKeyEvent) {
            hw_dbg_puts("KEY: ");
            hw_dbg_puts("0x");
            hw_dbg_hex8(evt.key);
            if (evt.key < 22){
            	hw_dbg_puts(" : ");
                hw_dbg_puts(key_names[evt.key]);
	    } else 
		    switch (evt.key){
			    case KEY_PTT: 
            				hw_dbg_puts(" : PTT");
					break;
			    case KEY_PTT2: 
            				hw_dbg_puts(" : PTT2");
					break;
		    }
	    /*
            else {
                hw_dbg_puts("0x");
                hw_dbg_hex8(evt.key);
            }
	    */
            switch (evt.type) {
            case KEY_EVT_PRESS:   hw_dbg_puts(" PRESS");   break;
            case KEY_EVT_REPEAT:  hw_dbg_puts(" REPEAT");  break;
            case KEY_EVT_RELEASE: hw_dbg_puts(" RELEASE"); break;
            default:              hw_dbg_puts(" ???");      break;
            }
            hw_dbg_newline();
        }

        /* Encoder */
        int8_t enc = encoder_poll();
        if (enc > 0) {
            hw_dbg_println("ENC: CW  (+1)");
        } else if (enc < 0) {
            hw_dbg_println("ENC: CCW (-1)");
        }

	if (uart_cps_rx_available()){
		dbg_putc(uart_cps_rx_read());
		dbg_putc('.');
	}


	//working
	/*
	 //moved to scan as a key
	if (gpio_read_pin (PTT_PORT,PTT_PIN )==0) //side key 1
	       hw_dbg_putc('T'); 	 //Ptt (1)
	if (gpio_read_pin (PTT2_PORT,PTT2_PIN )==0) //side key 2
	       hw_dbg_putc('t'); 	 //ptt (2)
	*/
					 
	//handled in keypad matrix
	/*
	if (gpio_read_pin (SIDE_KEY1_PORT,SIDE_KEY1_PIN )==0) //side key 3
	       dbg_putc('U'); //TOP_PROG_PORT 	
	if (gpio_read_pin (SIDE_KEY4_PORT,SIDE_KEY4_PIN )==0) //side key 3
	       dbg_putc('u'); //BOT_PROG_PORT 	
        */
        

	/* //never toggles
	if (gpio_read_pin (POWER_OFF_PORT,POWER_OFF_PIN )==0) //???
	       dbg_putc('+'); //BOT_PROG_PORT 	
	if (gpio_read_pin (GPIO_PC9_PORT,GPIO_PC9_PIN )==1) //always high???
	       dbg_putc('+'); //
	if (gpio_read_pin (SIDEPORT_RX_PORT,SIDEPORT_RX_PIN )==0) //???
	       dbg_putc('+'); //
	if (gpio_read_pin (SINGLE_IN_PORT,SINGLE_IN_PIN )==1) //???
	       dbg_putc('+'); //
        */

	if (gpio_read_pin (EXT_PTT_PORT, EXT_PTT_PIN)==0){ //external mic key
        	gpio_set_pin(LED_RED_PORT, LED_RED_PIN);
		gpio_clear_pin(LED_GREEN_PORT, LED_GREEN_PIN);
	        hw_dbg_putc('M'); 
		}
	else {
        	gpio_clear_pin(LED_RED_PORT, LED_RED_PIN);
		
		// power switch on vol knob
		// nested so that red can dominate over green
		if (gpio_read_pin (PWR_SWITCH_PORT, PWR_SWITCH_PIN)!=cPower) {
			cPower = gpio_read_pin (PWR_SWITCH_PORT, PWR_SWITCH_PIN);
			if (cPower ==0) {
				gpio_set_pin(LED_GREEN_PORT, LED_GREEN_PIN);
				hw_dbg_puts("Power ON\n");
			}
				else
					hw_dbg_puts("Power OFF\n");
		}

	}

	//not working
	if (gpio_read_pin (POWER_OFF_PORT, POWER_OFF_PIN)==0)
	       hw_dbg_putc('*'); 	
	//test
	if (gpio_read_pin (GPIO_PB9_PWREN_PORT,GPIO_PB9_PWREN_PIN)==1)
	       hw_dbg_putc('.'); 	
	/*
	if (gpio_read_pin (, )==0)
	       hw_dbg_putc('.'); 	
	       */

        delay_ms(5);  /* ~200 Hz poll rate */
    }
}

/* ==========================================================================
 *  TEST 10: GPS Display - Show NMEA sentences and parsed data
 *  Confirms: USART3 RX from GPS module, NMEA parsing
 * ========================================================================== */

void test_gps_display(void)
{
    uart_bt_init();
    uart_gps_init();
    hw_dbg_println("=== TEST 10: GPS NMEA Display ===");
    hw_dbg_println("Raw NMEA forwarded to USART1. Parsed data every 2s.");

    gps_init();
    uint32_t last_print = get_tick();

    while (1) {
        /* Forward raw NMEA bytes to debug port */
        if (gps_rx_available()) {
            uint8_t c = gps_rx_read();
	    hw_dbg_putc(c);
            //uart_send_byte(USART1, c);
        }

        /* Periodic parsed data dump */
        gps_process();
        uint32_t now = get_tick();
        if ((now - last_print) >= 2000) {
            last_print = now;
            const gps_data_t *gps = gps_get_data();
            hw_dbg_newline();
            hw_dbg_puts("[GPS] Fix=");
            hw_dbg_dec(gps->fix_quality);
            hw_dbg_puts(" Sats=");
            hw_dbg_dec(gps->num_satellites);
            hw_dbg_puts(" Time=");
            hw_dbg_dec(gps->hour);
            hw_dbg_puts(":");
            if (gps->minute < 10) hw_dbg_puts("0");
            hw_dbg_dec(gps->minute);
            hw_dbg_puts(":");
            if (gps->second < 10) hw_dbg_puts("0");
            hw_dbg_dec(gps->second);
            hw_dbg_newline();
        }
    }
}

/* ==========================================================================
 *  TEST 11: Full System Diagnostic - Init all peripherals, report status
 *  Confirms: Complete system bring-up sequence
 * ========================================================================== */

void test_full_diagnostic(void)
{
    uart_bt_init();
    hw_dbg_println("========================================");
    hw_dbg_println("  RT-950 Pro Custom Firmware Diagnostic");
    hw_dbg_println("  Build: " __DATE__ " " __TIME__);
    hw_dbg_println("========================================");
    hw_dbg_newline();

    /* Clock info ---------------------------------------------------- */
    hw_dbg_puts("SYSCLK: 120 MHz (HSE 8 MHz x 15)");
    hw_dbg_newline();
    hw_dbg_puts("SysTick: ");
    hw_dbg_dec(get_tick());
    hw_dbg_puts(" ms since boot");
    hw_dbg_newline();
    hw_dbg_newline();

    /* DMA ----------------------------------------------------------- */
    hw_dbg_puts("[DMA ] Init... ");
    dma_init();
    hw_dbg_println("OK");

    /* SPI Flash ----------------------------------------------------- */
    hw_dbg_puts("[FLASH] SPI2 init... ");
    spi2_init();
    uint32_t jedec = spi_flash_read_id();
    uint8_t flash_mfr  = (uint8_t)(jedec >> 16);
    uint8_t flash_type = (uint8_t)(jedec >> 8);
    uint8_t flash_cap  = (uint8_t)(jedec);
    hw_dbg_puts("JEDEC=");
    hw_dbg_hex8(flash_mfr);
    hw_dbg_hex8(flash_type);
    hw_dbg_hex8(flash_cap);
    if (flash_mfr != 0xFF && flash_mfr != 0x00) {
        hw_dbg_puts(" OK (");
        hw_dbg_dec((1UL << flash_cap) / 1024);
        hw_dbg_puts("KB)");
    } else {
        hw_dbg_puts(" FAIL");
    }
    hw_dbg_newline();

    /* BK4829 -------------------------------------------------------- */
    hw_dbg_puts("[RF  ] BK4829 #0 init... ");
    bk4829_init(BK4829_CHIP0);
    uint16_t rf_id0 = bk4829_read_reg(BK4829_CHIP0, 0x00);
    hw_dbg_puts("REG0=0x");
    hw_dbg_hex16(rf_id0);
    hw_dbg_puts(rf_id0 != 0xFFFF && rf_id0 != 0x0000 ? " OK" : " FAIL");
    hw_dbg_newline();

    hw_dbg_puts("[RF  ] BK4829 #1 init... ");
    bk4829_init(BK4829_CHIP1);
    uint16_t rf_id1 = bk4829_read_reg(BK4829_CHIP1, 0x00);
    hw_dbg_puts("REG0=0x");
    hw_dbg_hex16(rf_id1);
    hw_dbg_puts(rf_id1 != 0xFFFF && rf_id1 != 0x0000 ? " OK" : " FAIL");
    hw_dbg_newline();

    /* SI4732 -------------------------------------------------------- */
    hw_dbg_puts("[SI47] Init + power up FM... ");
    si4732_init();
    si4732_power_up_fm();
    delay_ms(500);
    uint8_t si_pn = 0;
    si4732_get_rev(&si_pn);
    hw_dbg_puts("PN=0x");
    hw_dbg_hex8(si_pn);
    hw_dbg_puts(si_pn == 0x20 ? " OK (SI4732)" :
             si_pn == 0x00 ? " FAIL (no response)" : " ? (unexpected PN)");
    hw_dbg_newline();

    /* ADC ----------------------------------------------------------- */
    hw_dbg_puts("[ADC ] Init + read... ");
    adc_init();
    uint8_t batt = adc_read_battery();
    uint8_t audio = adc_read_audio_level();
    hw_dbg_puts("Batt=");
    hw_dbg_dec(batt);
    hw_dbg_puts(" Audio=");
    hw_dbg_dec(audio);
    hw_dbg_puts(batt > 0 ? " OK" : " FAIL");
    hw_dbg_newline();

    /* GPS UART ------------------------------------------------------ */
    hw_dbg_puts("[GPS ] USART3 init... ");
    uart_gps_init();
    hw_dbg_puts("listening...");
    hw_dbg_newline();

    /* Wait up to 3 seconds for GPS data */
    uint32_t start = get_tick();
    int gps_ok = 0;
    while ((get_tick() - start) < 3000) {
        if (gps_rx_available()) {
            gps_ok = 1;
            break;
        }
    }
    hw_dbg_puts("[GPS ] Data: ");
    hw_dbg_println(gps_ok ? "RECEIVED" : "NO DATA (may need longer warm-up)");

    /* LCD ------------------------------------------------------------ */
    hw_dbg_puts("[LCD ] Init... ");
    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    lcd_init();
    hw_dbg_println("OK (visual check needed)");

    /* Keypad -------------------------------------------------------- */
    hw_dbg_puts("[KBD ] Init... ");
    keypad_init();
    hw_dbg_println("OK");

    /* Encoder ------------------------------------------------------- */
    hw_dbg_puts("[ENC ] Init... ");
    encoder_init();
    hw_dbg_println("OK");

    /* DAC ------------------------------------------------------------ */
    hw_dbg_puts("[DAC ] Init... ");
    dac_audio_init();
    hw_dbg_println("OK");

    /* Summary ------------------------------------------------------- */
    hw_dbg_newline();
    hw_dbg_println("========================================");
    hw_dbg_println("  Diagnostic complete.");
    hw_dbg_println("  Connect scope to PA4 for DAC test.");
    hw_dbg_println("  Check LCD for test pattern.");
    hw_dbg_println("  Turn encoder / press keys for input test.");
    hw_dbg_println("========================================");
    hw_dbg_newline();

    /* Fill LCD with a gradient test pattern */
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x001F); /* blue fill */

    /* Enter interactive mode: show keypad/encoder on debug + LCD */
    hw_dbg_println("Entering interactive mode (keys + encoder)...");
    while (1) {
        key_event_t evt;
        if (keypad_get_event(&evt) && evt.type == KEY_EVT_PRESS) {
            hw_dbg_puts("KEY ");
            hw_dbg_dec(evt.key);
            hw_dbg_newline();
        }
        int8_t enc = encoder_poll();
        if (enc != 0) {
		/*
            hw_dbg_puts("ENC ");
            hw_dbg_dec(enc > 0 ? 1 : (uint32_t)-1);
            hw_dbg_newline();
	    */
	    hw_dbg_println(enc > 0 ? "ENC: CW  (+1)":"ENC: CCW (-1)");
        }
        delay_ms(5);
    }
}
