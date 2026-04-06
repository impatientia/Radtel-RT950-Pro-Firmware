# Changelog

All notable changes to the RT-950 Pro custom firmware are documented here.

## [0.0.3] - 2026-04-06

First working speaker output, audio architecture hardware-verified.

### Audio
- DAC tones confirmed through speaker (V12 diagnostic: 6/6 tests pass)
- PE4 = amp power rail, PB8 = amp enable, PE1 = mute, PC12 = audio mux
- BK4829 R47/R48 must be zeroed before DAC playback (AF bus loading)
- Amp power-cycle sequence required for reliable cold-start audio
- PC12 polarity discrepancy: OEM SETs HIGH for beep, custom FW needs LOW

### Bootloader
- Decryption algorithm verified correct (encrypt_btf.py confirmed)
- UART update protocol fully decoded; bootloader assembly annotated

### Bug Fixes
- PE4 was cleared "for safety" - actually cuts amplifier power
- gpio.h comments had swapped OEM addresses/offsets (code was correct)
- HardFault workaround: debug builds skip SPI flash in vfo_load_state
- Assembly fix: 0x080038EA mislabeled PC10, corrected to PC12

## [0.0.2] - 2026-04-04

Hardware validation release. All keypad/encoder/PTT inputs confirmed working
on real hardware. Partial menu rendering tested. Audio playback under active
debugging.

### Hardware Confirmed
- All keypad buttons mapped and tested (4x5 matrix, PC0-3 cols, PD0-7 rows)
- Rotary encoder (PB4/PB5) confirmed working
- PTT1 (PE3), PTT2 (PE2), EXT_PTT (PE5), Side Key (PA12) all functional
- Power button (PE0) input confirmed
- DAC1 (PA4) outputting valid sine waves via TIM6+DMA2 (registers verified)
- SPI flash detected: Macronix MX25L1606E (JEDEC 0x00C22016)
- Wear-leveling probe reads calibration data from flash

### Bug Fixes
- Fixed HardFault during boot: WL probe buffer overflow (4 → 160 bytes)
- Fixed HardFault handler: naked ASM trampoline preserves stacked registers
- Fixed hex debug output corruption: arithmetic conversion immune to BTF .rodata issues
- Fixed PTT1/side buttons: PE0/2/3/5 and PA12 configured as inputs with pull-ups
- Fixed DAC DMA underrun: deferred EN1+DMAEN1 until DMA armed, clear DMAUDR1

### Display
- Partial menu rendering test (renders but not yet functional, visual glitches present)
- Boot splash screen with status text

### Repository
- V0.27 annotated disassembly moved to `assembly/` directory

### Known Limitations
- Audio amplifier not yet producing sound (DAC registers verified correct, analog path debugging in progress)
- Battery ADC reads zero after first sample
- SI4732 SSB patch binary not extracted from OEM flash
- Bluetooth audio streaming not implemented (AT commands only)

## [0.0.1] - 2026-04-03

Initial public release. Custom bare-metal firmware boots and runs on the
Radtel RT-950 Pro with LCD output, LED control, and OEM bootloader upload.

### Hardware Confirmed
- LCD ST7789V init and pixel writes via 8080 parallel bus (PD0-PD15)
- Embedded 8x8 bitmap font with 1x and 2x text rendering
- Boot screen with status display on 240x320 IPS panel
- Red LED (PC13) and Green LED (PC14) GPIO control
- LCD backlight primary (PC6) and secondary (PB3)
- Power latch (PB9) and band relay (PC4)
- Firmware upload via OEM bootloader (custom BTF encryption)
- Debug UART output for hardware bring-up

### Firmware
- Complete bare-metal C firmware for AT32F403A (Cortex-M4F @ 120 MHz)
- 94 source files (~19,600 lines of C) across include/ and src/
- Linker script with 12 KB bootloader reservation (ORIGIN=0x08003000)
- SystemInit: 120 MHz PLL (8 MHz HEXT x15), SysTick, IWDG watchdog

### Drivers (Roughly Code-Complete, Mostly Untested)
- BK4829 dual RF transceiver (bit-bang SPI, PE8/PE10/PE11/PE15)
- ST7789V LCD (8080 parallel bus, software bit-bang)
- SI4732 AM/FM/SSB/WB receiver (bit-bang I2C, PB6/PB7)
- W25Q16 SPI flash with wear-leveling (hardware SPI2, PB12-PB15)
- UART: Bluetooth (USART1 115200), GPS (USART3 9600), CPS (UART4 115200)
- ADC2 (PA0 VOX, PA1 battery), DAC1+TIM6+DMA tone generation
- GPIO with verified pin map (56 pins mapped, binary-verified + hardware-probed)

### Application (Roughly Code-Complete, Untested)
- Dual VFO (A/B/C), 990 memory channels with zone browsing
- APRS via BK4829 hardware AFSK (MIC-E encoding)
- DTMF encode/decode with contacts
- GPS NMEA parsing, FM/AM broadcast radio
- NOAA weather radio (7 channels via SI4732 WB)
- Channel scanner, spectrum analyzer
- Cross-band repeat (A->B, B->A, duplex), VOX
- Hierarchical 12-category menu (43 items)
- CPS wireless programming via Bluetooth

### Tools
- `firmware_upload.py` - Upload .BTF firmware with auto-restart and flood probe
- `encrypt_btf.py` - BTF encryption/decryption (keys for V0.15/V0.18/V0.21/V0.27)
- `cps_flash.py` - Read/write radio configuration via CPS serial protocol
- 11 hardware test modes (backlight, UART, LCD, BK4829, SI4732, flash, ADC, DAC, keypad, GPS, full diagnostic)

### Reverse Engineering
- Full V0.27 OEM binary disassembly (216K lines, radare2)
- OEM bootloader disassembly (4.7K lines, 95 functions)
- 80+ OEM function addresses mapped to C source equivalents
- GPIO pin map cross-referenced: binary analysis + hardware probing
- BTF encryption algorithm fully reversed (XOR cipher with bit-rotation key expansion)
- CPS, Bluetooth, bootloader, and KDH cloud protocols documented

### Known Limitations
- Most peripherals untested on hardware (SPI flash, BK4829, SI4732, GPS, keypad, encoder)
- SI4732 SSB patch binary not extracted from OEM flash
- SI4732 XOSCEN configuration needs PCB crystal verification
- Bluetooth audio streaming not implemented (AT commands only)
- Voice prompt audio samples not implemented (tone patterns only)
