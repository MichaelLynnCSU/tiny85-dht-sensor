# tiny85-dht-sensor
## Goal
Replace the unreliable DHT11 sensors from Arduino kits with a custom drop-in replacement built from an ATtiny85.
## Why
The DHT11 timing problem is real and well documented in production. On this project it showed up when the STM32F103 was running multiple DHT11 sensor reads alongside FRAM logging and UART output simultaneously — interrupt masking under shared load caused enough jitter to corrupt the DHT11 bit timing, returning garbage data. The fix required migrating the motor to a dedicated ESP32-C3 just to get clean timing isolation.
The root cause was the DHT11 protocol itself — it requires microsecond-accurate pulse timing with no tolerance for latency. Any shared interrupt load breaks it. The chip is fragile by design.
This project fixes that at the source. Instead of working around the DHT11's timing requirements, the ATtiny85 generates the protocol itself in firmware with full control over pulse width and timing. The host MCU sees a clean, reliable signal regardless of its own interrupt load.
## How It Works
- ATtiny85 reads internal temperature sensor via ADC
- ATtiny85 encodes the result and outputs a DHT11-compatible pulse signal in response to host start signal
- Any existing DHT11 library reads it without modification
- UART telemetry on PB4 for debugging (9600 baud, 8N1)
## Hardware
- ATtiny85 (8-pin DIP)
- USBtinyISP programmer
- avr-gcc / avrdude toolchain
- F_CPU: 8000000UL (internal 8MHz oscillator, lfuse=0xe2, CKDIV8 disabled)
- 4.7kΩ pullup resistor on DHT data line to 3.3V
## Wiring
- DHT data → PB0 (pin 5)
- UART TX  → PB4 (pin 3) — 9600 baud 8N1
- Host     → STM32F103 Blue Pill PA7
## Clock
- Internal RC @ 8MHz (lfuse=0xe2)
- Verified: _delay_us(24) measured 23.25µs on Saleae at 24MHz
- Actual clock ~8.26MHz — within ATtiny85 ±10% RC spec
## UART
- 9600 baud, 8N1, bit-banged on PB4
- BIT_DELAY = 1000000 / 9600 = 104µs per bit
- String literals stored in flash via PSTR() to avoid 512-byte SRAM overflow
## DHT11 Protocol
- Host pulls data line low ≥18ms then releases
- ATtiny85 responds within 30µs with 80µs low + 80µs high
- 40 bits: humidity(0,0) + temp integer + temp decimal(0) + checksum
- Bit encoding: 50µs low + 26µs high = 0, 50µs low + 70µs high = 1
## Known Bugs Fixed
1. Non-standard 2000 baud replaced with 9600 baud
2. SRAM overflow from string literals fixed with PSTR() flash storage
3. Wrong ADC reference (2.56V) corrected to 1.1V (REFS1=1 REFS0=0)
4. Wrong calibration anchor corrected from datasheet typical to measured value
5. ADC read in host trigger path caused timeout — fixed with cached ADC
6. DHT22 encoding sent to DHT11 host — fixed to DHT11 integer format
7. UART logging inside frame function caused timeout — moved to after transmission
## Status
- [x] Toolchain verified
- [x] Programmer verified (USBtinyISP, signature 0x1e930b confirmed)
- [x] Bringup hello world flashed and verified on hardware
- [x] Clock verified at 8MHz (23.25µs measured for 24µs pulse)
- [x] UART working at 9600 baud on PB4
- [x] DHT11-compatible waveform verified on Saleae (26µs/70µs bit widths)
- [x] Host-triggered DHT11 response working end-to-end
- [x] Blue Pill S4 task reading ATtiny85 on PA7 — confirmed live readings
- [x] All 7 firmware bugs found, fixed, and documented
- [ ] Replace internal temp sensor with LM35 on PB3
- [ ] Temperature scaling (10mV = 1°C)
