# tiny85-dht-sensor

## Goal

Replace the unreliable DHT11 sensors from Arduino kits with a custom drop-in replacement built from an LM35 and ATtiny85.

## Why

The DHT11 timing problem is real and well documented in production. On this project it showed up when the STM32F411 was running the dashboard, BLE stack, and DHT11 reads simultaneously — interrupt masking under shared load caused enough jitter to corrupt the DHT11 bit timing, returning garbage data. The fix required migrating the motor and sensor reads to a dedicated ESP32-C3 just to get clean timing isolation.

The root cause was the DHT11 protocol itself — it requires microsecond-accurate pulse timing with no tolerance for latency. Any shared interrupt load breaks it. The chip is fragile by design.

This project fixes that at the source. Instead of working around the DHT11's timing requirements, the ATtiny85 generates the protocol itself in firmware with full control over pulse width and timing. The host MCU sees a clean, reliable signal regardless of its own interrupt load.

## How It Works

- LM35 reads temperature as an analog voltage
- ATtiny85 reads that voltage via its built-in ADC
- ATtiny85 encodes the result and outputs a DHT11-compatible pulse signal
- Any existing DHT11 library reads it without modification

## Hardware

- ATtiny85 (8-pin DIP)
- LM35 analog temperature sensor
- USBtinyISP programmer
- avr-gcc / avrdude toolchain
- F_CPU: 1000000UL (internal 1MHz oscillator, default fuses)

## Status

- [x] Toolchain verified
- [x] Programmer verified (USBtinyISP, signature 0x1e930b confirmed)
- [x] Bringup hello world flashed and verified on hardware
- [ ] LM35 ADC read
- [ ] Temperature scaling (10mV = 1°C)
- [ ] DHT11-compatible pulse output
