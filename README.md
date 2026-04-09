# tiny85-dht-sensor

## Goal

Replace the unreliable DHT11 sensors that come in Arduino kits with a custom-built drop-in replacement using an LM35 and ATtiny85.

The stock DHT11 is notorious for timing failures, missed reads, and garbage data on busy buses. This project builds a sensor that does the same job but with reliable, controlled timing generated in firmware on the ATtiny85.

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

## Status

- [x] Toolchain verified
- [x] Programmer verified (USBtinyISP)
- [x] Bringup hello world flashed
- [ ] LM35 ADC read
- [ ] DHT11-compatible output
