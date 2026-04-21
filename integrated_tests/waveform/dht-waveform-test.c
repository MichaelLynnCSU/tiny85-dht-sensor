/*
 * dht-waveform-test.c
 * Outputs a static AM2302/DHT22-compatible frame on PB0
 * Data: humidity=0.0, temp=25.0, encoded as DHT22 16-bit format
 *
 * Clock: ATtiny85 @ 8MHz internal oscillator (lfuse=0xe2, CKDIV8 disabled)
 * At 8MHz, _delay_us() has 8 cycles/µs resolution.
 * Branch and function call overhead (~1µs) is within DHT11 ±10µs tolerance.
 *
 * DHT22 bit timing:
 *   separator low : 50µs
 *   0 bit high    : 26µs
 *   1 bit high    : 70µs
 *   response      : 80µs low, 80µs high
 *
 * DHT22 data format:
 *   byte[0] = humidity high byte
 *   byte[1] = humidity low byte   (humidity = (b0<<8|b1) / 10.0)
 *   byte[2] = temperature high byte
 *   byte[3] = temperature low byte (temp = (b2<<8|b3) / 10.0)
 *   byte[4] = checksum = b0+b1+b2+b3
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define DHT_PIN PB0

static void dht_send_frame(void)
{
    /* 25.0°C = 250 = 0x00FA, humidity 0.0 = 0x0000 */
    uint8_t data[5] = {0x00, 0x00, 0x00, 0xFA, 0xFA};
    uint8_t bits[40];

    /* precompute all 40 bits MSB first */
    for (uint8_t i = 0; i < 5; i++)
        for (uint8_t j = 0; j < 8; j++)
            bits[i * 8 + j] = (data[i] >> (7 - j)) & 1;

    /* simulate host start: pull low 20ms then release */
    DDRB  |=  (1 << DHT_PIN);
    PORTB &= ~(1 << DHT_PIN);
    _delay_ms(20);
    PORTB |=  (1 << DHT_PIN);
    _delay_us(40);

    /* response signal: 80µs low, 80µs high */
    PORTB &= ~(1 << DHT_PIN);
    _delay_us(80);
    PORTB |=  (1 << DHT_PIN);
    _delay_us(80);

    /* send all 40 bits */
    for (uint8_t i = 0; i < 40; i++)
    {
        PORTB &= ~(1 << DHT_PIN);   /* 50µs low separator */
        _delay_us(50);
        PORTB |=  (1 << DHT_PIN);
        if (bits[i])
            _delay_us(70);          /* 1 bit = 70µs high */
        else
            _delay_us(26);          /* 0 bit = 26µs high */
    }

    /* end pulse */
    PORTB &= ~(1 << DHT_PIN);
    _delay_us(50);
    PORTB |=  (1 << DHT_PIN);
}

int main(void)
{
    DDRB  |=  (1 << DHT_PIN);
    PORTB |=  (1 << DHT_PIN);  /* idle high */
    _delay_ms(100);

    while (1)
    {
        dht_send_frame();
        _delay_ms(2000);
    }

    return 0;
}
