/******************************************************************************
 * \file    tiny85-dht-sensor.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   ATtiny85 internal temperature sensor + bit-banged UART telemetry.
 *
 * \details Reads internal temperature sensor via ADC and transmits raw ADC
 *          and calibrated temperature over a software UART on PB4.
 *
 *          Hardware:
 *          - MCU:      ATtiny85 @ 1 MHz internal oscillator
 *          - UART TX:  PB4 (bit-banged, 8N1)
 *          - ADC:      Internal temperature sensor (MUX = 0x0F)
 *          - Vref:     Internal 1.1V reference
 *
 *          UART configuration:
 *          - Baud: ~2000 baud (BIT_DELAY = 500 µs)
 *          - Format: 8 data bits, no parity, 1 stop bit
 *
 *          Temperature model:
 *          - Raw ADC reading is used directly from internal sensor
 *          - Linear calibration applied using:
 *              TEMP_CAL_ADC = 282 @ TEMP_CAL_C = 22°C
 *          - Result: temp ≈ (adc - 282) + 22
 *
 * \note    ADC temp sensor behavior:
 *          The ATtiny85 internal temperature sensor is not absolute accurate.
 *          It is strongly affected by:
 *          - Vcc variation
 *          - chip-to-chip offset
 *          - factory calibration differences
 *
 *          This implementation uses single-point calibration.
 *          For production accuracy, a 2-point calibration (cold + hot) is required.
 *
 * \note    ADC configuration correctness (critical fix):
 *          - MUX = 0x0F selects internal temperature sensor
 *          - REFS1:0 = 1 selects internal 1.1V reference
 *          - First conversion after MUX switch is discarded (required)
 *
 * \note    Known issue (resolved from debugging session):
 *          Initial builds produced constant or invalid readings due to:
 *          - incorrect ADC reference configuration
 *          - missing dummy ADC conversion after MUX change
 *
 * \note    UART implementation:
 *          - Fully bit-banged (no hardware USI used)
 *          - Timing depends on F_CPU stability (1 MHz internal RC)
 *          - Clock drift may affect baud accuracy over temperature
 *
 * \warning Clock stability:
 *          Internal RC oscillator can drift significantly with temperature,
 *          causing UART framing errors at higher baud rates.
 *
 ******************************************************************************/

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define UART_PIN       PB4
#define BIT_DELAY      500

#define TEMP_CAL_ADC   282
#define TEMP_CAL_C     22

/* -------------------------------------------------------------------------- */

static void delay_bit(void)
{
    _delay_us(BIT_DELAY);
}

/* -------------------------------------------------------------------------- */

static void uart_tx_char(char c)
{
    PORTB &= ~(1 << UART_PIN);   /* start bit */
    delay_bit();

    for (uint8_t i = 0; i < 8; i++)
    {
        if (c & 1)
            PORTB |= (1 << UART_PIN);
        else
            PORTB &= ~(1 << UART_PIN);

        c >>= 1;
        delay_bit();
    }

    PORTB |= (1 << UART_PIN);    /* stop bit */
    delay_bit();
}

/* -------------------------------------------------------------------------- */

static void uart_print(const char *s)
{
    while (*s)
        uart_tx_char(*s++);
}

/* -------------------------------------------------------------------------- */

static void uart_print_int(int val)
{
    char buf[8];
    int i = 0;

    if (val < 0)
    {
        uart_tx_char('-');
        val = -val;
    }

    do
    {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    } while (val);

    while (i--)
        uart_tx_char(buf[i]);
}

/* -------------------------------------------------------------------------- */

static uint16_t adc_read_temp(void)
{
    /* Enable ADC (prescaler /128 for stability at 1 MHz) */
    ADCSRA =
        (1 << ADEN)  |
        (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    /* Internal 1.1V reference + temperature sensor (MUX = 0x0F) */
    ADMUX =
        (1 << REFS1) | (1 << REFS0) |
        0x0F;

    _delay_ms(20);

    /* discard first conversion after mux switch */
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));

    /* real conversion */
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));

    return ADC;
}

/* -------------------------------------------------------------------------- */

static int16_t temp_from_adc(uint16_t adc)
{
    /* single-point calibration model */
    return TEMP_CAL_C + (adc - TEMP_CAL_ADC);
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    DDRB  |= (1 << UART_PIN);
    PORTB |= (1 << UART_PIN);

    _delay_ms(100);

    while (1)
    {
        uint16_t adc = adc_read_temp();
        int16_t temp = temp_from_adc(adc);

        uart_print("RAW: ");
        uart_print_int(adc);
        uart_print(" | TEMP: ");
        uart_print_int(temp);
        uart_print(" C\r\n");

        _delay_ms(1000);
    }

    return 0;
}
