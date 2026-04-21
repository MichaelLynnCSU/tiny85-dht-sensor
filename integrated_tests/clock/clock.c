/*
 * clock.c
 * Clock verification test for ATtiny85 @ 8MHz
 * 24µs low pulse every 70µs
 */

#include <avr/io.h>
#include <util/delay.h>

#define TEST_PIN PB4

int main(void)
{
    DDRB  |=  (1 << TEST_PIN);
    PORTB |=  (1 << TEST_PIN);
    _delay_ms(100);

    while (1)
    {
        PORTB &= ~(1 << TEST_PIN);  /* low 24µs */
        _delay_us(24);
        PORTB |=  (1 << TEST_PIN);  /* high 46µs (70-24) */
        _delay_us(46);
    }

    return 0;
}
