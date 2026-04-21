#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 8000000UL   // ← change to your actual clock speed

// Minicom only supports standard baud rates (9600, 19200, 38400, 57600, 115200, etc.) and 2000 isn't one of them.

#define UART_PIN  PB4
#define BAUD      9600
#define BIT_DELAY (1000000UL / BAUD)

static void uart_tx_char(char c)
{
    PORTB &= ~(1 << UART_PIN);
    _delay_us(BIT_DELAY);

    for (uint8_t i = 0; i < 8; i++)
    {
        if (c & 1)
            PORTB |= (1 << UART_PIN);
        else
            PORTB &= ~(1 << UART_PIN);
        c >>= 1;
        _delay_us(BIT_DELAY);
    }

    PORTB |= (1 << UART_PIN);
    _delay_us(BIT_DELAY);
}

static void uart_print(const char *s)
{
    while (*s)
        uart_tx_char(*s++);
}

int main(void)
{
    DDRB  |=  (1 << UART_PIN);
    PORTB |=  (1 << UART_PIN);
    _delay_ms(100);

    while (1)
    {
        uart_print("Hello\r\n");
        _delay_ms(1000);
    }

    return 0;
}
