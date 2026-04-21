/******************************************************************************
 * \file    tiny85-dht-sensor.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   ATtiny85 internal temperature sensor + bit-banged UART telemetry
 *          + DHT11-compatible waveform output.
 *
 * \details Reads internal temperature sensor via ADC and transmits raw ADC
 *          and calibrated temperature over a software UART on PB4.
 *          Outputs a DHT11-compatible waveform on PB0 in response to host
 *          start signal (host pulls line low 20ms then releases).
 *
 *          Hardware:
 *          - MCU:      ATtiny85 @ 8MHz internal oscillator (lfuse=0xe2)
 *          - UART TX:  PB4 (bit-banged, 8N1)
 *          - DHT OUT:  PB0 (DHT11-compatible waveform)
 *          - ADC:      Internal temperature sensor (MUX = 0x0F)
 *          - Vref:     Internal 1.1V reference
 *
 * ============================================================================
 * BUG FIXES (all found and fixed during hardware debugging)
 * ============================================================================
 *
 * \bug     [1] Non-standard baud rate
 *              ORIGINAL:  #define BIT_DELAY 500  →  2000 baud
 *              FIXED:     #define BAUD 9600, BIT_DELAY = 1000000UL / BAUD
 *              SYMPTOM:   minicom showed no output at any standard baud rate.
 *              CAUSE:     2000 baud is not a standard rate. minicom only
 *                         supports standard rates (9600, 19200, 38400,
 *                         57600, 115200). The bit timing was valid but no
 *                         host could sync to it.
 *              RULE:      Always derive BIT_DELAY from a named BAUD constant
 *                         so the rate is obvious and checked against standards.
 *
 * \bug     [2] SRAM overflow from string literals
 *              ORIGINAL:  uart_print("literal string")
 *              FIXED:     LOG("literal string") via PSTR() + pgm_read_byte()
 *              SYMPTOM:   Linker error: section `.data' overflowed by 132 bytes
 *              CAUSE:     ATtiny85 has only 512 bytes SRAM. Every string
 *                         literal passed to uart_print() is copied into RAM
 *                         at startup by the C runtime. Adding debug logging
 *                         pushed total string data to 644 bytes — 132 over
 *                         limit. Code never ran.
 *              FIX:       PSTR() places literals in flash (.progmem.data).
 *                         pgm_read_byte() emits LPM to read them back one
 *                         byte at a time. Flash is 8KB — strings fit easily.
 *              RULE:      On AVR, any const string that is never modified
 *                         belongs in flash. Never pass a literal to a function
 *                         that takes char* on ATtiny85.
 *
 * \bug     [3] Wrong ADC voltage reference
 *              ORIGINAL:  ADMUX = (1<<REFS1) | (1<<REFS0) | 0x0F
 *              FIXED:     ADMUX = (1<<REFS1) | 0x0F
 *              SYMPTOM:   RAW reading was 92-94 instead of expected ~282.
 *                         TEMP computed as -168°C at room temperature.
 *              CAUSE:     On ATtiny85, REFS1=1 REFS0=1 selects the 2.56V
 *                         internal reference, not 1.1V. With 2.56V ref the
 *                         ADC counts are ~2.3x lower for the same voltage.
 *                         REFS1=1 REFS0=0 is the correct 1.1V reference.
 *                         The original TEMP_CAL_ADC=282 was derived from
 *                         datasheet typical values at 1.1V, so the mismatch
 *                         corrupted the entire calibration.
 *              RULE:      Always verify REFS bits against the specific MCU
 *                         datasheet. ATtiny85 reference selection differs
 *                         from ATmega series.
 *
 * \bug     [4] Wrong calibration anchor
 *              ORIGINAL:  TEMP_CAL_ADC 282  (datasheet typical value)
 *              FIXED:     TEMP_CAL_ADC 248  (measured on this chip at 22°C)
 *              SYMPTOM:   After reference fix, RAW read 248 but TEMP still
 *                         showed -12°C because anchor was still 282.
 *              CAUSE:     Internal RC temperature sensor varies significantly
 *                         chip-to-chip. Datasheet typical values are not
 *                         usable for calibration without trimming. This chip
 *                         reads 248 counts at 22°C (72°F room temperature),
 *                         not 282.
 *              RULE:      Always calibrate TEMP_CAL_ADC on the actual chip
 *                         at a known temperature. One count ≈ one degree C
 *                         once the anchor is correct.
 *
 * \bug     [5] ADC read blocking DHT response
 *              ORIGINAL:  host trigger → _delay_us(30) → adc_read_temp()
 *                         → dht_send_frame()
 *              FIXED:     ADC cached at boot and refreshed every 8 cycles.
 *                         host trigger → _delay_us(30) → dht_send_frame()
 *                         (cached value) → UART log → ADC refresh if due.
 *              SYMPTOM:   [S4-ATtiny] FAILED / EVENT: DHT11_FAIL every cycle.
 *                         ATtiny UART log showed correct readings.
 *              CAUSE:     adc_read_temp() takes >20ms (20ms settle +
 *                         2 conversions). STM32 DHT11 driver timeout waiting
 *                         for response signal is ~1ms (8000 DWT cycles at
 *                         72MHz). Host gave up before ATtiny drove the pin.
 *              RULE:      On bit-bang DHT, the response signal must start
 *                         within microseconds of the host releasing the line.
 *                         Any slow operation (ADC, UART, delay) before
 *                         dht_send_frame() will cause the host to time out.
 *
 * \bug     [6] DHT22 encoding sent to DHT11 host
 *              ORIGINAL:  data[2] = (temp*10) >> 8
 *                         data[3] = (temp*10) & 0xFF
 *              FIXED:     data[2] = (uint8_t)temp_c
 *                         data[3] = 0
 *              SYMPTOM:   [S4-ATtiny] FAILED even after timing was fixed.
 *                         Event log said DHT11_FAIL, not DHT22_FAIL.
 *              CAUSE:     DHT22 encodes temperature as (temp * 10) split
 *                         across two bytes for one decimal place precision.
 *                         DHT11 encodes temperature as a plain integer in
 *                         byte[2] with byte[3]=0. The STM32 driver was
 *                         parsing DHT11 protocol. For temp=22°C, DHT22
 *                         encoding gives data[2]=0, data[3]=220 — host
 *                         read 0°C and failed the validity check.
 *              RULE:      Match encoding to whatever protocol the host
 *                         driver expects. The event name in the host log
 *                         (DHT11_FAIL vs DHT22_FAIL) identifies the parser.
 *
 * \bug     [7] UART logging inside dht_send_frame() blocking response
 *              ORIGINAL:  LOG("[DHT] building frame...") before pin activity
 *              FIXED:     All LOG() calls moved to after line is released
 *              SYMPTOM:   [S4-ATtiny] FAILED even after encoding fix.
 *                         ATtiny log showed "[DHT] building frame" before
 *                         "[DHT] sending response signal".
 *              CAUSE:     Each LOG() call transmits ~30 characters at 9600
 *                         baud ≈ ~31ms per line. Three LOG() calls before
 *                         the response signal = ~91ms before the pin moved.
 *                         STM32 timeout is ~1ms — host had already given up.
 *                         Fix [5] cached the ADC but left UART in the frame
 *                         function, which reintroduced the same timing
 *                         violation.
 *              RULE:      Inside any bit-bang protocol response function,
 *                         no UART, no delays, no ADC — only pin operations
 *                         and _delay_us(). Log only after the last pin
 *                         transition is complete.
 *
 * ============================================================================
 *
 * \note    F_CPU must match fuse setting. lfuse=0xe2 = 8MHz internal RC.
 *          If F_CPU is wrong all _delay_us/_delay_ms timings are wrong.
 *
 * \note    Clock verification (hardware confirmed):
 *          _delay_us(24) measured on Saleae at 24MHz: 23.25µs actual.
 *          Actual clock = (24 / 23.25) * 8MHz = ~8.26MHz.
 *          Within ATtiny85 internal RC ±10% spec.
 *
 * \warning Internal RC oscillator can drift with temperature, causing UART
 *          framing errors at higher baud rates. 9600 baud is the practical
 *          maximum for reliable bit-bang UART on the ATtiny85 internal RC.
 *
 ******************************************************************************/

#include <avr/io.h>
#include <avr/pgmspace.h>      /* BUG [2]: required for PSTR, pgm_read_byte */
#include <util/delay.h>
#include <stdint.h>

#define F_CPU          8000000UL

#define UART_PIN       PB4
#define BAUD           9600                   /* BUG [1]: was 2000 (500us), non-standard */
#define BIT_DELAY      (1000000UL / BAUD)     /* BUG [1]: 104us per bit */

#define DHT_PIN        PB0

#define TEMP_CAL_ADC   248    /* BUG [4]: was 282 (datasheet typical, not this chip) */
#define TEMP_CAL_C     22

/* ── UART ─────────────────────────────────────────────────── */

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

/*
 * BUG [2]: kept for runtime-built strings only (uart_print_int writes into
 * a stack buffer, not a literal). Never pass a string literal to this
 * function — it will be copied into SRAM and overflow the 512 byte limit.
 */
static void uart_print(const char *s)
{
    while (*s)
        uart_tx_char(*s++);
}

/*
 * BUG [2]: reads from flash one byte at a time via pgm_read_byte().
 * Without LPM the AVR reads the wrong address space and prints garbage.
 */
static void uart_print_P(const char *s)
{
    char c;
    while ((c = pgm_read_byte(s++)))
        uart_tx_char(c);
}

/*
 * BUG [2]: PSTR() places the literal in .progmem.data (flash, not SRAM).
 * LOG("text") is a drop-in for uart_print("text") at zero SRAM cost.
 */
#define LOG(s) uart_print_P(PSTR(s))

static void uart_print_int(int val)
{
    char buf[8];   /* stack allocation — fine, not a static literal */
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

/* ── ADC ──────────────────────────────────────────────────── */

static uint16_t adc_read_temp(void)
{
    LOG("[ADC] enabling...\r\n");

    ADCSRA =
        (1 << ADEN)  |
        (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    /*
     * BUG [3]: REFS1=1 REFS0=0 = internal 1.1V reference.
     * Original had (1<<REFS1)|(1<<REFS0) = 2.56V reference.
     * With 2.56V ref ADC reads ~2.3x lower (~92 instead of ~248),
     * which destroyed the single-point calibration by ~190 degrees.
     * ATtiny85 reference table:
     *   REFS1=0 REFS0=0  VCC
     *   REFS1=0 REFS0=1  External AREF
     *   REFS1=1 REFS0=0  Internal 1.1V  ← correct
     *   REFS1=1 REFS0=1  Internal 2.56V ← original (wrong)
     */
    ADMUX = (1 << REFS1) | 0x0F;   /* BUG [3]: REFS0 removed */

    LOG("[ADC] settling (20ms)...\r\n");
    _delay_ms(20);
    LOG("[ADC] settle done\r\n");

    LOG("[ADC] dummy conversion...\r\n");
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    LOG("[ADC] dummy done\r\n");

    LOG("[ADC] real conversion...\r\n");
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    LOG("[ADC] read done\r\n");

    return ADC;
}

static int16_t temp_from_adc(uint16_t adc)
{
    return TEMP_CAL_C + (adc - TEMP_CAL_ADC);
}

/* ── DHT11 waveform ───────────────────────────────────────── */

static void dht_send_frame(int16_t temp_c)
{
    /*
     * BUG [7]: NO logging before or during transmission.
     * Each LOG() line takes ~31ms at 9600 baud. Three lines = ~91ms.
     * STM32 DHT11 driver timeout is ~1ms — host gives up before the
     * pin ever moves. All logging moved to after line is released.
     *
     * BUG [5]: No ADC reads here either. adc_read_temp() takes >20ms.
     * Same timeout violation. ADC is cached in main() and refreshed
     * every 8 cycles, well away from the host trigger.
     *
     * RULE: from host release to first pin transition must be <1ms.
     * Only pin operations and _delay_us() belong in this function.
     */

    uint8_t data[5];
    /*
     * BUG [6]: DHT11 encoding, not DHT22.
     * DHT22: data[2] = (temp*10)>>8, data[3] = (temp*10)&0xFF
     * DHT11: data[2] = temp integer,  data[3] = 0
     * Host driver is DHT11 (confirmed by EVENT: DHT11_FAIL in host log).
     * DHT22 encoding sent data[2]=0 for any temp <25.6C — host read
     * 0°C and failed the validity check every cycle.
     */
    data[0] = 0;                  /* humidity integer  — no sensor, fixed 0 */
    data[1] = 0;                  /* humidity decimal  — always 0 for DHT11 */
    data[2] = (uint8_t)temp_c;   /* BUG [6]: was (temp*10)>>8 (DHT22 encoding) */
    data[3] = 0;                  /* BUG [6]: was (temp*10)&0xFF (DHT22 encoding) */
    data[4] = data[0] + data[1] + data[2] + data[3];

    uint8_t bits[40];
    for (uint8_t i = 0; i < 5; i++)
        for (uint8_t j = 0; j < 8; j++)
            bits[i * 8 + j] = (data[i] >> (7 - j)) & 1;

    /* response signal — must start within ~20-40us of host releasing line */
    DDRB  |=  (1 << DHT_PIN);
    PORTB &= ~(1 << DHT_PIN);
    _delay_us(80);
    PORTB |=  (1 << DHT_PIN);
    _delay_us(80);

    for (uint8_t i = 0; i < 40; i++)
    {
        PORTB &= ~(1 << DHT_PIN);
        _delay_us(50);
        PORTB |=  (1 << DHT_PIN);
        if (bits[i])
            _delay_us(70);
        else
            _delay_us(26);
    }

    PORTB &= ~(1 << DHT_PIN);
    _delay_us(50);
    DDRB  &= ~(1 << DHT_PIN);
    PORTB &= ~(1 << DHT_PIN);

    /* BUG [7]: log only after line is released */
    LOG("[DHT] frame sent, temp=");
    uart_print_int(temp_c);
    LOG("C chk=");
    uart_print_int(data[4]);
    LOG("\r\n");
}

/* ── Main ─────────────────────────────────────────────────── */

int main(void)
{
    DDRB  |=  (1 << UART_PIN);
    PORTB |=  (1 << UART_PIN);
    DDRB  &= ~(1 << DHT_PIN);
    PORTB &= ~(1 << DHT_PIN);
    _delay_ms(100);

    LOG("\r\n[BOOT] tiny85-dht-sensor started\r\n");
    LOG("[BOOT] UART 9600 8N1 on PB4\r\n");
    LOG("[BOOT] DHT out on PB0\r\n");

    /*
     * BUG [5]: read ADC once at boot to prime the cache.
     * Subsequent reads happen every 8 cycles after the frame is sent,
     * never in the host trigger path where the 20ms settle would cause
     * a timeout.
     */
    uint16_t adc  = adc_read_temp();
    int16_t  temp = temp_from_adc(adc);

    LOG("[BOOT] initial read: RAW=");
    uart_print_int(adc);
    LOG(" TEMP=");
    uart_print_int(temp);
    LOG("C\r\n");
    LOG("[BOOT] waiting for host...\r\n");

    uint8_t cycle = 0;

    while (1)
    {
        /* wait for host pull-low */
        while (PINB & (1 << DHT_PIN));

        /* wait for host release */
        while (!(PINB & (1 << DHT_PIN)));

        /*
         * BUG [5][7]: respond immediately with cached value.
         * Nothing between here and dht_send_frame() except the
         * 30us spec gap. No ADC, no UART, no delay >30us.
         */
        _delay_us(30);
        dht_send_frame(temp);

        LOG("[MAIN] RAW: ");
        uart_print_int(adc);
        LOG(" | TEMP: ");
        uart_print_int(temp);
        LOG(" C\r\n");
        LOG("[MAIN] cycle complete\r\n");
        LOG("-----------------------------\r\n");

        /*
         * BUG [5]: ADC refresh happens here, after frame and UART are
         * done. Every 8 cycles ≈ every 8 seconds at 1Hz host polling.
         * The 20ms settle is harmless here — host will trigger again
         * long after this completes and the cache will be fresh.
         */
        cycle++;
        if (cycle >= 8)
        {
            cycle = 0;
            LOG("[ADC] refreshing...\r\n");
            adc  = adc_read_temp();
            temp = temp_from_adc(adc);
            LOG("[ADC] updated RAW=");
            uart_print_int(adc);
            LOG(" TEMP=");
            uart_print_int(temp);
            LOG("C\r\n");
        }
    }

    return 0;
}
