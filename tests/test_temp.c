#include <stdio.h>
#include <stdint.h>

/* ── calibration constants — must match tiny85-dht-sensor.c ── */
#define TEMP_CAL_ADC   248
#define TEMP_CAL_C     22

/* ── functions under test ─────────────────────────────────── */

int16_t temp_from_adc(uint16_t adc)
{
    return TEMP_CAL_C + (adc - TEMP_CAL_ADC);
}

/* DHT11 encoding — BUG [6] fix */
void dht11_encode(int16_t temp_c, uint8_t data[5])
{
    data[0] = 0;
    data[1] = 0;
    data[2] = (uint8_t)temp_c;
    data[3] = 0;
    data[4] = data[0] + data[1] + data[2] + data[3];
}

/* DHT22 encoding — the wrong one, kept to prove it fails */
void dht22_encode(int16_t temp_c, uint8_t data[5])
{
    uint16_t t = (uint16_t)(temp_c * 10);
    data[0] = 0;
    data[1] = 0;
    data[2] = (t >> 8) & 0xFF;
    data[3] = t & 0xFF;
    data[4] = data[0] + data[1] + data[2] + data[3];
}

/* BIT_DELAY formula — BUG [1] fix */
uint32_t bit_delay_us(uint32_t baud)
{
    return 1000000UL / baud;
}

/* ── test helpers ─────────────────────────────────────────── */

static int pass = 0, fail = 0;

static void assert_eq(int a, int b, const char *msg)
{
    if (a != b) { printf("FAIL: %s  (got %d, want %d)\n", msg, a, b); fail++; }
    else        { printf("PASS: %s\n", msg); pass++; }
}

static void assert_ne(int a, int b, const char *msg)
{
    if (a == b) { printf("FAIL: %s  (both %d, expected different)\n", msg, a); fail++; }
    else        { printf("PASS: %s\n", msg); pass++; }
}

/* ── test groups ──────────────────────────────────────────── */

/*
 * temp_from_adc() — BUG [3] and BUG [4]
 * Calibration anchor is 248 (this chip, 1.1V ref).
 * Old tests used 282 which was the wrong reference/wrong chip value.
 */
static void test_temp_from_adc(void)
{
    printf("\n--- temp_from_adc ---\n");

    /* anchor point — chip reads 248 at 22C */
    assert_eq(temp_from_adc(248), 22, "calibration anchor: 248 -> 22C");

    /* one count per degree above anchor */
    assert_eq(temp_from_adc(258), 32, "10 counts above anchor -> 32C");
    assert_eq(temp_from_adc(268), 42, "20 counts above anchor -> 42C");

    /* one count per degree below anchor */
    assert_eq(temp_from_adc(238), 12, "10 counts below anchor -> 12C");
    assert_eq(temp_from_adc(228),  2, "20 counts below anchor -> 2C");

    /* body heat sanity — finger on chip should read ~28C = ADC 254 */
    assert_eq(temp_from_adc(254), 28, "body heat: 254 -> 28C");

    /*
     * BUG [3] regression: old 2.56V ref gave ADC ~92 at room temp.
     * With correct 1.1V ref and new anchor, 92 must NOT read as room temp.
     */
    assert_ne(temp_from_adc(92), 22, "BUG[3] regression: 2.56V ref value 92 must not read 22C");

    /*
     * BUG [4] regression: old anchor was 282.
     * With correct anchor 248, ADC=282 must not read as room temp.
     */
    assert_ne(temp_from_adc(282), 22, "BUG[4] regression: old anchor 282 must not read 22C");
}

/*
 * DHT11 encoding — BUG [6]
 * byte[2] = raw integer, byte[3] = 0, checksum = sum of all four bytes.
 * Host parses DHT11 — DHT22 encoding caused every read to fail.
 */
static void test_dht11_encoding(void)
{
    printf("\n--- DHT11 encoding ---\n");
    uint8_t data[5];

    dht11_encode(22, data);
    assert_eq(data[0], 0,   "DHT11: humidity high byte = 0");
    assert_eq(data[1], 0,   "DHT11: humidity low byte = 0");
    assert_eq(data[2], 22,  "DHT11: temp byte = raw integer 22");
    assert_eq(data[3], 0,   "DHT11: temp decimal byte = 0");
    assert_eq(data[4], 22,  "DHT11: checksum = 22");

    dht11_encode(28, data);
    assert_eq(data[2], 28,  "DHT11: body heat temp byte = 28");
    assert_eq(data[4], 28,  "DHT11: body heat checksum = 28");

    dht11_encode(0, data);
    assert_eq(data[2], 0,   "DHT11: 0C temp byte = 0");
    assert_eq(data[4], 0,   "DHT11: 0C checksum = 0");

    /* checksum must equal sum of bytes 0-3 */
    dht11_encode(35, data);
    assert_eq(data[4], (data[0]+data[1]+data[2]+data[3]) & 0xFF,
              "DHT11: checksum formula correct");
}

/*
 * DHT22 encoding regression — BUG [6]
 * Proves why DHT22 encoding fails a DHT11 host.
 * For any temp < 256, DHT22 data[2] will often be 0, which the
 * DHT11 host reads as 0C and rejects.
 */
static void test_dht22_encoding_wrong(void)
{
    printf("\n--- DHT22 encoding (wrong for this host, regression) ---\n");
    uint8_t data[5];

    /*
     * BUG [6] regression: at 22C, DHT22 puts 220 in byte[3] and 0 in
     * byte[2]. DHT11 host reads byte[2] as temperature → reads 0C → FAIL.
     */
    dht22_encode(22, data);
    assert_eq(data[2], 0,   "BUG[6] regression: DHT22 data[2]=0 at 22C (host reads 0C)");
    assert_ne(data[2], 22,  "BUG[6] regression: DHT22 data[2] != raw temp");

    /* at 28C: DHT22 data[2] = (280>>8) = 1, not 28 */
    dht22_encode(28, data);
    assert_ne(data[2], 28,  "BUG[6] regression: DHT22 data[2] != 28 at 28C");
}

/*
 * Baud rate formula — BUG [1]
 * BIT_DELAY must derive from a standard baud rate.
 * 2000 baud (500us) is non-standard, minicom cannot sync to it.
 */
static void test_baud_rate(void)
{
    printf("\n--- baud rate formula ---\n");

    assert_eq(bit_delay_us(9600),   104, "9600 baud  -> 104us");
    assert_eq(bit_delay_us(19200),   52, "19200 baud -> 52us");
    assert_eq(bit_delay_us(38400),   26, "38400 baud -> 26us");

    /* BUG [1] regression: 500us = 2000 baud, non-standard */
    assert_ne(bit_delay_us(9600), 500,  "BUG[1] regression: 9600 baud must not be 500us");
}

/* ── main ─────────────────────────────────────────────────── */

int main(void)
{
    test_temp_from_adc();
    test_dht11_encoding();
    test_dht22_encoding_wrong();
    test_baud_rate();

    printf("\n================================\n");
    printf("Results: %d passed, %d failed\n", pass, fail);
    printf("================================\n");

    return fail > 0 ? 1 : 0;
}
