#include <stdio.h>
#include <stdint.h>

int16_t temp_from_adc(uint16_t adc)
{
    return 22 + (adc - 282);
}

void assert_eq(int a, int b, const char *msg)
{
    if (a != b)
        printf("FAIL: %s (%d != %d)\n", msg, a, b);
    else
        printf("PASS: %s\n", msg);
}

int main()
{
    assert_eq(temp_from_adc(282), 22, "room temp");
    assert_eq(temp_from_adc(292), 32, "hotter");
    assert_eq(temp_from_adc(272), 12, "colder");

    return 0;
}
