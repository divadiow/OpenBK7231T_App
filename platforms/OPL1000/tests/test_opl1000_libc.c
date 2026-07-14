#include "../opl1000_libc.h"

#include <stdio.h>
#include <string.h>

static int s_failures;

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
        s_failures++; \
    } \
} while (0)

static double absolute(double value)
{
    return value < 0.0 ? -value : value;
}

static void test_double_parser(void)
{
    char *end;
    const char *invalid = "no-number";

    CHECK(absolute(OpenOPL1000_ParseDouble("  -12.375tail", &end) + 12.375) < 0.000001);
    CHECK(strcmp(end, "tail") == 0);
    CHECK(absolute(OpenOPL1000_ParseDouble("1.25e3", &end) - 1250.0) < 0.000001);
    CHECK(*end == '\0');
    CHECK(absolute(OpenOPL1000_ParseDouble(".5E-2x", &end) - 0.005) < 0.000001);
    CHECK(*end == 'x');
    CHECK(OpenOPL1000_ParseDouble(invalid, &end) == 0.0);
    CHECK(end == invalid);
    CHECK(OpenOPL1000_ParseDouble("7e+", &end) == 7.0);
    CHECK(*end == 'e');
}

static void test_integer_scanner(void)
{
    int a = 0;
    int b = 0;
    int c = 0;
    unsigned int x = 0;
    long long wide = 0;
    unsigned char ip[4] = {0};
    char suffix = '\0';

    CHECK(OpenOPL1000_Scan("-2:30", "%i:%i", &a, &b) == 2 && a == -2 && b == 30);
    CHECK(OpenOPL1000_Scan("11 0", "%d %d", &a, &b) == 2 && a == 11 && b == 0);
    CHECK(OpenOPL1000_Scan("922337203685477580", "%lld", &wide) == 1 && wide == 922337203685477580LL);
    CHECK(OpenOPL1000_Scan("1a2b-20", "%x-%x", &x, &a) == 2 && x == 0x1a2b && a == 0x20);
    CHECK(OpenOPL1000_Scan("IO21", "IO%d", &a) == 1 && a == 21);
    CHECK(OpenOPL1000_Scan("192.168.4.1", "%hhu.%hhu.%hhu.%hhu", &ip[0], &ip[1], &ip[2], &ip[3]) == 4);
    CHECK(ip[0] == 192 && ip[1] == 168 && ip[2] == 4 && ip[3] == 1);
    CHECK(OpenOPL1000_Scan("12:34:56", "%2d:%2d:%2d", &a, &b, &c) == 3 && a == 12 && b == 34 && c == 56);
    CHECK(OpenOPL1000_Scan("42.txt", "%u%c", &x, &suffix) == 2 && x == 42 && suffix == '.');
}

static void test_float_and_string_scanner(void)
{
    double value = 0.0;
    char word[8] = {0};
    int number = 0;

    CHECK(OpenOPL1000_Scan("-0.125e2", "%lg", &value) == 1);
    CHECK(absolute(value + 12.5) < 0.000001);
    CHECK(OpenOPL1000_Scan("abc 17", "%3s %d", word, &number) == 2);
    CHECK(strcmp(word, "abc") == 0 && number == 17);
}

int main(void)
{
    test_double_parser();
    test_integer_scanner();
    test_float_and_string_scanner();

    if (s_failures != 0)
    {
        printf("%d OpenOPL1000 libc test(s) failed\n", s_failures);
        return 1;
    }
    puts("OpenOPL1000 libc tests passed");
    return 0;
}
