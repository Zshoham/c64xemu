#include <stdio.h>
#include <string.h>
/* printf conversions: widths, flags, precision, long long and dyadic fractions. */
static char line[128];
static char other[192];

static int compute(void)
{
    int total = 0;
    int used;
    unsigned int i;

    used = sprintf(line, "[%d][%5d][%-5d][%05d][%+d]", -42, -42, -42, 42, 42);
    for (i = 0; i < (unsigned int)used; i++) {
        total = total * 31 + (unsigned char)line[i];
    }
    used = sprintf(line, "[%u][%x][%X][%#x][%o][%c][%s][%.3s]", 4294967295u, 48879u, 48879u,
                   48879u, 511u, 'Z', "abcdef", "abcdef");
    for (i = 0; i < (unsigned int)used; i++) {
        total = total * 31 + (unsigned char)line[i];
    }
    used = sprintf(line, "[%ld][%lu][%lld][%llu]", -123456789L, 4294967295UL,
                   -1234567890123456789LL, 12345678901234567890ULL);
    for (i = 0; i < (unsigned int)used; i++) {
        total = total * 31 + (unsigned char)line[i];
    }
    used = sprintf(line, "[%.4f][%.4f][%.2f][%8.3f]", 1.5, 0.25, -3.75, 2.125);
    for (i = 0; i < (unsigned int)used; i++) {
        total = total * 31 + (unsigned char)line[i];
    }
    used = sprintf(other, "[%d%%][%s]", 7, line);
    for (i = 0; i < (unsigned int)used; i++) {
        total = total * 31 + (unsigned char)other[i];
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "printf_formats", compute());
    return 0;
}
