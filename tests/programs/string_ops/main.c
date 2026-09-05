#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* RTS string and conversion routines over overlapping and aligned buffers. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static char buffer[128];
static char other[128];

static int compute(void)
{
    int total = 0;
    int i;

    for (i = 0; i < 64; i++) {
        buffer[i] = (char)('a' + (int)(rng_next() % 26u));
    }
    buffer[64] = 0;
    strcpy(other, buffer);
    total += strcmp(buffer, other);
    total += strlen(buffer) * 3;
    other[10] = 'Z';
    total += strcmp(buffer, other) < 0 ? -5 : 5;
    total += memcmp(buffer, other, 64) < 0 ? -7 : 7;
    memmove(other + 4, other, 40);
    for (i = 0; i < 48; i++) {
        total = (total << 1) ^ (unsigned char)other[i];
    }
    /* A value that fits in 32 bits, because long is 32-bit on this target. */
    strncpy(other, "204512345", 10);
    total += (int)strtol(other, NULL, 10) >> 3;
    total += atoi("-4231") * 2;
    total ^= (int)strtoul("0xfeed", NULL, 16);
    total += (int)(strchr(buffer, buffer[20]) - buffer);
    total ^= (int)strcspn(buffer, "xyz");
    total += (int)strspn(buffer, "abcdefghijklmnopqrstuvwxyz");
    memset(other, '#', sizeof other);
    total ^= (int)strlen(memchr(other, '#', 8) ? "found" : "missing");
    return total;
}

int main(void)
{
    printf("%s=%d\n", "string_ops", compute());
    return 0;
}
