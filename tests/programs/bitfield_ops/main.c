#include <stdio.h>
/* Bit fields and packed structures: EXT/SET/CLR heavy code. */
struct control {
    unsigned int enable : 1;
    unsigned int mode : 3;
    unsigned int channel : 5;
    int offset : 9;
    unsigned int tag : 14;
};

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static struct control table[32];

static int compute(void)
{
    int i;
    unsigned int acc = 0u;

    for (i = 0; i < 32; i++) {
        unsigned int raw = rng_next();

        table[i].enable = raw & 1u;
        table[i].mode = (raw >> 3) & 7u;
        table[i].channel = (raw >> 7) & 31u;
        table[i].offset = (int)((raw >> 13) & 511u) - 256;
        table[i].tag = (raw >> 17) & 16383u;
    }
    for (i = 0; i < 32; i++) {
        if (table[i].enable) {
            acc += (unsigned int)table[i].mode * 3u;
        }
        acc ^= (unsigned int)table[i].channel << (i & 7);
        acc += (unsigned int)table[i].offset;
        acc = (acc << 1) | (acc >> 31);
        acc ^= table[i].tag;
    }
    return (int)acc;
}

int main(void)
{
    printf("%s=%d\n", "bitfield_ops", compute());
    return 0;
}
