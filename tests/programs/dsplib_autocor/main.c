#include <stdio.h>
/* DSPLIB autocorrelation over 16-bit data. */
extern void DSP_autocor_cn(short *r, const short *x, int nx, int nr);

static unsigned int rng_state = 13579u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[128 + 32];
static short r[32];

int main(void)
{
    int i;
    unsigned int total = 0;
    for (i = 0; i < 128 + 32; i++) x[i] = (short)((int)(rng_next() >> 21) - 1024);
    /* The reference reads x[nr .. nx+nr-1] and x[k-i], so it takes the array start. */
    DSP_autocor_cn(r, x, 128, 32);
    for (i = 0; i < 32; i++) total = total * 31u + (unsigned int)r[i];
    printf("%s=%u\n", "dsplib_autocor", total);
    return 0;
}
