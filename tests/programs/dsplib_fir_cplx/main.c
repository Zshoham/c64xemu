#include <stdio.h>
/* DSPLIB complex FIR over interleaved 16-bit real/imaginary pairs. */
extern void DSP_fir_cplx_cn(const short *x, const short *h, short *r, int nh, int nr);

static unsigned int rng_state = 24680u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[2 * (48 + 8 - 1)];
static short h[2 * 8];
static short r[2 * 48];

int main(void)
{
    int i;
    unsigned int total = 0;
    for (i = 0; i < 2 * (48 + 8 - 1); i++) x[i] = (short)((int)(rng_next() >> 20) - 2048);
    for (i = 0; i < 2 * 8; i++) h[i] = (short)((int)(rng_next() >> 22) - 512);
    /* The reference indexes x[i-j], so the pointer must sit nh-1 complex samples in. */
    DSP_fir_cplx_cn(x + 2 * (8 - 1), h, r, 8, 48);
    for (i = 0; i < 2 * 48; i++) total = total * 31u + (unsigned int)r[i];
    printf("%s=%u\n", "dsplib_fir_cplx", total);
    return 0;
}
