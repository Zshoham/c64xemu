#include <stdio.h>
/* DSPLIB single-sample IIR run over a stream, and the FIR with a radix-4 kernel. */
extern short DSP_iir_ss_cn(short input, const short *coefs, int ncoefs, short *state);
extern void DSP_fir_r4_cn(const short *x, const short *h, short *r, int nh, int nr);

static unsigned int rng_state = 777001u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short coefs[8];
static short state[8];
static short x[64 + 8 - 1];
static short h[8];
static short r[64];

int main(void)
{
    int i;
    unsigned int total = 0;
    for (i = 0; i < 8; i++) coefs[i] = (short)((int)(rng_next() >> 23) - 256);
    for (i = 0; i < 8; i++) state[i] = 0;
    for (i = 0; i < 200; i++) {
        short sample = (short)((int)(rng_next() >> 21) - 1024);
        total = total * 31u + (unsigned int)DSP_iir_ss_cn(sample, coefs, 8, state);
    }
    for (i = 0; i < 64 + 8 - 1; i++) x[i] = (short)((int)(rng_next() >> 21) - 1024);
    for (i = 0; i < 8; i++) h[i] = (short)((int)(rng_next() >> 23) - 256);
    DSP_fir_r4_cn(x, h, r, 8, 64);
    for (i = 0; i < 64; i++) total = total * 17u + (unsigned int)r[i];
    printf("%s=%u\n", "dsplib_iir_ss", total);
    return 0;
}
