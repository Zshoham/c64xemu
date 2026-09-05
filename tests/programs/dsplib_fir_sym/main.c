#include <stdio.h>
/* DSPLIB symmetric FIR plus a matrix transpose over the same data. */
extern void DSP_fir_sym_cn(const short *x, const short *h, short *r, int nh, int nr, int s);
extern void DSP_mat_trans_cn(const short *x, short rows, short columns, short *r);

static unsigned int rng_state = 555u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[64 + 2 * 8 + 1];
static short h[8 + 1];
static short r[64];
static short transposed[64];

int main(void)
{
    int i;
    unsigned int total = 0;
    for (i = 0; i < 64 + 2 * 8 + 1; i++) x[i] = (short)((int)(rng_next() >> 21) - 1024);
    for (i = 0; i < 8 + 1; i++) h[i] = (short)((int)(rng_next() >> 23) - 256);
    DSP_fir_sym_cn(x, h, r, 8, 64, 15);
    DSP_mat_trans_cn(r, 8, 8, transposed);
    for (i = 0; i < 64; i++) total = total * 31u + (unsigned int)r[i] + 7 * transposed[i];
    printf("%s=%u\n", "dsplib_fir_sym", total);
    return 0;
}
