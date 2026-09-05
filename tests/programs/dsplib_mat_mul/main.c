#include <stdio.h>
/* DSPLIB Q-format matrix multiply and the GSM minimum-error search. */
extern void DSP_mat_mul_cn(const short *x, int r1, int c1, const short *y, int c2, short *r, int qs);
extern int DSP_minerror_cn(const short *GSP0_TABLE, const short *errCoefs, int *max_index);

static unsigned int rng_state = 60708090u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[8 * 8];
static short y[8 * 8];
static short r[8 * 8];
/* GSP0_NUM = 256 vectors of GSP0_TERMS = 9 coefficients each. */
static short table[256 * 9];
static short coefficients[9];

int main(void)
{
    int i;
    int index = 0;
    unsigned int total = 0;
    for (i = 0; i < 8 * 8; i++) x[i] = (short)((int)(rng_next() >> 21) - 1024);
    for (i = 0; i < 8 * 8; i++) y[i] = (short)((int)(rng_next() >> 21) - 1024);
    DSP_mat_mul_cn(x, 8, 8, y, 8, r, 6);
    for (i = 0; i < 8 * 8; i++) total = total * 31u + (unsigned int)r[i];
    for (i = 0; i < 256 * 9; i++) table[i] = (short)((int)(rng_next() >> 22) - 512);
    for (i = 0; i < 9; i++) coefficients[i] = (short)((int)(rng_next() >> 22) - 512);
    total = total * 17u + (unsigned int)DSP_minerror_cn(table, coefficients, &index);
    total = total * 17u + (unsigned int)index;
    printf("%s=%u\n", "dsplib_mat_mul", total);
    return 0;
}
