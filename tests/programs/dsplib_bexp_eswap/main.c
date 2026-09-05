#include <stdio.h>
/* DSPLIB block exponent, endian swaps of every width and a block move. */
extern int DSP_bexp_cn(const int *x, short nx);
extern void DSP_blk_eswap16_cn(void *src, void *dst, int n_hwords);
extern void DSP_blk_eswap32_cn(void *src, void *dst, int n_words);
extern void DSP_blk_eswap64_cn(void *src, void *dst, int n_dwords);
extern void DSP_blk_move_cn(const short *x, short *r, int nx);

static unsigned int rng_state = 9182736u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int values[64];
static short data[128];
static short swapped[128];
static short moved[128];

int main(void)
{
    int i;
    unsigned int total = 0;
    for (i = 0; i < 64; i++) values[i] = (int)(rng_next() >> 9);
    for (i = 0; i < 128; i++) data[i] = (short)(rng_next() >> 16);
    total = (unsigned int)DSP_bexp_cn(values, 64);
    DSP_blk_eswap16_cn(data, swapped, 128);
    for (i = 0; i < 128; i++) total = total * 31u + (unsigned int)swapped[i];
    DSP_blk_eswap32_cn(data, swapped, 64);
    for (i = 0; i < 128; i++) total = total * 31u + (unsigned int)swapped[i];
    DSP_blk_eswap64_cn(data, swapped, 32);
    for (i = 0; i < 128; i++) total = total * 31u + (unsigned int)swapped[i];
    DSP_blk_move_cn(swapped, moved, 128);
    for (i = 0; i < 128; i++) total = total * 17u + (unsigned int)moved[i];
    printf("%s=%u\n", "dsplib_bexp_eswap", total);
    return 0;
}
