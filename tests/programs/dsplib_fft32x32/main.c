#include <stdio.h>
/* DSPLIB 32-bit FFT and the 16x32 mixed-precision FFT, both over deterministic data. */
extern void DSP_fft32x32_cn(int *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_fft16x32_cn(const short *ptr_w, int npoints, int *ptr_x, int *ptr_y);

static unsigned int rng_state = 22114433u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

#define POINTS 64

static int w32[2 * POINTS];
static short w16[2 * POINTS];
static int x[2 * POINTS];
static int y[2 * POINTS];

int main(void)
{
    int i;
    unsigned int total = 0;
    for (i = 0; i < 2 * POINTS; i++) w32[i] = (int)(rng_next() >> 8) - 8388608;
    for (i = 0; i < 2 * POINTS; i++) w16[i] = (short)((int)(rng_next() >> 20) - 2048);
    for (i = 0; i < 2 * POINTS; i++) x[i] = (int)(rng_next() >> 8) - 8388608;
    DSP_fft32x32_cn(w32, POINTS, x, y);
    for (i = 0; i < 2 * POINTS; i++) total = total * 31u + (unsigned int)y[i];
    for (i = 0; i < 2 * POINTS; i++) x[i] = (int)(rng_next() >> 8) - 8388608;
    DSP_fft16x32_cn(w16, POINTS, x, y);
    for (i = 0; i < 2 * POINTS; i++) total = total * 17u + (unsigned int)y[i];
    printf("%s=%u\n", "dsplib_fft32x32", total);
    return 0;
}
