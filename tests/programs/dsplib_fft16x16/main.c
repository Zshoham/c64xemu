#include <stdio.h>
/* DSPLIB radix-4 FFT and its inverse over 16-bit data. The twiddle table is filled with a
   deterministic pattern rather than real twiddles: what is being checked is that the emulator
   computes what the host computes from the same code and the same bits, and a real table would
   pull in libm and its rounding. */
extern void DSP_fft16x16_cn(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_ifft16x16_cn(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);

static unsigned int rng_state = 44556677u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

#define POINTS 64

static short w[2 * POINTS];
static short x[2 * POINTS];
static short y[2 * POINTS];
static short z[2 * POINTS];

int main(void)
{
    int i;
    unsigned int total = 0;
    for (i = 0; i < 2 * POINTS; i++) w[i] = (short)((int)(rng_next() >> 20) - 2048);
    for (i = 0; i < 2 * POINTS; i++) x[i] = (short)((int)(rng_next() >> 20) - 2048);
    DSP_fft16x16_cn(w, POINTS, x, y);
    for (i = 0; i < 2 * POINTS; i++) total = total * 31u + (unsigned int)y[i];
    DSP_ifft16x16_cn(w, POINTS, y, z);
    for (i = 0; i < 2 * POINTS; i++) total = total * 17u + (unsigned int)z[i];
    printf("%s=%u\n", "dsplib_fft16x16", total);
    return 0;
}
