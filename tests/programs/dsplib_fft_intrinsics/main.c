#include <stdio.h>
/* Differential execution for the ten DSPLIB FFT kernels that exist only as intrinsic C, which
   nothing in this suite had ever run: the forward and inverse transforms over 16x16, 16x32 and
   32x32 data, the imaginary-real ordered variants, the real-input 16x16 transform and the scaled
   32x32 one. Each is called beside its natural-C reference over identical inputs and the two
   results must agree bit for bit.

   The intrinsic versions are written with aligned double-word accesses, packed arithmetic and
   software-pipelined loops, so they reach parts of the machine the references never touch. The
   host compiler cannot build them, so on the host each intrinsic call is the reference call: the
   host therefore supplies one value that both target implementations have to produce, and the
   match count says the target agreed with itself.

   The twiddle tables are a deterministic pattern rather than real twiddles - a real table would
   pull in libm and its rounding, and what is being checked is that two implementations of the
   same arithmetic agree. */

#define ALIGNED8 __attribute__((aligned(8)))

extern void DSP_fft16x16_cn(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_ifft16x16_cn(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_fft16x16_imre_cn(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_ifft16x16_imre_cn(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_fft16x16r_cn(int n, short *ptr_x, const short *ptr_w, short *ptr_y, int radix,
                             int offset, int nmax);
extern void DSP_fft16x32_cn(const short *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_ifft16x32_cn(const short *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_fft32x32_cn(const int *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_ifft32x32_cn(const int *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_fft32x32s_cn(const int *ptr_w, int npoints, int *ptr_x, int *ptr_y);

#if defined(__TI_COMPILER_VERSION__)
extern void DSP_fft16x16_i(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_ifft16x16_i(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_fft16x16_imre_i(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_ifft16x16_imre_i(const short *ptr_w, int npoints, short *ptr_x, short *ptr_y);
extern void DSP_fft16x16r_i(int n, short *ptr_x, const short *ptr_w, short *ptr_y, int radix,
                            int offset, int nmax);
extern void DSP_fft16x32_i(const short *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_ifft16x32_i(const short *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_fft32x32_i(const int *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_ifft32x32_i(const int *ptr_w, int npoints, int *ptr_x, int *ptr_y);
extern void DSP_fft32x32s_i(const int *ptr_w, int npoints, int *ptr_x, int *ptr_y);
#else
#define DSP_fft16x16_i        DSP_fft16x16_cn
#define DSP_ifft16x16_i       DSP_ifft16x16_cn
#define DSP_fft16x16_imre_i   DSP_fft16x16_imre_cn
#define DSP_ifft16x16_imre_i  DSP_ifft16x16_imre_cn
#define DSP_fft16x16r_i       DSP_fft16x16r_cn
#define DSP_fft16x32_i        DSP_fft16x32_cn
#define DSP_ifft16x32_i       DSP_ifft16x32_cn
#define DSP_fft32x32_i        DSP_fft32x32_cn
#define DSP_ifft32x32_i       DSP_ifft32x32_cn
#define DSP_fft32x32s_i       DSP_fft32x32s_cn
#endif

static unsigned int rng_state = 0x13579BDFu;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

#define POINTS 16

static short w16[2 * POINTS] ALIGNED8;
static short x16[2 * POINTS] ALIGNED8;
static short y16[2 * POINTS] ALIGNED8;
static short z16[2 * POINTS] ALIGNED8;
static int w32[2 * POINTS] ALIGNED8;
static int x32[2 * POINTS] ALIGNED8;
static int y32[2 * POINTS] ALIGNED8;
static int z32[2 * POINTS] ALIGNED8;

static unsigned int total;
static unsigned int matched;

static void fold16(const short *reference, const short *test)
{
    int index;
    int same = 1;

    for (index = 0; index < 2 * POINTS; index++) {
        total = total * 31u + (unsigned int)(unsigned short)reference[index];
        if (reference[index] != test[index]) {
            same = 0;
        }
    }
    matched += (unsigned int)same;
}

static void fold32(const int *reference, const int *test)
{
    int index;
    int same = 1;

    for (index = 0; index < 2 * POINTS; index++) {
        total = total * 31u + (unsigned int)reference[index];
        if (reference[index] != test[index]) {
            same = 0;
        }
    }
    matched += (unsigned int)same;
}

static void fill16(void)
{
    int index;

    for (index = 0; index < 2 * POINTS; index++) {
        w16[index] = (short)((int)(rng_next() >> 20) - 2048);
    }
    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = (short)((int)(rng_next() >> 20) - 2048);
    }
}

static void fill32(void)
{
    int index;

    for (index = 0; index < 2 * POINTS; index++) {
        w32[index] = (int)(rng_next() >> 8) - 0x800000;
        w16[index] = (short)((int)(rng_next() >> 20) - 2048);
    }
    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = (int)(rng_next() >> 8) - 0x800000;
    }
}

int main(void)
{
    /* Every kernel is run twice from the same input: the reference into y, the intrinsic version
       into z, with x restored in between because several of them work in place. */
    short saved16[2 * POINTS];
    int saved32[2 * POINTS];
    int index;

    total = 0;
    matched = 0;

    fill16();
    for (index = 0; index < 2 * POINTS; index++) {
        saved16[index] = x16[index];
    }
    DSP_fft16x16_cn(w16, POINTS, x16, y16);
    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_fft16x16_i(w16, POINTS, x16, z16);
    fold16(y16, z16);

    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_ifft16x16_cn(w16, POINTS, x16, y16);
    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_ifft16x16_i(w16, POINTS, x16, z16);
    fold16(y16, z16);

    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_fft16x16_imre_cn(w16, POINTS, x16, y16);
    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_fft16x16_imre_i(w16, POINTS, x16, z16);
    fold16(y16, z16);

    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_ifft16x16_imre_cn(w16, POINTS, x16, y16);
    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_ifft16x16_imre_i(w16, POINTS, x16, z16);
    fold16(y16, z16);

    /* The real-input transform works in place on x and takes its radix and window explicitly. */
    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_fft16x16r_cn(POINTS, x16, w16, y16, 4, 0, POINTS);
    for (index = 0; index < 2 * POINTS; index++) {
        x16[index] = saved16[index];
    }
    DSP_fft16x16r_i(POINTS, x16, w16, z16, 4, 0, POINTS);
    fold16(y16, z16);

    fill32();
    for (index = 0; index < 2 * POINTS; index++) {
        saved32[index] = x32[index];
    }
    DSP_fft16x32_cn(w16, POINTS, x32, y32);
    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_fft16x32_i(w16, POINTS, x32, z32);
    fold32(y32, z32);

    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_ifft16x32_cn(w16, POINTS, x32, y32);
    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_ifft16x32_i(w16, POINTS, x32, z32);
    fold32(y32, z32);

    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_fft32x32_cn(w32, POINTS, x32, y32);
    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_fft32x32_i(w32, POINTS, x32, z32);
    fold32(y32, z32);

    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_ifft32x32_cn(w32, POINTS, x32, y32);
    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_ifft32x32_i(w32, POINTS, x32, z32);
    fold32(y32, z32);

    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_fft32x32s_cn(w32, POINTS, x32, y32);
    for (index = 0; index < 2 * POINTS; index++) {
        x32[index] = saved32[index];
    }
    DSP_fft32x32s_i(w32, POINTS, x32, z32);
    fold32(y32, z32);

    printf("dsplib_fft_intrinsics=%u matched=%u\n", total, matched);
    return 0;
}
