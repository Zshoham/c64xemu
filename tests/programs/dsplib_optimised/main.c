#include <stdio.h>
/* Differential execution: DSPLIB's hand-optimised kernels against their natural-C references,
   over identical inputs, inside one program. The optimised versions are written with the TI
   intrinsics - aligned double-word accesses, packed dot products, software-pipelined loops - so
   they exercise the parts of the model the reference implementations never reach. The host build
   cannot compile intrinsics, so there the optimised call is the reference call: the host
   therefore supplies one value that both target implementations have to produce. */

#define ALIGNED8 __attribute__((aligned(8)))
#define ALIGNED16 __attribute__((aligned(16)))

extern void DSP_autocor_cn(short *r, const short *x, int nx, int nr);
extern int DSP_dotprod_cn(const short *x, const short *y, int nx);
extern void DSP_fir_gen_cn(const short *x, const short *h, short *r, int nh, int nr);
extern short DSP_maxval_cn(const short *x, int nx);
extern int DSP_vecsumsq_cn(const short *x, int nx);
extern void DSP_w_vec_cn(const short *x, const short *y, short m, short *r, int nr);
extern void DSP_mat_trans_cn(const short *x, short rows, short columns, short *r);
extern void DSP_mul32_cn(const int *x, const int *y, int *r, short nx);
extern void DSP_add16_cn(short *x, short *y, short *r, int nx);
extern void DSP_add32_cn(const int *x, const int *y, int *r, int nx);
extern int DSP_bexp_cn(const int *x, short nx);
extern void DSP_blk_eswap16_cn(void *src, void *dst, int n_hwords);
extern void DSP_blk_eswap32_cn(void *src, void *dst, int n_words);
extern void DSP_blk_eswap64_cn(void *src, void *dst, int n_dbls);
extern void DSP_blk_move_cn(short *x, short *r, int nx);
extern void DSP_fir_r4_cn(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_r8_cn(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_sym_cn(const short *x, const short *h, short *r, int nh, int nr, int s);
extern void DSP_mat_mul_cn(const short *x, int r1, int c1, const short *y, int c2, short *r, int qs);
extern void DSP_mat_mul_cplx_cn(const short *x, int r1, int c1, const short *y, int c2, short *r, int qs);
extern int DSP_dotp_sqr_cn(int G, const short *x, const short *y, int *r, int nx);
extern void DSP_fltoq15_cn(const float *flt, short *q15, short nx);
extern void DSP_q15tofl_cn(const short *q15, float *flt, int nx);
extern int DSP_maxidx_cn(const short *x, int nx);
extern short DSP_minval_cn(const short *x, int nx);
extern void DSP_neg32_cn(const int *x, int *r, int nx);
extern void DSP_fir_cplx_cn(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_cplx_hM4X4_cn(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_gen_hM17_rA8X8_cn(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_r8_hM16_rM8A8X8_cn(const short *x, const short *h, short *r, int nh, int nr);
extern int DSP_firlms2_cn(short *h, const short *x, short b, int nh);
extern void DSP_iir_cn(short *r1, const short *x, short *r2, const short *h2, const short *h1, int nr);
extern void DSP_iir_lat_cn(short *x, int nx, const short *k, int nk, int *b, short *r);
extern short DSP_iir_ss_cn(short input, const short *coefs, int ncoefs, short *state);
extern int DSP_minerror_cn(const short *table, const short *coefficients, int *max_index);

#if defined(__TI_COMPILER_VERSION__)
extern void DSP_autocor(short *r, const short *x, int nx, int nr);
extern int DSP_dotprod(const short *x, const short *y, int nx);
extern void DSP_fir_gen(const short *x, const short *h, short *r, int nh, int nr);
extern short DSP_maxval(const short *x, int nx);
extern int DSP_vecsumsq(const short *x, int nx);
extern void DSP_w_vec(const short *x, const short *y, short m, short *r, int nr);
extern void DSP_mat_trans(const short *x, short rows, short columns, short *r);
extern void DSP_mul32(const int *x, const int *y, int *r, short nx);
extern void DSP_add16(short *x, short *y, short *r, int nx);
extern void DSP_add32(const int *x, const int *y, int *r, int nx);
extern int DSP_bexp(const int *x, short nx);
extern void DSP_blk_eswap16(void *src, void *dst, int n_hwords);
extern void DSP_blk_eswap32(void *src, void *dst, int n_words);
extern void DSP_blk_eswap64(void *src, void *dst, int n_dbls);
extern void DSP_blk_move(short *x, short *r, int nx);
extern void DSP_fir_r4(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_r8(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_sym(const short *x, const short *h, short *r, int nh, int nr, int s);
extern void DSP_mat_mul(const short *x, int r1, int c1, const short *y, int c2, short *r, int qs);
extern void DSP_mat_mul_cplx(const short *x, int r1, int c1, const short *y, int c2, short *r, int qs);
extern int DSP_dotp_sqr(int G, const short *x, const short *y, int *r, int nx);
extern void DSP_fltoq15(const float *flt, short *q15, short nx);
extern void DSP_q15tofl(const short *q15, float *flt, int nx);
extern int DSP_maxidx(const short *x, int nx);
extern short DSP_minval(const short *x, int nx);
extern void DSP_neg32(const int *x, int *r, int nx);
extern void DSP_recip16(const short *x, short *rfrac, short *rexp, short nx);
extern void DSP_fir_cplx(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_cplx_hM4X4(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_gen_hM17_rA8X8(const short *x, const short *h, short *r, int nh, int nr);
extern void DSP_fir_r8_hM16_rM8A8X8(const short *x, const short *h, short *r, int nh, int nr);
extern int DSP_firlms2(short *h, short *x, short b, int nh);
extern void DSP_iir(short *r1, const short *x, short *r2, const short *h2, const short *h1, int nr);
extern void DSP_iir_lat(short *x, int nx, const short *k, int nk, int *b, short *r);
extern short DSP_iir_ss(short input, const short *coefs, int ncoefs, short *state);
extern int DSP_minerror(const short *table, const short *coefficients, int *max_index);
#else
#define DSP_autocor   DSP_autocor_cn
#define DSP_dotprod   DSP_dotprod_cn
#define DSP_fir_gen   DSP_fir_gen_cn
#define DSP_maxval    DSP_maxval_cn
#define DSP_vecsumsq  DSP_vecsumsq_cn
#define DSP_w_vec     DSP_w_vec_cn
#define DSP_mat_trans DSP_mat_trans_cn
#define DSP_mul32     DSP_mul32_cn
#define DSP_add16       DSP_add16_cn
#define DSP_add32       DSP_add32_cn
#define DSP_bexp        DSP_bexp_cn
#define DSP_blk_eswap16 DSP_blk_eswap16_cn
#define DSP_blk_eswap32 DSP_blk_eswap32_cn
#define DSP_blk_eswap64 DSP_blk_eswap64_cn
#define DSP_blk_move    DSP_blk_move_cn
#define DSP_fir_r4       DSP_fir_r4_cn
#define DSP_fir_r8       DSP_fir_r8_cn
#define DSP_fir_sym      DSP_fir_sym_cn
#define DSP_mat_mul      DSP_mat_mul_cn
#define DSP_mat_mul_cplx DSP_mat_mul_cplx_cn
#define DSP_dotp_sqr     DSP_dotp_sqr_cn
#define DSP_fltoq15 DSP_fltoq15_cn
#define DSP_q15tofl DSP_q15tofl_cn
#define DSP_maxidx  DSP_maxidx_cn
#define DSP_minval  DSP_minval_cn
#define DSP_neg32   DSP_neg32_cn
#define DSP_fir_cplx              DSP_fir_cplx_cn
#define DSP_fir_cplx_hM4X4        DSP_fir_cplx_hM4X4_cn
#define DSP_fir_gen_hM17_rA8X8    DSP_fir_gen_hM17_rA8X8_cn
#define DSP_fir_r8_hM16_rM8A8X8  DSP_fir_r8_hM16_rM8A8X8_cn
#define DSP_firlms2 DSP_firlms2_cn
#define DSP_iir DSP_iir_cn
#define DSP_iir_lat DSP_iir_lat_cn
#define DSP_iir_ss DSP_iir_ss_cn
#define DSP_minerror DSP_minerror_cn
#define DSP_recip16 recip16_reference
#endif

static unsigned int rng_state = 1234567u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

#define NX 128
#define NR 32
#define NH 16

static short x[NX + NR] ALIGNED8;
static short h[NH + 1] ALIGNED8;
static short reference[NX] ALIGNED8;
static short optimised[NX] ALIGNED8;
static int wide_x[NX + 2] ALIGNED8;
static int wide_y[NX] ALIGNED8;
static int wide_reference[NX] ALIGNED8;
static int wide_optimised[NX] ALIGNED8;
static short matrix_y[NX] ALIGNED8;
static short extrema_input[NX] ALIGNED16;
static short exponent_reference[NX] ALIGNED8;
static short exponent_optimised[NX] ALIGNED8;
static float float_input[NX] ALIGNED8;
static float float_reference[NX] ALIGNED8;
static float float_optimised[NX] ALIGNED8;
static short firlms_reference[NH] ALIGNED8;
static short firlms_optimised[NH] ALIGNED8;
static const short iir_h1[5] ALIGNED8 = { 0, 1024, -512, 256, -128 };
static const short iir_h2[5] ALIGNED8 = { 8192, 4096, 2048, 1024, 512 };
static short iir_state_reference[NR + 4] ALIGNED8;
static short iir_state_optimised[NR + 4] ALIGNED8;
static const short lattice_k[8] ALIGNED8 =
    { 1024, -768, 640, -512, 384, -256, 192, -128 };
static int lattice_state_reference[9] ALIGNED8;
static int lattice_state_optimised[9] ALIGNED8;
static const short iir_ss_coefficients[8] ALIGNED8 =
    { 1024, -512, 768, -384, 640, -320, 512, -256 };
static short iir_ss_state_reference[4] ALIGNED8;
static short iir_ss_state_optimised[4] ALIGNED8;
static short minerror_codebook[256 * 9] ALIGNED8;
static const short minerror_coefficients[9] ALIGNED8 =
    { 1, 2, 3, 4, 5, 6, 7, 8, 9 };

static unsigned int fold(unsigned int total, const short *values, int count)
{
    int i;
    for (i = 0; i < count; i++) {
        total = total * 31u + (unsigned int)values[i];
    }
    return total;
}

static unsigned int fold_wide(unsigned int total, const int *values, int count)
{
    int i;
    for (i = 0; i < count; i++) {
        total = total * 31u + (unsigned int)values[i];
    }
    return total;
}

static unsigned int fold_float(unsigned int total, const float *values, int count)
{
    int i;
    union {
        float f;
        unsigned int u;
    } bits;

    for (i = 0; i < count; i++) {
        bits.f = values[i];
        total = total * 31u + bits.u;
    }
    return total;
}

static void recip16_reference(const short *input, short *fraction, short *exponent, short count)
{
    int i;

    for (i = 0; i < count; i++) {
        int signed_value = input[i];
        unsigned int magnitude = signed_value < 0
            ? (unsigned int)-signed_value
            : (unsigned int)signed_value;
        unsigned int normalized = magnitude;
        unsigned int quotient = 0x80000000u;
        int normal = 0;
        int bit;

        while ((normalized & 0x40000000u) == 0) {
            normalized <<= 1;
            normal++;
        }
        exponent[i] = (short)(normal - 15);
        for (bit = 0; bit < 15; bit++) {
            quotient = quotient >= normalized
                ? ((quotient - normalized) << 1) | 1u
                : quotient << 1;
        }
        fraction[i] = (short)(quotient & 0x7fffu);
        if (signed_value < 0) fraction[i] = (short)-fraction[i];
    }
}

int main(void)
{
    unsigned int ref = 0;
    unsigned int fast = 0;
    int i;
    int ref_index;
    int fast_index;

    for (i = 0; i < NX + NR; i++) x[i] = (short)((int)(rng_next() >> 21) - 1024);
    for (i = 0; i < NH; i++) h[i] = (short)((int)(rng_next() >> 23) - 256);
    h[NH] = 37;
    for (i = 0; i < NX; i++) wide_x[i] = (int)(rng_next() >> 12) - 262144;
    for (i = 0; i < NX; i++) wide_y[i] = (int)(rng_next() >> 12) - 262144;
    for (i = 0; i < NX; i++) matrix_y[i] = (short)((int)(rng_next() >> 21) - 1024);
    for (i = 0; i < 256 * 9; i++)
        minerror_codebook[i] = (short)(((i * 73 + 19) % 128) - 64);
    for (i = 0; i < 9; i++)
        minerror_codebook[137 * 9 + i] = 20000;
    for (i = 0; i < NX; i++) {
        extrema_input[i] = x[i];
        if (matrix_y[i] == 0) matrix_y[i] = 1;
        float_input[i] = (float)matrix_y[i] / 32768.0f;
    }
    extrema_input[17] = 30000;
    wide_x[NX] = 0;
    wide_x[NX + 1] = 0;

    /* Autocorrelation: the optimised version reads the input eight halfwords at a time. */
    DSP_autocor_cn(reference, x, NX, NR);
    DSP_autocor(optimised, x, NX, NR);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);

    /* Dot product of the same vector against itself, shifted. */
    ref = ref * 31u + (unsigned int)DSP_dotprod_cn(x, x + NR, NX);
    fast = fast * 31u + (unsigned int)DSP_dotprod(x, x + NR, NX);

    /* A general FIR over the whole input. */
    DSP_fir_gen_cn(x, h, reference, NH, NR);
    DSP_fir_gen(x, h, optimised, NH, NR);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);

    /* Reductions. */
    ref = ref * 31u + (unsigned int)DSP_maxval_cn(x, NX);
    fast = fast * 31u + (unsigned int)DSP_maxval(x, NX);
    ref = ref * 31u + (unsigned int)DSP_vecsumsq_cn(x, NX);
    fast = fast * 31u + (unsigned int)DSP_vecsumsq(x, NX);

    /* A weighted vector sum and a matrix transpose. */
    DSP_w_vec_cn(x, x + NR, 300, reference, NX);
    DSP_w_vec(x, x + NR, 300, optimised, NX);
    ref = fold(ref, reference, NX);
    fast = fold(fast, optimised, NX);
    DSP_mat_trans_cn(x, 8, 16, reference);
    DSP_mat_trans(x, 8, 16, optimised);
    ref = fold(ref, reference, 128);
    fast = fold(fast, optimised, 128);

    /* 32-bit by 32-bit multiply into the upper word. */
    DSP_mul32_cn(wide_x, wide_y, wide_reference, NX);
    DSP_mul32(wide_x, wide_y, wide_optimised, NX);
    ref = fold_wide(ref, wide_reference, NX);
    fast = fold_wide(fast, wide_optimised, NX);

    /* Elementwise vector operations and block exponent reduction. */
    DSP_add16_cn(x, x + NR, reference, NX);
    DSP_add16(x, x + NR, optimised, NX);
    ref = fold(ref, reference, NX);
    fast = fold(fast, optimised, NX);
    DSP_add32_cn(wide_x, wide_y, wide_reference, NX);
    DSP_add32(wide_x, wide_y, wide_optimised, NX);
    ref = fold_wide(ref, wide_reference, NX);
    fast = fold_wide(fast, wide_optimised, NX);
    ref = ref * 31u + (unsigned int)DSP_bexp_cn(wide_x, NX);
    fast = fast * 31u + (unsigned int)DSP_bexp(wide_x, NX);

    /* Out-of-place endian swaps at every supported element width. */
    DSP_blk_eswap16_cn(x, reference, NX);
    DSP_blk_eswap16(x, optimised, NX);
    ref = fold(ref, reference, NX);
    fast = fold(fast, optimised, NX);
    DSP_blk_eswap32_cn(wide_x, wide_reference, NX);
    DSP_blk_eswap32(wide_x, wide_optimised, NX);
    ref = fold_wide(ref, wide_reference, NX);
    fast = fold_wide(fast, wide_optimised, NX);
    DSP_blk_eswap64_cn(wide_x, wide_reference, NX / 2);
    DSP_blk_eswap64(wide_x, wide_optimised, NX / 2);
    ref = fold_wide(ref, wide_reference, NX);
    fast = fold_wide(fast, wide_optimised, NX);

    DSP_blk_move_cn(x, reference, NX);
    DSP_blk_move(x, optimised, NX);
    ref = fold(ref, reference, NX);
    fast = fold(fast, optimised, NX);

    /* Four-output FIRs and a symmetric 33-tap FIR. */
    DSP_fir_r4_cn(x, h, reference, NH, NR);
    DSP_fir_r4(x, h, optimised, NH, NR);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);
    DSP_fir_r8_cn(x, h, reference, NH, NR);
    DSP_fir_r8(x, h, optimised, NH, NR);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);
    DSP_fir_sym_cn(x, h, reference, NH, NR, 12);
    DSP_fir_sym(x, h, optimised, NH, NR, 12);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);

    /* Dense real and complex matrix multiplication. */
    DSP_mat_mul_cn(x, 4, 8, matrix_y, 8, reference, 8);
    DSP_mat_mul(x, 4, 8, matrix_y, 8, optimised, 8);
    ref = fold(ref, reference, 32);
    fast = fold(fast, optimised, 32);
    DSP_mat_mul_cplx_cn(x, 4, 4, matrix_y, 4, reference, 8);
    DSP_mat_mul_cplx(x, 4, 4, matrix_y, 4, optimised, 8);
    ref = fold(ref, reference, 32);
    fast = fold(fast, optimised, 32);

    wide_reference[0] = 123;
    wide_optimised[0] = 123;
    ref = ref * 31u + (unsigned int)DSP_dotp_sqr_cn(456, x, matrix_y, wide_reference, NX);
    fast = fast * 31u + (unsigned int)DSP_dotp_sqr(456, x, matrix_y, wide_optimised, NX);
    ref = ref * 31u + (unsigned int)wide_reference[0];
    fast = fast * 31u + (unsigned int)wide_optimised[0];

    /* Fixed-point/float conversion, extrema, negation, and reciprocal. */
    DSP_fltoq15_cn(float_input, reference, NX);
    DSP_fltoq15(float_input, optimised, NX);
    ref = fold(ref, reference, NX);
    fast = fold(fast, optimised, NX);
    DSP_q15tofl_cn(x, float_reference, NX);
    DSP_q15tofl(x, float_optimised, NX);
    ref = fold_float(ref, float_reference, NX);
    fast = fold_float(fast, float_optimised, NX);
    ref = ref * 31u + (unsigned int)DSP_maxidx_cn(extrema_input, NX);
    fast = fast * 31u + (unsigned int)DSP_maxidx(extrema_input, NX);
    ref = ref * 31u + (unsigned int)DSP_minval_cn(x, NX);
    fast = fast * 31u + (unsigned int)DSP_minval(x, NX);
    DSP_neg32_cn(wide_x, wide_reference, NX);
    DSP_neg32(wide_x, wide_optimised, NX);
    ref = fold_wide(ref, wide_reference, NX);
    fast = fold_wide(fast, wide_optimised, NX);
    recip16_reference(matrix_y, reference, exponent_reference, NX);
    DSP_recip16(matrix_y, optimised, exponent_optimised, NX);
    ref = fold(ref, reference, NX);
    fast = fold(fast, optimised, NX);
    ref = fold(ref, exponent_reference, NX);
    fast = fold(fast, exponent_optimised, NX);

    /* Complex and fixed-dimension FIR variants. */
    DSP_fir_cplx_cn(x + 2 * (NH - 1), matrix_y, reference, NH, NR);
    DSP_fir_cplx(x + 2 * (NH - 1), matrix_y, optimised, NH, NR);
    ref = fold(ref, reference, 2 * NR);
    fast = fold(fast, optimised, 2 * NR);
    DSP_fir_cplx_hM4X4_cn(x + 2 * (NH - 1), matrix_y, reference, NH, NR);
    DSP_fir_cplx_hM4X4(x + 2 * (NH - 1), matrix_y, optimised, NH, NR);
    ref = fold(ref, reference, 2 * NR);
    fast = fold(fast, optimised, 2 * NR);
    DSP_fir_gen_hM17_rA8X8_cn(x, h, reference, NH, NR);
    DSP_fir_gen_hM17_rA8X8(x, h, optimised, NH, NR);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);
    DSP_fir_r8_hM16_rM8A8X8_cn(x, h, reference, NH, NR);
    DSP_fir_r8_hM16_rM8A8X8(x, h, optimised, NH, NR);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);

    for (i = 0; i < NH; i++) {
        firlms_reference[i] = h[i];
        firlms_optimised[i] = h[i];
    }
    ref = ref * 31u + (unsigned int)DSP_firlms2_cn(firlms_reference, x, 123, NH);
    fast = fast * 31u + (unsigned int)DSP_firlms2(firlms_optimised, x, 123, NH);
    ref = fold(ref, firlms_reference, NH);
    fast = fold(fast, firlms_optimised, NH);

    DSP_iir_cn(iir_state_reference, x, reference, iir_h2, iir_h1, NR);
    DSP_iir(iir_state_optimised, x, optimised, iir_h2, iir_h1, NR);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);
    ref = fold(ref, iir_state_reference, NR + 4);
    fast = fold(fast, iir_state_optimised, NR + 4);

    /* The optimized kernel keeps its delay state at Q16 while natural C uses Q15.
       Compare outputs over consecutive calls rather than representation-private state. */
    DSP_iir_lat_cn(x, NR, lattice_k, 8, lattice_state_reference, reference);
    DSP_iir_lat(x, NR, lattice_k, 8, lattice_state_optimised, optimised);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);
    DSP_iir_lat_cn(x + NR, NR, lattice_k, 8, lattice_state_reference, reference);
    DSP_iir_lat(x + NR, NR, lattice_k, 8, lattice_state_optimised, optimised);
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);

    for (i = 0; i < NR; i++) {
        reference[i] = DSP_iir_ss_cn(x[i], iir_ss_coefficients, 8, iir_ss_state_reference);
        optimised[i] = DSP_iir_ss(x[i], iir_ss_coefficients, 8, iir_ss_state_optimised);
    }
    ref = fold(ref, reference, NR);
    fast = fold(fast, optimised, NR);

    ref_index = -1;
    fast_index = -1;
    ref = ref * 31u +
        (unsigned int)DSP_minerror_cn(minerror_codebook, minerror_coefficients, &ref_index);
    fast = fast * 31u +
        (unsigned int)DSP_minerror(minerror_codebook, minerror_coefficients, &fast_index);
    ref = ref * 31u + (unsigned int)ref_index;
    fast = fast * 31u + (unsigned int)fast_index;

    printf("dsplib_optimised=%u fast=%u\n", ref, fast);
    return 0;
}
