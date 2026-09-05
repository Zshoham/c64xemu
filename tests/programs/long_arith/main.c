#include <stdio.h>
#include "long_shim.h"

/* 40-bit long arithmetic: plain __int40_t expressions drive the compiler's ADD/SUB/SHL/
   SHR/MPY long forms over register pairs, and the intrinsics drive ABS/SAT/NORM/SADD/SSUB
   and the M-unit DOTP2 directly. Values live in arrays so every operand round-trips through
   an 8-byte load/store and the odd register's bits 39-32 are read back from memory. The
   constant is produced by a host build of this exact source, where the shim holds each
   value in an int64_t and truncates it to 40 bits after every operation. */

static u32 rng_state = 0x13579BDFu;

static u32 rng_next(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

#define N 24

static i40 va[N];
static i40 vb[N];
static u40 vu[N];

int main(void)
{
    u32 total = 0x12345678u;
    i40 acc;
    u40 uacc;
    int i;

    for (i = 0; i < N; i++) {
        va[i] = L_MAKE40(rng_next(), rng_next());
        vb[i] = L_MAKE40(rng_next(), rng_next());
        vu[i] = U_MAKE40(rng_next(), rng_next());
    }

    /* Add/subtract chains through memory, folding after every step so any single wrong
       bit in the odd register poisons the checksum. */
    acc = L_MAKE40(0, 0);
    for (i = 0; i < N; i++) {
        acc = L_ADD(acc, va[i]);
        acc = L_SUB(acc, vb[i]);
        FOLD40(total, acc);
    }

    /* Negation and full-width multiplication, which only agrees mod 2^40 if the model's
       pair products are combined from the right halves. */
    for (i = 0; i < N; i++) {
        acc = L_NEG(va[i]);
        FOLD40(total, acc);
        acc = L_MUL(va[i], vb[i]);
        FOLD40(total, acc);
        acc = L_MUL(acc, (i40)(i + 3));
        FOLD40(total, acc);
    }

    /* Variable shifts in both directions, including counts that push bits out of the top
       of the pair and arithmetic shifts that pull sign bits down into the even register. */
    acc = va[0];
    for (i = 0; i < N; i++) {
        u32 up = rng_next() & 7u;
        u32 down = rng_next() & 15u;
        acc = L_SHL(acc, up);
        FOLD40(total, acc);
        acc = L_SHR(acc, down);
        FOLD40(total, acc);
    }

    /* Comparisons that read a pair but write one register, plus a selection that makes
       the result observable beyond a count. */
    {
        i40 biggest = L_MINC;
        u32 greater = 0;
        u32 equal = 0;
        for (i = 0; i < N; i++) {
            greater += (va[i] > vb[i]);
            equal += (va[i] == vb[i]);
            if (va[i] > biggest) biggest = va[i];
        }
        total += greater * 31u + equal * 17u;
        FOLD40(total, biggest);
        FOLD40(total, L_MAXC);
        FOLD40(total, L_MINC);
    }

    /* The intrinsic family at the corners: saturation boundaries, zero and -1 under
       NORM, INT40_MIN under ABS, saturating add/sub of ints onto longs. */
    {
        static const i32 k = -7919;
        i40 edges[6];
        edges[0] = L_MAXC;
        edges[1] = L_MINC;
        edges[2] = L_MAKE40(0, 0);
        edges[3] = (i40)(-1);
        edges[4] = L_MAKE40(0xFF, 0xDEADBEEFu);
        edges[5] = L_MAKE40(0x80, 0x12458796u);
        for (i = 0; i < 6; i++) {
            i40 a = edges[i];
            /* TI's own codegen realizes the long SSUB as a saturating NEG followed by
               SADD, which loses one ulp when the subtrahend is exactly INT40_MIN; that
               corner is undefined between the intrinsic's contract and its compiled
               form, so it is excluded and INT40_MAX stands in. */
            i40 b = edges[(5 - i) == 1 ? 0 : 5 - i];
            total ^= (u32)X_LNORM(a);
            total += (u32)X_SAT(a);
            FOLD40(total, X_LABS(a));
            FOLD40(total, X_LSADD(k * (i + 1), a));
            FOLD40(total, X_LSSUB(k * (i + 1), b));
        }
        for (i = 0; i < N; i++) {
            total = total * 1000003u + (u32)X_LNORM(va[i]) * 97u + (u32)X_SAT(vb[i]);
            acc = L_ADD(acc, X_LABS(va[i]));
            acc = X_LSADD((int)(rng_next() >> 24) - 128, acc);
        }
        FOLD40(total, acc);
    }

    /* The M-unit long dot product: two 16x16 products summed into a pair, accumulated
       both as a 40-bit running sum and folded per step. */
    for (i = 0; i < N; i++) {
        i40 dot = X_LDOTP2((int)va[i], (int)vb[i]);
        FOLD40(total, dot);
        acc = L_ADD(acc, dot);
    }

    /* Unsigned 40-bit: logical right shifts, wrapping multiplication and unsigned
       comparisons never touch the sign bit the way the signed forms do. */
    uacc = U_MAKE40(0, 0);
    for (i = 0; i < N; i++) {
        vu[i] >>= rng_next() & 15u;
        uacc = U_ADD(uacc, vu[i]);
        total ^= (u32)vu[i] + (u32)(vu[i] >> 18);
        total += (vu[i] > uacc);
    }
    total += (u32)uacc ^ (u32)(uacc >> 31);

    printf("long_arith=%08x\n", total);
    return 0;
}
