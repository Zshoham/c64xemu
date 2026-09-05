#ifndef LONG_SHIM_H
#define LONG_SHIM_H
/* The 40-bit long family. On the target, __int40_t lands in register pairs and every
   operation below compiles into the long forms of ADD/SUB/SHL/SHR/ABS/SAT/NORM and the
   M-unit DOTP2. A host compiler has no 40-bit type, so there the same source runs against
   an int64_t that is truncated back to 40 bits after every operation, which makes the host
   an honest oracle: the two builds can only agree if the model keeps bits 39-32 of the odd
   register exactly right. Host equivalents of the intrinsics are written from the
   instruction descriptions and worked examples in SPRU732J. */

typedef unsigned int u32;
typedef int i32;
typedef long long i64;
typedef unsigned long long u64;

#define MASK40 0xFFFFFFFFFFull
#define INT40_MAX_LL 549755813887ll
#define INT40_MIN_LL (-549755813888ll)

#if defined(__TI_COMPILER_VERSION__)

#include <c6x.h>

typedef __int40_t i40;
typedef unsigned __int40_t u40;

/* Casting a 64-bit pattern to __int40_t keeps its low 40 bits, which is exactly how a
   register pair is built. */
#define L_MAKE40(high8, low32) ((i40)((((u64)(u32)(high8)) << 32) | (u64)(u32)(low32)))
#define U_MAKE40(high8, low32) ((u40)((((u64)(u32)(high8)) << 32) | (u64)(u32)(low32)))
#define U_ADD(a, b)            ((a) + (b))

#define L_MAXC ((i40)0x7FFFFFFFFFll)
#define L_MINC ((i40)(-549755813888ll))

#define L_ADD(a, b)  ((a) + (b))
#define L_SUB(a, b)  ((a) - (b))
#define L_NEG(a)     (-(a))
#define L_MUL(a, b)  ((a) * (b))
#define L_SHL(a, n)  ((a) << (n))
#define L_SHR(a, n)  ((a) >> (n))

#define X_LABS(v)   _labs(v)
#define X_LSADD(n, v) _lsadd((n), (v))
#define X_LSSUB(n, v) _lssub((n), (v))
#define X_SAT(v)    _sat(v)
#define X_LNORM(v)  _lnorm(v)
#define X_LDOTP2(a, b) _ldotp2((a), (b))

#else

typedef i64 i40;
typedef u64 u40;

static i64 extend40(i64 v)
{
    return (i64)((((u64)v) & MASK40) ^ 0x8000000000ull) - 0x8000000000ull;
}

static i64 sat40(i64 v)
{
    if (v > INT40_MAX_LL) return INT40_MAX_LL;
    if (v < INT40_MIN_LL) return INT40_MIN_LL;
    return v;
}

#define L_MAKE40(high8, low32) extend40((i64)((((u64)(u32)(high8)) << 32) | (u64)(u32)(low32)))
#define U_MAKE40(high8, low32) ((((u64)(u32)(high8) & 0xFFull) << 32) | (u64)(u32)(low32))

/* The host has no unsigned 40-bit type, so the wrap at 2^40 is explicit. */
#define U_ADD(a, b) ((((a) + (b)) & MASK40))

#define L_MAXC INT40_MAX_LL
#define L_MINC INT40_MIN_LL

#define L_ADD(a, b) extend40((a) + (b))
#define L_SUB(a, b) extend40((a) - (b))
#define L_NEG(a)    extend40(-(a))

/* Only the low 40 bits of the product matter, so the multiply runs in 20-bit limbs to
   stay inside 64 bits even when both operands are full width; the low limb's overflow
   carries into the middle limb. */
static i40 mul40(i40 a, i40 b)
{
    u64 ua = (u64)(u40)a;
    u64 ub = (u64)(u40)b;
    u64 al = ua & 0xFFFFFull, ah = ua >> 20;
    u64 bl = ub & 0xFFFFFull, bh = ub >> 20;
    u64 p0 = al * bl;
    u64 p1 = (ah * bl + al * bh + (p0 >> 20)) & 0xFFFFFull;
    return extend40((i64)(((p1 << 20) | (p0 & 0xFFFFFull)) & MASK40));
}
#define L_MUL(a, b) mul40((a), (b))

#define L_SHL(a, n) extend40((i64)((((u64)(u40)(a))) << (n)))
#define L_SHR(a, n) ((a) >> (n))

/* ABS .L src2_h:src2_l,dst_h:dst_l (SPRU732J): negate, saturating -2^39 to 2^39-1. */
static i40 labs40(i40 v)
{
    if (v == INT40_MIN_LL) return INT40_MAX_LL;
    return v < 0 ? -v : v;
}
#define X_LABS(v) labs40(v)

/* SADD/SSUB long forms (SPRU732J): the 32-bit src1 is sign-extended and added to or
   subtracted from the 40-bit src2, saturating at the 40-bit bounds. */
#define X_LSADD(n, v) sat40(extend40((i64)(n)) + (v))
#define X_LSSUB(n, v) sat40(extend40((i64)(n)) - (v))
static i64 sat32(i64 v)
{
    if (v > 2147483647ll) return 2147483647ll;
    if (v < -2147483648ll) return -2147483648ll;
    return v;
}
/* SAT (SPRU732J): a 40-bit value clamped into 32 bits. */
#define X_SAT(v) ((i32)sat32(v))

/* NORM .L src2_h:src2_l,dst (SPRU732J): the number of redundant sign bits. The manual's
   long-form example returns 36 for the value 7, i.e. the leading run of sign-equal bits
   excluding the sign bit itself, which for a zero operand is 39 - the 40-bit datapath's
   width minus one. */
static u32 norm40(i40 v)
{
    u64 flipped = v < 0 ? ~(u64)(u40)v : (u64)(u40)v;
    u32 lz = 0;
    while (lz < 40 && !((flipped >> (39 - lz)) & 1u)) lz++;
    return lz - 1;
}
#define X_LNORM(v) norm40(v)

/* DOTP2 pair form (SPRU732J): the odd register receives msb x msb + lsb x lsb; the
   intrinsic returns that sum as a 40-bit value. Two 16x16 products cannot overflow it. */
static i40 ldotp2(i32 a, i32 b)
{
    i64 hi = (i64)(i32)(short)((u32)a >> 16) * (i64)(i32)(short)((u32)b >> 16);
    i64 lo = (i64)(i32)(short)((u32)a & 0xffffu) * (i64)(i32)(short)((u32)b & 0xffffu);
    return extend40(hi + lo);
}
#define X_LDOTP2(a, b) ldotp2((a), (b))
#endif

/* Fold a 40-bit value's high bits into a checksum word; identical arithmetic on both
   sides because both hold the same two's-complement value. */
#define FOLD40(total, v)                                                                    \
    do {                                                                                    \
        i40 fold_v_ = (v);                                                                  \
        (total) = (total) * 1000003u + (u32)((fold_v_) ^ ((fold_v_) >> 31)) * 2654435761u;  \
    } while (0)



#endif
