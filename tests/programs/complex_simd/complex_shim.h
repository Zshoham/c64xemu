#ifndef COMPLEX_SHIM_H
#define COMPLEX_SHIM_H
/* The C64x+ complex-multiply family, with plain C equivalents for a host compiler. Each
   equivalent is written from the instruction descriptions in SPRUFE8B and reproduces that
   instruction's worked examples. */

typedef unsigned int u32;
typedef int i32;
typedef unsigned long long u64;
typedef long long i64;

#if defined(__TI_COMPILER_VERSION__)
#include <c6x.h>
#define X_CMPY(a, b)          _cmpy((a), (b))
#define X_CMPYR(a, b)         _cmpyr((a), (b))
#define X_CMPYR1(a, b)        _cmpyr1((a), (b))
#define X_DDOTP4(a, b)        _ddotp4((a), (b))
#define X_DDOTPH2(a, b)       _ddotph2((a), (b))
#define X_DDOTPH2R(a, b)      _ddotph2r((a), (b))
#define X_DDOTPL2(a, b)       _ddotpl2((a), (b))
#define X_DDOTPL2R(a, b)      _ddotpl2r((a), (b))
#define X_MPY2IR(a, b)        _mpy2ir((a), (b))
#define X_SMPY32(a, b)        _smpy32((a), (b))
#define X_MPYHIR(a, b)        _mpyhir((a), (b))
#define X_MPYLIR(a, b)        _mpylir((a), (b))
#else
static i32 half_low(u32 v) { return (i32)(short)(v & 0xffffu); }
static i32 half_high(u32 v) { return (i32)(short)(v >> 16); }

static i32 saturate32(i64 value)
{
    if (value > 2147483647ll) return (i32)2147483647ll;
    if (value < -2147483648ll) return (i32)(-2147483648ll);
    return (i32)value;
}

static u64 pair(u32 even, u32 odd)
{
    return ((u64)odd << 32) | even;
}

static u64 host_cmpy(u32 a, u32 b)
{
    i32 odd = saturate32((i64)half_high(a) * half_high(b) - (i64)half_low(a) * half_low(b));
    i32 even = saturate32((i64)half_high(a) * half_low(b) + (i64)half_low(a) * half_high(b));
    return pair((u32)even, (u32)odd);
}

static u32 host_cmpyr(u32 a, u32 b, int one)
{
    i64 round = one ? 0x4000 : 0x8000;
    i64 real = (i64)half_high(a) * half_high(b) - (i64)half_low(a) * half_low(b);
    i64 imag = (i64)half_high(a) * half_low(b) + (i64)half_low(a) * half_high(b);
    i32 r = saturate32(real + round);
    i32 i = saturate32(imag + round);
    if (one) {
        r = saturate32((i64)r << 1);
        i = saturate32((i64)i << 1);
    }
    return ((u32)(r >> 16) << 16) | ((u32)(i >> 16) & 0xffffu);
}

static u64 host_ddotp4(u32 a, u32 b)
{
    i32 byte0 = (signed char)(b & 0xffu);
    i32 byte1 = (signed char)((b >> 8) & 0xffu);
    i32 byte2 = (signed char)((b >> 16) & 0xffu);
    i32 byte3 = (signed char)(b >> 24);
    i32 even = half_high(a) * byte1 + half_low(a) * byte0;
    i32 odd = half_high(a) * byte3 + half_low(a) * byte2;
    return pair((u32)even, (u32)odd);
}

static u64 host_ddotp2(u64 src1, u32 b, int high_form)
{
    u32 even_word = (u32)src1;
    u32 odd_word = (u32)(src1 >> 32);
    i64 first = high_form
        ? (i64)half_low(odd_word) * half_low(b) + (i64)half_high(odd_word) * half_high(b)
        : (i64)half_low(even_word) * half_low(b) + (i64)half_high(even_word) * half_high(b);
    i64 second = (i64)half_high(b) * half_low(odd_word) + (i64)half_low(b) * half_high(even_word);
    if (high_form) {
        return pair((u32)saturate32(second), (u32)saturate32(first));
    }
    return pair((u32)saturate32(first), (u32)saturate32(second));
}

static u32 host_ddotp2r(u64 src1, u32 b, int high_form)
{
    u32 even_word = (u32)src1;
    u32 odd_word = (u32)(src1 >> 32);
    i64 first = high_form
        ? (i64)half_low(odd_word) * half_low(b) + (i64)half_high(odd_word) * half_high(b)
        : (i64)half_low(even_word) * half_low(b) + (i64)half_high(even_word) * half_high(b);
    i64 second = (i64)half_high(b) * half_low(odd_word) + (i64)half_low(b) * half_high(even_word);
    i32 a1 = saturate32(first + 0x8000);
    i32 a2 = saturate32(second + 0x8000);
    if (high_form) {
        return ((u32)(a1 >> 16) << 16) | ((u32)(a2 >> 16) & 0xffffu);
    }
    return ((u32)(a2 >> 16) << 16) | ((u32)(a1 >> 16) & 0xffffu);
}

static u64 host_mpy2ir(u32 a, u32 b)
{
    i32 even = saturate32(((i64)half_low(a) * (i32)b + 0x4000) >> 15);
    i32 odd = saturate32(((i64)half_high(a) * (i32)b + 0x4000) >> 15);
    return pair((u32)even, (u32)odd);
}

static u32 host_smpy32(u32 a, u32 b)
{
    i64 product = (i64)(i32)a * (i32)b;
    return (u32)saturate32((product * 2) >> 32);
}

static u32 host_mpyir(u32 a, u32 b, int high)
{
    i32 half = high ? half_high(a) : half_low(a);
    return (u32)(i32)(((i64)half * (i32)b + 0x4000) >> 15);
}

#define X_CMPY(a, b)          host_cmpy((a), (b))
#define X_CMPYR(a, b)         host_cmpyr((a), (b), 0)
#define X_CMPYR1(a, b)        host_cmpyr((a), (b), 1)
#define X_DDOTP4(a, b)        host_ddotp4((a), (b))
#define X_DDOTPH2(a, b)       host_ddotp2((a), (b), 1)
#define X_DDOTPH2R(a, b)      host_ddotp2r((a), (b), 1)
#define X_DDOTPL2(a, b)       host_ddotp2((a), (b), 0)
#define X_DDOTPL2R(a, b)      host_ddotp2r((a), (b), 0)
#define X_MPY2IR(a, b)        host_mpy2ir((a), (b))
#define X_SMPY32(a, b)        host_smpy32((a), (b))
#define X_MPYHIR(a, b)        host_mpyir((a), (b), 1)
#define X_MPYLIR(a, b)        host_mpyir((a), (b), 0)
#endif

#endif
