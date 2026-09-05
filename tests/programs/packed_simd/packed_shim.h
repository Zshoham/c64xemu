#ifndef PACKED_SHIM_H
#define PACKED_SHIM_H
/* The C64x+ packed intrinsics, with plain C equivalents for a host compiler. Each equivalent
   is written from the instruction descriptions in SPRUFE8B, so the host and the target agree
   only if the emulator implements the instruction the way the manual defines it. */

#include <limits.h>

typedef unsigned int u32;
typedef int i32;
typedef unsigned long long u64;
typedef long long i64;

static short lane_low(u32 v)  { return (short)(v & 0xffff); }
static short lane_high(u32 v) { return (short)(v >> 16); }
static u32 pack_lanes(short high, short low)
{
    return ((u32)(unsigned short)high << 16) | (u32)(unsigned short)low;
}
static short sat_short(i32 v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (short)v;
}
static unsigned char sat_uchar(i32 v)
{
    if (v > 255) return 255;
    if (v < 0) return 0;
    return (unsigned char)v;
}

#if defined(__TI_COMPILER_VERSION__)
#include <c6x.h>
#define P_ADD2(a, b)     _add2((a), (b))
#define P_SUB2(a, b)     _sub2((a), (b))
#define P_SADD2(a, b)    _sadd2((a), (b))
#define P_SSUB2(a, b)    _ssub2((a), (b))
#define P_MAX2(a, b)     _max2((a), (b))
#define P_MIN2(a, b)     _min2((a), (b))
#define P_AVG2(a, b)     _avg2((a), (b))
#define P_AVGU4(a, b)    _avgu4((a), (b))
#define P_ADD4(a, b)     _add4((a), (b))
#define P_SUB4(a, b)     _sub4((a), (b))
#define P_MAXU4(a, b)    _maxu4((a), (b))
#define P_MINU4(a, b)    _minu4((a), (b))
#define P_SUBABS4(a, b)  _subabs4((a), (b))
#define P_PACK2(a, b)    _pack2((a), (b))
#define P_PACKH2(a, b)   _packh2((a), (b))
#define P_PACKLH2(a, b)  _packlh2((a), (b))
#define P_PACKH4(a, b)   _packh4((a), (b))
#define P_PACKL4(a, b)   _packl4((a), (b))
#define P_SPACK2(a, b)   _spack2((a), (b))
#define P_SPACKU4(a, b)  _spacku4((a), (b))
#define P_DOTP2(a, b)    _dotp2((a), (b))
#define P_DOTPN2(a, b)   _dotpn2((a), (b))
#define P_DOTPU4(a, b)   _dotpu4((a), (b))
#define P_DOTPSU4(a, b)  _dotpsu4((a), (b))
#define P_MPY2LL(a, b)   _mpy2ll((a), (b))
#define P_SHR2(a, b)     _shr2((a), (b))
#define P_SHRU2(a, b)    _shru2((a), (b))
#define P_SHLMB(a, b)    _shlmb((a), (b))
#define P_SHRMB(a, b)    _shrmb((a), (b))
#define P_SWAP4(a)       _swap4((a))
#define P_ROTL(a, b)     _rotl((a), (b))
#define P_BITR(a)        _bitr((a))
#define P_BITC4(a)       _bitc4((a))
#define P_DEAL(a)        _deal((a))
#define P_SHFL(a)        _shfl((a))
#define P_XPND2(a)       _xpnd2((a))
#define P_XPND4(a)       _xpnd4((a))
#define P_ABS2(a)        _abs2((a))
#define P_SSHVL(a, b)    _sshvl((a), (b))
#define P_SSHVR(a, b)    _sshvr((a), (b))
#define P_GMPY4(a, b)    _gmpy4((a), (b))
#define P_XORMPY(a, b)   _xormpy((a), (b))
#define P_CMPEQ2(a, b)   _cmpeq2((a), (b))
#define P_CMPGT2(a, b)   _cmpgt2((a), (b))
#define P_CMPEQ4(a, b)   _cmpeq4((a), (b))
#define P_CMPGTU4(a, b)  _cmpgtu4((a), (b))
#define P_NORM(a)        _norm((a))
#define P_SADD(a, b)     _sadd((a), (b))
#define P_SSUB(a, b)     _ssub((a), (b))
#define P_ABS(a)         _abs((a))
#define P_LMBD(a, b)     _lmbd((a), (b))
#else
static u32 host_add2(u32 a, u32 b)
{
    return pack_lanes((short)(lane_high(a) + lane_high(b)), (short)(lane_low(a) + lane_low(b)));
}
static u32 host_sub2(u32 a, u32 b)
{
    return pack_lanes((short)(lane_high(a) - lane_high(b)), (short)(lane_low(a) - lane_low(b)));
}
static u32 host_sadd2(u32 a, u32 b)
{
    return pack_lanes(sat_short((i32)lane_high(a) + lane_high(b)),
                      sat_short((i32)lane_low(a) + lane_low(b)));
}
static u32 host_ssub2(u32 a, u32 b)
{
    return pack_lanes(sat_short((i32)lane_high(a) - lane_high(b)),
                      sat_short((i32)lane_low(a) - lane_low(b)));
}
static u32 host_max2(u32 a, u32 b)
{
    short h = lane_high(a) > lane_high(b) ? lane_high(a) : lane_high(b);
    short l = lane_low(a) > lane_low(b) ? lane_low(a) : lane_low(b);
    return pack_lanes(h, l);
}
static u32 host_min2(u32 a, u32 b)
{
    short h = lane_high(a) < lane_high(b) ? lane_high(a) : lane_high(b);
    short l = lane_low(a) < lane_low(b) ? lane_low(a) : lane_low(b);
    return pack_lanes(h, l);
}
static u32 host_avg2(u32 a, u32 b)
{
    return pack_lanes((short)(((i32)lane_high(a) + lane_high(b) + 1) >> 1),
                      (short)(((i32)lane_low(a) + lane_low(b) + 1) >> 1));
}
static u32 host_bytes(u32 a, u32 b, int op)
{
    u32 result = 0;
    int lane;
    for (lane = 0; lane < 4; lane++) {
        unsigned char x = (unsigned char)(a >> (lane * 8));
        unsigned char y = (unsigned char)(b >> (lane * 8));
        unsigned char v;
        switch (op) {
            case 0: v = (unsigned char)(x + y); break;
            case 1: v = (unsigned char)(x - y); break;
            case 2: v = x > y ? x : y; break;
            case 3: v = x < y ? x : y; break;
            case 4: v = (unsigned char)(x > y ? x - y : y - x); break;
            default: v = (unsigned char)((x + y + 1) >> 1); break;
        }
        result |= (u32)v << (lane * 8);
    }
    return result;
}
static u32 host_shr2(u32 a, u32 count)
{
    unsigned int c = count & 0x1f;
    if (c > 16u) c = 16u;
    return pack_lanes((short)(lane_high(a) >> c), (short)(lane_low(a) >> c));
}
static u32 host_shru2(u32 a, u32 count)
{
    unsigned int c = count & 0x1f;
    if (c > 16u) c = 16u;
    return pack_lanes((short)((unsigned short)(a >> 16) >> c),
                      (short)((unsigned short)(a & 0xffff) >> c));
}
static i32 host_dotp2(u32 a, u32 b)
{
    return (i32)lane_low(a) * lane_low(b) + (i32)lane_high(a) * lane_high(b);
}
static i32 host_dotpn2(u32 a, u32 b)
{
    return (i32)lane_high(a) * lane_high(b) - (i32)lane_low(a) * lane_low(b);
}
static u32 host_dotpu4(u32 a, u32 b)
{
    u32 total = 0;
    int lane;
    for (lane = 0; lane < 4; lane++) {
        total += (u32)(unsigned char)(a >> (lane * 8)) * (unsigned char)(b >> (lane * 8));
    }
    return total;
}
static i32 host_dotpsu4(u32 a, u32 b)
{
    i32 total = 0;
    int lane;
    for (lane = 0; lane < 4; lane++) {
        total += (i32)(signed char)(a >> (lane * 8)) * (unsigned char)(b >> (lane * 8));
    }
    return total;
}
static u64 host_mpy2ll(u32 a, u32 b)
{
    i32 low = (i32)lane_low(a) * lane_low(b);
    i32 high = (i32)lane_high(a) * lane_high(b);
    return ((u64)(u32)high << 32) | (u32)low;
}
static u32 host_swap4(u32 a)
{
    return ((a & 0x00ff00ffu) << 8) | ((a >> 8) & 0x00ff00ffu);
}
static u32 host_rotl(u32 a, u32 count)
{
    unsigned int c = count & 0x1f;
    return c == 0 ? a : ((a << c) | (a >> (32 - c)));
}
static u32 host_bitr(u32 a)
{
    u32 result = 0;
    int bit;
    for (bit = 0; bit < 32; bit++) {
        if ((a >> bit) & 1u) result |= 1u << (31 - bit);
    }
    return result;
}
static u32 host_bitc4(u32 a)
{
    u32 result = 0;
    int lane, bit;
    for (lane = 0; lane < 4; lane++) {
        unsigned char byte = (unsigned char)(a >> (lane * 8));
        unsigned char count = 0;
        for (bit = 0; bit < 8; bit++) count = (unsigned char)(count + ((byte >> bit) & 1u));
        result |= (u32)count << (lane * 8);
    }
    return result;
}
static u32 host_deal(u32 a)
{
    u32 even = 0, odd = 0;
    int bit;
    for (bit = 0; bit < 16; bit++) {
        even |= ((a >> (2 * bit)) & 1u) << bit;
        odd |= ((a >> (2 * bit + 1)) & 1u) << bit;
    }
    return (odd << 16) | even;
}
static u32 host_shfl(u32 a)
{
    u32 result = 0;
    int bit;
    for (bit = 0; bit < 16; bit++) {
        result |= ((a >> bit) & 1u) << (2 * bit);
        result |= ((a >> (bit + 16)) & 1u) << (2 * bit + 1);
    }
    return result;
}
static u32 host_xpnd2(u32 a)
{
    u32 high = (a & 2u) ? 0xffff0000u : 0u;
    u32 low = (a & 1u) ? 0x0000ffffu : 0u;
    return high | low;
}
static u32 host_xpnd4(u32 a)
{
    u32 result = 0;
    int lane;
    for (lane = 0; lane < 4; lane++) {
        if ((a >> lane) & 1u) result |= 0xffu << (lane * 8);
    }
    return result;
}
static u32 host_abs2(u32 a)
{
    short h = lane_high(a), l = lane_low(a);
    short ah = h == -32768 ? 32767 : (short)(h < 0 ? -h : h);
    short al = l == -32768 ? 32767 : (short)(l < 0 ? -l : l);
    return pack_lanes(ah, al);
}
static u32 host_sshvl(u32 a, i32 count)
{
    i32 shift = count;
    i64 value = (i32)a;
    if (shift < 0) {
        shift = -shift;
        if (shift > 31) shift = 31;
        return (u32)(i32)(value >> shift);
    }
    if (shift > 31) shift = 31;
    {
        i64 shifted = value << shift;
        if (shifted > INT_MAX) return 0x7fffffffu;
        if (shifted < INT_MIN) return 0x80000000u;
        return (u32)(i32)shifted;
    }
}
static u32 host_sshvr(u32 a, i32 count)
{
    return host_sshvl(a, -count);
}
static u32 host_gmpy4(u32 a, u32 b)
{
    u32 result = 0;
    int lane, bit;
    for (lane = 0; lane < 4; lane++) {
        unsigned int x = (unsigned char)(a >> (lane * 8));
        unsigned int y = (unsigned char)(b >> (lane * 8));
        unsigned int product = 0;
        for (bit = 7; bit >= 0; bit--) {
            product <<= 1;
            if ((y >> bit) & 1u) product ^= x;
            /* Reduce modulo the default polynomial 0x1d with the implicit x^8 term. */
            if (product & 0x100u) product ^= 0x11du;
        }
        result |= (product & 0xffu) << (lane * 8);
    }
    return result;
}
static u32 host_xormpy(u32 a, u32 b)
{
    u32 result = 0;
    int bit;
    for (bit = 0; bit < 9; bit++) {
        if ((b >> bit) & 1u) result ^= a << bit;
    }
    return result;
}
static u32 host_cmpeq2(u32 a, u32 b)
{
    return (u32)((lane_high(a) == lane_high(b) ? 2u : 0u) | (lane_low(a) == lane_low(b) ? 1u : 0u));
}
static u32 host_cmpgt2(u32 a, u32 b)
{
    return (u32)((lane_high(a) > lane_high(b) ? 2u : 0u) | (lane_low(a) > lane_low(b) ? 1u : 0u));
}
static u32 host_cmpeq4(u32 a, u32 b)
{
    u32 result = 0;
    int lane;
    for (lane = 0; lane < 4; lane++) {
        if ((unsigned char)(a >> (lane * 8)) == (unsigned char)(b >> (lane * 8))) {
            result |= 1u << lane;
        }
    }
    return result;
}
static u32 host_cmpgtu4(u32 a, u32 b)
{
    u32 result = 0;
    int lane;
    for (lane = 0; lane < 4; lane++) {
        if ((unsigned char)(a >> (lane * 8)) > (unsigned char)(b >> (lane * 8))) {
            result |= 1u << lane;
        }
    }
    return result;
}
static u32 host_norm(i32 a)
{
    u32 count = 0;
    i32 value = a < 0 ? ~a : a;
    while (count < 31u && ((value >> (30 - count)) & 1) == 0) count++;
    return count;
}
static i32 host_sadd(i32 a, i32 b)
{
    i64 sum = (i64)a + b;
    if (sum > INT_MAX) return INT_MAX;
    if (sum < INT_MIN) return INT_MIN;
    return (i32)sum;
}
static i32 host_ssub(i32 a, i32 b)
{
    i64 difference = (i64)a - b;
    if (difference > INT_MAX) return INT_MAX;
    if (difference < INT_MIN) return INT_MIN;
    return (i32)difference;
}
static u32 host_lmbd(u32 a, u32 b)
{
    u32 searched = a & 1u;
    u32 count = 0;
    int bit;
    for (bit = 31; bit >= 0; bit--) {
        if (((b >> bit) & 1u) == searched) break;
        count++;
    }
    return count;
}
#define P_ADD2(a, b)     host_add2((a), (b))
#define P_SUB2(a, b)     host_sub2((a), (b))
#define P_SADD2(a, b)    host_sadd2((a), (b))
#define P_SSUB2(a, b)    host_ssub2((a), (b))
#define P_MAX2(a, b)     host_max2((a), (b))
#define P_MIN2(a, b)     host_min2((a), (b))
#define P_AVG2(a, b)     host_avg2((a), (b))
#define P_AVGU4(a, b)    host_bytes((a), (b), 5)
#define P_ADD4(a, b)     host_bytes((a), (b), 0)
#define P_SUB4(a, b)     host_bytes((a), (b), 1)
#define P_MAXU4(a, b)    host_bytes((a), (b), 2)
#define P_MINU4(a, b)    host_bytes((a), (b), 3)
#define P_SUBABS4(a, b)  host_bytes((a), (b), 4)
#define P_PACK2(a, b)    pack_lanes(lane_low(a), lane_low(b))
#define P_PACKH2(a, b)   pack_lanes(lane_high(a), lane_high(b))
#define P_PACKLH2(a, b)  pack_lanes(lane_low(a), lane_high(b))
/* dst takes src1 bytes 3 and 1 then src2 bytes 3 and 1 (SPRUFE8B PACKH4). */
#define P_PACKH4(a, b)   ((u32)(((a) & 0xff000000u) | (((a) & 0x0000ff00u) << 8) | \
                                 ((((b) >> 24) & 0xffu) << 8) | (((b) >> 8) & 0xffu)))
#define P_PACKL4(a, b)   ((u32)((((a) & 0x00ff0000u) << 8) | (((a) & 0xffu) << 16) | \
                                 (((b) & 0x00ff0000u) >> 8) | ((b) & 0xffu)))
#define P_SPACK2(a, b)   pack_lanes(sat_short((i32)(a)), sat_short((i32)(b)))
#define P_SPACKU4(a, b)  ((u32)(((u32)sat_uchar(lane_high(a)) << 24) | \
                                 ((u32)sat_uchar(lane_low(a)) << 16) | \
                                 ((u32)sat_uchar(lane_high(b)) << 8) | \
                                 (u32)sat_uchar(lane_low(b))))
#define P_DOTP2(a, b)    host_dotp2((a), (b))
#define P_DOTPN2(a, b)   host_dotpn2((a), (b))
#define P_DOTPU4(a, b)   host_dotpu4((a), (b))
#define P_DOTPSU4(a, b)  host_dotpsu4((a), (b))
#define P_MPY2LL(a, b)   host_mpy2ll((a), (b))
#define P_SHR2(a, b)     host_shr2((a), (b))
#define P_SHRU2(a, b)    host_shru2((a), (b))
/* src2 shifts left one byte and src1's top byte lands in the bottom (SPRUFE8B SHLMB). */
#define P_SHLMB(a, b)    ((u32)((((b) & 0xffffffu) << 8) | (((a) >> 24) & 0xffu)))
#define P_SHRMB(a, b)    ((u32)((((a) & 0xffu) << 24) | ((b) >> 8)))
#define P_SWAP4(a)       host_swap4((a))
#define P_ROTL(a, b)     host_rotl((a), (b))
#define P_BITR(a)        host_bitr((a))
#define P_BITC4(a)       host_bitc4((a))
#define P_DEAL(a)        host_deal((a))
#define P_SHFL(a)        host_shfl((a))
#define P_XPND2(a)       host_xpnd2((a))
#define P_XPND4(a)       host_xpnd4((a))
#define P_ABS2(a)        host_abs2((a))
#define P_SSHVL(a, b)    host_sshvl((a), (b))
#define P_SSHVR(a, b)    host_sshvr((a), (b))
#define P_GMPY4(a, b)    host_gmpy4((a), (b))
#define P_XORMPY(a, b)   host_xormpy((a), (b))
#define P_CMPEQ2(a, b)   host_cmpeq2((a), (b))
#define P_CMPGT2(a, b)   host_cmpgt2((a), (b))
#define P_CMPEQ4(a, b)   host_cmpeq4((a), (b))
#define P_CMPGTU4(a, b)  host_cmpgtu4((a), (b))
#define P_NORM(a)        host_norm((a))
#define P_SADD(a, b)     host_sadd((a), (b))
#define P_SSUB(a, b)     host_ssub((a), (b))
#define P_ABS(a)         ((i32)(a) == INT_MIN ? INT_MAX : ((i32)(a) < 0 ? -(i32)(a) : (i32)(a)))
#define P_LMBD(a, b)     host_lmbd((a), (b))
#endif

#endif
