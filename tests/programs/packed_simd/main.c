#include <stdio.h>
#include "packed_shim.h"
/* Drive every packed operation the shim covers over pseudo-random inputs and fold the results
   into one value. The host builds the same source with plain C equivalents written from the
   instruction descriptions, so the two agree only if the emulator matches the manual. */

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static u32 compute(void)
{
    u32 total = 0;
    int round;

    for (round = 0; round < 24; round++) {
        u32 a = rng_next();
        u32 b = rng_next();
        u32 c = rng_next() >> 11;
        i32 sa = (i32)a;
        i32 sb = (i32)b;
        u64 wide;

        total += (u32)P_ADD2(a, b);
        total ^= (u32)P_SUB2(a, b);
        total += (u32)P_SADD2(a, b);
        total ^= (u32)P_SSUB2(a, b);
        total += (u32)P_MAX2(a, b);
        total ^= (u32)P_MIN2(a, b);
        total += (u32)P_AVG2(a, b);
        total ^= (u32)P_AVGU4(a, b);
        total += (u32)P_ADD4(a, b);
        total ^= (u32)P_SUB4(a, b);
        total += (u32)P_MAXU4(a, b);
        total ^= (u32)P_MINU4(a, b);
        total += (u32)P_SUBABS4(a, b);
        total ^= (u32)P_PACK2(a, b);
        total += (u32)P_PACKH2(a, b);
        total ^= (u32)P_PACKLH2(a, b);
        total += (u32)P_PACKH4(a, b);
        total ^= (u32)P_PACKL4(a, b);
        total += (u32)P_SPACK2(sa >> 8, sb >> 8);
        total ^= (u32)P_SPACKU4(a, b);
        total += (u32)P_DOTP2(a, b);
        total ^= (u32)P_DOTPN2(a, b);
        total += (u32)P_DOTPU4(a, b);
        total ^= (u32)P_DOTPSU4(a, b);
        wide = P_MPY2LL(a, b);
        total += (u32)(wide >> 32) ^ (u32)wide;
        total ^= (u32)P_SHR2(a, c & 15u);
        total += (u32)P_SHRU2(a, c & 15u);
        total ^= (u32)P_SHLMB(a, b);
        total += (u32)P_SHRMB(a, b);
        total ^= (u32)P_SWAP4(a);
        total += (u32)P_ROTL(a, c & 31u);
        total ^= (u32)P_BITR(a);
        total += (u32)P_BITC4(a);
        total ^= (u32)P_DEAL(a);
        total += (u32)P_SHFL(a);
        total ^= (u32)P_XPND2(c);
        total += (u32)P_XPND4(c);
        total ^= (u32)P_ABS2(a);
        total += (u32)P_SSHVL(a, (i32)(c & 7u));
        total ^= (u32)P_SSHVR(a, (i32)(c & 7u));
        total += (u32)P_GMPY4(a, b);
        total ^= (u32)P_XORMPY(a & 0x1ffu, b & 0x1ffu);
        total += (u32)P_CMPEQ2(a, b) + (int)P_CMPGT2(a, b);
        total ^= (u32)P_CMPEQ4(a, b) + (int)P_CMPGTU4(a, b);
        total += (u32)P_NORM(sa >> 3);
        total ^= (u32)P_SADD(sa, sb);
        total += (u32)P_SSUB(sa, sb);
        total ^= (u32)P_ABS(sa);
        total += (u32)P_LMBD(a & 1u, b);
        total = (total << 1) ^ (total >> 31);
    }
    return total;
}

int main(void)
{
    printf("%s=%u\n", "packed_simd", compute());
    return 0;
}
