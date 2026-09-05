#include <stdio.h>
#include "complex_shim.h"
/* Drive the complex-multiply family over pseudo-random inputs, including the saturating corner
   cases, and fold every result. The host builds the same source against equivalents written
   from the manual, so the constant only matches if the emulator implements CMPY, CMPYR,
   CMPYR1, DDOTP4, DDOTPH2, DDOTPH2R, DDOTPL2, DDOTPL2R, MPY2IR, SMPY32, MPYHIR and MPYLIR the
   way the manual defines them, register pairs and all. */

static unsigned int rng_state = 987654321u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static u32 compute(void)
{
    u32 total = 0;
    int round;

    for (round = 0; round < 20; round++) {
        u32 a = rng_next();
        u32 b = rng_next();
        u64 wide = ((u64)rng_next() << 32) | rng_next();
        u64 result;

        if (round == 0) {
            a = 0x7fff7fffu;
            b = 0x7fff8000u;
            wide = 0x1234800080005678ull;
        }
        result = X_CMPY(a, b);
        total += (u32)result ^ (u32)(result >> 32);
        total = (total << 1) | (total >> 31);
        total ^= X_CMPYR(a, b);
        total += X_CMPYR1(a, b);
        result = X_DDOTP4(a, b);
        total ^= (u32)result + (u32)(result >> 32);
        result = X_DDOTPH2(wide, b);
        total += (u32)result ^ (u32)(result >> 32);
        result = X_DDOTPL2(wide, b);
        total ^= (u32)result + (u32)(result >> 32);
        total += X_DDOTPH2R(wide, b);
        total ^= X_DDOTPL2R(wide, b);
        result = X_MPY2IR(a, b);
        total += (u32)result ^ (u32)(result >> 32);
        total ^= X_SMPY32(a, b);
        total += X_MPYHIR(a, b);
        total ^= X_MPYLIR(a, b);
        total = (total << 3) | (total >> 29);
    }
    return total;
}

int main(void)
{
    printf("%s=%u\n", "complex_simd", compute());
    return 0;
}
