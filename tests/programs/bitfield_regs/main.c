#include <stdio.h>
#include "bitfield_shim.h"
/* Drive the register form of EXT, EXTU, SET and CLR - on both sides, across the cross path,
   and under a predicate - plus the constant form at the ends of its range, and fold every
   result into one value.  The host builds the same driver against plain C models written from
   SPRU732J, so the two agree only if the emulator implements the instructions as documented. */

static u32 rng_state = 22695477u;

static u32 rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static u32 compute(void)
{
    u32 total = 0u;
    int round;

    for (round = 0; round < 40; round++) {
        u32 value = rng_next();
        u32 draw = rng_next();
        u32 csta = (draw >> 3) & 31u;
        u32 cstb = (draw >> 11) & 31u;
        u32 low = csta < cstb ? csta : cstb;
        u32 high = csta < cstb ? cstb : csta;
        u32 noise = rng_next() & 0xfffffc00u;
        /* Only bits 9-0 of the specifier register are read, so the rest carries garbage.
           SET and CLR get an ordered field: the manual defines csta as the LSB and cstb as
           the MSB, and leaves the reversed case undefined. */
        u32 shift_control = ((csta << 5) | cstb) | noise;
        u32 field_control = ((low << 5) | high) | (noise ^ 0xa5a5a400u);
        u32 predicate = (draw >> 20) & 1u;

        total += bf_extu_s1(value, shift_control);
        total ^= bf_extu_s2x(value, shift_control);
        total += bf_ext_s1(value, shift_control);
        total ^= bf_ext_s2x(value, shift_control);
        total += bf_set_s1(value, field_control);
        total ^= bf_set_s2x(value, field_control);
        total += bf_clr_s1(value, field_control);
        total ^= bf_clr_s2x(value, field_control);
        total += bf_clr_pred(value, field_control, predicate);
        total ^= bf_ext_npred(value, shift_control, predicate);
        total += bf_const_corners(value);
        total = (total << 1) | (total >> 31);
    }
    return total;
}

int main(void)
{
    printf("%s=%u\n", "bitfield_regs", compute());
    return 0;
}
