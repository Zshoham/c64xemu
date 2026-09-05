#include <stdio.h>
/* The 64-bit memory intrinsics the hand-optimised DSPLIB kernels are built on: aligned and
   non-aligned double-word loads and stores through _amem8/_mem8, and the halves via _loll,
   _hill and _itoll. */
#define ALIGNED8 __attribute__((aligned(8)))
typedef unsigned long long u64;
typedef unsigned int u32;

#if defined(__TI_COMPILER_VERSION__)
#include <c6x.h>
#define LD_A(p)      _amem8_const(p)
#define LD_N(p)      _mem8_const(p)
#define ST_A(p, v)   _amem8((p)) = (v)
#define ST_N(p, v)   _mem8((p)) = (v)
#define LOW(v)       _loll(v)
#define HIGH(v)      _hill(v)
#define JOIN(h, l)   _itoll((h), (l))
#else
static u64 host_load(const void *p)
{
    const unsigned char *bytes = (const unsigned char *)p;
    u64 value = 0;
    int i;
    for (i = 0; i < 8; i++) value |= (u64)bytes[i] << (i * 8);
    return value;
}
static void host_store(void *p, u64 value)
{
    unsigned char *bytes = (unsigned char *)p;
    int i;
    for (i = 0; i < 8; i++) bytes[i] = (unsigned char)(value >> (i * 8));
}
#define LD_A(p)      host_load((const void *)(p))
#define LD_N(p)      host_load((const void *)(p))
#define ST_A(p, v)   host_store((void *)(p), (v))
#define ST_N(p, v)   host_store((void *)(p), (v))
#define LOW(v)       ((u32)(v))
#define HIGH(v)      ((u32)((v) >> 32))
#define JOIN(h, l)   ((((u64)(h)) << 32) | (u32)(l))
#endif

static unsigned int rng_state = 5150u;
static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short data[64] ALIGNED8;
static short out[64] ALIGNED8;

int main(void)
{
    int i;
    u32 total = 0;
    for (i = 0; i < 64; i++) data[i] = (short)rng_next();
    for (i = 0; i < 64; i++) out[i] = 0;

    /* Aligned loads walk the array four halfwords at a time. */
    for (i = 0; i + 4 <= 64; i += 4) {
        u64 value = LD_A(&data[i]);
        total = total * 31u + LOW(value);
        total = total * 31u + HIGH(value);
    }
    /* Non-aligned loads walk it one halfword at a time, which is what the FIR kernels do. */
    for (i = 0; i + 4 <= 64; i++) {
        u64 value = LD_N(&data[i]);
        total = total * 17u + (LOW(value) ^ HIGH(value));
    }
    /* Stores: aligned pairs, then a non-aligned pass that overlaps them. */
    u32 rh, rl;
    for (i = 0; i + 4 <= 64; i += 4) {
        rh = rng_next();
        rl = rng_next();
        ST_A(&out[i], JOIN(rh, rl));
    }
    for (i = 1; i + 4 <= 64; i += 5) {
        rh = rng_next();
        rl = rng_next();
        ST_N(&out[i], JOIN(rh, rl));
    }
    for (i = 0; i < 64; i++) {
        total = total * 31u + (u32)(unsigned short)out[i];
    }
    printf("%s=%u\n", "memory_intrinsics", total);
    return 0;
}
