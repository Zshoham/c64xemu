#ifndef BITFIELD_SHIM_H
#define BITFIELD_SHIM_H
/* The bit-field instructions in their register form, reached through hand-written assembly on
   the target and modelled in plain C for a host compiler.  Every model below is written from
   the EXT, EXTU, SET and CLR pages of SPRU732J: the field specifier lives in bits 9-5 (csta)
   and 4-0 (cstb) of src1 and the rest of src1 is ignored; EXT is a left shift by csta followed
   by an arithmetic right shift by cstb, EXTU the same with a logical right shift, and SET and
   CLR set or clear the bits from csta up to cstb inclusive. */

typedef unsigned int u32;
typedef int i32;

#if defined(__TI_COMPILER_VERSION__)

extern u32 bf_extu_s1(u32 value, u32 control);
extern u32 bf_extu_s2x(u32 value, u32 control);
extern u32 bf_ext_s1(u32 value, u32 control);
extern u32 bf_ext_s2x(u32 value, u32 control);
extern u32 bf_set_s1(u32 value, u32 control);
extern u32 bf_set_s2x(u32 value, u32 control);
extern u32 bf_clr_s1(u32 value, u32 control);
extern u32 bf_clr_s2x(u32 value, u32 control);
extern u32 bf_clr_pred(u32 value, u32 control, u32 predicate);
extern u32 bf_ext_npred(u32 value, u32 control, u32 predicate);
extern u32 bf_const_corners(u32 value);

#else

static u32 field_csta(u32 control) { return (control >> 5) & 31u; }
static u32 field_cstb(u32 control) { return control & 31u; }

static u32 model_extu(u32 value, u32 control)
{
    return (value << field_csta(control)) >> field_cstb(control);
}

static u32 model_ext(u32 value, u32 control)
{
    i32 shifted = (i32)(value << field_csta(control));
    return (u32)(shifted >> field_cstb(control));
}

/* The mask covers csta up to cstb inclusive.  Both are five bits, so a field that reaches
   bit 31 needs the width-32 case spelled out rather than a shift by 32. */
static u32 field_mask(u32 control)
{
    u32 low = field_csta(control);
    u32 high = field_cstb(control);
    u32 width = high - low + 1u;

    if (width >= 32u) {
        return 0xffffffffu;
    }
    return ((1u << width) - 1u) << low;
}

static u32 model_set(u32 value, u32 control) { return value | field_mask(control); }
static u32 model_clr(u32 value, u32 control) { return value & ~field_mask(control); }

static u32 bf_extu_s1(u32 value, u32 control) { return model_extu(value, control); }
static u32 bf_extu_s2x(u32 value, u32 control) { return model_extu(value, control); }
static u32 bf_ext_s1(u32 value, u32 control) { return model_ext(value, control); }
static u32 bf_ext_s2x(u32 value, u32 control) { return model_ext(value, control); }
static u32 bf_set_s1(u32 value, u32 control) { return model_set(value, control); }
static u32 bf_set_s2x(u32 value, u32 control) { return model_set(value, control); }
static u32 bf_clr_s1(u32 value, u32 control) { return model_clr(value, control); }
static u32 bf_clr_s2x(u32 value, u32 control) { return model_clr(value, control); }

static u32 bf_clr_pred(u32 value, u32 control, u32 predicate)
{
    return predicate ? model_clr(value, control) : value;
}

static u32 bf_ext_npred(u32 value, u32 control, u32 predicate)
{
    return predicate ? value : model_ext(value, control);
}

/* The same nine constant-form operations the assembly helper performs, folded the same way. */
static u32 bf_const_corners(u32 value)
{
    u32 total = model_extu(value, (0u << 5) | 0u);

    total += model_extu(value, (31u << 5) | 31u);
    total ^= model_ext(value, (31u << 5) | 31u);
    total += model_ext(value, (0u << 5) | 31u);
    total ^= model_ext(value, (16u << 5) | 16u);
    total += model_set(value, (0u << 5) | 31u);
    total ^= model_clr(value, (0u << 5) | 31u);
    total += model_set(value, (5u << 5) | 5u);
    total ^= model_clr(value, (31u << 5) | 31u);
    return total;
}

#endif

#endif
