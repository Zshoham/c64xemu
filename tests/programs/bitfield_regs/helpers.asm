* The bit-field family in the forms compiled C never produces.  cl6x lowers _ext and _extu with
* a run-time field specifier to a SHL/SHR pair and only ever emits the constant form of SET and
* CLR with literals a bit-field declaration can express, so the register form and the extreme
* constants are untested by every other program in the suite.
*
* SPRU732J, EXT/EXTU/SET/CLR: in the register form the field specifier is taken from src1, csta
* from bits 9-5 and cstb from bits 4-0, and the remaining bits of src1 are ignored.  src2 holds
* the value and is the operand the cross path may supply; src1 and dst stay on the unit's own
* side.  EXT shifts left by csta then arithmetically right by cstb, EXTU shifts right logically,
* and SET and CLR set or clear the field whose LSB is csta and whose MSB is cstb.
*
* EABI: the first argument arrives in A4, the second in B4, the third in A6, the return value
* leaves in A4 and the return address is in B3.  Each helper does its work before the branch so
* that no result depends on delay-slot counting.
        .global bf_extu_s1, bf_extu_s2x, bf_ext_s1, bf_ext_s2x
        .global bf_set_s1, bf_set_s2x, bf_clr_s1, bf_clr_s2x
        .global bf_clr_pred, bf_ext_npred, bf_const_corners
        .sect ".text"

* EXTU, register form, both operands on the A side.
bf_extu_s1:
        MV      .L1X    B4,A5
        EXTU    .S1     A4,A5,A4
        B       .S2     B3
        NOP     5

* EXTU, register form, the value crossed onto the B side (x = 1).
bf_extu_s2x:
        EXTU    .S2X    A4,B4,B5
        MV      .L1X    B5,A4
        B       .S2     B3
        NOP     5

* EXT, register form, both operands on the A side.
bf_ext_s1:
        MV      .L1X    B4,A5
        EXT     .S1     A4,A5,A4
        B       .S2     B3
        NOP     5

* EXT, register form, the value crossed onto the B side.
bf_ext_s2x:
        EXT     .S2X    A4,B4,B5
        MV      .L1X    B5,A4
        B       .S2     B3
        NOP     5

* SET, register form, both operands on the A side.
bf_set_s1:
        MV      .L1X    B4,A5
        SET     .S1     A4,A5,A4
        B       .S2     B3
        NOP     5

* SET, register form, the value crossed onto the B side.
bf_set_s2x:
        SET     .S2X    A4,B4,B5
        MV      .L1X    B5,A4
        B       .S2     B3
        NOP     5

* CLR, register form, both operands on the A side.
bf_clr_s1:
        MV      .L1X    B4,A5
        CLR     .S1     A4,A5,A4
        B       .S2     B3
        NOP     5

* CLR, register form, the value crossed onto the B side.
bf_clr_s2x:
        CLR     .S2X    A4,B4,B5
        MV      .L1X    B5,A4
        B       .S2     B3
        NOP     5

* CLR, register form, predicated on B0: the value passes through untouched when B0 is zero.
bf_clr_pred:
        MV      .L1X    B4,A5
||      MV      .L2X    A6,B0
        NOP     1
 [B0]   CLR     .S1     A4,A5,A4
        B       .S2     B3
        NOP     5

* EXT, register form, predicated on !B0: the extraction happens only when B0 is zero.
bf_ext_npred:
        MV      .L1X    B4,A5
||      MV      .L2X    A6,B0
        NOP     1
 [!B0]  EXT     .S1     A4,A5,A4
        B       .S2     B3
        NOP     5

* The constant form at the ends of its range: a zero-width shift pair, the single-bit
* extractions, the sign spread, and fields that cover the whole word.
bf_const_corners:
        EXTU    .S1     A4,0,0,A5
        EXTU    .S1     A4,31,31,A6
        EXT     .S1     A4,31,31,A7
        EXT     .S1     A4,0,31,A8
        EXT     .S1     A4,16,16,A9
        SET     .S1     A4,0,31,A0
        CLR     .S1     A4,0,31,A1
        SET     .S1     A4,5,5,A2
        CLR     .S1     A4,31,31,A3
        ADD     .L1     A5,A6,A5
        XOR     .L1     A5,A7,A5
        ADD     .L1     A5,A8,A5
        XOR     .L1     A5,A9,A5
        ADD     .L1     A5,A0,A5
        XOR     .L1     A5,A1,A5
        ADD     .L1     A5,A2,A5
        XOR     .L1     A5,A3,A5
        MV      .L1     A5,A4
        B       .S2     B3
        NOP     5
