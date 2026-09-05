* Circular addressing through AMR: two blocks (BK0 = 32 bytes, BK1 = 64 bytes) walked with
* post-increment, pre-increment, pre-decrement, register-offset and ADDA/SUBA forms. Only bits
* N..0 of each address update may change (SPRUFE8B 2.8.3, 3.9.2). The checksum folds both the
* loaded data and the wrapped pointer offsets, so a pointer that fails to wrap shows up.
        .global main
        .sect ".text"
        .align 32
main:
* AMR: A4 circular on BK0, A5 circular on BK1, A6 circular on BK0.
* BK0 = 4 (32 bytes), BK1 = 5 (64 bytes).
        MVKL    .S2     0x00a40019,B0
        MVKH    .S2     0x00a40019,B0
        MVC     .S2     B0,AMR
        NOP     4
        MVKL    .S1     buf32,A4
        MVKH    .S1     buf32,A4
        MVKL    .S1     buf32,A10
        MVKH    .S1     buf32,A10
        MVK     .L1     0,A9
        MVK     .S2     16,B1
loop32:
        LDW     .D1T1   *A4++[3],A8
        NOP     4
        SUB     .L1     A4,A10,A12
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9
        ADD     .L1     A9,A12,A9
        SUB     .L2     B1,1,B1
        NOP     1
 [B1]   B       .S2     loop32
        NOP     5

        MVKL    .S1     buf64,A5
        MVKH    .S1     buf64,A5
        MVKL    .S1     buf64,A11
        MVKH    .S1     buf64,A11
        MVK     .L2     12,B1
loop64:
        LDBU    .D1T1   *A5++[5],A8
        NOP     4
        SUB     .L1     A5,A11,A12
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9
        ADD     .L1     A9,A12,A9
        SUB     .L2     B1,1,B1
        NOP     1
 [B1]   B       .S2     loop64
        NOP     5

* ADDA/SUBA on a circular pointer.
        MVKL    .S1     buf32,A6
        MVKH    .S1     buf32,A6
        NOP     1
        ADDAH   .D1     A6,19,A6
        NOP     1
        SUB     .L1     A6,A10,A12
        SHL     .S1     A9,3,A9
        ADD     .L1     A9,A12,A9
        SUBAW   .D1     A6,5,A6
        NOP     1
        SUB     .L1     A6,A10,A12
        SHL     .S1     A9,3,A9
        ADD     .L1     A9,A12,A9
        MVK     .L1     7,A13
        NOP     1
        ADDAW   .D1     A6,A13,A6
        NOP     1
        SUB     .L1     A6,A10,A12
        SHL     .S1     A9,3,A9
        ADD     .L1     A9,A12,A9
        ADDAB   .D1     A6,29,A6
        NOP     1
        SUB     .L1     A6,A10,A12
        SHL     .S1     A9,3,A9
        ADD     .L1     A9,A12,A9

* Pre-decrement and pre-increment word loads, the second matching Example 3-4's shape.
        LDW     .D1T1   *--A4[5],A8
        NOP     4
        SUB     .L1     A4,A10,A12
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9
        ADD     .L1     A9,A12,A9
        LDW     .D1T1   *++A4[9],A8
        NOP     4
        SUB     .L1     A4,A10,A12
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9
        ADD     .L1     A9,A12,A9

* A register-offset load that does not update the pointer, then a byte load with a constant.
        MVK     .L1     6,A13
        NOP     1
        LDW     .D1T1   *+A4[A13],A8
        NOP     4
        SUB     .L1     A4,A10,A12
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9
        ADD     .L1     A9,A12,A9

* Stores through a circular pointer, then a linear read-back of the whole block.
        MVK     .L2     5,B1
stores:
        STW     .D1T1   A9,*A4++[7]
        NOP     1
        SUB     .L1     A4,A10,A12
        SHL     .S1     A9,3,A9
        ADD     .L1     A9,A12,A9
        SUB     .L2     B1,1,B1
        NOP     1
 [B1]   B       .S2     stores
        NOP     5

        MVK     .L2     8,B1
        MVKL    .S1     buf32,A14
        MVKH    .S1     buf32,A14
readback:
        LDW     .D1T1   *A14++[1],A8
        NOP     4
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9
        SUB     .L2     B1,1,B1
        NOP     1
 [B1]   B       .S2     readback
        NOP     5

* Galois multiplies: GMPY reduces by GPLYA on the A side, XORMPY by a zero polynomial.
        MVKL    .S2     0x87654321,B0
        MVKH    .S2     0x87654321,B0
        MVC     .S2     B0,GPLYA
        NOP     4
        MVK     .S1     0x126,A15
        NOP     1
        GMPY    .M1     A9,A15,A8
        NOP     3
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9
        XORMPY  .M1     A9,A15,A8
        NOP     3
        SHL     .S1     A9,3,A9
        XOR     .L1     A9,A8,A9

        MV      .L1     A9,A4
        B       .S2     B3
        NOP     5

        .data
        .align  32
buf32:  .word   0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f
        .word   0x10111213, 0x14151617, 0x18191a1b, 0x1c1d1e1f
        .align  64
buf64:  .byte   0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
        .byte   16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
        .byte   32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47
        .byte   48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63
