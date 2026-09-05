* CSR.SAT and SSR: a saturating functional unit sets its own bit in SSR and the SAT bit in CSR
* one cycle after the result is written. Only MVC can clear SAT, and writing a one to it has no
* effect (SPRUFE8B Table 2-9 and Table 2-22). SSR names the unit: bit 0 L1, 1 L2, 2 S1, 3 S2,
* 4 M1, 5 M2. The checksum folds the observed CSR.SAT bit and SSR after each step.
        .global main
        .sect ".text"
        .align 32
main:
        MVK     .L1     0,A9                    ; checksum
        ZERO    .L2     B4
        MVC     .S2     B4,SSR
        MVC     .S2     B4,CSR
        NOP     4

* A non-saturating SADD on .L1 leaves both clean.
        MVKL    .S1     0x00001000,A4
        MVKH    .S1     0x00001000,A4
        MVK     .S1     1,A5
        NOP     2
        SADD    .L1     A4,A5,A6
        NOP     4
        MVC     .S2     CSR,B5
        MVC     .S2     SSR,B6
        NOP     4
        EXTU    .S2     B5,22,31,B7             ; CSR bit 9 alone
        NOP
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B7,A9
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B6,A9

* A saturating SADD on .L1: 0x7fffffff + 1 clamps, so SAT and SSR bit 0 appear.
        MVKL    .S1     0x7fffffff,A4
        MVKH    .S1     0x7fffffff,A4
        NOP     2
        SADD    .L1     A4,A5,A6
        NOP     4
        MVC     .S2     CSR,B5
        MVC     .S2     SSR,B6
        NOP     4
        EXTU    .S2     B5,22,31,B7
        NOP
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B7,A9
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B6,A9

* Writing a one to CSR.SAT must not set it, and writing zero clears it. SSR is writable.
        ZERO    .L2     B4
        MVC     .S2     B4,SSR
        MVC     .S2     B4,CSR
        NOP     4
        MVC     .S2     CSR,B5
        MVC     .S2     SSR,B6
        NOP     4
        EXTU    .S2     B5,22,31,B7
        NOP
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B7,A9
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B6,A9
        MVK     .S2     0x0200,B4
        NOP
        MVC     .S2     B4,CSR
        NOP     4
        MVC     .S2     CSR,B5
        NOP     4
        EXTU    .S2     B5,22,31,B7
        NOP
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B7,A9

* SSHL on .S2 saturates: SSR bit 3.
        ZERO    .L2     B4
        MVC     .S2     B4,SSR
        MVKL    .S2     0x40000000,B8
        MVKH    .S2     0x40000000,B8
        MVK     .S2     4,B9
        NOP     3
        SSHL    .S2     B8,B9,B10
        NOP     4
        MVC     .S2     CSR,B5
        MVC     .S2     SSR,B6
        NOP     4
        EXTU    .S2     B5,22,31,B7
        NOP
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B7,A9
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B6,A9

* SMPY on .M1 saturates only for 0x8000 * 0x8000: SSR bit 4.
        ZERO    .L2     B4
        MVC     .S2     B4,SSR
        MVC     .S2     B4,CSR
        MVKL    .S1     0x00008000,A10
        MVKH    .S1     0x00008000,A10
        NOP     3
        SMPY    .M1     A10,A10,A11
        NOP     4
        MVC     .S2     CSR,B5
        MVC     .S2     SSR,B6
        NOP     4
        EXTU    .S2     B5,22,31,B7
        NOP
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B7,A9
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B6,A9

* ABS on .L2 of the most negative int saturates: SSR bit 1, and the flags accumulate.
        MVKL    .S2     0x80000000,B8
        MVKH    .S2     0x80000000,B8
        NOP     2
        ABS     .L2     B8,B11
        NOP     4
        MVC     .S2     CSR,B5
        MVC     .S2     SSR,B6
        NOP     4
        EXTU    .S2     B5,22,31,B7
        NOP
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B7,A9
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B6,A9

* SPACK2 on .S1 saturates both halves: SSR bit 2.
        ZERO    .L2     B4
        MVC     .S2     B4,SSR
        MVKL    .S1     0x00400000,A12
        MVKH    .S1     0x00400000,A12
        NOP     3
        SPACK2  .S1     A12,A12,A13
        NOP     4
        MVC     .S2     SSR,B6
        NOP     4
        SHL     .S1     A9,4,A9
        ADD     .L1X    A9,B6,A9

        MV      .L1     A9,A4
        B       .S2     B3
        NOP     5
