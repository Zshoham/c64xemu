* When a register-sourced branch samples its target register (SPRUFE8B, `B .S2 src2`), which
* no compiled program exposes: cl6x never writes a branch's target register inside that
* branch's own delay slots, so only hand-written assembly can tell the two candidate models
* apart. The instruction's pipeline table reads src2 in E1 - at issue - so a write that lands
* while the branch is in flight cannot redirect it. TI's own SYS/BIOS task switch depends on
* this: ti_sysbios_family_c62_TaskSupport_swap ends with `b b4` and then reloads b3 and the
* callee-saved registers in the branch's delay slots.
*
* Two things are checked. The branch must reach the target that src2 held when it issued, and
* the delay-slot write must still have landed by the time the target runs, so the register
* holds the value the branch did not use.
        .nocmp
        .global main
        .sect ".text"

FOLD    .macro  src
        SHL     .S1     A4,5,A8
        SHRU    .S1     A4,27,A9
        OR      .L1     A8,A9,A4
        XOR     .L1X    A4,src,A4
        .endm

FOLDA   .macro  src
        SHL     .S1     A4,5,A8
        SHRU    .S1     A4,27,A9
        OR      .L1     A8,A9,A4
        XOR     .L1     A4,src,A4
        .endm

main:
        MV      .L2     B3,B10                  ; caller's return address
        MVK     .L1     0,A4                    ; checksum

        MVKL    .S2     sampled,B4
        MVKH    .S2     sampled,B4
        MVKL    .S1     redirected,A6
        MVKH    .S1     redirected,A6

        B       .S2     B4                      ; src2 is read in E1, so this goes to sampled
        MV      .L2X    A6,B4                   ; delay slot 1: B4 becomes redirected
        NOP     4                               ; delay slots 2 to 5

* Reached only if the branch used the register value written while it was in flight.
redirected:
        MVKL    .S1     0x0bad0001,A4
        MVKH    .S1     0x0bad0001,A4
        B       .S2     B10
        NOP     5

sampled:
        MVK     .L1     7,A5                    ; the branch landed where src2 pointed at issue
        SUB     .L2X    B4,A6,B5                ; and the delay-slot write did land: zero
        FOLDA   A5
        FOLD    B5
        B       .S2     B10
        NOP     5
