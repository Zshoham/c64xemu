;; Minimal C64x+ startup for emulator program tests.
;;
;; Deliberately does *not* use the TI RTS boot path (_c_int00 / auto-init):
;; these tests exercise compiler-generated code, not RTS initialisation.
;; Programs that need auto-init link against the real RTS instead and use
;; _c_int00 as their entry point.
;;
;; Contract:
;;   B15 = top of the reserved stack (8-byte aligned, grows down)
;;   B14 = static base for near data access
;;   A4  = value returned by main() when execution stops
;;   PC  = C$$EXIT when the program is finished (the runner stops there)
        .global _start
        .global C$$EXIT
        .global main
        .ref __TI_STATIC_BASE

stack_base  .usect ".stack", 2048, 8

        .sect ".text"
        .align 32
_start:
        MVKL    .S2     stack_base + 2040,B15
        MVKH    .S2     stack_base + 2040,B15
        MVKL    .S2     __TI_STATIC_BASE,B14
        MVKH    .S2     __TI_STATIC_BASE,B14
        AND     .D2     -8,B15,B15
        MVK     .L2     0,B3
        NOP     2
        CALLP   .S2     main,B3
        NOP     4
        B       .S1     C$$EXIT
        NOP     5

        .align 32
C$$EXIT:
        NOP     5
        NOP     5
