* The megamodule memory protection unit (SPRU871K chapter 8), which no compiled program
* reaches: cl6x never programs a page attribute register.
*
* Each of L2, L1P and L1D has thirty-two per-page attribute registers - L2MPPA at 0x0184A200,
* L1PMPPA at 0x0184A600, L1DMPPA at 0x0184AE00 - carrying LOCAL in bit 8 and SR SW SX UR UW UX
* in bits 5 to 0, with the fault address, status and clear registers 0x200 below each array.
* A CPU access the page does not grant is discarded, its address and the permission it asked
* for are latched in MPFAR and MPFSR, and the unit raises a system event: 124 for L2, 120 for
* L1P, 122 for L1D. That event reaches the CPU through the exception combiner as EXCEP, which
* is why this program unmasks those three bits in EXPMASK3 and enables TSR.XEN.
*
* The page used here is L2 page 1 at 0x00810000, well clear of the code in page 0. It is given
* everything except supervisor write, so a store must fault and leave memory untouched, and the
* same store must succeed once the page grants SW.
        .nocmp
        .global main
        .sect ".text"

* Fold one observation into the checksum. A rotate rather than a shift, so that nothing
* observed early is shifted out before the end.
FOLD    .macro  src
        SHL     .S1     A4,5,A6
        SHRU    .S1     A4,27,A7
        OR      .L1     A6,A7,A4
        XOR     .L1X    A4,src,A4
        .endm

EXPMASK3    .set    0x018000cc
MEXPFLAG3   .set    0x018000ec
EVTCLR3     .set    0x0180004c
L2MPPA1     .set    0x0184a204
L2MPFAR     .set    0x0184a000
L2MPFSR     .set    0x0184a004
L2MPFCLR    .set    0x0184a008
TARGET      .set    0x00810000

main:
        MV      .L2     B3,B10                  ; caller's return address

        MVKL    .S2     ist,B4
        MVKH    .S2     ist,B4
        MVC     .S2     B4,ISTP

        MVC     .S2     TSR,B4
        NOP     1
        SET     .S2     B4,2,3,B4               ; TSR.GEE and TSR.XEN
        MVC     .S2     B4,TSR
        MVK     .S2     2,B5
        MVC     .S2     B5,IER                  ; IER.NMIE
        NOP     4

* Let only the three protection-fault events reach EXCEP: 120, 122 and 124 are bits 24, 26
* and 28 of the fourth event bank.
        MVKL    .S2     0x018000c0,B5
        MVKH    .S2     0x018000c0,B5
        MVK     .S2     -1,B4
        STW     .D2T2   B4,*B5
        STW     .D2T2   B4,*+B5[1]
        STW     .D2T2   B4,*+B5[2]
        MVKL    .S2     0xeaffffff,B4
        MVKH    .S2     0xeaffffff,B4
        STW     .D2T2   B4,*+B5[3]

        MVK     .L1     0,A4                    ; checksum
        MVK     .L1     0,A5                    ; faults serviced

* L2 page 1: everything except supervisor write.
        MVKL    .S2     L2MPPA1,B5
        MVKH    .S2     L2MPPA1,B5
        MVK     .S2     0x12f,B4
        STW     .D2T2   B4,*B5
        NOP     4

* The store must be discarded and must fault.
        MVKL    .S2     TARGET,B5
        MVKH    .S2     TARGET,B5
        MVKL    .S2     0x5a5a5a5a,B4
        MVKH    .S2     0x5a5a5a5a,B4
        STW     .D2T2   B4,*B5
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        ADDK    .S1     1,A5

* Reading is still granted, and the word must be untouched.
        MVKL    .S2     TARGET,B5
        MVKH    .S2     TARGET,B5
        LDW     .D2T2   *B5,B4
        NOP     4
        FOLD    B4

* Grant supervisor write and repeat: the store must land this time and raise nothing.
        MVKL    .S2     L2MPPA1,B5
        MVKH    .S2     L2MPPA1,B5
        MVK     .S2     0x1ff,B4
        STW     .D2T2   B4,*B5
        NOP     4
        MVKL    .S2     TARGET,B5
        MVKH    .S2     TARGET,B5
        MVKL    .S2     0x5a5a5a5a,B4
        MVKH    .S2     0x5a5a5a5a,B4
        STW     .D2T2   B4,*B5
        NOP     9
        NOP     9
        LDW     .D2T2   *B5,B4
        NOP     4
        FOLD    B4

        MVC     .S2     EFR,B4
        NOP     1
        FOLD    B4
        MV      .L2X    A5,B4
        FOLD    B4
        B       .S2     B10
        NOP     5

* The exception vector. A protection fault is external, so it folds the combiner's masked
* flags and the unit's fault registers, then clears the fault, the event and the flag.
exception_entry:
        MVC     .S2     EFR,B4
        NOP     1
        FOLD    B4
        MVKL    .S2     MEXPFLAG3,B7
        MVKH    .S2     MEXPFLAG3,B7
        LDW     .D2T2   *B7,B8
        NOP     4
        FOLD    B8
        MVKL    .S2     L2MPFAR,B7
        MVKH    .S2     L2MPFAR,B7
        LDW     .D2T2   *B7,B9
        LDW     .D2T2   *+B7[1],B11
        NOP     4
        FOLD    B9
        FOLD    B11

* Clear the fault status, the latched event and the exception flag, in that order.
        MVKL    .S2     L2MPFCLR,B7
        MVKH    .S2     L2MPFCLR,B7
        STW     .D2T2   B11,*B7
        MVKL    .S2     EVTCLR3,B7
        MVKH    .S2     EVTCLR3,B7
        STW     .D2T2   B8,*B7
        NOP     4
        MVC     .S2     B4,ECR
        NOP     1
        B       .S2     NRP
        NOP     5

* A nested exception vectors through the service table's RESET entry, which nothing here
* should reach.
reset_entry:
        MVKL    .S1     0xdead0004,A4
        MVKH    .S1     0xdead0004,A4
        B       .S2     B10
        NOP     5

        .align 1024
ist:
        B       .S1     reset_entry
        NOP     5
        .align 32
        B       .S1     exception_entry
        NOP     5
        .align 32
