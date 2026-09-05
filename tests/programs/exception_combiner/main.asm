* The megamodule exception combiner and the maskable external exception input (SPRU871K 7.2.4,
* SPRU732J 6.1.2), neither of which any compiled program reaches.
*
* EXPMASK0-3 at 0x018000C0 select which of the 128 latched system events feed the single EXCEP
* input, with the same polarity as EVTMASK: a set bit keeps the event out. MEXPFLAG0-3 at
* 0x018000E0 give the masked view, which is how the handler finds out what fired. EXCEP needs
* TSR.XEN, TSR.GEE and IER.NMIE, sets EFR.EXF, and saves the context with NTSR.HWE set. Taking
* it clears TSR.XEN, which comes back with the rest of TSR on `B NRP`.
*
* The EFR flag is not cleared by taking the exception, so an input that is still asserted
* cannot re-enter the handler until ECR acknowledges it - the handler here clears the event
* first and acknowledges second.
*
* Three steps: an unmasked event must vector, a masked one must not, and unmasking it later
* must vector at once because the flag stayed latched.
        .nocmp
        .global main
        .sect ".text"

* Fold one observation into the checksum. A rotate rather than a shift, so that nothing
* observed early is shifted out before the end: with a plain shift the first exception the
* program takes would not be checked at all.
FOLD    .macro  src
        SHL     .S1     A4,5,A6
        SHRU    .S1     A4,27,A7
        OR      .L1     A6,A7,A4
        XOR     .L1X    A4,src,A4
        .endm

EVTSET0     .set    0x01800020
EVTCLR0     .set    0x01800040
EXPMASK0    .set    0x018000c0
MEXPFLAG0   .set    0x018000e0

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

* Everything masked out of the exception combiner except event 20.
        MVKL    .S2     EXPMASK0,B5
        MVKH    .S2     EXPMASK0,B5
        MVKL    .S2     0xffefffff,B4
        MVKH    .S2     0xffefffff,B4
        STW     .D2T2   B4,*B5
        MVK     .S2     -1,B4
        STW     .D2T2   B4,*+B5[1]
        STW     .D2T2   B4,*+B5[2]
        STW     .D2T2   B4,*+B5[3]

        MVK     .L1     0,A4                    ; checksum
        MVK     .L1     0,A5                    ; exceptions serviced

* Raise event 20: it is unmasked, so EXCEP asserts and the handler runs inside the pad.
        MVKL    .S2     EVTSET0,B5
        MVKH    .S2     EVTSET0,B5
        MVKL    .S2     0x00100000,B4
        MVKH    .S2     0x00100000,B4
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

* Raise event 21, which the combiner masks: nothing may happen and EFR must stay clear.
* The handler runs on the B-side scratch registers, so the pointer is rebuilt rather than
* carried across a pad the handler may have run in.
        MVKL    .S2     EVTSET0,B5
        MVKH    .S2     EVTSET0,B5
        MVKL    .S2     0x00200000,B4
        MVKH    .S2     0x00200000,B4
        STW     .D2T2   B4,*B5
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        NOP     9
        MVC     .S2     EFR,B4
        NOP     1
        FOLD    B4

* Unmask event 21. Its flag is still latched, so EXCEP asserts as soon as the mask changes.
        MVKL    .S2     EXPMASK0,B5
        MVKH    .S2     EXPMASK0,B5
        MVKL    .S2     0xffdfffff,B4
        MVKH    .S2     0xffdfffff,B4
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

        MVC     .S2     EFR,B4
        MVC     .S2     IERR,B5
        MVC     .S2     TSR,B6
        NOP     1
        FOLD    B4
        FOLD    B5
        FOLD    B6
        MV      .L2X    A5,B4
        FOLD    B4
        B       .S2     B10
        NOP     5

* The exception vector. It folds EFR, NTSR, the masked exception flags and IERR, clears the
* events that fired, acknowledges the flag and returns - the restore brings TSR.XEN back.
exception_entry:
        MVC     .S2     EFR,B4
        MVC     .S2     NTSR,B5
        MVKL    .S2     MEXPFLAG0,B7
        MVKH    .S2     MEXPFLAG0,B7
        LDW     .D2T2   *B7,B8
        MVC     .S2     IERR,B6
        NOP     4
        FOLD    B4
        FOLD    B5
        FOLD    B8
        FOLD    B6
        MVKL    .S2     EVTCLR0,B7
        MVKH    .S2     EVTCLR0,B7
        STW     .D2T2   B8,*B7                  ; clear exactly the events that fired
        NOP     4
        MVC     .S2     B4,ECR                  ; then acknowledge the flag
        NOP     1
        B       .S2     NRP
        NOP     5

* A nested exception vectors through the service table's RESET entry, which nothing here
* should reach.
reset_entry:
        MVKL    .S1     0xdead0003,A4
        MVKH    .S1     0xdead0003,A4
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
