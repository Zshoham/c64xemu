* The Supervisor/User privilege model (SPRU732J chapter 8), which no compiled program reaches:
* cl6x never leaves Supervisor mode and never touches the restricted control registers.
*
* User mode is entered the only way the architecture allows - by returning from an exception
* whose saved context has TSR.CXM set - and the restored mode takes effect coincident with the
* branch target, not in the delay slots of the `B NRP`.
*
* Four things are then checked, each of which must raise an exception and resume afterwards:
*
*   MVC from IER    - a fully restricted control register: IERR reports the resource access
*   MVC to ISTP       and the privilege together, RAX and PRX (SPRU732J 8.2.4.1)
*   IDLE            - a restricted instruction: IERR reports OPX with PRX (8.2.4.3)
*   B IRP
*
* and two that must not raise but must be filtered:
*
*   MVC to TSR      - only GIE and SGIE are writable in User mode, so a write of all ones
*                     leaves GEE and CXM exactly as they were
*   MVC from CSR    - readable, and reports the GIE that the TSR write just set
*
* `SWENR` then requests the supervisor through REP, and that handler clears NTSR.CXM so its
* return lands back in Supervisor mode. Every observed EFR, NTSR and IERR is folded into the
* checksum, along with the number of times User mode resumed after a fault.
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

main:
        MV      .L2     B3,B10                  ; caller's return address

        MVKL    .S2     ist,B4
        MVKH    .S2     ist,B4
        MVC     .S2     B4,ISTP

        MVKL    .S2     rep_handler,B4
        MVKH    .S2     rep_handler,B4
        MVC     .S2     B4,REP

        MVC     .S2     TSR,B4
        NOP     1
        SET     .S2     B4,2,2,B4               ; TSR.GEE
        MVC     .S2     B4,TSR
        NOP     4

        MVK     .L1     0,A4                    ; checksum
        MVK     .L1     0,A5                    ; resumptions in User mode

* Build the context that returns into User mode: GEE stays on, CXM says User.
        MVK     .S2     0x44,B4
        MVC     .S2     B4,NTSR
        MVKL    .S2     user_start,B4
        MVKH    .S2     user_start,B4
        MVC     .S2     B4,NRP
        NOP     2
        B       .S2     NRP
        NOP     5

user_start:
* A fully restricted control register, read then written.
        MVC     .S2     IER,B4
        ADDK    .S1     1,A5
        MVC     .S2     B4,ISTP
        ADDK    .S1     1,A5
* Restricted instructions.
        IDLE
        ADDK    .S1     1,A5
        B       .S2     IRP
        ADDK    .S1     1,A5
* Partially restricted: a write of all ones reaches GIE and SGIE and nothing else.
        MVK     .S2     -1,B4
        MVC     .S2     B4,TSR
        NOP     1
        MVC     .S2     TSR,B5
        MVC     .S2     CSR,B6
        NOP     1
        FOLD    B5
        FOLD    B6
* Request the supervisor through the restricted entry point.
        SWENR
        NOP     5

supervisor_done:
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

* The exception vector. Every fault here is internal, so NRP names the offending packet and
* the handler steps over it before returning to User mode.
exception_entry:
        MVC     .S2     EFR,B4
        MVC     .S2     NTSR,B5
        MVC     .S2     IERR,B6
        NOP     1
        FOLD    B4
        FOLD    B5
        FOLD    B6
        AND     .L2     B4,2,B0                 ; EFR.IXF
        NOP     1
 [B0]   MVC     .S2     NRP,B7
        NOP     1
 [B0]   ADDK    .S2     4,B7
        NOP     1
 [B0]   MVC     .S2     B7,NRP
        MVC     .S2     B4,ECR
        MVK     .L2     0,B7
        MVC     .S2     B7,IERR
        NOP     1
        B       .S2     NRP
        NOP     5

* SWENR lands here through REP. This is the way back to Supervisor: clearing NTSR.CXM makes
* the return install Supervisor mode at its target.
rep_handler:
        MVC     .S2     EFR,B4
        MVC     .S2     NTSR,B5
        NOP     1
        FOLD    B4
        FOLD    B5
        XOR     .L1     15,A4,A4
        CLR     .S2     B5,6,6,B5               ; TSR.CXM back to Supervisor
        MVC     .S2     B5,NTSR
        MVKL    .S2     supervisor_done,B5
        MVKH    .S2     supervisor_done,B5
        MVC     .S2     B5,NRP
        MVC     .S2     B4,ECR
        NOP     1
        B       .S2     NRP
        NOP     5

* A nested exception would vector through the service table's RESET entry, which nothing here
* should reach: landing on it poisons the result rather than running away.
reset_entry:
        MVKL    .S1     0xdead0002,A4
        MVKH    .S1     0xdead0002,A4
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
