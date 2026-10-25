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

* SPRU871K Tables 2-12, 3-16, 4-23: cache register permissions are per register,
* not a blanket ban on User cache maintenance. Scratch registers do not affect FOLD.
WRREG   .macro  address,value
        MVKL    .S1     address,A8
        MVKH    .S1     address,A8
        MVKL    .S1     value,A9
        MVKH    .S1     value,A9
        STW     .D1     A9,*A8
        NOP     5
        .endm
RDREG   .macro  address,dst
        MVKL    .S1     address,A8
        MVKH    .S1     address,A8
        LDW     .D1     *A8,dst
        NOP     4
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

* Supervisor writes work. User must leave these values unchanged.
        WRREG   0x01848380,1                 ; MAR224
        RDREG   0x01848380,A16               ; Supervisor control: 1
        WRREG   0x01848380,0
        WRREG   0x01840000,1                 ; L2CFG: 32 KB
        RDREG   0x01840000,A17               ; Supervisor control: 1
        WRREG   0x01840020,1                 ; L1PCFG: 4 KB
        WRREG   0x01840040,1                 ; L1DCFG: 4 KB
* A second MAR holds a dirty line for the permitted User block-writeback control.
        WRREG   0x01848384,1                 ; MAR225
        WRREG   0xe1000000,0
        RDREG   0xe1000000,A8                ; allocate in L1D
        WRREG   0xe1000000,0x55              ; dirty only in L1D

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
* User writes to read-only cache registers have no effect. L1PCFG additionally
* records a local UW fault (SPRU871K 2.9.1.2, p.37); EXPMASK leaves it masked.
        WRREG   0x01848380,1
        WRREG   0x01840000,2
        WRREG   0x01840040,2
        WRREG   0x01840020,2
        WRREG   0x01845048,1                 ; L1DINV: Supervisor only
        WRREG   0x01845008,1                 ; L2INV: Supervisor only
* Freeze controls and block writeback remain writable by User.
        WRREG   0x01840044,1
        RDREG   0x01840044,A24
        WRREG   0x01840024,1
        RDREG   0x01840024,A25
        WRREG   0x01840044,0
        WRREG   0x01840024,0
        WRREG   0x01844000,0xe1000000         ; L2WBAR
        WRREG   0x01844004,1                  ; L2WWC: write dirty data to DDR
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
        RDREG   0x01848380,A18
        RDREG   0x01840000,A19
        RDREG   0x01840040,A20
        RDREG   0x01840020,A21
        RDREG   0x0184a400,A22               ; L1PMPFAR
        RDREG   0x0184a404,A23               ; L1PMPFSR: LOCAL | UW
* Discard copies without writing them back, then observe what the User block
* operation delivered to DDR. A refused block writeback would leave zero there.
        WRREG   0x01845048,1
        WRREG   0x01845008,1
        RDREG   0xe1000000,A26
        MVC     .S2     EFR,B4
        MVC     .S2     IERR,B5
        MVC     .S2     TSR,B6
        NOP     1
        FOLD    B4
        FOLD    B5
        FOLD    B6
        MV      .L2X    A5,B4
        FOLD    B4
* Include every cache observation in the program's externally asserted result.
        MV      .L2X    A16,B4
        FOLD    B4
        MV      .L2X    A17,B4
        FOLD    B4
        MV      .L2X    A18,B4
        FOLD    B4
        MV      .L2X    A19,B4
        FOLD    B4
        MV      .L2X    A20,B4
        FOLD    B4
        MV      .L2X    A21,B4
        FOLD    B4
        MV      .L2X    A22,B4
        FOLD    B4
        MV      .L2X    A23,B4
        FOLD    B4
        MV      .L2X    A24,B4
        FOLD    B4
        MV      .L2X    A25,B4
        FOLD    B4
        MV      .L2X    A26,B4
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
