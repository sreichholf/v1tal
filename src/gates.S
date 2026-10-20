/* gates.S -- Thumb trampolines that let the LD_PRELOAD shim (addon.c) take over
 * four mid-function sites in the V1 AppProxy without any code-cave in the
 * binary itself.
 *
 * Mechanism (see TECHNICAL.md §4):
 *   * each site is overwritten with a 12-byte "movw r12,#lo / movt r12,#hi /
 *     bx r12 / nop" sequence (emitted at runtime by addon.c) that branches to
 *     one of the gate functions below;
 *   * a gate sets up the arguments for the C hook, calls it, and then branches
 *     to the *continuation* address inside AppProxy (fetched from a writable
 *     word so it stays independent of the exact binary build).
 *
 * All gates are entered with `bx`, so they must not rely on lr, and they must
 * leave the stack balanced.  The continuation word is set from addon.c.
 *
 * AppProxy is a non-PIE ET_EXEC, but we still keep every absolute address in a
 * writable data word so the shim is portable to a different AppProxy build.
 */
    .syntax unified
    .arch armv7-a
    .thumb

/* Everything here is private to the shim: hidden visibility keeps `ldr r12,=`
 * literals PC-relative (no dynamic relocations, no GOT). */
#define HIDDEN .hidden

/* ------------------------------------------------------------------ gates */
HIDDEN v1_gate_hdr
HIDDEN v1_gate_copy
HIDDEN v1_gate_deser
HIDDEN v1_gate_rcv
HIDDEN v1_hook_hdr
HIDDEN v1_cont_hdr
HIDDEN v1_hook_copy
HIDDEN v1_cont_copy
HIDDEN v1_hook_deser
HIDDEN v1_cont_deser
HIDDEN v1_hook_rcv
HIDDEN v1_rcv_tramp
HIDDEN v1_gate_rcv2
HIDDEN v1_rcv_tramp2

    .section .text.v1seg_gates,"ax",%progbits
    .align 2

/* header gate: r7 = this.  Writes the 28-byte IMAGE header. */
    .thumb_func
    .global v1_gate_hdr
    .type   v1_gate_hdr, %function
v1_gate_hdr:
    push    {r0, r1, r2, r3}
    mov     r0, r7
    ldr     r12, =v1_hook_hdr
    ldr     r12, [r12]
    blx     r12
    pop     {r0, r1, r2, r3}
    ldr     r12, =v1_cont_hdr
    ldr     r12, [r12]
    bx      r12
    .ltorg
    .size   v1_gate_hdr, . - v1_gate_hdr

/* copy gate: r7 = this, r4 = dst (this+76).  Returns the new dst in r4. */
    .thumb_func
    .global v1_gate_copy
    .type   v1_gate_copy, %function
v1_gate_copy:
    push    {r0, r1, r2, r3}
    mov     r0, r7
    mov     r1, r4
    ldr     r12, =v1_hook_copy
    ldr     r12, [r12]
    blx     r12
    mov     r4, r0
    pop     {r0, r1, r2, r3}
    ldr     r12, =v1_cont_copy
    ldr     r12, [r12]
    bx      r12
    .ltorg
    .size   v1_gate_copy, . - v1_gate_copy

/* deserialize gate: r7 = this, r5 = block pointer. */
    .thumb_func
    .global v1_gate_deser
    .type   v1_gate_deser, %function
v1_gate_deser:
    push    {r0, r1, r2, r3}
    mov     r0, r7
    mov     r1, r5
    ldr     r12, =v1_hook_deser
    ldr     r12, [r12]
    blx     r12
    pop     {r0, r1, r2, r3}
    ldr     r12, =v1_cont_deser
    ldr     r12, [r12]
    bx      r12
    .ltorg
    .size   v1_gate_deser, . - v1_gate_deser

/* receive gate: r0 = AppProxyServer*, r1 = json text.
 *   hook returns  const char* (unchanged / rewritten) -> run the original body
 *                  NULL                                -> suppress the original
 * The original body is entered through v1_rcv_tramp (built at runtime: the
 * overwritten prologue bytes followed by a branch back into AppProxy). */
    .thumb_func
    .global v1_gate_rcv
    .type   v1_gate_rcv, %function
v1_gate_rcv:
    /* 24-byte (6-register) push, NOT 5: this gate is entered from a C function
     * entry, where AAPCS guarantees sp is 8-aligned.  Pushing 5 registers
     * (20 bytes) left sp == 4 (mod 8) for the hook and for the trampolined
     * original body, so glibc's va_arg(long long) in reply_to_app's
     * snprintf("{\"id\":%llu,...}") rounded to the wrong slot (corrupt id, %s
     * through a stray pointer).  An even register count keeps sp 8-aligned
     * across both blx calls; r4 is spilled only for that alignment.  The saved
     * AppProxyServer* stays the first pushed word, so ldr r0,[sp,#0] is valid. */
    push    {r0, r1, r2, r3, r4, lr}
    ldr     r12, =v1_hook_rcv
    ldr     r12, [r12]
    blx     r12
    cmp     r0, #0
    beq     1f
    mov     r1, r0
    ldr     r0, [sp, #0]                 /* saved AppProxyServer* */
    ldr     r12, =v1_rcv_tramp
    ldr     r12, [r12]
    blx     r12
1:  pop     {r0, r1, r2, r3, r4, pc}
    .ltorg
    .size   v1_gate_rcv, . - v1_gate_rcv

/* receive gate #2: the per-message dispatch inside RcvMessageFromMIIO's split
 * loop (RcvSingleMessageFromMIIO(char const*), 0x37d08).  The entry gate defers
 * a datagram that carries several JSON objects, so this site handles each split
 * message individually with the same hook. */
    .thumb_func
    .global v1_gate_rcv2
    .type   v1_gate_rcv2, %function
v1_gate_rcv2:
    push    {r0, r1, r2, r3, r4, lr}
    ldr     r12, =v1_hook_rcv
    ldr     r12, [r12]
    blx     r12
    cmp     r0, #0
    beq     1f
    mov     r1, r0
    ldr     r0, [sp, #0]                 /* saved AppProxyServer* */
    ldr     r12, =v1_rcv_tramp2
    ldr     r12, [r12]
    blx     r12
1:  pop     {r0, r1, r2, r3, r4, pc}
    .ltorg
    .size   v1_gate_rcv2, . - v1_gate_rcv2

/* ------------------------------------------------- writable hook/cont words */
    .section .data.v1seg_gates,"aw",%progbits
    .align 2
    .global v1_hook_hdr
v1_hook_hdr:    .word 0
    .global v1_cont_hdr
v1_cont_hdr:    .word 0
    .global v1_hook_copy
v1_hook_copy:   .word 0
    .global v1_cont_copy
v1_cont_copy:   .word 0
    .global v1_hook_deser
v1_hook_deser:  .word 0
    .global v1_cont_deser
v1_cont_deser:  .word 0
    .global v1_hook_rcv
v1_hook_rcv:    .word 0
    .global v1_rcv_tramp
v1_rcv_tramp:   .word 0
    .global v1_rcv_tramp2
v1_rcv_tramp2:  .word 0

    .section .note.GNU-stack,"",%progbits
