.syntax unified
.arch   armv7-m
.thumb
.global ulaw_decode_asm_8
.type   ulaw_decode_asm_8, %function

.macro decode_4_samples
    ldr     r4, [r0], #4

    uxtb    r5, r4
    lsr     r4, r4, #8
    uxtb    r6, r4
    lsr     r4, r4, #8
    uxtb    r7, r4
    lsr     r4, r4, #8

    ldrh    r5, [r3, r5, lsl #1]
    ldrh    r6, [r3, r6, lsl #1]
    ldrh    r7, [r3, r7, lsl #1]
    ldrh    r4, [r3, r4, lsl #1]

    lsl     r6, r6, #16
    orr     r5, r5, r6
    
    lsl     r4, r4, #16
    orr     r7, r7, r4
.endm

ulaw_decode_asm_8:
    push    {r4-r11, lr}

loop8:
    decode_4_samples
    mov     r8, r5
    mov     r9, r7

    decode_4_samples
    mov     r10, r5
    mov     r11, r7

    stmia   r1!, {r8-r11}

    subs    r2, r2, #8
    bne     loop8

    pop     {r4-r11, pc}
    .size   ulaw_decode_asm_8, . - ulaw_decode_asm_8