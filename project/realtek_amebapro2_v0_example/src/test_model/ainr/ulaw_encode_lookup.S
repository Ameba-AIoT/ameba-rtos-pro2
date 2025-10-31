.syntax unified
.arch   armv7-m
.thumb
.global ulaw_encode_asm_16
.type   ulaw_encode_asm_16, %function

.macro encode_4_samples
    ldrh    r4, [r0], #2
    ldrh    r5, [r0], #2
    ldrh    r6, [r0], #2
    ldrh    r7, [r0], #2

    ldrb    r4, [r3, r4]
    ldrb    r5, [r3, r5]
    ldrb    r6, [r3, r6]
    ldrb    r7, [r3, r7]

    lsls    r5, r5, #8
    lsls    r6, r6, #16
    lsls    r7, r7, #24

    orrs    r4, r4, r5
    orrs    r6, r6, r7
    orrs    r4, r4, r6
.endm

ulaw_encode_asm_16:
    push    {r4-r11, lr}

loop16:
    encode_4_samples
    mov     r8, r4

    encode_4_samples
    mov     r9, r4

    encode_4_samples
    mov     r10, r4

    encode_4_samples
    mov     r11, r4

    stmia   r1!, {r8-r11}

    subs    r2, r2, #16
    bne     loop16

    pop     {r4-r11, pc}
    .size   ulaw_encode_asm_16, . - ulaw_encode_asm_16