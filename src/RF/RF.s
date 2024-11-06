.syntax unified
.cpu cortex-m0
.fpu softvfp
.thumb


.global blue_unit_conversion


// blue_unit_conversion
.section .text.blue_unit_conversion
.type blue_unit_conversion, %function
blue_unit_conversion:
    push    {r4, lr}
    cmp     r0, r2
    bls     mul32
    uxth    r2, r1
    lsrs    r3, r0, #16
    lsrs    r1, r1, #16
    mov     r4, r1
    muls    r1, r3, r1
    uxth    r0, r0
    muls    r3, r2, r3
    muls    r4, r0, r4
    muls    r0, r2, r0
    adds    r3, r3, r4
    lsls    r2, r3, #16
    lsrs    r3, r3, #16
    adds    r0, r2, r0

    movs    r2, #128
    lsls    r2, r2, #13
    movs    r3, #0
    adds    r2, r2, r0
    adcs    r3, r1
    lsrs    r2, r2, #21
    lsls    r0, r3, #11

    pop {r4, pc}

    bx lr
.size blue_unit_conversion, .-blue_unit_conversion


// mul32
.section .text.mul32
.type mul32, %function
mul32:
    muls    r0, r1, r0
    movs    r2, #128
    lsls    r2, r2, #13
    adds    r2, r2, r0
    lsrs    r2, r2, #21

    pop {r4, pc}

    bx lr
.size mul32, .-mul32
