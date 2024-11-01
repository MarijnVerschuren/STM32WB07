.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"

/*!<
 * functions
 */
.global GPIO_to_int
.global int_to_GPIO
.global enable_GPIO
.global disable_GPIO


/*!< misc */
.section .text.GPIO_to_int
.type GPIO_to_int, %function
GPIO_to_int:
	ldr r1, =AHBPERIPH_BASE
	movs r2, #0b1
	subs r0, r0, r1
	lsrs r0, #20
	ands r0, r0, r2
	adds r0, r0, #2
	bx lr
.size GPIO_to_int, .-GPIO_to_int

.section .text.int_to_GPIO
.type int_to_GPIO, %function
int_to_GPIO:
	ldr r1, =AHBPERIPH_BASE
	movs r2, #0b1
	subs r0, r0, #2
	ands r0, r0, r2
	lsls r0, #20
	adds r0, r0, r1
	bx lr
.size int_to_GPIO, .-int_to_GPIO


/*!< init / disable */
//.section .text.enable_GPIO
//.type enable_GPIO, %function
//enable_GPIO:
//	mov r2, lr
//	bl GPIO_to_int
//	movs r1, #1
//	lsls r0, r1, r0
//	ldrs ip, =RCC_BASE
//	ldrs r1, [ip, RCC_AHB1ENR]
//	orrs r1, r1, r0
//	strs r1, [ip, RCC_AHB1ENR]
//	bx r2
//.size enable_GPIO, .-enable_GPIO
//
//.section .text.disable_GPIO
//.type disable_GPIO, %function
//disable_GPIO:
//	mov r2, lr
//	bl GPIO_to_int
//	movs r1, #1
//	lsls r0, r1, r0
//	ldrs ip, =RCC_BASE
//	ldrs r1, [ip, RCC_AHB1ENR]
//	bics r1, r1, r0
//	strs r1, [ip, RCC_AHB1ENR]
//	bx r2
//.size disable_GPIO, .-disable_GPIO
