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
	bx lr
.size GPIO_to_int, .-GPIO_to_int

.section .text.int_to_GPIO
.type int_to_GPIO, %function
int_to_GPIO:
	ldr r1, =AHBPERIPH_BASE
	movs r2, #0b1
	ands r0, r0, r2
	lsls r0, #20
	adds r0, r0, r1
	bx lr
.size int_to_GPIO, .-int_to_GPIO
