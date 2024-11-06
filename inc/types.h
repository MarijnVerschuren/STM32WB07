//
// Created by marijn on 9/4/24.
//

#ifndef STM32WB07_TYPES_H
#define STM32WB07_TYPES_H


typedef char				int8_t;
typedef short				int16_t;
typedef long				int32_t;
typedef long long			int64_t;

typedef unsigned char		uint8_t;
typedef unsigned short		uint16_t;
typedef unsigned long		uint32_t;
typedef unsigned long long	uint64_t;

#define _I	volatile const
#define _O	volatile
#define _IO	volatile

#define NULL ((void*)0x00000000UL)

typedef struct {
	uint32_t	clk		: 4;		// clock bus
	uint32_t	periph	: 8;		// peripheral
	uint32_t	misc	: 8;		// misc info (tim channel number etc...)
	uint32_t	port	: 4;		// GPIO port
	uint32_t	pin		: 4;		// GPIO pin
	uint32_t	alt		: 4;		// alternate function
} dev_pin_t;  // 32 bit

// TODO: assemble all functions that contain these
__attribute__((always_inline)) static inline void __NOP(void) {
	__asm volatile ("nop");
}
__attribute__((always_inline)) static inline void __ISB(void) {
	__asm volatile ("isb 0xF":::"memory");
}
__attribute__((always_inline)) static inline void __DSB(void) {
	__asm volatile ("dsb 0xF":::"memory");
}
__attribute__((always_inline)) static inline void __DMB(void) {
	__asm volatile ("dmb 0xF":::"memory");
}
__attribute__((always_inline)) static inline void __IRQ_D(void) {
	__asm volatile ("cpsid i":::"memory");
}
__attribute__((always_inline)) static inline void __IRQ_E(void) {
	__asm volatile ("cpsie i":::"memory");
}
__attribute__((always_inline)) static inline void __SET_PRIMASK(uint32_t primask) {
	__asm volatile ("MSR primask, %0" : : "r" (primask) : "memory");
}
__attribute__((always_inline)) static inline uint32_t __GET_PRIMASK(void) {
	uint32_t r;
	__asm volatile ("MRS %0, primask" : "=r" (r) );
	return(r);
}

#endif //STM32WB07_TYPES_H
