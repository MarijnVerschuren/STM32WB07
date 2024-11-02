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

#endif //STM32WB07_TYPES_H
