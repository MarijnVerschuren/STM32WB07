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


#endif //STM32WB07_TYPES_H
