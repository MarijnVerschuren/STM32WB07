//
// Created by marijn on 11/1/24.
//

#ifndef STM32WB07_VECTOR_H
#define STM32WB07_VECTOR_H
#include "types.h"

typedef void(*IVT_FUNC)(void);

typedef union {
	void*		ptr;
	IVT_FUNC	func;
} IVT_ELEMENT_t;

extern const IVT_ELEMENT_t IVT[];

#endif //STM32WB07_VECTOR_H
