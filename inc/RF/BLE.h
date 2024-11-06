//
// Created by quvx on 05/11/24.
//

#ifndef BLE_H
#define BLE_H

#include "types.h"

#define UTIL_SEQ_ENTER_CRITICAL_SECTION( )   UTILS_ENTER_CRITICAL_SECTION()
#define UTILS_ENTER_CRITICAL_SECTION() uint32_t primask_bit= __GET_PRIMASK();\
__IRQ_D()
#define UTIL_SEQ_EXIT_CRITICAL_SECTION( )    UTILS_EXIT_CRITICAL_SECTION()
#define UTILS_EXIT_CRITICAL_SECTION()  __SET_PRIMASK(primask_bit)


void ble_init(void);
void modules_init(void);
void blenvm_init(void)

#endif //BLE_H
