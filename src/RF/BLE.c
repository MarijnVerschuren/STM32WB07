//
// Created by quvx on 05/11/24.
//

#include "RF/BLE.h"

void ble_init(void) {
    /* reset AES and turn on */

}

void UTIL_SEQ_RegTask(UTIL_SEQ_bm_t TaskId_bm, uint32_t Flags, void (*Task)( void ))
{
    (void)Flags;
    UTIL_SEQ_ENTER_CRITICAL_SECTION();

    TaskCb[SEQ_BitPosition(TaskId_bm)] = Task;

    UTIL_SEQ_EXIT_CRITICAL_SECTION();

    return;
}
