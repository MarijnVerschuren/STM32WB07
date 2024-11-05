//
// Created by quvx on 05/11/24.
//

#ifndef PKA_H
#define PKA_H

#include "types.h"
#include "periph.h"
#include "NVIC.h"

typedef struct {
    _IO uint32_t	CSR;			/* control and status                0x00 */
    _IO uint32_t	ISR;			/* reload value                      0x04 */
    _IO uint32_t	IEN;			/* current value                     0x08 */
} PKA_t;
#define PKA						((PKA_t*)PKA_BASE)

/*!<
 * functions
 * */
void pka_init(void);

#endif //PKA_H
