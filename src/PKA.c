//
// Created by quvx on 05/11/24.
//

#include "PKA.h"


/*!<
 * functions
 * */
void pka_init(void) {
    /* set & clear PKA */
    PKA->CSR |= 0b1U << 8U;
    PKA->CSR &= ~(0b1U << 8U);

    /* enable clock */
    RCC->AHBENR |= 0b1U << 16U;

    /* set NVIC */
    NVIC_set_IRQ_priority(PKA_IRQn, 1);
    NVIC_enable_IRQ(PKA_IRQn);

    /* set & clear intterupt status */
    PKA->ISR |= 0x0DU;
    PKA->ISR &= ~ 0x0DU;

    /* enable interrupts */
    PKA->IEN |= 0x0DU;
}
