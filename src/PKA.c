//
// Created by quvx on 05/11/24.
//

#include "PKA.h"


/*!<
 * functions
 * */
void pka_init(void) {  // TODO NOTE: VALID!!!
	/* enable clock */
	RCC->AHBENR |= 0b1U << 16U;

	/* set NVIC */
	NVIC_set_IRQ_priority(PKA_IRQn, 1);
	NVIC_enable_IRQ(PKA_IRQn);

    /* set & clear PKA */
    PKA->CSR |= 0x80;
    PKA->CSR &= ~0x80;

    /* set & clear interrupt status */
    PKA->ISR |= 0x0DU;
    PKA->ISR &= ~0x0DU;

    /* enable interrupts */
    PKA->IEN |= 0x0DU;	// TODO: cant be found in func (but is logical)
}
