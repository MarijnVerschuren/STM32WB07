.syntax unified
.thumb

.global IVT


.section .vector, "a", %progbits
.type IVT, %object
IVT:
/*!< ARM cortex-M0+ core interrupts */
.word _stack_end
.word reset_handler
.word NMI_handler
.word hard_fault_handler
.word memory_fault_handler
.word bus_fault_handler
.word exception_handler
.word 0x00000000
.word 0x00000000
.word 0x00000000
.word 0x00000000
.word SVC_handler
.word debug_handler
.word 0x00000000
.word pending_SV_handler
.word sys_tick_handler
/*!< STM32WB07 interrupts */
.word 0x00000000	// flash_handler
.word 0x00000000	// RCC_handler
.word 0x00000000	// PVD_handler
.word 0x00000000	// I2C1_handler
.word 0x00000000	// I2C2_handler
.word 0x00000000	// SPI1_handler
.word 0x00000000	// SPI2_handler
.word 0x00000000	// SPI3_handler
.word 0x00000000	// USART1_handler
.word 0x00000000	// LPUART1_handler
.word 0x00000000	// TIM1_handler
.word 0x00000000	// RTC_handler
.word 0x00000000	// ADC_handler
.word 0x00000000	// PKA_handler
.word 0x00000000	// -
.word GPIOA_handler
.word GPIOB_handler
.word 0x00000000	// DMA_handler
.word 0x00000000	// RADIO_TXRX_handler
.word 0x00000000	// -
.word 0x00000000	// RADIO_TIM_error_handler
.word 0x00000000	// -
.word 0x00000000	// -
.word 0x00000000	// RADIO_TIM_CPU_WKUP_handler
.word 0x00000000	// RADIO_TIM_TXRX_WKUP_handler
.word 0x00000000	// RADIO_TXRX_SEQ_handler
.word 0x00000000	// -
.word 0x00000000	// -
.word 0x00000000	// -
.word 0x00000000	// -


// default handler
.section .text.default_handler, "ax", %progbits
default_handler:
	b default_handler
.size default_handler, .-default_handler


// weak definitions
/*!< ARM cortex-M0+ core interrupts */
.weak NMI_handler
.thumb_set NMI_handler,			default_handler
.weak hard_fault_handler
.thumb_set hard_fault_handler,	default_handler
.weak memory_fault_handler
.thumb_set memory_fault_handler,default_handler
.weak bus_fault_handler
.thumb_set bus_fault_handler,	default_handler
.weak exception_handler
.thumb_set exception_handler,	default_handler
.weak SVC_handler
.thumb_set SVC_handler,			default_handler
.weak debug_handler
.thumb_set debug_handler,		default_handler
.weak pending_SV_handler
.thumb_set pending_SV_handler,	default_handler
.weak sys_tick_handler
.thumb_set sys_tick_handler,	default_handler

/*!< STM32WB07 interrupts */
.weak GPIOA_handler
.thumb_set GPIOA_handler,		default_handler
.weak GPIOB_handler
.thumb_set GPIOB_handler,		default_handler

