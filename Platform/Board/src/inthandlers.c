#include "stm32f4xx.h"
#include "boardDefinition.h"
#include "osapi.h"


/**********************************************
* @brief
*
***********************************************/
void NMI_Handler(void)
{
    while(1){};
}

static  struct sHardFaultStacked {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} HardFault;


/*******************************************
 * @brief 
 * 
 * @param pulFaultStackAddress ==
********************************************/
void prvGetRegistersFromStack( uint32_t pulFaultStackAddress[] )
{

    HardFault.r0 = pulFaultStackAddress[ 0 ];
    HardFault.r1 = pulFaultStackAddress[ 1 ];
    HardFault.r2 = pulFaultStackAddress[ 2 ];
    HardFault.r3 = pulFaultStackAddress[ 3 ];

    HardFault.r12 = pulFaultStackAddress[ 4 ];
    HardFault.lr = pulFaultStackAddress[ 5 ];
    HardFault.pc = pulFaultStackAddress[ 6 ];
    HardFault.psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; ){};
}

/**********************************************
* @brief
*
***********************************************/
void HardFault_Handler(void)
{

    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );

    /* Go to infinite loop when Hard Fault exception occurs */
   while (1){}
}


void EXTI1_IRQHandler(void)
{
	OSEnterISR();
	EXTI->PR = ACCEL_DATA_READY_EXTI_LINE;
	OSExitISR();
}


// Handle the magnetometer and accelerometer interrupts by checking the individual interrupt status
void EXTI9_5_IRQHandler(void)
{
    OSEnterISR();

    EXTI->PR = ACCEL_DATA_READY_EXTI_LINE;
    EXTI->PR = MAG_DATA_READY_EXTI_LINE;

    OSExitISR();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @return None
  */
static uint32_t  tick = 0U;
void SysTick_Handler(void)
{
    tick++;
}
