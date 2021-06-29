/******************************************************************************
* @file bsp.c
* @brief
* File description:
*		Use the file to Configure Cortex M3
******************************************************************************/
#include "stm32f4xx_conf.h"
#include "bsp.h"
#include "configureGPIO.h"
#include "SpiBitBang.h"
#include "eepromAPI.h"
#include "halAPI.h"

GPIO_TypeDef* PORT_A;
GPIO_TypeDef* PORT_B;
GPIO_TypeDef* PORT_C;
GPIO_TypeDef* PORT_D;


/*******************************************
 * @brief 
 * 
********************************************/
static void InitGpioReferences()
{
    PORT_A = GPIO_GetInstancePtr(GPIO_A);
    PORT_B = GPIO_GetInstancePtr(GPIO_B);
    PORT_C = GPIO_GetInstancePtr(GPIO_C);
    PORT_D = GPIO_GetInstancePtr(GPIO_D);
}

/**********************************************
 * @brief 
 * 
 ***********************************************/
 static void InitSystemTimer(void)
 {
     /// need 1s interrupt as well from GPS, hooked in to the system to force
     ///   sampling on that boundary.
     SysTick_Config(SystemCoreClock / 1000U);
 }


/**********************************************
* @brief 
* 
***********************************************/
static void BSP_init(void)
{
    int32_t i;

	InitSystemTimer();

    for(i = WWDG_IRQn; i <= FPU_IRQn; i++){
         NVIC_SetPriority((IRQn_Type)i, 0x02U);
    }

    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
}

/**********************************************
* @brief Set the Int Vector Offset object
* 
* @param offset ===
***********************************************/
static void SetIntVectorOffset(uint32_t const offset)
{
	 NVIC_SetVectorTable(NVIC_VectTab_FLASH, offset);   
}

/**********************************************
* @brief 
* 
***********************************************/
void HW_Init(void)  
{
    static uint32_t const  APP_NVIC_OFFSET = 0X10000U;

    // Initialize the system clock, PLL, etc
    InitGpioReferences();
    SystemInit();            // system_stm32f2xx.c
    EEPROM_Init();
    TIMER_Init();
    HW_InitADC();


    SetIntVectorOffset(APP_NVIC_OFFSET);

    /*int32_t tmp = */SystemCoreClockUpdate(); // system_stm32f2xx.c

    // In the case of the HSI clock configuration, change "system_stm32fxx.c"
    //   based on the Excel-generated file and comment out RCC_config() in
    //   BSP_init.  The variable 'tmp' is set based on whether the internal or
    //   external clock is used by the processor.  Both will generate
    //   approximately a 120 MHz system clock using PLLs.
    BSP_init();              // bsp.c
    
    ControlPortInit();
    RCC_ClearFlag(); ///< reset flags - stm32f2xx_rccc.c
    // ---------------------------- BOOT CAUSE MESSAGING -----------------------
    /// Initialize data-ready (DR), configuration, and (1-PPS) pins as input pins
    ///   (the board is configured according to the signal levels on these pins)
    InitBoardConfiguration_GPIO();  // configureGPIO.c
    SpiBitBangInit();
}


/**********************************************
* @brief 
* 
***********************************************/
void    BoardConfigureSpiBitBangInterface()
{
    GPIO_InitSpiBitBangInterface();
}

/**********************************************
* @brief 
* 
***********************************************/
void HW_SystemReset(void)
{
    *((uint32_t *)0xE000ED0C) = 0x05FA0004U; 
}

/**********************************************
* @brief 
* 
***********************************************/
void HW_InitWatchdod()
{
    IWDG_Enable();
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_16);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
}

/**********************************************
* @brief 
* 
***********************************************/
void HW_FeedWatchdog()
{
    IWDG_ReloadCounter();
}


