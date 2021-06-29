#include "osapi.h"
#include "halAPI.h"
#include "stm32f4xx_conf.h"

static int32_t gIsrDisableCount;

/**********************************************
* @brief 
* 
* @return uint64_t 
***********************************************/
uint64_t  OS_GetCurrTimeStamp()
{
    return TIMER_GetCurrTimeStamp();

}

/**********************************************
* @brief 
* 
* @return uint64_t 
***********************************************/
uint64_t  OS_GetDacqTimeStamp()
{
    return TIMER_GetDacqTimeStamp();
}

/**********************************************
* @brief 
* 
* @param dacqInterval ---
***********************************************/
void OS_StartTimers(uint32_t const dacqInterval)
{
    // Configure and enable timers
    TIMER_SetDacqInterval(dacqInterval);
    TIMER_InitOSTick();
}

/**********************************************
* @brief 
* 
* @return BOOL 
***********************************************/
BOOL OS_IsDacqOverrun()
{
    return TIMER_IsDacqOverrun();
}

/**********************************************
* @brief 
* 
***********************************************/
void OS_WaitForDacqTick()
{
    TIMER_WaitForNewDacqTick();
} 

/**********************************************
* @brief 
* 
* @param ms ---
***********************************************/
void OS_Delay(int32_t const ms)
{
    TIMER_DelayMs((uint32_t)ms);
}

/**********************************************
* @brief 
* 
***********************************************/
void   OSEnterISR() 
{  gIsrDisableCount++; 
   __ASM  ("cpsid i");
};

/**********************************************
* @brief 
* 
***********************************************/
void   OSExitISR() 
{    gIsrDisableCount--; 
     if (gIsrDisableCount == 0) {
         __ASM  ("cpsie i"); 
     }
}

/**********************************************
* @brief 
* 
***********************************************/
void  ENTER_CRITICAL() 
    { 
        gIsrDisableCount++; 
        __ASM  ("cpsid i");
}

/**********************************************
* @brief 
* 
***********************************************/
void  EXIT_CRITICAL() 
{   gIsrDisableCount--; 
    if (gIsrDisableCount == 0) { 
        __ASM  ("cpsie i"); 
    }
}
