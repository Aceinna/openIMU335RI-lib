
#include <stdint.h>
#include "GlobalConstants.h"
#include "halAPI.h"



/************************************************************************/
/* Constants                                                            */
/************************************************************************/
static uint32_t const PAINT_VALUE = 0xC5C5C5C5U;

/************************************************************************/
/* Static variables                                                     */
/************************************************************************/
static uint32_t * stackMaxTop;
static uint32_t areaToCheck = 0U;

/************************************************************************/
/* Public functions                                                     */
/************************************************************************/

/* *
 * Paint the stack starting from MAX_STACK_SIZE to STACK_POINTER
 * where,
 *  MAX_STACK_SIZE = _ebss (end of .bbs section) + heap_size;
 *  This is assuming heap size is fixed and doesnt grow
 * */

/**********************************************
* @brief
*
***********************************************/
void HW_FillStackPattern()
{
	// Tolerance is in percentage, range is [1,100]
	// Stack overflow error when 10% of stack is remaining
	static uint32_t const TOLERANCE =  10U;

    uint32_t const * heap_size;
    uint32_t const * bss_end;
    heap_size = (uint32_t const * )&_Min_Heap_Size;
    bss_end = (uint32_t const * )&_ebss;

    // Set stack max top
    uint32_t const topOfStack = (uint32_t)bss_end + (uint32_t)heap_size;
    stackMaxTop = (uint32_t *)topOfStack;
    uint32_t * itr = (uint32_t *)topOfStack;

    // Set stack base
    uint32_t const * const stackBase = (uint32_t const * const)&_estack;

    // Check top TOLERANCE % of ((Stack End - Stack Start) / Word)
    areaToCheck = (uint32_t) ((((uint32_t)stackBase - (uint32_t)stackMaxTop)/sizeof(uint32_t)) / (uint32_t)TOLERANCE);

    // Calculate area to paint
    uint32_t const areaToPaint = areaToCheck;

    // Paint
    for(uint32_t i = 0U; i < areaToPaint; i++){
        *itr = PAINT_VALUE;
		itr++;
    }
}


// This method checks top region of the stack
// Returns true when value other than PAIN_VALUE found
// Returns false when critical stack region is intact
/**********************************************
* @brief
*
* @return BOOL
***********************************************/
BOOL HW_IsStackOverflow()
{
    uint32_t * itr = (uint32_t *)stackMaxTop;

    for (uint32_t i = 0U; i < areaToCheck; i++){
        if (*itr != PAINT_VALUE){
            return TRUE;
        }
		itr++;
    }
    return FALSE;
}

// function memorySelfTest moved to bootloader where it belongs
