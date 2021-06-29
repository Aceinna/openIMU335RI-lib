#include <stdint.h>
#include "GlobalConstants.h"

float64_t const  GRAVITY =  9.80665;
uint32_t  const   MAX_ITOW   = 604800000U;    // ms in a week (assuming exactly 24 hours in a day)
float64_t const  TWO_PI      =  6.283185307179586;
float64_t const  PI          =  3.141592653589793;
// Specify constants used to limit variables in the algorithm
float64_t const ONE_DEGREE_IN_RAD   =  0.017453292519943;
float64_t const TWO_DEGREES_IN_RAD  =  0.034906585039887;
float64_t const TWO_PT_FIVE_DEGREES_IN_RAD  =  0.043633231299858;
float64_t const THREE_DEGREES_IN_RAD =  0.052359877559830;
float64_t const FIVE_DEGREES_IN_RAD  = 0.087266462599716;
float64_t const SIX_DEGREES_IN_RAD   = 0.104719755119660;
float64_t const TEN_DEGREES_IN_RAD   = 0.17453292519943;
float64_t const TWENTY_DEGREES_IN_RAD =0.349065850398866;
float64_t const THREE_HUNDRED_EIGHTY_DEGREES_IN_RAD = 6.632251157578453;
float64_t const MIN_TO_MILLISECONDS = 60000.0;

uint32_t const MAXUINT32 = 4294967295U; ///< max unsigned 32 bit int32_t=> ((2^32)-1)
int32_t const MAXINT32 = 0x7FFFFFFF; ///< max signed 32 bit int32_t
int32_t const MININT32 = 0x80000000; ///< max negative signed 32 bit int32_t

uint16_t const MAXUINT16 = 65535U;  ///< max unsigned 16 bit int32_t=> ((2^16)-1)
int16_t const MAXINT16 = 32767;  ///< max signed 16 bit int32_t=> ((2^15)-1)
int16_t const MININT16 = -32768; ///< max negative signed 16 bit int32_t=> (-(2^15))
int16_t const INT16_LIMIT = 32765;

float64_t const RAD_TO_DEG = 57.29577951308232;
float64_t const DEG_TO_RAD = 0.017453292519943;

uint32_t const ROLL_INCIDENCE_LIMIT = 0x1000U;
uint32_t const PITCH_INCIDENCE_LIMIT = 0x1000U;
uint32_t const HARD_IRON_LIMIT = 8192U; // 0.25 G
uint32_t const SOFT_IRON_LIMIT = 6554U; // 20%
void     *null = 0;

/*******************************************
 * @brief 
 * 
 * @param x ==
 * @return float64_t 
********************************************/
float64_t SQUARE(float64_t const x)
{
    return x * x;
}
