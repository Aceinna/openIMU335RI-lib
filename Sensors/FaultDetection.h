#ifndef FAULT_DETECTION_H
#define FAULT_DETECTION_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "GlobalConstants.h"
#include "Indices.h"

/**
 * @brief
 *
 */
void    FaultDetectionScaled(void);

// algorith defaults
enum {
//********************* Defaults ***************************************
    ACCEL_CONSISTENCY_DETECT_THRESHOLD_DEFAULT = 100  ,     // 100   (mG)
    RATE_CONSISTENCY_DETECT_THRESHOLD_DEFAULT  = 10000 ,    // 10000  (mdeg/s)

    ACCEL_CONSISTENCY_DETECT_PERIOD_DEFAULT    = 300 ,      // 300 (msec)
    RATE_CONSISTENCY_DETECT_PERIOD_DEFAULT    =  300 ,      // 300 (msec)

//********************* Minimum ***************************************
    ACCEL_CONSISTENCY_DETECT_THRESHOLD_MIN     = 50  ,      // 50   (mG)
    RATE_CONSISTENCY_DETECT_THRESHOLD_MIN      = 500 ,      // 500  (mdeg/s)

    ACCEL_CONSISTENCY_DETECT_PERIOD_MIN       =  50 ,       // 50 (msec)
    RATE_CONSISTENCY_DETECT_PERIOD_MIN        =  50 ,       // 50 (msec)
};

// failure cause
enum{
    CAUSE_ACCEL_NOISE        =   1,
    CAUSE_RATE_NOISE         =   2,
    CAUSE_ACCEL_IMPROB       =   3,
    CAUSE_ACCEL_CONSISTENCY  =   4,
    CAUSE_RATE_CONSISTENCY   =   5,
};


typedef struct {
    int32_t chip1;
    int32_t chip2;
}comb_t;

#ifdef __cplusplus
}
#endif

#endif /* FAULT_DETECTION_H */
