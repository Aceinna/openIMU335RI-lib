/******************************************************************************
 * @file StateIndices.h
 * @author Joe Motyka
 * @brief State indices of EKF states and measurements.
 * @version 1.0
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef STATEINDICES_H
#define STATEINDICES_H

#ifdef __cplusplus
extern "C" {
#endif

enum { NUMBER_OF_EKF_STATES = 16 };

enum {
    STATE_RX  = 0,
    STATE_RY  = 1,
    STATE_RZ  = 2,
    STATE_VX  = 3,
    STATE_VY  = 4,
    STATE_VZ  = 5,
    STATE_Q0  = 6,
    STATE_Q1  = 7,
    STATE_Q2  = 8,
    STATE_Q3  = 9,
    STATE_WBX = 10,
    STATE_WBY = 11,
    STATE_WBZ = 12,
    STATE_ABX = 13,
    STATE_ABY = 14,
    STATE_ABZ = 15
};

enum {
    STATE_ROLL   = 6,
    STATE_PITCH  = 7,
    STATE_YAW    = 8
};

#ifdef __cplusplus
}
#endif

#endif /* STATEINDICES_H */
