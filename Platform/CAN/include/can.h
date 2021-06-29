/** ***************************************************************************
 * @file can.h control area network functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef CAN_H
#define CAN_H
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif


enum
{
    USER_CAN_IDE = 1,
    USER_CAN_RTR = 0,
};


#ifdef __cplusplus
}
#endif

#endif
