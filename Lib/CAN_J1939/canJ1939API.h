/*******************************************************************************
 * File:   canJ1938API.h
 ******************************************************************************/
/*******************************************************************************
Copyright 2020 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef CAN_J1939_API_H
#define CAN_J1939_API_H

#include <stdint.h>

/*******************************************
 * @brief 
 * 
 * @return int32_t 
********************************************/
extern int32_t      NeedResetUnit();

/*******************************************
 * @brief Get the Requested Packets object
 * 
 * @return uint16_t 
********************************************/
extern uint16_t     GetRequestedPackets();

/*******************************************
 * @brief 
 * 
********************************************/
extern void         ProcessCANMessages();

#ifdef UNIT_TEST
void setDelayCounter(int32_t cnt);
#endif

#endif /* J1939_API */
