/** ***************************************************************************
 * @file eeprom_api.h 
 * @Author
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef EEPROM_API_H
#define EEPROM_API_H

#include <stdint.h> 
#include "GlobalConstants.h"

/**********************************************
* @brief 
* 
* @param addr ==
* @param num ==
* @param source ==
* @return int32_t 
***********************************************/
extern int32_t EEPROM_WriteWords(uint16_t addr, uint16_t const num, uint8_t source[]) ;

/**********************************************
* @brief 
* 
* @param addr ==
* @param num ==
* @param destination ==
***********************************************/
extern void EEPROM_ReadByte(uint16_t addr, uint16_t num, uint8_t destination[]) ;

/**********************************************
* @brief 
* 
* @param addr ==
* @param num ==
* @param source ==
* @return int32_t 
***********************************************/
extern int32_t EEPROM_WriteByte(uint16_t addr, uint16_t num, uint8_t source[]) ;

/**********************************************
* @brief 
* 
* @param idx == 
* @param destination ==
* @param size ==
***********************************************/
extern void EEPROM_ReadCalibration(int32_t idx, void* destination, int32_t size);

/**********************************************
* @brief 
* 
* @param destination ==
***********************************************/
extern void EEPROM_ReadFactoryConfiguration(void* destination);

/**********************************************
* @brief 
* 
* @param offset ==
* @param num ==
* @param destination ==
* @return BOOL 
***********************************************/
extern BOOL EEPROM_ReadFromCalPartition( uint16_t offset, uint16_t num, uint8_t destination[]);
/**********************************************
* @brief 
* 
* @param offset ==
* @param num ==
* @param source ==
* @return BOOL 
***********************************************/
extern BOOL EEPROM_WriteToCalPartition(uint16_t offset, uint16_t num, uint8_t source[]);

/**********************************************
* @brief 
* 
* @return BOOL 
***********************************************/
extern BOOL EEPROM_LockCalSectors(void);

/**********************************************
* @brief 
* 
* @param payload == 
* @return BOOL 
***********************************************/
extern BOOL EEPROM_UnlockCalSectors(void *payload);

/**********************************************
* @brief 
* 
* @return BOOL 
***********************************************/
extern BOOL EEPROM_IsFactoryMode(void);

/**********************************************
* @brief 
* 
* @param idx ==
* @param size ==
* @return uint8_t* 
***********************************************/
extern uint8_t *EEPROM_GetCalTabPtr(int32_t idx, uint32_t *size);

/**********************************************
* @brief 
* 
* @param ptrToUserConfigInRam ==
* @param userConfigSize ==
* @return BOOL 
***********************************************/
extern BOOL  EEPROM_LoadUserConfig(uint8_t *ptrToUserConfigInRam, uint16_t *userConfigSize);

/**********************************************
* @brief 
* 
* @param userConfigSize ==
* @return BOOL 
***********************************************/
extern BOOL  EEPROM_ValidateUserConfig(uint16_t *userConfigSize);

/**********************************************
* @brief 
* 
* @param ptrToEcuSettingsStruct ==
* @param ecuSettingsStructSize ==
* @return BOOL 
***********************************************/
extern BOOL  EEPROM_SaveEcuSettings(uint8_t *ptrToEcuSettingsStruct, uint32_t ecuSettingsStructSize);

/**********************************************
* @brief 
* 
* @return BOOL 
***********************************************/
extern BOOL  EEPROM_IsAppStartedFirstTime(void);

/**********************************************
* @brief 
* 
* @return BOOL 
***********************************************/
extern int32_t EEPROM_PrepareToEnterBootloader(void);

/*******************************************
 * @brief 
 * 
 * @param idx ==
 * @param sensor ==
 * @param row ==
 * @param bCoeffs ==
********************************************/
extern void EEPROM_GetBMatrix(uint32_t const idx, uint32_t const sensor, uint32_t const row, float32_t bCoeffs[]);

/**********************************************
* @brief 
* 
***********************************************/
extern void  EEPROM_Init();

/*******************************************
 * @brief 
 * 
********************************************/
void  EEPROM_SetAppSignature();


/*******************************************
 * @brief 
 * 
 * @return int32_t 
********************************************/
int32_t EEPROM_PrepareToEnterBootloader(void);

/*******************************************
 * @brief 
 * 
 * @param addr ==
 * @return BOOL 
********************************************/
BOOL    EEPROM_GetLastSavedEcuAddress(uint8_t *addr);

/*******************************************
 * @brief 
 * 
 * @param ecuAddress ==
 * @return BOOL 
********************************************/
BOOL EEPROM_SaveEcuAddress(uint8_t ecuAddress);

/*******************************************
 * @brief 
 * 
********************************************/
uint16_t    EEPROM_GetApplicationCRC();


/*******************************************
 * @brief 
 * 
 * @param dm1Config ==
 * @return BOOL 
********************************************/
BOOL EEPROM_SaveDM1Config(uint64_t dm1Config);

/*******************************************
 * @brief 
 * 
 * @param dm1Config ==
 * @return BOOL 
********************************************/
BOOL    EEPROM_GetLastSavedDM1Config(uint64_t *dm1Config);

/*******************************************
 * @brief 
 * 
 * @param version ==
 * @return BOOL 
********************************************/
BOOL EEPROM_GetBootVersion(uint8_t *version);


/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL EEPROM_IsConfigLoaded(uint16_t size);

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL EEPROM_InvalidateConfigSignature(uint16_t size);

#endif /* S_EEPROM_H */ 


