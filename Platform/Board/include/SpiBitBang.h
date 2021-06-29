#ifndef SPI_BITBANG_H
#define SPI_BITBANG_H


#ifdef __cplusplus
 extern "C" {
#endif
 

#include "stdint.h"

enum {
    START_DATA_LEVEL = 0U,
    START_CLK_LEVEL  = 1U,
    SPI_MIN_SPEED    = 20U,
    SPI_MAX_SPEED    = 2U
};


/*******************************************
 * @brief 
 * 
 * @param reg ==
 * @param dst1 ==
 * @param dst2 ==
 * @param dst3 ==
 * @param dst4 ==
 * @param len ==
********************************************/
void SpiBitBangReadTransaction(uint8_t const reg, uint8_t *dst1, uint8_t *dst2, uint8_t *dst3, uint8_t *dst4, int32_t len);

/*******************************************
 * @brief 
 * 
 * @param reg ==
 * @param src ==
 * @param len ==
********************************************/
void SpiBitBangWriteTransaction(uint8_t const reg, uint8_t* const src, int32_t const len);

/*******************************************
 * @brief 
 * 
********************************************/
void SpiBitBangInit();

/*******************************************
 * @brief 
 * 
 * @param prescaler ==
********************************************/
void SpiBitBangSetBaud(int16_t prescaler);

/*******************************************
 * @brief 
 * 
 * @param sensorsMap ==
********************************************/
void SpiBitBangSelectActiveSensors(uint16_t sensorsMap);

/*******************************************
 * @brief 
 * 
 * @param sensorsMap ==
********************************************/
void SpiBitBangSelectSaveActiveSensors(uint16_t sensorsMap);

/*******************************************
 * @brief 
 * 
********************************************/
void SpiBitBangRestoreActiveSensors();

#ifdef __cplusplus
}
#endif

#endif
