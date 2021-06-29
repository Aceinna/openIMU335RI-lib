#include "SpiBitBang.h"
#include "boardDefinition.h"
#include "bsp.h"

static int32_t   clockCycle = SPI_MIN_SPEED;
static uint16_t  spiNssMask = 0x0007U;       //all 4 pins 
static uint32_t spiNssSaveMask;
static int32_t  saved = 0;

/******************************************************************************
 * @brief
 * @param data  
 * @param len   
 *  
 ******************************************************************************/
static void  SpiDataOut(uint8_t data[10], int32_t const len)
{
    uint8_t  byte;
    uint8_t  level;
    uint8_t  mask = 0x80U;

    for(int32_t i = 0; i < len; i++){
        byte  = data[i];
        mask  = 0x80U;
        for(int32_t j = 0; j < 8; j++){
            level = mask & byte; 
            if(level){
                PORT_B->BSRRL = SPI_BITBANG_MOSI_PIN;
            }else{
                PORT_B->BSRRH = SPI_BITBANG_MOSI_PIN;
            }
            // clk low
            PORT_B->BSRRH     = SPI_BITBANG_SCK_PIN;
            for(int32_t ii = 0; ii < clockCycle; ii++){};
            // clk high
            PORT_B->BSRRL    = SPI_BITBANG_SCK_PIN;
            mask >>=1U;
            for(int32_t aa = 0; aa < clockCycle; aa++){};
        }
    }
}

/******************************************************************************
 * @brief
 * @param data   [in] 3 sample delay buffer
 * @param len    [in] sample new data to push onto the queue
 *  
 ******************************************************************************/
static void  SpiDataIn(uint8_t data[100], int32_t const len)
{
    uint8_t tmp;
    int32_t idx = 0;
    uint32_t const misoPins = (uint32_t)SPI_BITBANG_MISO_PINS;

    for(int32_t j = 0; j < len; j++){
        for(int32_t i = 0; i < 8; i++){
            // clk low
            PORT_B->BSRRH     = SPI_BITBANG_SCK_PIN;
            for(int32_t m = 0; m < clockCycle; m++){};
            // clk high
            PORT_B->BSRRL    = SPI_BITBANG_SCK_PIN;
            tmp = (uint8_t)(PORT_A->IDR & misoPins);
            data[idx] = tmp;
            idx++;
            for(int32_t m = 0; m < clockCycle; m++){};
        }
    }
}

/******************************************************************************
 * @brief
 *  
 ******************************************************************************/
static void  SpiChipSelect()
{
    GPIO_WriteBit(PORT_B, ((SPI_BITBANG_NSS_PINS) & spiNssMask), Bit_RESET);
}


/******************************************************************************
 * @brief
 *  
 ******************************************************************************/
static void SpiSetStartCondition()
{
// CLK
        GPIO_WriteBit(PORT_B, SPI_BITBANG_SCK_PIN, Bit_SET);
// data MOSI
    if(START_DATA_LEVEL){
        GPIO_WriteBit(PORT_B, SPI_BITBANG_MOSI_PIN, Bit_SET);
    }else{
        GPIO_WriteBit(PORT_B, SPI_BITBANG_MOSI_PIN, Bit_RESET);
    }
// CS
    GPIO_WriteBit(PORT_B, ((SPI_BITBANG_NSS_PINS) & spiNssMask), Bit_SET);
}

static uint8_t tmpBuf[1000];

/******************************************************************************
 * @brief
 * @param dst1   [in] 3 sample delay buffer
 * @param dst2   [in] 3 sample delay buffer
 * @param dst3   [in] 3 sample delay buffer
 * @param dst4   [in] 3 sample delay buffer
 * @param len    [in] sample new data to push onto the queue
 *  
 ******************************************************************************/
static void SpiParseData(uint8_t dst1[10], uint8_t dst2[10], uint8_t dst3[10], uint8_t dst4[10], int32_t const len)
{
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    int32_t idx  = 0;
    
    for(int32_t i = 0; i < len; i++){
        byte1 = 0U;
        byte2 = 0U;
        byte3 = 0U;
        byte4 = 0U;
        for(int32_t j = 0; j < 8; j++){
            byte1 <<=1U;
            byte2 <<=1U;
            byte3 <<=1U;
            byte4 <<=1U;
            if(tmpBuf[idx] & SPI_BITBANG_MISO1_PIN){
                byte1 |= 1U;
            }
            if(tmpBuf[idx] & SPI_BITBANG_MISO2_PIN){
                byte2 |= 1U;
            }
            if(tmpBuf[idx] & SPI_BITBANG_MISO3_PIN){
                byte3 |= 1U;
            }
            idx++;
        }
        dst1[i]   = byte1;
        dst2[i]   = byte2;
        dst3[i]   = byte3;
        dst4[i]   = byte4;
    }
    

}

/******************************************************************************
 * @brief
 * @param reg    [in]
 * @param dst1   [in] 3 sample delay buffer
 * @param dst2   [in] 3 sample delay buffer
 * @param dst3   [in] 3 sample delay buffer
 * @param dst4   [in] 3 sample delay buffer
 * @param len    [in] sample new data to push onto the queue
 *  
 ******************************************************************************/
void SpiBitBangReadTransaction(uint8_t  const reg, uint8_t dst1[10], uint8_t dst2[10], uint8_t dst3[10], uint8_t dst4[10], int32_t const len)
{
    uint8_t tmp[10];
    tmp[0] =  reg;
    SpiChipSelect();
    SpiDataOut(tmp, 1);
    SpiDataIn(tmpBuf, len);
    SpiSetStartCondition();
    SpiParseData(dst1, dst2, dst3, dst4, len);
}

/******************************************************************************
 * @brief
 * @param reg    [in]
 * @param src    [in]
 * @param len    [in]
 *  
 ******************************************************************************/
void SpiBitBangWriteTransaction(uint8_t const reg, uint8_t* const src, int32_t const len)
{
    uint8_t tmp[10];
    tmp[0] =  reg;

    SpiChipSelect();
    SpiDataOut(tmp, 1);
    SpiDataOut(src, len);
    SpiSetStartCondition();
}

/******************************************************************************
 * @brief
 * @param prescaler [in] 
 *  
 ******************************************************************************/
void SpiBitBangSetBaud(int16_t const prescaler)
{
    clockCycle = prescaler;
}


 /******************************************************************************
 * @brief
 *  
 ******************************************************************************/
void SpiBitBangInit()
{

    BoardConfigureSpiBitBangInterface();
    SpiSetStartCondition();
}

/******************************************************************************
 * @brief
 * @param sensorsMap [in] 
 *  
 ******************************************************************************/
void SpiBitBangSelectActiveSensors(uint16_t const sensorsMap)
{
    spiNssMask = sensorsMap; 
} 

/******************************************************************************
 * @brief
 * @param sensorsMap [in]
 *  
 ******************************************************************************/
void SpiBitBangSelectSaveActiveSensors(uint16_t const sensorsMap)
{
    spiNssSaveMask = spiNssMask;
    spiNssMask     = sensorsMap; 
    saved          = 1;
} 

 /******************************************************************************
 * @brief
 *  
 ******************************************************************************/
void SpiBitBangRestoreActiveSensors()
{
    if(saved){
        spiNssMask = (uint16_t)spiNssSaveMask;
        saved      = 0;
    }
}


