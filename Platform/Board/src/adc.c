/** ***************************************************************************
 * @file   adc.c
*******************************************************************************/
#include "boardDefinition.h" ///< For ONE_PPS_PIN, etc
#include "stm32f4xx_adc.h"
#include "configureGPIO.h"
#include "GlobalConstants.h"
#include "halAPI.h"

static ADC_TypeDef* UNIT_ADC;

/*******************************************
 * @brief 
 * 
********************************************/
static void HW_InitADC_GPIO()
{
    InitPin_GPIO_ANALOG( ADC1_PORT_CLK, PORT_C, ADC1_PIN);   // ADC 123  in  11    V Input First
    InitPin_GPIO_ANALOG( ADC2_PORT_CLK, PORT_C, ADC2_PIN);   // ADC 123  in  10    V Input Second
    InitPin_GPIO_ANALOG( ADC3_PORT_CLK, PORT_C, ADC3_PIN);   // ADC 12   in  14    5V Input First
    InitPin_GPIO_ANALOG( ADC4_PORT_CLK, PORT_C, ADC4_PIN);   // ADC 12   in  15    5V Input Second
    InitPin_GPIO_ANALOG( ADC5_PORT_CLK, PORT_C, ADC5_PIN);   // ADC 123  IN  12    3.3 V Input First
    InitPin_GPIO_ANALOG( ADC6_PORT_CLK, PORT_C, ADC6_PIN);   // ADC 123  in  13    3.3 V Input Second
}

/*******************************************
 * @brief 
 * 
********************************************/
void HW_InitADC()
{

#ifdef MTLT335_V2    
    ADC_InitTypeDef         init;
    ADC_CommonInitTypeDef   commonInit;
    NVIC_InitTypeDef        NVIC_InitStructure;

    UNIT_ADC = ADC_GetInstancePtr(ADC_1);

    HW_InitADC_GPIO();

    RCC_APB2PeriphClockCmd(ADC1_CLK, ENABLE);


    ADC_StructInit(&init);
    ADC_CommonStructInit(&commonInit);

    init.ADC_Resolution             = ADC_Resolution_12b;
    init.ADC_ContinuousConvMode     = DISABLE;
    init.ADC_DataAlign              = ADC_DataAlign_Right;
    init.ADC_NbrOfConversion        = 7U;
    init.ADC_ScanConvMode           = ENABLE;
    init.ADC_ExternalTrigConvEdge   = ADC_ExternalTrigConvEdge_None;

    commonInit.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    commonInit.ADC_Mode             = ADC_Mode_Independent;
    commonInit.ADC_Prescaler        = ADC_Prescaler_Div8;
    commonInit.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;

    ADC_Init(UNIT_ADC, &init);
    ADC_CommonInit(&commonInit);
    ADC_TempSensorVrefintCmd(ENABLE); // enables temp sensor
    ADC_RegularChannelConfig(UNIT_ADC, ADC_Channel_10, 2U, ADC_SampleTime_144Cycles); // ADC_SQR
    ADC_RegularChannelConfig(UNIT_ADC, ADC_Channel_11, 1U, ADC_SampleTime_144Cycles);
    ADC_RegularChannelConfig(UNIT_ADC, ADC_Channel_12, 5U, ADC_SampleTime_144Cycles);
    ADC_RegularChannelConfig(UNIT_ADC, ADC_Channel_13, 6U, ADC_SampleTime_144Cycles);
    ADC_RegularChannelConfig(UNIT_ADC, ADC_Channel_14, 3U, ADC_SampleTime_144Cycles);
    ADC_RegularChannelConfig(UNIT_ADC, ADC_Channel_15, 4U, ADC_SampleTime_144Cycles);
    ADC_RegularChannelConfig(UNIT_ADC, ADC_Channel_16, 7U, ADC_SampleTime_144Cycles);
    ADC_EOCOnEachRegularChannelCmd(UNIT_ADC, ENABLE);
    ADC_ITConfig(UNIT_ADC, ADC_IT_EOC, ENABLE);
    ADC_Cmd(UNIT_ADC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x5U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0U;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
#endif

}


static uint32_t convVal[16] = {0U};
static uint32_t tmpConvVal[16] = {0U};
static int32_t convIdx     = 0;

/*******************************************
 * @brief 
 * 
********************************************/
void HW_StartADC()
{
#ifdef MTLT335_V2    
    ADC_SoftwareStartConv(UNIT_ADC);
#endif
}

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL HW_IsADCConversionComplete()
{
    if(convIdx >= 7){
        convIdx = 0;
        return TRUE;
    }
    
    return FALSE;
}

/*******************************************
 * @brief 
 * 
********************************************/
void ADC_IRQHandler(void)
{
    ITStatus  status;
  
    status = ADC_GetITStatus(UNIT_ADC, (uint16_t)ADC_IT_EOC);
    if(status == SET){
        ADC_ClearFlag(UNIT_ADC, (uint8_t)ADC_FLAG_EOC);
        ADC_ClearITPendingBit(UNIT_ADC, (uint16_t)ADC_IT_EOC);
        if(convIdx < 16){
            convVal[convIdx] += ADC_GetConversionValue(UNIT_ADC);
            convIdx++;
        }
    }
}


// For temperature conversion:
// T = (V-0.76)/0.0025 + 25

/*******************************************
 * @brief 
 * 
 * @param voltages ==
********************************************/
void    HW_GetADCReadings(float32_t voltages[])
{
    static float32_t const adcScaleFactor[7] = {13.0F, 13.0F, 2.0F, 2.0F, 2.0F, 2.0F, 1.0F};
    float32_t tmp;

    // here we have 10 samples accumulated
    for(int32_t i = 0; i < 7; i++){
        tmp = (float32_t)convVal[i];
        voltages[i] = ((tmp * adcScaleFactor[i])/40960.0F)*3.3F; 
        convVal[i]  = 0U;
    } 
}

/*******************************************
 * @brief 
 * 
 * @param i ==
 * @param voltAvg ==
********************************************/
void    ADC_SetVoltCounts(uint16_t i, float32_t voltAvg)
{
    tmpConvVal[i] = (uint32_t)(int32_t)(voltAvg * 1000.0F);
}

/*******************************************
 * @brief 
 * 
 * @param channel ==
 * @return int16_t 
********************************************/
int16_t     ADC_GetChannelReadings(int32_t channel)
{
    return ((int16_t) tmpConvVal[channel]);
}


