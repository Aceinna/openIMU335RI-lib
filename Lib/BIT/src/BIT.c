#include <string.h>
#include "GlobalConstants.h"
#include "Indices.h"
#include "BITStatus.h"
#include "bitAPI.h"
#include "halAPI.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"

static BITStatusStruct     gBitStatus;
static BOOL needSendWarning = FALSE;
static BOOL mustSendWarning = FALSE;
static BOOL needTestCal     = TRUE;
static BOOL needTestStack   = TRUE;
static BOOL daqCycleStart   = FALSE;
static uint16_t faultedChipsMask    = 0U;
static uint16_t configuredChipsMask = 0U;
static uint16_t fifoResetMask       = 0U;
static BOOL DTC1_Started    = FALSE;
static BOOL DTC2_Started    = FALSE;
static uint8_t   FMI_CODE_1 = 0xFFU;
static uint8_t   FMI_CODE_2 = 0xFFU;
static uint16_t  sendDTC1    = 0U;
static uint16_t  sendDTC2    = 0U;
static BOOL      reportConfigError  = FALSE;
static uint32_t  chipOverTempError_cntr[3] = {0U,0U,0U};
static int32_t   voltErrorCnt[7] = {0,0,0,0,0,0,0};
static uint16_t  cpuOverTempError_cntr = 0U;
static uint8_t   DTC1_Count     = 0U;
static uint8_t   DTC2_Count     = 0U;
static int32_t   extOverVoltageCounter = 0;

/**
 * @brief
 *
 * @param fHighGain
 */
void BIT_SetHighGainMode(BOOL const fHighGain)
{
    gBitStatus.algoStatus.bit.algoMode = (fHighGain != 0) ? 1U : 0U;
}

/****************************************************
 * @brief Function called one time upon unit startup
 *
*****************************************************/
static void BIT_UpdateResetCause()
{
    uint8_t const cause = (uint8_t)HW_GetResetRootCause();

    if(cause > (uint8_t)MAX_RESET){
        return;
    }

    gBitStatus.hwStatus.bit.lastResetCause = (uint16_t)cause & (uint16_t)0x07U;

    if((gBitStatus.hwStatus.bit.lastResetCause & 0x04U) != 0U)
    {
        needSendWarning = TRUE;
    }
}

/**
 * @brief
 *
 */
static void BIT_UpdateAccelDegradationStatus()
{
    uint8_t const disagreement  = (gBitStatus.sensorStatus.bit.accelDisagreement != 0U) ? (uint8_t)1U : (uint8_t)0U;
    
    uint8_t const overRange = (gBitStatus.sensorStatus.bit.accelOverRange != 0U) ? (uint8_t)1U : (uint8_t)0U;
    
    uint8_t const overRun = (uint8_t)gBitStatus.swStatus.bit.overRunError ? (uint8_t)0x07U : (uint8_t)0U;

    uint8_t moreThanOne = 0U;
    if((gBitStatus.sensorStatus.bit.accelSensorStatus == 0x03U) ||
        (gBitStatus.sensorStatus.bit.accelSensorStatus == 0x05U) ||
        (gBitStatus.sensorStatus.bit.accelSensorStatus == 0x06U) ||
        (gBitStatus.sensorStatus.bit.accelSensorStatus == 0x07U))
    {
        moreThanOne = (uint8_t)gBitStatus.sensorStatus.bit.accelSensorStatus;
    }

    gBitStatus.sensorStatus.bit.accelDegradationError = (uint32_t)moreThanOne | (uint32_t)overRange | (uint32_t)overRun | (uint32_t)disagreement;
}

/**
 * @brief
 *
 */
static void BIT_UpdateGyroDegradationStatus()
{

    uint8_t const disagreement    = (gBitStatus.sensorStatus.bit.rateDisagreement != 0U) ? (uint8_t)1U : (uint8_t)0U;
    
    uint8_t const overRange       = (gBitStatus.sensorStatus.bit.rateOverRange != 0U) ? (uint8_t)1U : (uint8_t)0U;
    
    uint8_t const overRun         = (uint8_t)gBitStatus.swStatus.bit.overRunError ? (uint8_t)0x07U : (uint8_t)0U;
    
    uint8_t moreThanOne = 0U;
    if((gBitStatus.sensorStatus.bit.rateSensorStatus == 0x03U) ||
        (gBitStatus.sensorStatus.bit.rateSensorStatus == 0x05U) ||
        (gBitStatus.sensorStatus.bit.rateSensorStatus == 0x06U) ||
        (gBitStatus.sensorStatus.bit.rateSensorStatus == 0x07U)) 
    {
        moreThanOne = (uint8_t)gBitStatus.sensorStatus.bit.rateSensorStatus;
    }

    gBitStatus.sensorStatus.bit.rateDegradationError = (uint32_t)moreThanOne | (uint32_t)overRange | (uint32_t)overRun | (uint32_t)disagreement;
}

// When disagreement is found a specific bit is set for that chip and axis
// All bits are zero when no disagreement
// -------------------------------
//  Accel/Rate      | X | Y | Z |
// -------------------------------
//  axisInfo[chip0] | 0 | 0 | 0 |
//  axisInfo[chip1] | 0 | 0 | 0 |
//  axisInfo[chip2] | 0 | 0 | 0 |
// -------------------------------
/**
 * @brief 
 * 
 * @param axisInfo 
 * @param fAccel 
 * @param fChipFailure 
 */
static void BIT_UpdateSensorDisagreement(uint8_t const axisInfo[], BOOL const fAccel, BOOL const fChipFailure)
{
    // Do nothing when there is a chip failure because of disagreement
    if(fChipFailure)
    {
        return;
    }
        
    //TODO create axis from axis Info
    uint8_t mask = 0U;

    if(axisInfo[0] != 0U){
        mask |= 1U;
    }
    if(axisInfo[1] != 0U){
        mask |= 2U;
    }
    if(axisInfo[2] != 0U){
        mask |= 4U;
    }

    if(fAccel)
    {
        gBitStatus.sensorStatus.bit.accelDisagreement  = mask;
        BIT_UpdateAccelDegradationStatus();
    }
    else
    {
        gBitStatus.sensorStatus.bit.rateDisagreement  = mask;
        BIT_UpdateGyroDegradationStatus();
    }

}

/**
 * @brief
 *
 * @param chip
 */
static void BIT_MarkChipAsFaulty(uint8_t const chip)
{
    if(chip < (uint8_t)NUM_SENSOR_CHIPS)
    {   
        // Here it becomes sticky. So do cleanup
        // 1. set error bits
        gBitStatus.sensorStatus.bit.accelSensorStatus   |= (uint32_t)1U << (uint32_t)chip;
        gBitStatus.sensorStatus.bit.rateSensorStatus    |= (uint32_t)1U << (uint32_t)chip;
        // 2. reset error bits
        gBitStatus.sensorStatus.bit.accelOverRangeError &= (uint32_t)1U << (uint32_t)chip;
        gBitStatus.sensorStatus.bit.rateOverRangeError  &= (uint32_t)1U << (uint32_t)chip;
        gBitStatus.sensorStatus.bit.accelOverRange      &= (uint32_t)1U << (uint32_t)chip;
        gBitStatus.sensorStatus.bit.rateOverRange       &= (uint32_t)1U << (uint32_t)chip;

        faultedChipsMask |= (uint16_t)((uint16_t)1U << (uint16_t)chip);
    }
}

// test performed periodically
/**
 * @brief
 *
 * @param chip
 */
static void BIT_UpdateChipCommError(uint8_t const chip)
{
    if(chip < (uint8_t)NUM_SENSOR_CHIPS){
        // Sensor chip Communication error also reflects in accel/rate sensor status
        // to be used for degradation mechanism
        // Also, if a chip is busted accel and rate both sensor statuses are set
        gBitStatus.comStatus.bit.sensorCommStatus   |= (uint16_t)((uint16_t)1U << (uint16_t)chip);                                // Sticky
        gBitStatus.hwStatus.bit.sensorComm          |= (uint16_t)((uint16_t)1U << (uint16_t)chip);                                        // Sticky

        if(gBitStatus.hwStatus.bit.sensorComm == (uint16_t)0x07U){
            mustSendWarning = TRUE;
            sendDTC1       |= COMM_ERROR_DTC_MASK;
        }

        BIT_MarkChipAsFaulty(chip);
        BIT_UpdateAccelDegradationStatus();
        BIT_UpdateGyroDegradationStatus();
    }
}

/**
 * @brief
 *
 */
static void BIT_UpdateAlgorithmError()
{
    if(gBitStatus.algoStatus.bit.algoInitError){
          gBitStatus.algoStatus.bit.algoError = 1U;
          mustSendWarning = TRUE;
      }else{
          gBitStatus.algoStatus.bit.algoError = 0U;
      }
}

/**
 * @brief
 *
 * @param status
 */
static void BIT_UpdateAlgorithmInitStatus(BOOL const status)
{
    static uint16_t algoInitError_cntr = 0U;
    gBitStatus.algoStatus.bit.algoInitStatus = (uint16_t)status;                                            // Not Sticky
    algoInitError_cntr = (status == TRUE) ? (algoInitError_cntr + 1U) : 0U;

    // Critical when init status remains after 3 second of startup
    if(algoInitError_cntr > BIT_ALGO_INIT_THRESHOLD)
    {
        // set error condition
        algoInitError_cntr = BIT_ALGO_INIT_THRESHOLD;
        gBitStatus.algoStatus.bit.algoInitError = 1U;                                              // Sticky
    }else{
        // remove error condition
        gBitStatus.algoStatus.bit.algoInitError = 0U;                                              // Sticky
    }

    BIT_UpdateAlgorithmError();

}

/**
 * @brief
 *
 */
static void BIT_SetStackOverflowError()
{
    // This is always a critical error, sets SW Error bit in master status
    gBitStatus.swStatus.bit.stackOverflowError = 1U;                                             
    mustSendWarning = TRUE;
    sendDTC1       |= STACK_OVERFLOW_DTC_MASK;     // Sticky
}

/**
 * @brief
 *
 * @param status
 */
void BIT_UpdateTurnSwitchError(BOOL const status)
{
    gBitStatus.algoStatus.bit.turnSwitch = (uint16_t)status;                                                // Not Sticky???
     BIT_UpdateAlgorithmError();
}

// This method is called when a chip temperature is 5 degrees
// above max temperature limit or 5 degreees below min temp limit
// if chip temperature stays above threshold continuously
// for TMP_THRESHOLD times, set HW Error bit in master Status
/**
 * @brief
 *
 * @param chip
 * @param fOverTemp
 */
void BIT_UpdateChipOverTempError(uint8_t const chip, BOOL const fOverTemp)
{

    if(chip < (uint16_t)NUM_SENSOR_CHIPS){
        uint16_t const mask = (uint16_t)((uint16_t)1U << (uint16_t)chip);

        if(fOverTemp){
            if(chipOverTempError_cntr[chip] >= BIT_TEMP_HI_THRESHOLD){
                gBitStatus.hwStatus.bit.sensorOverTempError = 1U;                // Sticky
                mustSendWarning = TRUE;
            }else{
                chipOverTempError_cntr[chip]++;
            }
            gBitStatus.hwStatus.bit.sensorOverTemp |= mask;             // not sticky
        }else{
            if(chipOverTempError_cntr[chip] != 0U){
                chipOverTempError_cntr[chip]--;
                // Only reset overTemp bit when hwError is not set and chipTemp is gradually back to normal
                if((chipOverTempError_cntr[chip] == 0U) && (gBitStatus.hwStatus.bit.sensorOverTempError != 1U))
                {
                    gBitStatus.hwStatus.bit.sensorOverTemp &= (uint16_t const)~mask;        // not sticky if hwError is not set
                }
            }
        }
    }
    
    int32_t  tempErrCnt = 0;
    for(int32_t i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(chipOverTempError_cntr[i] >= BIT_TEMP_HI_THRESHOLD){
            tempErrCnt++;
        }
    }

    if(tempErrCnt >= NUM_SENSOR_CHIPS){
        // set condition if all three chips are over temperature
        sendDTC1 |= SENSOR_OVER_TEMP_DTC_MASK;
    }else{
        // remove condition otherwise
        sendDTC1 &= ~SENSOR_OVER_TEMP_DTC_MASK;
    }
}

/**
 * @brief
 *
 * @param chipMask
 */
void   BIT_UpdateSensorSelfTestStatus(uint8_t const chipMask)
{
    for(uint8_t i = 0U; i < (uint8_t)NUM_SENSOR_CHIPS; i++)
    {
        if((chipMask & (1U << i)) != 0U){
            BIT_UpdateChipCommError(i);
        }
    }
}

/**
 * @brief
 *
 */
void  BIT_Iinitialize()
{
    memset(&gBitStatus, 0, sizeof(gBitStatus));
    BIT_UpdateResetCause();
    HW_FillStackPattern();
}

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t BIT_GetMasterStatusWord()
{
    master_status_word  status;
    uint16_t hwError       = 0U;
    uint16_t swError       = 0U;

    BOOL cpuOverTemp_f     = FALSE;
    BOOL extPowSup_f       = FALSE;
    BOOL intPowerSup_f     = FALSE;
    BOOL powerCons_f       = FALSE;
    BOOL senorOverTemp_f   = FALSE;
    BOOL hwSensorComm_f    = FALSE;

    if(gBitStatus.hwStatus.bit.cpuOverTempError != 0U){
        cpuOverTemp_f   = TRUE;   
    }
    if(gBitStatus.hwStatus.bit.extPowerSupError != 0U){
        extPowSup_f     = TRUE;
    }
    if(gBitStatus.hwStatus.bit.intPowerSupError != 0U){
        intPowerSup_f   = TRUE;   
    }
    if(gBitStatus.hwStatus.bit.powerConsumptionError != 0U){
        powerCons_f     = TRUE;   
    }
    if(gBitStatus.hwStatus.bit.sensorOverTempError != 0U){
        senorOverTemp_f = TRUE;   
    }
    if(gBitStatus.hwStatus.bit.sensorComm == 0x07U){
        hwSensorComm_f  = TRUE;   
    }
    hwError = (uint16_t)cpuOverTemp_f | (uint16_t)extPowSup_f | (uint16_t)intPowerSup_f | (uint16_t)powerCons_f | (uint16_t)senorOverTemp_f | (uint16_t)hwSensorComm_f;

    BOOL swSensorComm_f    = FALSE;    
    BOOL stackOverFlow_f   = FALSE;
    BOOL accelOverRng_f    = FALSE;
    BOOL rateOverRng_f     = FALSE;
    BOOL algoError_f       = FALSE;

    if(gBitStatus.hwStatus.bit.sensorComm == 0x07U){
        swSensorComm_f  = TRUE;
    }
    if(gBitStatus.swStatus.bit.stackOverflowError != 0U){
        stackOverFlow_f = TRUE;
    }
    if(gBitStatus.sensorStatus.bit.accelOverRangeError != 0U){
        accelOverRng_f  = TRUE;
    }
    if(gBitStatus.sensorStatus.bit.rateOverRangeError != 0U){
        rateOverRng_f   = TRUE;
    }
    if(gBitStatus.algoStatus.bit.algoError != 0U){
        algoError_f     = TRUE;
    }
    swError = (uint16_t)swSensorComm_f | (uint16_t)stackOverFlow_f | (uint16_t)accelOverRng_f | (uint16_t)rateOverRng_f | (uint16_t)algoError_f;

    status.bit.masterStatus         = (hwError | swError) ? 1U : 0U;
    status.bit.hardwareStatus       = hwError;
    status.bit.softwareStatus       = swError;
    status.bit.configurationStatus  = gBitStatus.swStatus.bit.configurationError;
    status.bit.calibrationStatus    = (gBitStatus.swStatus.bit.calibrationError == 0x07U) ? 1U :0U;
    status.bit.accelDegradation     = gBitStatus.sensorStatus.bit.accelDegradationError ? 1U : 0U;
    status.bit.rateDegradation      = gBitStatus.sensorStatus.bit.rateDegradationError ? 1U : 0U;
    status.bit.forcedReset          = (gBitStatus.hwStatus.bit.lastResetCause & 0x04U) ? 1U : 0U;
    status.bit.applicationCRC       = gBitStatus.swStatus.bit.applicationCRCError;
    status.bit.txOverFlowError      = gBitStatus.comStatus.bit.txQueueOverflowError;
    status.bit.rsvd                 = 0U;

    return status.all;
}

/**
 * @brief
 *
 * @return uint32_t
 */
uint32_t BIT_GetSWStatusWord()
{
    sw_status_word status;

    status.bit.stackOverflowError           = gBitStatus.swStatus.bit.stackOverflowError;
    status.bit.algoError                    = gBitStatus.algoStatus.bit.algoError;
    status.bit.algoInitStatus               = gBitStatus.algoStatus.bit.algoInitStatus;
    status.bit.accelOverRangeStatus         = gBitStatus.sensorStatus.bit.accelOverRange;
    status.bit.rateOverRangeStatus          = gBitStatus.sensorStatus.bit.rateOverRange;
    status.bit.configurationError           = gBitStatus.swStatus.bit.configurationError;
    status.bit.calibrationDataStatus        = gBitStatus.swStatus.bit.calibrationError;
    status.bit.accelSensorStatus            = gBitStatus.sensorStatus.bit.accelSensorStatus;
    status.bit.rateSensorStatus             = gBitStatus.sensorStatus.bit.rateSensorStatus;
    status.bit.accelDisagreement            = gBitStatus.sensorStatus.bit.accelDisagreement ? 1U : 0U;
    status.bit.rateDisagreement             = gBitStatus.sensorStatus.bit.rateDisagreement ? 1U : 0U;
    status.bit.lastResetStatus              = gBitStatus.hwStatus.bit.lastResetCause;
    status.bit.dataProcessingOverRunStatus  = gBitStatus.swStatus.bit.overRunStatus;
    status.bit.turnSwitchStatus             = gBitStatus.algoStatus.bit.turnSwitch;
    status.bit.algoMode                     = gBitStatus.algoStatus.bit.algoMode;
    status.bit.txQueueOverflowStatus        = gBitStatus.comStatus.bit.txQueueOverflow;
    status.bit.rsvd0                        = 0U;
    status.bit.rsvd1                        = 0U;

    return status.all;
}

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t BIT_GetSensorStatusWord_MSB()
{
    return (uint16_t)(gBitStatus.sensorStatus.all >> 16U);
}

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t BIT_GetSensorStatusWord_LSB()
{
    return (uint16_t)(gBitStatus.sensorStatus.all & 0xFFFFU);
}

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t BIT_GetHWStatusWord()
{
    hw_status_word status;

    status.bit.powerConsumptionError    = gBitStatus.hwStatus.bit.powerConsumptionError;
    status.bit.extPowerSupError         = gBitStatus.hwStatus.bit.extPowerSupError;
    status.bit.intPowerSupError         = gBitStatus.hwStatus.bit.intPowerSupError;
    status.bit.cpuOverTemp              = gBitStatus.hwStatus.bit.cpuOverTemp;
    status.bit.chipOverTemp             = gBitStatus.hwStatus.bit.sensorOverTemp;
    status.bit.sensorCommStatus         = gBitStatus.hwStatus.bit.sensorComm;
    status.bit.rsvd0                    = 0U;

    return status.all;
}

/**
 * @brief
 *
 * @return BOOL
 */
BOOL BIT_IsEepromUnlocked()
{
    return (BOOL)gBitStatus.hwStatus.bit.unlockedEEPROM;
}

/**
 * @brief
 *
 * @param locked
 */
void BIT_SetEepromLockStatus(BOOL const locked)
{
    gBitStatus.hwStatus.bit.unlockedEEPROM = (uint16_t)locked;
}

/**
 * @brief
 *
 * @param status
 */
void  BIT_UpdateAlgorithmStatus(uint16_t const status)
{
    BOOL gainStatus = FALSE;
    BOOL initStatus = FALSE;
    BOOL turnSwitchStatus = FALSE;

    if(status & (1U << 1U)){
        gainStatus = TRUE;
    }
    if(status & (1U << 0U)){
        initStatus = TRUE;
    }
    if(status & (1U << 3U)){
        turnSwitchStatus = TRUE;
    }
    BIT_UpdateAlgorithmInitStatus(initStatus);
    BIT_SetHighGainMode(gainStatus);
    BIT_UpdateTurnSwitchError(turnSwitchStatus);
}

/**
 * @brief
 *
 */
void BIT_UpdateDaqCycleStartFlag()
{
    daqCycleStart = TRUE;
}

/**
 * @brief
 *
 * @param overflowed
 * @return BOOL
 */
BOOL    BIT_CANTxOverflow(BOOL const overflowed)
{
    static uint32_t CANTxOverflowCnt = 0U;
    static uint32_t txOverflowError_counter = 0U;                      // keeps track of time in mili second

    if(overflowed){
        CANTxOverflowCnt++;                                             // Increments asmany times as Unit is trying to send a message and overflow detected

        // set warning status here
        gBitStatus.comStatus.bit.txQueueOverflow = 1U;

        if(CANTxOverflowCnt > BIT_CAN_TX_OVERFLOW_THRESHOLD)
        {
            gBitStatus.comStatus.bit.txQueueOverflowError = 1U;
            CANTxOverflowCnt = BIT_CAN_TX_OVERFLOW_THRESHOLD;
            return TRUE;
        }

        // Once a Daq cycle check if TX Overflow still there
        if(daqCycleStart)                                               // Only sets in void TaskDataAcquisition()
        {
            daqCycleStart = FALSE;
            txOverflowError_counter++;                                  // Increment once a Dacq cycle

            // Reset Unit here
            if(txOverflowError_counter > BIT_CAN_OVERFLOW_THRESHOLD)
            {
                HW_SetResetRootCause(TXOVERFLOW_RESET);                 // Reset the Unit if TX Overflow error persists more than 20 seconds
                HW_SystemReset();
            }
        }

    }else{
        // Send out notification when recovered from TX Overflow
        if(CANTxOverflowCnt > 0U)
        {
            needSendWarning = TRUE;
        }
        CANTxOverflowCnt = 0U;
        txOverflowError_counter = 0U;

            // remove warning status here
        gBitStatus.comStatus.bit.txQueueOverflow = 0U;
        gBitStatus.comStatus.bit.txQueueOverflowError = 0U;
    }
    return FALSE;
}

/**
 * @brief
 *
 * @param overflowed
 * @return BOOL
 */
BOOL    BIT_CANRxOverflow(BOOL const overflowed)
{
    static uint32_t CANRxOverflowCnt = 0U;

    if(overflowed){
        CANRxOverflowCnt++;
            // set warning status here
            gBitStatus.comStatus.bit.rxQueueOverflow = 1U;
        if(CANRxOverflowCnt > BIT_CAN_RX_OVERFLOW_THRESHOLD)
        {
            gBitStatus.comStatus.bit.rxQueueOverflowError = 1U;
            CANRxOverflowCnt = BIT_CAN_RX_OVERFLOW_THRESHOLD;
            return TRUE;
        }
    }else{
        CANRxOverflowCnt = 0U;
            // remove warning status here
        gBitStatus.comStatus.bit.rxQueueOverflow = 0U;
        gBitStatus.comStatus.bit.rxQueueOverflowError = 0U;
    }
    return FALSE;
}

/**
 * @brief
 *
 * @return BOOL
 */
BOOL BIT_NeedToSendStatus()
{
    static uint32_t delay  = 0;
    static int32_t period = 0;
    static BOOL started   = FALSE;

    if(needSendWarning){
        // one time send
        needSendWarning = FALSE;
        period  = 0;
        return TRUE;
    }

    if(reportConfigError){
        mustSendWarning = TRUE;
    }

    delay++;
    
    if(delay < 250){
        return FALSE;
    }
    // here we are at 1.25 seconds intervals

    delay = 0;

    if(!mustSendWarning){
        // reset condition
        started = FALSE;
        return FALSE;
    }

    if(!started){
        // first time send
        started = TRUE;
        period  = 0;
        return TRUE;
    }
    
    period += 1;

    if(period >= (int32_t)BIT_OUT_PERIOD){
        // first time send
        period          = 0;
        mustSendWarning = FALSE;
        return TRUE;
    }

    return FALSE;
}



/*******************************************
 * @brief 
 * 
 * @param FMI1 ==
 * @param FMI2 ==
********************************************/
void BIT_SetFMICodes(uint8_t FMI1, uint8_t FMI2)
{
    FMI_CODE_1 = FMI1;
    FMI_CODE_2 = FMI2;
}

/*******************************************
 * @brief 
 * 
********************************************/
void BIT_ResetDTC()
{
    // Reset only nonsticky conditions

    if((sendDTC1 & EXT_POWER_DTC_MASK) != 0){
        voltErrorCnt[EXT_VOLTAGE_MAX] = 0;
        gBitStatus.hwStatus.bit.powerConsumptionError = 0U; 
        sendDTC1 &= ~EXT_POWER_DTC_MASK;
    }

    if((sendDTC1 & EXT_VOLTAGE_DTC_MASK) != 0){
        gBitStatus.hwStatus.bit.extPowerSupError = 0U;   
        extOverVoltageCounter = 0;
        sendDTC1 &= ~EXT_VOLTAGE_DTC_MASK;
    }

    if((sendDTC1 & INT_POWER_DTC_MASK) != 0){
        voltErrorCnt[INT_VOLTAGE_1] = 0;
        voltErrorCnt[INT_VOLTAGE_2] = 0;
        voltErrorCnt[INT_VOLTAGE_3] = 0;
        voltErrorCnt[INT_VOLTAGE_4] = 0;
        gBitStatus.hwStatus.bit.intPowerSupError = 0U; 
        sendDTC1 &= ~INT_POWER_DTC_MASK;
    }

    if((sendDTC1 & CPU_OVER_TEMP_DTC_MASK) != 0){
        voltErrorCnt[CPU_TEMP] = 0;
        cpuOverTempError_cntr  = 0U;
        gBitStatus.hwStatus.bit.cpuOverTempError = 0U;
        sendDTC1 &= ~CPU_OVER_TEMP_DTC_MASK;
    }

    if((sendDTC1 & SENSOR_OVER_TEMP_DTC_MASK) != 0){
        // reset condition
        chipOverTempError_cntr[0] = 0U;
        chipOverTempError_cntr[1] = 0U;
        chipOverTempError_cntr[2] = 0U;
        gBitStatus.hwStatus.bit.sensorOverTempError = 0;   
        sendDTC1 &= ~SENSOR_OVER_TEMP_DTC_MASK;
    }
    
    DTC1_Count = 0U;
}



/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL BIT_NeedToSendDM1Message(uint8_t *fmiCode, uint8_t *count)
{

    // called with rate 200Hz need send messages out with 10Hz
    static int32_t DTC1_Period    = 0;
    static int32_t DTC2_Period    = 0;
    static BOOL    DTC1_Triggered = FALSE;
    static BOOL    DTC2_Triggered = FALSE;

    *fmiCode = 0xFFU;
    *count   = 0x7FU;

    // More priority
    if((sendDTC2 != 0) && (FMI_CODE_2 != 0xFFU)){

        if(DTC2_Triggered == FALSE){
            DTC2_Triggered = TRUE;
            DTC2_Count++;
            if(DTC2_Count > 126){
                DTC2_Count = 126;
            }
        }
        
        if(!DTC2_Started){
            DTC2_Started = TRUE;
            DTC2_Period  = 0;
            *fmiCode     = FMI_CODE_2;
            *count       = DTC2_Count;
            return TRUE;
        }
        
        DTC2_Period += 1;
        
        if(DTC2_Period >= (int32_t)BIT_DM1_PERIOD){
            DTC2_Period = 0;
            *fmiCode    = FMI_CODE_2;
            *count      = DTC2_Count;
            return TRUE;
        }
        
        return FALSE;
    }
    else {
        DTC2_Triggered = FALSE; 
        DTC2_Started   = FALSE;
    }
    

    if((sendDTC1 != 0) && (FMI_CODE_1 != 0xFFU)){
        
        if(DTC1_Triggered == FALSE){
            DTC1_Triggered = TRUE;
            DTC1_Count++;
            if(DTC1_Count > 126){
                DTC1_Count = 126;
            }
        }

        if(!DTC1_Started){
            DTC1_Started = TRUE;
            DTC1_Period  = 0;
            *fmiCode     = FMI_CODE_1;
            *count       = DTC1_Count;
            return TRUE;
        }
        
        DTC1_Period += 1;
        
        if(DTC1_Period >= (int32_t)BIT_DM1_PERIOD){
            DTC1_Period = 0;
            *fmiCode    = FMI_CODE_1;
            *count      = DTC1_Count;
            return TRUE;
        }
    }
    else {
        DTC1_Triggered = FALSE; 
        DTC1_Started   = FALSE;
    }

    return FALSE;
}



/*******************************************
 * @brief 
 * 
 * @param calId ==
********************************************/
void    BIT_TestCalibrationPartition(uint8_t const calId)
{
    if(!needTestCal){
        return;
    }

    if(checkCalibrationStructCrc(calId) == FALSE){
        BIT_SetInvalidCalStatus(calId);
    }
}


/**
 * @brief
 *
 */
void    BIT_TestStack()
{
    if(!needTestStack){
        return;
    }

    if(HW_IsStackOverflow() == TRUE){
        // sticky bit
        BIT_SetStackOverflowError();
    }
}

/**
 * @brief
 *
 */
void    BIT_PerformPeriodicTest()
{
    static int32_t cal_test_period    = 0;
    static int32_t stack_test_period  = 0;

    ++cal_test_period;
    if(cal_test_period >= (int32_t)BIT_CAL_TEST_PERIOD){
        static uint8_t calIdx = 0U;
        BIT_TestCalibrationPartition(calIdx);
        calIdx++;
        calIdx %= 3U;
        cal_test_period = 0;
    }

    ++stack_test_period;
    if(stack_test_period >= (int32_t)BIT_STACK_TEST_PERIOD){
        BIT_TestStack();
        stack_test_period = 0;
    }
    
    BIT_UpdateConfiguredSensorChips();
    // Put extra tests here if needed
    BIT_CheckPower();
}



/**
 * @brief
 *
 * @param chip
 * @param sensorAxis
 * @param fOverRange
 */
void BIT_UpdateSensorOverRangeStatus(uint8_t const chip, uint8_t sensorAxis, BOOL const fOverRange)
{
    uint8_t mask = 0U;
    uint8_t chipMask = 0U;

    if(chip < (uint8_t)NUM_SENSOR_CHIPS){
    
        chipMask = (uint8_t)((uint8_t)1U << chip);
        
        // Find sensor type and Axis
        if(sensorAxis <= (uint8_t)ZACCEL){
            
            static uint8_t  accelOverRange_state[NUM_AXIS] = {0U};
            static uint16_t accelOverRange_cntr[NUM_SENSOR_CHIPS][NUM_AXIS] = {0U};

            mask   = (uint8_t)~gBitStatus.sensorStatus.bit.accelSensorStatus;

            if((mask & chipMask) == 0U){
                accelOverRange_state[sensorAxis] &= (uint8_t)~((uint8_t)1U << chip);
                // nothing to do
                return;
            }

            if(fOverRange){
                
                gBitStatus.sensorStatus.bit.accelOverRange |= (uint32_t)1U << (uint32_t)sensorAxis;
                accelOverRange_state[sensorAxis] |= (uint8_t)((uint8_t)1U << chip);
                
                if(accelOverRange_cntr[chip][sensorAxis] >= BIT_MAX_OVERRANGE_LIMIT){
                    gBitStatus.sensorStatus.bit.accelOverRangeError |= (uint32_t)1U << (uint32_t)sensorAxis;
                    mustSendWarning = TRUE;
                }else{
                    accelOverRange_cntr[chip][sensorAxis]++;
                }

            }else{
                if(accelOverRange_cntr[chip][sensorAxis]){
                    
                    if(accelOverRange_cntr[chip][sensorAxis] < BIT_MIN_OVERRANGE_LIMIT){
                        
                        accelOverRange_cntr[chip][sensorAxis] = 0U;
                        accelOverRange_state[sensorAxis] &= (uint8_t)~((uint8_t)1U << chip);
                        gBitStatus.sensorStatus.bit.accelOverRange  &= (uint32_t)~((uint32_t)1U << (uint32_t)sensorAxis);
                        
                        if(accelOverRange_state[sensorAxis] == 0U){
                            
                            gBitStatus.sensorStatus.bit.accelOverRangeError &= (uint32_t)~((uint32_t)1U << (uint32_t)sensorAxis);
                        }
                    }
                    else{
                        accelOverRange_cntr[chip][sensorAxis]--;
                    }
                }
            }
            BIT_UpdateAccelDegradationStatus();

        }else if(sensorAxis <= (uint8_t)ZRATE){
            
            static uint8_t  rateOverRange_state[NUM_AXIS]  = {0U};
            static uint16_t rateOverRange_cntr[NUM_SENSOR_CHIPS][NUM_AXIS] = {0U};

            sensorAxis -= (uint8_t)XRATE;
            mask   = (uint8_t)~gBitStatus.sensorStatus.bit.rateSensorStatus;

            if((mask & chipMask) == 0U){
                rateOverRange_state[sensorAxis]  &= (uint8_t)~((uint8_t)1U << chip);
                // nothing to do
                return;
            }
            if(fOverRange){
                
                gBitStatus.sensorStatus.bit.rateOverRange |= (uint32_t)1U << (uint32_t)sensorAxis;
                rateOverRange_state[sensorAxis]         |= (uint8_t)((uint8_t)1U << chip);
                
                if(rateOverRange_cntr[chip][sensorAxis] >= BIT_MAX_OVERRANGE_LIMIT){
                
                    gBitStatus.sensorStatus.bit.rateOverRangeError |= (uint32_t)1U << (uint32_t)sensorAxis;
                    mustSendWarning = TRUE;
                }else{
                    rateOverRange_cntr[chip][sensorAxis]++;
                }
            }else{
                if(rateOverRange_cntr[chip][sensorAxis]){
                
                    if(rateOverRange_cntr[chip][sensorAxis] < BIT_MIN_OVERRANGE_LIMIT){

                        rateOverRange_cntr[chip][sensorAxis] = 0U;
                        rateOverRange_state[sensorAxis]     &= (uint8_t)~((uint8_t)1U << chip);
                        gBitStatus.sensorStatus.bit.rateOverRange &= (uint32_t)~((uint32_t)1U << (uint32_t)sensorAxis);
                        
                        if(rateOverRange_state[sensorAxis] == 0U){
                            gBitStatus.sensorStatus.bit.rateOverRangeError &= (uint32_t)~((uint32_t)1U << (uint32_t)sensorAxis);
                        }
                    }
                    else{
                        rateOverRange_cntr[chip][sensorAxis]--;
                    }
                }
            }
            BIT_UpdateGyroDegradationStatus();
        }
        else
        {
            // Should never get here
        }   
    }
}

/**
 * @brief
 *
 */
void BIT_SetInvalidConfigStatus()
{
    // This is always a critical error, sets SW Error bit in master status
    gBitStatus.swStatus.bit.configurationError = 1U;                                             // Sticky
    sendDTC2 = TRUE;
    reportConfigError = TRUE;
}

/**
 * @brief
 *
 * @param chip
 */
void BIT_SetInvalidCalStatus(uint8_t const chip)
{
    if(chip < (uint8_t)NUM_SENSOR_CHIPS){
        gBitStatus.swStatus.bit.calibrationError |= (uint16_t)((uint16_t)1U << (uint16_t)chip);                                   // Sticky
        BIT_MarkChipAsFaulty(chip);
    }
}

// This methods is called when fault is detected among active sensors
/**
 * @brief
 *
 * @param chip
 * @param fAccel
 */
void BIT_UpdateFaultDetectionStatus(uint8_t const chip, BOOL const fAccel)
{
    if(chip < (uint8_t)NUM_SENSOR_CHIPS){
        uint8_t mask = 1U << chip;

        // Update sensor status
        if(fAccel){
            gBitStatus.sensorStatus.bit.accelSensorStatus |= (uint32_t)mask;  // sticky
        } else  {
            gBitStatus.sensorStatus.bit.rateSensorStatus  |= (uint32_t)mask;  // sticky
        }

        mask &= (uint8_t)gBitStatus.sensorStatus.bit.accelSensorStatus;
        mask &= (uint8_t)gBitStatus.sensorStatus.bit.rateSensorStatus;

        if(mask){
            // chip failed completely
            BIT_MarkChipAsFaulty(chip);
        }
    }
}

/**
 * @brief
 *
 * @param status
 */
void BIT_DacqOverrun(BOOL const status)
{
    static uint16_t overRun_cntr = 0U;

    if(status){
        if(overRun_cntr >= DATA_OVERRUN_MAX){
            gBitStatus.swStatus.bit.overRunError = 1U;            // Sticky
        }else{
            overRun_cntr++;
        }
        gBitStatus.swStatus.bit.overRunStatus = 1U;               // Sticky
    }else{
        overRun_cntr = 0U;
        if(gBitStatus.swStatus.bit.overRunError != 1U)
        {
            gBitStatus.swStatus.bit.overRunStatus = 0U;           // Sticky
        }
    }
    BIT_UpdateAccelDegradationStatus();
    BIT_UpdateGyroDegradationStatus();
}

// *********************************** //
// *** Hardware BIT Status Methods *** //
// *********************************** //

/**
 * @brief
 *
 * @param fOverConsumption
 */
void BIT_UpdatePowerConsumptionError(BOOL const fOverConsumption)
{
    if(!fOverConsumption)
    {
        gBitStatus.hwStatus.bit.powerConsumptionError = 0U;      // Sticky
        sendDTC1 &= ~EXT_POWER_DTC_MASK;
        return;     // No action
    }
    gBitStatus.hwStatus.bit.powerConsumptionError = 1U;      // Sticky
    mustSendWarning = TRUE;
    sendDTC1       |= EXT_POWER_DTC_MASK;
}

/**
 * @brief
 *
 * @param fError
 */
void BIT_UpdateExtPowerSupError(BOOL const fError)
{
    if(!fError)
    {
        if(extOverVoltageCounter > 0){
            extOverVoltageCounter--;
        }
        gBitStatus.hwStatus.bit.extPowerSupError = 0U;   
        sendDTC1 &= ~EXT_VOLTAGE_DTC_MASK;
        return;     // No action
    }
    // external overvoltage
    extOverVoltageCounter++;
    if(extOverVoltageCounter > POWER_OUTAGE_TRIGGER_TIME)
    {
        extOverVoltageCounter = POWER_OUTAGE_TRIGGER_TIME; 
        gBitStatus.hwStatus.bit.extPowerSupError = 1U; 
        sendDTC1 |= EXT_VOLTAGE_DTC_MASK;
    mustSendWarning = TRUE;
    }
}

/**
 * @brief
 *
 * @param fError
 */
void BIT_UpdateIntPowerSupError(BOOL const fError)
{
    if(!fError)
    {
        gBitStatus.hwStatus.bit.intPowerSupError = 0U; 
        sendDTC1 &= ~INT_POWER_DTC_MASK;
        return;  
    }

    gBitStatus.hwStatus.bit.intPowerSupError = 1U;     
    sendDTC1 |= INT_POWER_DTC_MASK;
    mustSendWarning = TRUE;
}

/**
 * @brief
 *
 * @param axisInfo
 * @param fAccel
 * @param fChipFailure
 */
void    BIT_UpdateSensorsDisagreementStatus(uint8_t const * const axisInfo, BOOL const fAccel, BOOL const fChipFailure)
{
    BIT_UpdateSensorDisagreement(axisInfo, fAccel, fChipFailure);
}

// This method is called when CPU temperature is 5 degrees
// above max temperature limit or 5 degreees below min temp limit
/**
 * @brief
 *
 * @param fError
 */
void BIT_UpdateCpuOverTempError(BOOL const fError)
{

    if(fError){
        if((uint32_t)cpuOverTempError_cntr >= BIT_TEMP_CPU_THRESHOLD){
            gBitStatus.hwStatus.bit.cpuOverTempError = 1U;
            mustSendWarning = TRUE;
            sendDTC1 |= CPU_OVER_TEMP_DTC_MASK;
        }else{
            cpuOverTempError_cntr++;
        }
        gBitStatus.hwStatus.bit.cpuOverTemp = 1U;
    }else{
        sendDTC1 &= ~CPU_OVER_TEMP_DTC_MASK;
        // reset if condition cleared but can be triggered sooner based on counter
        gBitStatus.hwStatus.bit.cpuOverTempError = 0U;
        gBitStatus.hwStatus.bit.cpuOverTemp      = 0U;  
        if(cpuOverTempError_cntr != 0U){
            cpuOverTempError_cntr--;
        }
    }
}

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t BIT_GetCommStatusWord()
{
    return gBitStatus.comStatus.all;
}

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t BIT_GetAlgoStatusWord()
{
    return gBitStatus.algoStatus.all;
}

/**
 * @brief
 *
 * @return BOOL
 */
BOOL    BIT_IsAlgorithmError()
{
    return (gBitStatus.algoStatus.bit.algoInitStatus || gBitStatus.algoStatus.bit.algoError || gBitStatus.algoStatus.bit.algoInitError);

}

/**
 * @brief
 *
 * @return BOOL
 */
BOOL    BIT_IsAlgorithmDegraded()
{
    return (
            gBitStatus.algoStatus.bit.turnSwitch              ||
            gBitStatus.sensorStatus.bit.accelDisagreement     ||
            gBitStatus.sensorStatus.bit.rateDisagreement      ||
            gBitStatus.sensorStatus.bit.accelOverRangeError   ||
            gBitStatus.sensorStatus.bit.rateOverRangeError    ||
            gBitStatus.sensorStatus.bit.rateDegradationError  ||
            gBitStatus.sensorStatus.bit.accelDegradationError
    );


}

/**
 * @brief
 *
 * @param axis
 * @return BOOL
 */
BOOL     BIT_IsAccelerationDegraded(int32_t const axis)
{
    (void) axis;        // chip level resolution only
    return ( gBitStatus.sensorStatus.bit.accelDisagreement || gBitStatus.sensorStatus.bit.accelDegradationError);
}

/**
 * @brief
 *
 * @param axis
 * @return BOOL
 */
BOOL     BIT_IsAccelerationError(int32_t const axis)
{
    if(axis < NUM_AXIS){
        uint16_t mask;
        mask  = (uint16_t)((uint16_t)1U << (uint32_t)axis);

        if((gBitStatus.sensorStatus.bit.accelOverRangeError & (uint32_t)mask) || (gBitStatus.sensorStatus.bit.accelSensorStatus & (uint32_t)mask)){
            return TRUE;
        }
    }
    return FALSE;
}

/**
 * @brief
 *
 * @param axis
 * @return BOOL
 */
BOOL     BIT_IsRatesDegraded(int32_t const axis)
{
    (void)axis;     // chip level resolution only

    return (gBitStatus.sensorStatus.bit.rateDisagreement || gBitStatus.sensorStatus.bit.rateDegradationError);
}

/**
 * @brief
 *
 * @param axis
 * @return BOOL
 */
BOOL     BIT_IsRatesError(int32_t axis)
{
    if((XRATE <= axis) && (axis <= ZRATE)){
        uint16_t mask;
        axis -= XRATE;

        mask  = 1U << (uint16_t)axis;

        if(((gBitStatus.sensorStatus.bit.rateOverRangeError & (uint32_t)mask) != 0U) || ((gBitStatus.sensorStatus.bit.rateSensorStatus & (uint32_t)mask) != 0U)){
            return  TRUE;
        }
    }
    return FALSE;
}

/**
 * @brief
 *
 */
void     BIT_UpdateConfiguredSensorChips()
{   
    configuredChipsMask = config_GetUsedChips();
}

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t BIT_GetUsedChips()
{
    return (configuredChipsMask & (uint16_t)~faultedChipsMask);
}

// returns bitmask of used on chip sensors
// bits 0,1,2  - accelerometer X Y Z,
// bits 3,4,5  - gyro          X Y Z,
/**
 * @brief
 *
 * @param chip
 * @return uint16_t
 */
uint16_t BIT_GetUsedSensors(int32_t const chip)
{
    if(chip < NUM_SENSOR_CHIPS)
    {
        uint8_t const chipMask =  1U << (uint32_t)chip;
        uint16_t usedSensors  =  config_GetUsedSensors(chip);
        
        if(gBitStatus.sensorStatus.bit.accelSensorStatus & chipMask){
            usedSensors &= 0xFFF8U;  // remove accelerometers - all axes
        }
        if(gBitStatus.sensorStatus.bit.rateSensorStatus & chipMask){
            usedSensors &= 0xFFC7U;  // remove gyros - all axes
        }
        return  usedSensors;
    }
    return 0U;
}

// here getting mask of sensors being polled and calibrated
/*******************************************
 * @brief 
 * 
 * @param accels ==
 * @param rates ==
********************************************/
void BIT_GetUsedSensorsMask(uint8_t accels[], uint8_t rates[])
{
    uint16_t usedChips = BIT_GetUsedChips();
    uint16_t usedSensors;
    for (int32_t i = 0; i < NUM_SENSOR_CHIPS; i++){
        accels[i] = 0U;
        rates[i]  = 0U;
        if((usedChips & 0x01U) != 0U){
            usedSensors = BIT_GetUsedSensors(i);
            accels[i]   = (uint8_t)usedSensors & (uint8_t)0x07U;
            rates[i]    = (uint8_t)usedSensors & (uint8_t)0x38U;
        }
        usedChips >>= 1U;
    }
}

/**
 * @brief
 *
 */
void    BIT_NotifyEepromWriteEvent()
{
    fifoResetMask |= FIFO_RESET_CAUSE_EEPROM;
}

/**
 * @brief
 *
 */
void    BIT_ClearFifoResetEvents()
{
    fifoResetMask = 0U;
}

/**
 * @brief
 *
 * @return BOOL
 */
BOOL    BIT_NeedResetFifo()
{
    return fifoResetMask != 0U;
}

/**
 * @brief 
 * 
 */
void    BIT_BegintPowerCheck()
{
    HW_StartADC();
}


/*******************************************
 * @brief 
 * 
********************************************/
void    BIT_CheckPower()
{   
    static int32_t   numConversions = 0;
    static int32_t   numChecks     = 0;
    numChecks++;

    if(HW_IsADCConversionComplete()){
        static int32_t adcCnt = 0;
        numConversions++;
        adcCnt++;
        if(adcCnt == 10){
            static int32_t   numAdcSamples = 0; 
            
            float32_t voltages[8] = {};
            static float32_t voltSum[7] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
            HW_GetADCReadings(voltages);
            numAdcSamples ++;
            for(int32_t i = 0; i < 7; i++){
                voltSum[i] += voltages[i];
            }       
            if(numChecks >= 200){
                // about 1 second interval here 
                BOOL intPowerError = FALSE;
                float32_t const extVoltageCoeffMax = 3.2F;
                float32_t const extVoltageCoeffMin = 1.8F;
                static float32_t voltAvg[7] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
                for(uint16_t i = 0U; i < 7U; i++){
                    float32_t const voltMin[7]   = {8.0F,  4.8F,  4.5F, 4.5F, 3.0F, 3.0F, -40.0F};
                    float32_t const voltMax[7]   = {32.0F, 31.9F, 5.5F, 5.5F, 3.5F, 3.5F, 90.0F};
                    voltAvg[i]  = voltSum[i]/(float32_t)numAdcSamples;       
                    voltSum[i]  = 0.0F;
                    ADC_SetVoltCounts(i, voltAvg[i]);
                    if(i == 0U) {
                        if(voltAvg[i] >= voltMax[i] || voltAvg[i] <= voltMin[i]){
                            // input overvoltage 
                            BIT_UpdateExtPowerSupError(TRUE);
                        }else{
                            BIT_UpdateExtPowerSupError(FALSE);
                        }
                    }
                    if((i > 1U) && (i < 6U)){
                        
                        // check power supply voltages here
                        if((voltAvg[i] > voltMax[i]) || (voltAvg[i] < voltMin[i])){
                            if(voltErrorCnt[i] >= (int32_t)POWER_OUTAGE_TRIGGER_TIME){
                                intPowerError = TRUE;
                            }else{
                                voltErrorCnt[i]++;
                            }
                        } else {
                            // remove error and decrease counter
                            if(voltErrorCnt[i] > 0){
                                voltErrorCnt[i]--;
                            }    
                        }
                    }else if(i==6U) {
                        // check temperature here
                        float32_t const temp = ((voltAvg[i] - 0.76F)/0.0025F) + 25.F;   // temp in celsius
                        float32_t const tempMax = 90.0F;
                        BOOL const fError = temp > tempMax; 
                        BIT_UpdateCpuOverTempError(fError);
                    }else{
                        // nothing to do here 
                    }
                }
                // Update internal power supply error status
                BIT_UpdateIntPowerSupError(intPowerError);

                // Check external power supply
                float32_t coeff = voltAvg[0] - voltAvg[1];
                coeff *= voltAvg[0];
                if((coeff > extVoltageCoeffMax) || (coeff < extVoltageCoeffMin)){
                    voltErrorCnt[0]++;
                    if(voltErrorCnt[0] >= (int32_t)POWER_OUTAGE_TRIGGER_TIME){
//                        BIT_UpdatePowerConsumptionError(TRUE);
                    } 
                } else {
                    // remove error and decrease counter
//                    BIT_UpdatePowerConsumptionError(FALSE);
                    if(voltErrorCnt[0] > 0){
                        voltErrorCnt[0]--;
                    }    
                }

                numChecks = 0;
                numAdcSamples = 0;
            }

            adcCnt = 0;
        }
        HW_StartADC();
    }
}

/*******************************************
 * @brief 
 * 
 * @return uint8_t 
********************************************/
uint8_t  BIT_GetFmiCode()
{
    // NOTE: implement FMI code logic here 
    return BIT_FMI_CODE_OK;

}



#ifdef UNIT_TEST
#include "bit_wrappers.h"
#endif
