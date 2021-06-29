/******************************************************************************
 * @file MagAlign.c
 * @brief 2D soft iron and hard iron algorithm.
 * @author Dong Xiaoguang
 * @date 2019.05.10
 * @version V1.0.0
 *-----------------------------------------------------------------------------
 * Change History
 * <Date>     | <Version> | <Author>       | <Description>
 * ----------------------------------------------------------------------------
 * 2019.05.09 | v1.0.0    | Dong Xiaoguang | Create file
 * ----------------------------------------------------------------------------
******************************************************************************/

#include "MagAlign.h"
#include "WorldMagneticModel.h"
#include "magApi.h"


MagAlignStruct  gMagAlign;
WorldMagModelStruct  gWorldMagModel;

/******************************************************************************
 * @brief Initialize the parameters for magnetic heading calculation
 *  
 *****************************************************************************/
void MagAlign_Init()
{
    gMagAlign.softIronScaleRatio = 1.0F;
    gMagAlign.softIronAngle      = 0.0F;
    gMagAlign.hardIronBias[0]    = 0.0F;
    gMagAlign.hardIronBias[1]    = 0.0F;
    gMagAlign.SF[0]              = 0.0F;
    gMagAlign.SF[1]              = 0.0F;
    gMagAlign.SF[2]              = 0.0F;
    gMagAlign.SF[3]              = 0.0F;

}

/******************************************************************************
 * @brief Initialzie the world magnetic mode.
 *  
 *****************************************************************************/
void WMM_Init()
{
    gWorldMagModel.decl_rad       = 0.0F;
    gWorldMagModel.validSoln      = FALSE;
    gWorldMagModel.timeOfLastSoln = 0U;

}
