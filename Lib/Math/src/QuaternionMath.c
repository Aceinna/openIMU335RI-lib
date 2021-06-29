/*
 * File:   QuaternionMath.cpp
 * Author: joemotyka
 *
 * Created on May 7, 2016, 5:03 PM
 */

#include "QuaternionMath.h"
#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "TimingVars.h"
#endif

#include "Indices.h"
#include <math.h>

/******************************************************************************
 * @brief Convert Euler angles (roll, pitch and yaw) to quaternions according to
 *        the ZYX rotation sequence.
 * 
 * @param EulerAngles   [in]    Euler angles. [rad].
 * @param Quaternion    [out]   Quaternion.
 *  
 *****************************************************************************/
void EulerAnglesToQuaternion(real EulerAngles[], real Quaternion[])
{
    real sinThetaXOver2;  // roll
    real cosThetaXOver2;
    real sinThetaYOver2;  // pitch
    real cosThetaYOver2;
    real sinThetaZOver2;  // yaw
    real cosThetaZOver2;

    // Divide the angle by two (the angles used in computing the quaternion
    //   are theta/2)
    real const ThetaXOver2 = 0.5F * EulerAngles[ROLL];
    real const ThetaYOver2 = 0.5F * EulerAngles[PITCH];
    real const ThetaZOver2 = 0.5F * EulerAngles[YAW];

    // Precompute sin/cos values used in the expressions below
    sinThetaXOver2 = sinf( ThetaXOver2 );
    cosThetaXOver2 = cosf( ThetaXOver2 );
    sinThetaYOver2 = sinf( ThetaYOver2 );
    cosThetaYOver2 = cosf( ThetaYOver2 );
    sinThetaZOver2 = sinf( ThetaZOver2 );
    cosThetaZOver2 = cosf( ThetaZOver2 );

    // q0 = SIN( ThetaX/2 ) * SIN( ThetaY/2 ) * SIN( ThetaZ/2 ) + COS( ThetaX/2 ) * COS( ThetaY/2 ) * COS( ThetaZ/2 )
    Quaternion[Q0] = ( sinThetaXOver2 * sinThetaYOver2 * sinThetaZOver2 ) +
                     ( cosThetaXOver2 * cosThetaYOver2 * cosThetaZOver2 );

    // q1 = SIN( ThetaX/2 ) * COS( ThetaY/2 ) * COS( ThetaZ/2 ) - COS( ThetaX/2 ) * SIN( ThetaY/2 ) * SIN( ThetaZ/2 )
    Quaternion[Q1] = ( sinThetaXOver2 * cosThetaYOver2 * cosThetaZOver2 ) -
                     ( cosThetaXOver2 * sinThetaYOver2 * sinThetaZOver2 );

    // q2 = SIN( ThetaX/2 ) * COS( ThetaY/2 ) * SIN( ThetaZ/2 ) + COS( ThetaX/2 ) * SIN( ThetaY/2 ) * COS( ThetaZ/2 )
    Quaternion[Q2] = ( sinThetaXOver2 * cosThetaYOver2 * sinThetaZOver2 ) +
                     ( cosThetaXOver2 * sinThetaYOver2 * cosThetaZOver2 );

    // q3 = COS( ThetaX/2 ) * COS( ThetaY/2 ) * SIN( ThetaZ/2 ) - SIN( ThetaX/2 ) * SIN( ThetaY/2 ) * COS( ThetaZ/2 )
    Quaternion[Q3] = ( cosThetaXOver2 * cosThetaYOver2 * sinThetaZOver2 ) -
                     ( sinThetaXOver2 * sinThetaYOver2 * cosThetaZOver2 );

    QuatNormalize( Quaternion );
}

/******************************************************************************
 * @brief Normalize a quaternion.
 * 
 * @param Quat  [in/out]    Quaternion.
 *   
 *****************************************************************************/
void QuatNormalize(real Quat[])
{
    real QuatSquared[4];
    real QuatMag;
    real temp;

    // Square the components of the quaternion (0 <= sSquared <= 1)
    QuatSquared[Q0] = Quat[Q0] * Quat[Q0];
    QuatSquared[Q1] = Quat[Q1] * Quat[Q1];
    QuatSquared[Q2] = Quat[Q2] * Quat[Q2];
    QuatSquared[Q3] = Quat[Q3] * Quat[Q3];

    // Find the RSS of the quaternion: sqrt( q1^2 + q2^2 + q3^2 + q4^2 )
    QuatMag = QuatSquared[Q0] +
              QuatSquared[Q1] +
              QuatSquared[Q2] +
              QuatSquared[Q3];
    QuatMag = sqrtf( QuatMag );

    // Normalize the quaternion
    temp = 1.0F / QuatMag;
    Quat[Q0] = Quat[Q0] * temp;
    Quat[Q1] = Quat[Q1] * temp;
    Quat[Q2] = Quat[Q2] * temp;
    Quat[Q3] = Quat[Q3] * temp;

#ifdef FORCE_Q0_POSITIVE
    // Force Q0 to be positive
    if (Quat[Q0] < 0.0) {
        // Flip signs on all quaternion elements
        Quat[Q0] = -Quat[Q0];
        Quat[Q1] = -Quat[Q1];
        Quat[Q2] = -Quat[Q2];
        Quat[Q3] = -Quat[Q3];
    }
#endif
}

/******************************************************************************
 * @brief Convert quaternion to Euler angles (roll, pitch and yaw) according to
 *        the ZYX rotation sequence.
 *
 * @param EulerAngles   [out]   Euler angles. [rad].
 * @param Quaternion    [in]    Quaternion.
 *  
 *****************************************************************************/
void QuaternionToEulerAngles(real EulerAngles[], real Quaternion[])
{
    real R[3][3];

    real q0Sq;
    real q0q1;
    real q0q2;
    real q0q3;
    real q1Sq;
    real q1q2;
    real q1q3;
    real q2Sq;
    real q2q3;
    real q3Sq;

    // Compute values used repeatedly in the function
    q0Sq = Quaternion[Q0] * Quaternion[Q0];
    q0q1 = Quaternion[Q0] * Quaternion[Q1];
    q0q2 = Quaternion[Q0] * Quaternion[Q2];
    q0q3 = Quaternion[Q0] * Quaternion[Q3];

    q1Sq = Quaternion[Q1] * Quaternion[Q1];
    q1q2 = Quaternion[Q1] * Quaternion[Q2];
    q1q3 = Quaternion[Q1] * Quaternion[Q3];

    q2Sq = Quaternion[Q2] * Quaternion[Q2];
    q2q3 = Quaternion[Q2] * Quaternion[Q3];

    q3Sq = Quaternion[Q3] * Quaternion[Q3];

    // Form the direction cosine matrix (DCM) from q
    R[0][0] = q0Sq + q1Sq - q2Sq - q3Sq;

    R[1][0] = 2.0F * (q1q2 + q0q3);

    R[2][0] = 2.0F * (q1q3 - q0q2);
    R[2][1] = 2.0F * (q2q3 + q0q1);
    R[2][2] = q0Sq - q1Sq - q2Sq + q3Sq;

    // Calculate the euler angles from the DCM
    // prevent NaN due to numerical error
    if (R[2][0] > 1.0F)
    {
        R[2][0] = 1.0F;
    }
    if (R[2][0] < -1.0F)
    {
        R[2][0] = -1.0F;
    }

    EulerAngles[ROLL]  = atan2f( R[2][1], R[2][2] );
    EulerAngles[PITCH] = -asinf( R[2][0] );
    EulerAngles[YAW]   = atan2f( R[1][0], R[0][0] );

    // What do do in the case that pitch = 90 degrees???  Indeterminate roll and yaw...
}

/******************************************************************************
 * @brief Convert quaternion to direction cosine matrix.
 *
 * @param Quaternion    [in]    Quaternion.
 * @param R321          [out]   Direction cosine matrix.
 *  
 *****************************************************************************/
void QuaternionToR321(real Quaternion[], real R321[3][3])
{
    real q0Sq;
    real q0q1;
    real q0q2;
    real q0q3;
    real q1Sq;
    real q1q2;
    real q1q3;
    real q2Sq;
    real q2q3;
    real q3Sq;

    // Compute values used repeatedly in the function
    q0Sq = Quaternion[Q0] * Quaternion[Q0];
    q0q1 = Quaternion[Q0] * Quaternion[Q1];
    q0q2 = Quaternion[Q0] * Quaternion[Q2];
    q0q3 = Quaternion[Q0] * Quaternion[Q3];

    q1Sq = Quaternion[Q1] * Quaternion[Q1];
    q1q2 = Quaternion[Q1] * Quaternion[Q2];
    q1q3 = Quaternion[Q1] * Quaternion[Q3];

    q2Sq = Quaternion[Q2] * Quaternion[Q2];
    q2q3 = Quaternion[Q2] * Quaternion[Q3];

    q3Sq = Quaternion[Q3] * Quaternion[Q3];

    //Form the direction cosine matrix (DCM) from q
    R321[0][0] = q0Sq + q1Sq - q2Sq - q3Sq;
    R321[0][1] = 2.0F * (q1q2 - q0q3);
    R321[0][2] = 2.0F * (q1q3 + q0q2);

    R321[1][0] = 2.0F * (q1q2 + q0q3);
    R321[1][1] = q0Sq - q1Sq + q2Sq - q3Sq;
    R321[1][2] = 2.0F * (q2q3 - q0q1);

    R321[2][0] = 2.0F * (q1q3 - q0q2);
    R321[2][1] = 2.0F * (q2q3 + q0q1);
    R321[2][2] = q0Sq - q1Sq - q2Sq + q3Sq;
}


