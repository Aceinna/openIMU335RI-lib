/**
******************************************************************************
* @file WMMInternal.h
*
*****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

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

#ifndef WMMINTERNAL_H
#define WMMINTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// internal constants
static const uint8_t WMM_MAX_MODEL_DEGREES                   = 12U;
static const uint8_t WMM_MAX_SECULAR_VARIATION_MODEL_DEGREES = 12U;
static const uint8_t NUMTERMS  = 91U;
static const uint8_t NUMPCUP   = 92U;
static const uint8_t NUMPCUPS  = 13U;

// internal structure definitions
typedef struct {
    float32_t EditionDate;
    float32_t epoch;						  //Base time of Geomagnetic model epoch (yrs)
    char  ModelName[20];
    float32_t Main_Field_Coeff_G[NUMTERMS];  // C - Gauss coefficients of main geomagnetic model (nT)
    float32_t Main_Field_Coeff_H[NUMTERMS];  // C - Gauss coefficients of main geomagnetic model (nT)
    float32_t Secular_Var_Coeff_G[NUMTERMS]; // CD - Gauss coefficients of secular geomagnetic model (nT/yr)
    float32_t Secular_Var_Coeff_H[NUMTERMS]; // CD - Gauss coefficients of secular geomagnetic model (nT/yr)
    uint16_t nMax;                             // Maximum degree of spherical harmonic model
    uint16_t nMaxSecVar;                       // Maxumum degree of spherical harmonic secular model
    uint16_t SecularVariationUsed;             // Whether or not the magnetic secular variation vector will be needed by program
} WMMtype_MagneticModel;

typedef struct {
    float32_t a;     // semi-major axis of the ellipsoid
    float32_t b;     // semi-minor axis of the ellipsoid
    float32_t fla;   // flattening
    float32_t epssq; // first eccentricity squared
    float32_t eps;   // first eccentricity
    float32_t re;    // mean radius of  ellipsoid
} WMMtype_Ellipsoid;

typedef struct {
    float32_t lambda; // longitude
    float32_t phi;    // geodetic latitude
    float32_t HeightAboveEllipsoid; // height above the ellipsoid (HaE)
} WMMtype_CoordGeodetic;

typedef struct {
    float32_t lambda; // longitude
    float32_t phig;   // geocentric latitude
    float32_t r;      // distance from the center of the ellipsoid
} WMMtype_CoordSpherical;

typedef struct {
    uint16_t	Year;
    uint16_t	Month;
    uint16_t	Day;
    float32_t DecimalYear;
} WMMtype_Date;

typedef struct {
    float32_t Pcup[NUMPCUP];  // Legendre Function
    float32_t dPcup[NUMPCUP]; // Derivative of Lagendre fn
} WMMtype_LegendreFunction;

typedef struct {
    float32_t Bx;    // North
    float32_t By;	 // East
    float32_t Bz;    // Down
} WMMtype_MagneticResults;

typedef struct {
    float32_t RelativeRadiusPower[WMM_MAX_MODEL_DEGREES + 1];  // [earth_reference_radius_km / sph. radius ]^n
    float32_t cos_mlambda[WMM_MAX_MODEL_DEGREES + 1]; // cp(m)  - cosine of (m*spherical coord. longitude
    float32_t sin_mlambda[WMM_MAX_MODEL_DEGREES + 1]; // sp(m)  - sine of (m*spherical coord. longitude)
} WMMtype_SphericalHarmonicVariables;

typedef struct {
    float32_t Decl; 	//  1. Angle between the magnetic field vector and true north, positive east
    float32_t Incl; 	//  2. Angle between the magnetic field vector and the horizontal plane, positive down
    float32_t F; 		//  3. Magnetic Field Strength
    float32_t H; 		//  4. Horizontal Magnetic Field Strength
    float32_t X; 		//  5. Northern component of the magnetic field vector
    float32_t Y; 		//  6. Eastern component of the magnetic field vector
    float32_t Z; 		//  7. Downward component of the magnetic field vector
    float32_t GV; 		//  8. The Grid Variation
    float32_t Decldot;  //  9. Yearly Rate of change in declination
    float32_t Incldot;  // 10. Yearly Rate of change in inclination
    float32_t Fdot; 	// 11. Yearly rate of change in Magnetic field strength
    float32_t Hdot; 	// 12. Yearly rate of change in horizontal field strength
    float32_t Xdot; 	// 13. Yearly rate of change in the northern component
    float32_t Ydot; 	// 14. Yearly rate of change in the eastern component
    float32_t Zdot; 	// 15. Yearly rate of change in the downward component
    float32_t GVdot;	// 16. Yearly rate of chnage in grid variation
} WMMtype_GeoMagneticElements;

// Internal Function Prototypes
void WMM_Set_Coeff_Array( float32_t coeffs[][6]);
void WMM_GeodeticToSpherical( WMMtype_Ellipsoid      Ellip,
                              WMMtype_CoordGeodetic  CoordGeodetic,
                              WMMtype_CoordSpherical *CoordSpherical);
uint16_t WMM_DateToYear( WMMtype_Date *CalendarDate, char *Error);
void WMM_TimelyModifyMagneticModel( WMMtype_Date          UserDate,
                                    WMMtype_MagneticModel *MagneticModel,
                                    WMMtype_MagneticModel *TimedMagneticModel );

uint16_t WMM_Geomag( WMMtype_Ellipsoid           Ellip,
                     WMMtype_CoordSpherical      CoordSpherical,
                     WMMtype_CoordGeodetic       CoordGeodetic,
                     WMMtype_MagneticModel       *TimedMagneticModel,
                     WMMtype_GeoMagneticElements *GeoMagneticElements );

uint16_t WMM_AssociatedLegendreFunction( WMMtype_CoordSpherical   CoordSpherical,
                                         uint16_t                 nMax,
                                         WMMtype_LegendreFunction *LegendreFunction );

uint16_t WMM_CalculateGeoMagneticElements( WMMtype_MagneticResults    *MagneticResultsGeo,
                                           WMMtype_GeoMagneticElements *GeoMagneticElements);

uint16_t WMM_CalculateSecularVariation( WMMtype_MagneticResults     MagneticVariation,
                                        WMMtype_GeoMagneticElements *MagneticElements);

uint16_t WMM_ComputeSphericalHarmonicVariables( WMMtype_Ellipsoid                  Ellip,
                                                WMMtype_CoordSpherical             CoordSpherical,
                                                uint16_t                           nMax,
                                                WMMtype_SphericalHarmonicVariables *SphVariables);

uint16_t WMM_PcupLow( float32_t    *Pcup,
                      float32_t    *dPcup,
                      float32_t    x,
                      uint16_t nMax );

uint16_t WMM_PcupHigh( float32_t    *Pcup,
                       float32_t    *dPcup,
                       float32_t    x,
                       uint16_t nMax );

uint16_t WMM_RotateMagneticVector( WMMtype_CoordSpherical,
                                   WMMtype_CoordGeodetic   CoordGeodetic,
                                   WMMtype_MagneticResults MagneticResultsSph,
                                   WMMtype_MagneticResults *MagneticResultsGeo );

uint16_t WMM_SecVarSummation( WMMtype_LegendreFunction           *LegendreFunction,
                              WMMtype_MagneticModel              *MagneticModel,
                              WMMtype_SphericalHarmonicVariables SphVariables,
                              WMMtype_CoordSpherical             CoordSpherical,
                              WMMtype_MagneticResults            *MagneticResults );

uint16_t WMM_SecVarSummationSpecial( WMMtype_MagneticModel              *MagneticModel,
                                     WMMtype_SphericalHarmonicVariables SphVariables,
                                     WMMtype_CoordSpherical             CoordSpherical,
                                     WMMtype_MagneticResults            *MagneticResults );

uint16_t WMM_Summation( WMMtype_LegendreFunction           *LegendreFunction,
                        WMMtype_MagneticModel              *MagneticModel,
                        WMMtype_SphericalHarmonicVariables SphVariables,
                        WMMtype_CoordSpherical             CoordSpherical,
                        WMMtype_MagneticResults            *MagneticResults );

uint16_t WMM_SummationSpecial( WMMtype_MagneticModel              *MagneticModel,
                               WMMtype_SphericalHarmonicVariables SphVariables,
                               WMMtype_CoordSpherical             CoordSpherical,
                               WMMtype_MagneticResults            *MagneticResults );

#ifdef __cplusplus
}
#endif

#endif /* WMMINTERNAL_H_ */
