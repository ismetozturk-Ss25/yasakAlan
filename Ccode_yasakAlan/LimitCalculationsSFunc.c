#define S_FUNCTION_NAME  LimitCalculationsSFunc
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdbool.h>

#if defined(MATLAB_MEX_FILE)
    #include "tmwtypes.h"
    #include "simstruc_types.h"
#else
    #include "rtwtypes.h"
#endif

#define RZONE_ROWS 32
#define RZONE_COLS 5
#define WZONE_LEN 4

#define RZONE_EN     0
#define RZONE_TRMIN  1
#define RZONE_ELMIN  2
#define RZONE_TRMAX  3
#define RZONE_ELMAX  4

#define WZONE_TRMIN  0
#define WZONE_ELMIN  1
#define WZONE_TRMAX  2
#define WZONE_ELMAX  3
/*====================*
 * S-function methods *
 *====================*/
// #define MDL_SET_INPUT_PORT_DIMENSION_INFO
// #define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

real32_T NormalizeAngle(real32_T angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);  // Now in [0, 360)
    if (angle < 0.0f) angle += 360.0f;      // Avoid negative wraparound
    return angle - 180.0f;                 // Now in [-180, +180)
}
real32_T Middle(real32_T a, real32_T b) {
    real32_T mid = (a + b) / 2.0f;
    if (b < a) {
        mid += 180.0f;
    }
    return NormalizeAngle(mid);
}
uint16_T IsInCorner(real32_T TraPos, real32_T ElvPos, const real32_T Interval[4], real32_T buffer) {
    if ((TraPos < Interval[0]) && (TraPos >= Interval[0] - buffer)) {
        if ((ElvPos > Interval[3]) && (ElvPos <= Interval[3] + buffer)) {
            return 1;  // Top-left corner
        } else if ((ElvPos < Interval[2]) && (ElvPos >= Interval[2] - buffer)) {
            return 4;  // Bottom-left corner
        } else {
            return 0;
        }
    } else if ((TraPos > Interval[1]) && (TraPos <= Interval[1] + buffer)) {
        if ((ElvPos > Interval[3]) && (ElvPos <= Interval[3] + buffer)) {
            return 2;  // Top-right corner
        } else if ((ElvPos < Interval[2]) && (ElvPos >= Interval[2] - buffer)) {
            return 3;  // Bottom-right corner
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}

real32_T IsInsideLC(real32_T angle, real32_T min, real32_T max)
{
    if (max >= min) {
        return (angle >= min && angle <= max);
    } else {
        return (angle >= min || angle <= max);
    }
}
uint16_T IsInsideWtBuffer(real32_T angle, real32_T min, real32_T max, real32_T buffer)
{
    if (max >= min) {
        return (angle >= min + buffer && angle <= max - buffer);
    } else {
        return (angle >= min + buffer || angle <= max - buffer);
    }
}
uint16_T fnSwitchPreOP(
    real32_T TraPos,
    real32_T ElvPos,
    const real32_T RZone[][5],  // assuming 5 columns
    uint16_T NumRZones,
    real32_T preOPBuffer
) {
    uint16_T count;
    for ( count = 0; count < NumRZones; count++) {
        if (RZone[count][RZONE_EN]) {
            uint16_T insideTra = IsInsideWtBuffer(TraPos, RZone[count][RZONE_TRMIN], RZone[count][RZONE_TRMAX], preOPBuffer);
            uint16_T insideElv = IsInsideWtBuffer(ElvPos, RZone[count][RZONE_ELMIN], RZone[count][RZONE_ELMAX], preOPBuffer);
            if (insideTra && insideElv) {
                return 1;  // inside both buffered ranges
            }
        }
    }
    return 0;
}

// Assumes WZONE_TRMIN, WZONE_TRMAX, WZONE_ELMIN, WZONE_ELMAX are defined (0-based)

void IsInsideWZone(
    real32_T TraLim[3],
    real32_T ElvLim[3],
    const real32_T *TraPos,
    const real32_T *ElvPos,
    const real32_T WZone[4],
    real32_T WZoneTraEn,
    real32_T WZoneElvEn,
    real32_T *TraStop,
    real32_T *ElvStop,
    uint16_T *ret
)
{
    *TraStop = 0.0f;
    *ElvStop = 0.0f;
    *ret = 0;

    uint16_T violation =
        ((WZoneTraEn != 0.0f && !IsInsideLC(*TraPos, WZone[WZONE_TRMIN], WZone[WZONE_TRMAX])) ||
         (WZoneElvEn != 0.0f && !IsInsideLC(*ElvPos, WZone[WZONE_ELMIN], WZone[WZONE_ELMAX])));

    if (violation) {
        if (WZoneTraEn != 0.0f && !IsInsideLC(*TraPos, WZone[WZONE_TRMIN], WZone[WZONE_TRMAX])) {
            if (WZone[WZONE_TRMAX] > WZone[WZONE_TRMIN]) {
                if ((*TraPos - WZone[WZONE_TRMAX]) < (WZone[WZONE_TRMIN] - *TraPos)) {
                    TraLim[0] = WZone[WZONE_TRMIN];
                    TraLim[1] = WZone[WZONE_TRMAX] - 360.0f;
                } else {
                    TraLim[0] = WZone[WZONE_TRMIN] + 360.0f;
                    TraLim[1] = WZone[WZONE_TRMAX];
                }
            } else {
                TraLim[0] = WZone[WZONE_TRMIN];
                TraLim[1] = WZone[WZONE_TRMAX];
            }
            *TraStop = 0.0f;
            *ElvStop = 1.0f;
        }

        if (WZoneElvEn != 0.0f && !IsInsideLC(*ElvPos, WZone[WZONE_ELMIN], WZone[WZONE_ELMAX])) {
            if ((*ElvPos - WZone[WZONE_ELMAX]) < (WZone[WZONE_ELMIN] - *ElvPos)) {
                ElvLim[0] = WZone[WZONE_ELMIN];
                ElvLim[1] = WZone[WZONE_ELMAX] - 360.0f;
            } else {
                ElvLim[0] = WZone[WZONE_ELMIN] + 360.0f;
                ElvLim[1] = WZone[WZONE_ELMAX];
            }
            *TraStop = 1.0f;
            *ElvStop = 0.0f;
        }

        *ret = 1;
    }
}
void IsInsideRZone(
    real32_T TraLim[3],
    real32_T ElvLim[3],
    const real32_T *TraPos,
    const real32_T *ElvPos,
    const real32_T RZone[RZONE_ROWS][RZONE_COLS],  // assumes fixed 5 columns per zone
    uint16_T NumRZones,
    const real32_T WZone[WZONE_LEN],
    real32_T *TraStop,
    real32_T *ElvStop,
    uint16_T *ret
)
{
    *TraStop = 0.0f;
    *ElvStop = 0.0f;
    *ret = 0;

    uint16_T violationFound = 0;
    uint16_T matchCount = 0;
    uint16_T count ;
    for ( count = 0; count < NumRZones; count++) {
        if (RZone[count][RZONE_EN]) {
            uint16_T violation = IsInsideLC(*TraPos, RZone[count][RZONE_TRMIN], RZone[count][RZONE_TRMAX]) &&
                            IsInsideLC(*ElvPos, RZone[count][RZONE_ELMIN], RZone[count][RZONE_ELMAX]);

            violationFound |= violation;

            if (violation) {
                matchCount++;
                if (matchCount == 1) {
                    TraLim[0] = RZone[count][RZONE_TRMAX];
                    TraLim[1] = RZone[count][RZONE_TRMIN];
                    ElvLim[0] = RZone[count][RZONE_ELMAX];
                    ElvLim[1] = RZone[count][RZONE_ELMIN];
                } else {
                    TraLim[0] = fmaxf(TraLim[0], RZone[count][RZONE_TRMAX]);
                    TraLim[1] = fminf(TraLim[1], RZone[count][RZONE_TRMIN]);
                    ElvLim[0] = fmaxf(ElvLim[0], RZone[count][RZONE_ELMAX]);
                    ElvLim[1] = fminf(ElvLim[1], RZone[count][RZONE_ELMIN]);
                }
            }
        }
    }

    if (violationFound) {
        if (ElvLim[0] != WZone[3] || ElvLim[1] != WZone[1]) {
            if (fminf(fabsf(TraLim[0] - *TraPos), fabsf(TraLim[1] - *TraPos)) <=
                fminf(fabsf(ElvLim[0] - *ElvPos), fabsf(ElvLim[1] - *ElvPos))) {
                if (fabsf(TraLim[0] - *TraPos) <= fabsf(TraLim[1] - *TraPos) && TraLim[0] == WZone[2]) {
                    *TraStop = 1.0f;
                    *ElvStop = 0.0f;
                } else if (fabsf(TraLim[0] - *TraPos) > fabsf(TraLim[1] - *TraPos) && TraLim[1] == WZone[0]) {
                    *TraStop = 1.0f;
                    *ElvStop = 0.0f;
                } else {
                    *TraStop = 0.0f;
                    *ElvStop = 1.0f;
                }
            } else {
                *TraStop = 1.0f;
                *ElvStop = 0.0f;
            }
        } else {
            *TraStop = 0.0f;
            *ElvStop = 1.0f;
        }

        *ret = 1;
    }
}

void updateBoundaries(
    real32_T TraPos,
    real32_T ElvPos,
    const real32_T RZone[RZONE_ROWS][RZONE_COLS],
    const real32_T TraLim[3],
    const real32_T ElvLim[3],
    real32_T endDampTraLim[3],
    real32_T endDampElvLim[3],
    real32_T posCmdShaperTraLim[3],
    real32_T posCmdShaperElvLim[3],
    real32_T slope,
    real32_T ExpandedZoneLimMin,
    real32_T ExpandedZoneLimMax
) {
    real32_T tempTraLim[3] = { TraLim[0], TraLim[1], TraLim[2] };
    real32_T spdModeTraLim[3] = { TraLim[0], TraLim[1], TraLim[2] };
    uint16_T count;
    for ( count = 0; count < RZONE_ROWS; count++) {
        if (RZone[count][RZONE_EN]) {
            uint16_T inTra = IsInsideLC(TraPos, RZone[count][RZONE_TRMIN], RZone[count][RZONE_TRMAX]);
            uint16_T inElv = IsInsideLC(ElvPos, RZone[count][RZONE_ELMIN], RZone[count][RZONE_ELMAX]);

            if (!inTra && inElv) {
                if (TraPos < RZone[count][RZONE_TRMIN]) {
                    tempTraLim[0] = RZone[count][RZONE_TRMAX] - 360.0f;
                    tempTraLim[1] = RZone[count][RZONE_TRMIN];
                } else {
                    tempTraLim[0] = RZone[count][RZONE_TRMAX];
                    tempTraLim[1] = RZone[count][RZONE_TRMIN] + 360.0f;
                }
            } else if (inTra && !inElv) {
                if (ElvPos < RZone[count][RZONE_ELMIN]) {
                    endDampElvLim[2] = RZone[count][RZONE_ELMIN];
                    posCmdShaperElvLim[2] = RZone[count][RZONE_ELMIN];
                } else {
                    endDampElvLim[0] = RZone[count][RZONE_ELMAX];
                    posCmdShaperElvLim[0] = RZone[count][RZONE_ELMAX];
                }
            } else {
                uint16_T corner = IsInCorner(TraPos, ElvPos, (real32_T[4]){
                    RZone[count][RZONE_TRMIN],
                    RZone[count][RZONE_TRMAX],
                    RZone[count][RZONE_ELMIN],
                    RZone[count][RZONE_ELMAX]
                }, 180.0f);

                switch (corner) {
                    case 1:
                        spdModeTraLim[0] = (RZone[count][RZONE_TRMIN] > RZone[count][RZONE_TRMAX] - (ElvPos - RZone[count][RZONE_ELMAX]) / slope)
                            ? ExpandedZoneLimMin
                            : (RZone[count][RZONE_TRMAX] - (ElvPos - RZone[count][RZONE_ELMAX]) / slope) - 360.0f;

                        spdModeTraLim[1] = (RZone[count][RZONE_TRMAX] < RZone[count][RZONE_TRMIN] + (ElvPos - RZone[count][RZONE_ELMAX]) / slope)
                            ? ExpandedZoneLimMax
                            : RZone[count][RZONE_TRMIN] + (ElvPos - RZone[count][RZONE_ELMAX]) / slope;

                        endDampElvLim[0] = fmaxf(endDampElvLim[0], RZone[count][RZONE_ELMAX] - (RZone[count][RZONE_TRMIN] - TraPos) / slope);
                        break;

                    case 2:
                        spdModeTraLim[0] = (RZone[count][RZONE_TRMIN] > RZone[count][RZONE_TRMAX] - (ElvPos - RZone[count][RZONE_ELMAX]) / slope)
                            ? ExpandedZoneLimMin
                            : RZone[count][RZONE_TRMAX] - (ElvPos - RZone[count][RZONE_ELMAX]) / slope;

                        spdModeTraLim[1] = (RZone[count][RZONE_TRMAX] < RZone[count][RZONE_TRMIN] + (ElvPos - RZone[count][RZONE_ELMAX]) / slope)
                            ? ExpandedZoneLimMax
                            : RZone[count][RZONE_TRMIN] + (ElvPos - RZone[count][RZONE_ELMAX]) / slope + 360.0f;

                        endDampElvLim[0] = fmaxf(endDampElvLim[0], RZone[count][RZONE_ELMAX] - (TraPos - RZone[count][RZONE_TRMAX]) / slope);
                        break;

                    case 3:
                        spdModeTraLim[0] = (RZone[count][RZONE_TRMIN] > RZone[count][RZONE_TRMAX] - (RZone[count][RZONE_ELMIN] - ElvPos) / slope)
                            ? ExpandedZoneLimMin
                            : RZone[count][RZONE_TRMAX] - (RZone[count][RZONE_ELMIN] - ElvPos) / slope;

                        spdModeTraLim[1] = (RZone[count][RZONE_TRMAX] < RZone[count][RZONE_TRMIN] + (RZone[count][RZONE_ELMIN] - ElvPos) / slope)
                            ? ExpandedZoneLimMax
                            : RZone[count][RZONE_TRMIN] + (RZone[count][RZONE_ELMIN] - ElvPos) / slope + 360.0f;

                        endDampElvLim[2] = fminf(endDampElvLim[2], RZone[count][RZONE_ELMIN] + (TraPos - RZone[count][RZONE_TRMAX]) / slope);
                        break;

                    case 4:
                        spdModeTraLim[0] = (RZone[count][RZONE_TRMIN] > RZone[count][RZONE_TRMAX] - (RZone[count][RZONE_ELMIN] - ElvPos) / slope)
                            ? ExpandedZoneLimMin
                            : (RZone[count][RZONE_TRMAX] - (RZone[count][RZONE_ELMIN] - ElvPos) / slope) - 360.0f;

                        spdModeTraLim[1] = (RZone[count][RZONE_TRMAX] < RZone[count][RZONE_TRMIN] + (RZone[count][RZONE_ELMIN] - ElvPos) / slope)
                            ? ExpandedZoneLimMax
                            : RZone[count][RZONE_TRMIN] + (RZone[count][RZONE_ELMIN] - ElvPos) / slope;

                        endDampElvLim[2] = fminf(endDampElvLim[2], RZone[count][RZONE_ELMIN] + (RZone[count][RZONE_TRMIN] - TraPos) / slope);
                        break;
                }
            }

            endDampTraLim[0] = fmaxf(fmaxf(tempTraLim[0], endDampTraLim[0]), spdModeTraLim[0]);
            endDampTraLim[1] = fminf(fminf(tempTraLim[1], endDampTraLim[1]), spdModeTraLim[1]);

            posCmdShaperTraLim[0] = fmaxf(tempTraLim[0], TraLim[0]);
            posCmdShaperTraLim[1] = fminf(tempTraLim[1], TraLim[1]);
        }
    }
    // // Middle = average of min and max limits
    posCmdShaperTraLim[2] = Middle(posCmdShaperTraLim[0] , posCmdShaperTraLim[1]) ;
    posCmdShaperElvLim[2] = Middle(posCmdShaperElvLim[0] , posCmdShaperElvLim[1]);
    endDampTraLim[2]      = Middle(endDampTraLim[0]      , endDampTraLim[1])     ;
    endDampElvLim[2]      = Middle(endDampElvLim[0]      , endDampElvLim[1])     ;
}








static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 0);
    if (!ssSetNumInputPorts(S, 10)) return;
    uint16_T i;
    for ( i = 0; i < 4; ++i) ssSetInputPortWidth(S, i, 1);
    ssSetInputPortWidth(S, 4, WZONE_LEN);
    ssSetInputPortMatrixDimensions(S, 5, RZONE_ROWS, RZONE_COLS);
    for ( i = 6; i < 10; ++i) ssSetInputPortWidth(S, i, 1);
    for ( i = 0; i < 10; ++i) ssSetInputPortDirectFeedThrough(S, i, 1);

    for ( i = 0; i < 10; ++i) {
        ssSetInputPortDataType(S, i, SS_SINGLE);
    }

    ssSetNumOutputPorts(S, 7);
    for ( i = 0; i < 7; ++i) {
        uint16_T width = (i == 2 || i == 3 || i == 6) ? 1 : 3;
        ssSetOutputPortWidth(S, i, width);
        ssSetOutputPortDataType(S, i, SS_SINGLE);
    }
    

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}


// Function to determine the command zone based on command position and direction




// 
// static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port,
//                                         const DimsInfo_T *dimsInfo)
// {
//     ssSetOutputPortMatrixDimensions(S, 0, inRow, inCol);
// 
// }
// 
// 
// 
// static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port,
//                                         const DimsInfo_T *dimsInfo)
// {
//     if (!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
// 
// }


static void mdlOutputs(SimStruct *S, int_T tid) {
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real32_T TraPos = *((const real32_T *) uPtrs[0]);
    InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S, 1);
    real32_T ElvPos = *((const real32_T *) uPtrs1[0]);
    
    InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S, 2);
    real32_T WZoneTraEn = *((const real32_T *) uPtrs2[0]);
    InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs(S, 3);
    real32_T WZoneElvEn = *((const real32_T *) uPtrs3[0]);
    // input 4
    InputRealPtrsType WZonePointer = ssGetInputPortRealSignalPtrs(S, 4);
    uint16_T i, j, idx;
    real32_T WZone[WZONE_LEN] = {0};  // ✅ Here
    for (i = 0; i < WZONE_LEN; i++) { 
        WZone[i] = *((const real32_T *)WZonePointer[i]);  // cast double to float
    }
    InputRealPtrsType flatRZone = ssGetInputPortRealSignalPtrs(S, 5);
    // input 5 
    uint16_T NumRZones = 0;
    real32_T RZones[RZONE_ROWS][RZONE_COLS] = {0};  // ✅ Here
    for (i = 0; i < RZONE_ROWS; i++) {
        for (j = 0; j < RZONE_COLS; j++) {
            idx = i + j * RZONE_ROWS;
            RZones[i][j] = *((const real32_T *)flatRZone[idx]);  // cast double to float
        }
    
        if (RZones[i][0] == 1 || 
            RZones[i][1] != 0 || 
            RZones[i][2] != 0 || 
            RZones[i][3] != 0 || 
            RZones[i][4] != 0)
        {
            NumRZones++;
        }
    }

    

    InputRealPtrsType uPtrs6 = ssGetInputPortRealSignalPtrs(S, 6);
    real32_T ExpandedZoneLimMin = *((const real32_T *) uPtrs6[0]);
    
    InputRealPtrsType uPtrs7 = ssGetInputPortRealSignalPtrs(S, 7);
    real32_T ExpandedZoneLimMax = *((const real32_T *) uPtrs7[0]);
    
    InputRealPtrsType uPtrs8 = ssGetInputPortRealSignalPtrs(S, 8);
    real32_T preOPBuffer = *((const real32_T *) uPtrs8[0]);
    
    InputRealPtrsType uPtrs9 = ssGetInputPortRealSignalPtrs(S, 9);
    real32_T slope = *((const real32_T *) uPtrs9[0]);
    // Logical operation Start 
    TraPos=NormalizeAngle(TraPos);
	ElvPos=NormalizeAngle(ElvPos); 
    real32_T TraLim[3]={0};
    uint16_T switchPreOP = 0.0f;
    uint16_T TraInf = (WZoneTraEn == 0.0f);
    uint16_T ElvInf = (WZoneElvEn == 0.0f);

    if (TraInf) {
        TraLim[0] = ExpandedZoneLimMin;
        TraLim[1] = ExpandedZoneLimMax;
        TraLim[2] = 0.0f;
    } else {
        if (WZone[WZONE_TRMIN] < WZone[WZONE_TRMAX]) {
            TraLim[0] = WZone[WZONE_TRMIN];
            TraLim[1] = WZone[WZONE_TRMAX];
            TraLim[2] = (WZone[WZONE_TRMIN] + WZone[WZONE_TRMAX]) / 2.0f;  // Middle value
        } else {
            if (TraPos > WZone[WZONE_TRMIN]) {
                TraLim[0] = WZone[WZONE_TRMIN];
                TraLim[1] = WZone[WZONE_TRMAX] + 360.0f;
                TraLim[2] = 0.0f;
            } else {
                TraLim[0] = WZone[WZONE_TRMIN] - 360.0f;
                TraLim[1] = WZone[WZONE_TRMAX];
                TraLim[2] = 0.0f;
            }
        }
    }
    real32_T endDampTraLim[3] = {0};
    real32_T posCmdShaperTraLim[3] = {0};

    for ( i = 0; i < 3; i++) {
        endDampTraLim[i] = TraLim[i];
        posCmdShaperTraLim[i] = TraLim[i];
    }


    // Assume the following are defined:
    // #define WZONE_ELMIN 1
    // #define WZONE_ELMAX 3
    
    real32_T ElvLim[3]= {0}; 
    real32_T endDampElvLim[3] = {0}; 
    real32_T posCmdShaperElvLim[3] = {0}; 
    
    if (ElvInf) {
        ElvLim[0] = ExpandedZoneLimMin;
        ElvLim[1] = ExpandedZoneLimMax;
        ElvLim[2] = 0.0f;
    } else {
        ElvLim[0] = WZone[WZONE_ELMIN];
        ElvLim[1] = WZone[WZONE_ELMAX];
        ElvLim[2] = (WZone[WZONE_ELMIN] + WZone[WZONE_ELMAX]) / 2.0f;  // Middle
    }
    
    for ( i = 0; i < 3; i++) {
        endDampElvLim[i] = ElvLim[i];
        posCmdShaperElvLim[i] = ElvLim[i];
    }

    uint16_T ret;
    real32_T TraStop,ElvStop;
    IsInsideWZone(
            TraLim,
            ElvLim,
            &TraPos,
            &ElvPos,
            WZone,
            WZoneTraEn,
            WZoneElvEn,
            &TraStop,
            &ElvStop,
            &ret
        );
    if (ret) {
        for ( i = 0; i < 3; i++) {
            endDampTraLim[i] = TraLim[i];
            endDampElvLim[i] = ElvLim[i];
        }
    } else {
         switchPreOP = fnSwitchPreOP(TraPos, ElvPos, RZones, NumRZones, preOPBuffer);
         
         IsInsideRZone(
                TraLim,
                ElvLim,
                &TraPos,         
                &ElvPos,         
                RZones,
                NumRZones,
                WZone,
                &TraStop,
                &ElvStop,
                &ret
            );
            if (ret) {
                for ( i = 0; i < 3; i++) {
                    endDampTraLim[i] = TraLim[i];
                    endDampElvLim[i] = ElvLim[i];
                }
            } else {
                updateBoundaries(
                    TraPos,
                    ElvPos,
                    RZones,
                    TraLim,
                    ElvLim,
                    endDampTraLim,
                    endDampElvLim,
                    posCmdShaperTraLim,
                    posCmdShaperElvLim,
                    slope,
                    ExpandedZoneLimMin,
                    ExpandedZoneLimMax
                );
            }
     
    }
       
        


    
   





    // Logical operation End  
    // output 
    real32_T *endDampTraLimOut = (real32_T *)ssGetOutputPortSignal(S, 0);
    real32_T *endDampElvLimOut = (real32_T *)ssGetOutputPortSignal(S, 1);
    real32_T *TraStopOut = (real32_T *)ssGetOutputPortSignal(S, 2);
    real32_T *ElvStopOut = (real32_T *)ssGetOutputPortSignal(S, 3);
    real32_T *posCmdShaperTraLimOut = (real32_T *)ssGetOutputPortSignal(S, 4);
    real32_T *posCmdShaperElvLimOut = (real32_T *)ssGetOutputPortSignal(S, 5);
    real32_T *switchPreOPOut = (real32_T *)ssGetOutputPortSignal(S, 6);

    endDampTraLimOut[0] = endDampTraLim[0];
    endDampTraLimOut[1] = endDampTraLim[1];
    endDampTraLimOut[2] = endDampTraLim[2];

    endDampElvLimOut[0] = endDampElvLim[0];
    endDampElvLimOut[1] = endDampElvLim[1];
    endDampElvLimOut[2] = endDampElvLim[2];

    TraStopOut[0] = (real32_T)TraStop;

    ElvStopOut[0] = (real32_T)ElvStop;

    posCmdShaperTraLimOut[0] = posCmdShaperTraLim[0];
    posCmdShaperTraLimOut[1] = posCmdShaperTraLim[1];
    posCmdShaperTraLimOut[2] = posCmdShaperTraLim[2];

    posCmdShaperElvLimOut[0] = posCmdShaperElvLim[0];
    posCmdShaperElvLimOut[1] = posCmdShaperElvLim[1];
    posCmdShaperElvLimOut[2] = posCmdShaperElvLim[2];

    switchPreOPOut[0] = switchPreOP;
    
}

static void mdlTerminate(SimStruct *S) {}
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
