#define S_FUNCTION_NAME  trajplan
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#if defined(MATLAB_MEX_FILE)
    #include "tmwtypes.h"
    #include "simstruc_types.h"
#else
    #include "rtwtypes.h"
#endif
/* Static weights and biases from your original code */
#define MAX_SIZE 160  // 32*5
#define inRow 32
#define inCol 5
// === Zone Indexes ===
#define RZoneEn         0
#define RZoneTrMin      1
#define RZoneElMin      2
#define RZoneTrMax      3
#define RZoneElMax      4

#define WZoneTrMin      0
#define WZoneElMin      1
#define WZoneTrMax      2
#define WZoneElMax      3

// === Directional Constants ===
#define ZONE_NONE          -1   // New: represents 'no zone'
#define LEFT            0
#define RIGHT           1
#define TOP             2
#define BOTTOM          3
#define LEFT_TOP_CORNER     4
#define RIGHT_TOP_CORNER    5
#define LEFT_BOTTOM_CORNER  6
#define RIGHT_BOTTOM_CORNER 7

#define DIR_LEFT_UP     0
#define DIR_RIGHT_UP    1
#define DIR_LEFT_DOWN   2
#define DIR_RIGHT_DOWN  3







/*====================*
 * S-function methods *
 *====================*/
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
uint16_T IsInside(real32_T Angle, real32_T Min, real32_T Max)
{
    if (Max >= Min) {
        return (Angle >= Min) && (Angle <= Max);
    } else {
        return (Angle >= Min) || (Angle <= Max);
    }
}

uint16_T changeDirectionIfDeadEnd(uint16_T direction, uint16_T foundDeadEnd)
{
       if (foundDeadEnd) {
            if (direction == DIR_LEFT_UP) {
                direction = DIR_RIGHT_UP;
            } else if (direction == DIR_LEFT_DOWN) {
                direction = DIR_RIGHT_DOWN;
            } else if (direction == DIR_RIGHT_UP) {
                direction = DIR_LEFT_UP;
            } else if (direction == DIR_RIGHT_DOWN) {
                direction = DIR_LEFT_DOWN;
            }
       }
    return direction;
}

uint16_T IsInPath(uint16_T direction, const real32_T CurZone[5], real32_T TraCmd, real32_T TraPos)
{
    uint16_T inPath = 0;

    uint16_T isRight = (direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN);

    uint16_T checkMin = IsInside(CurZone[RZoneTrMin], TraPos, TraCmd);
    uint16_T checkMax = IsInside(CurZone[RZoneTrMax], TraPos, TraCmd);

    if (isRight) {
        inPath = (checkMin || checkMax);
    } else {
        inPath = (!checkMin || !checkMax);
    }

    return inPath;
}

uint16_T findDeadEnd(
    uint16_T direction,
    real32_T TraCmd,
    real32_T TraPos,
    const real32_T WZone[4],
    real32_T WZoneTrvEn,
    const real32_T RZone[inRow][inCol],
    uint16_T NumRZones,
    real32_T buffer
)
{
    uint16_T Count = 0;
    for ( Count = 0; Count < NumRZones; Count++) {
        if (RZone[Count][RZoneEn]) {
            if ((RZone[Count][RZoneElMin] - buffer <= WZone[WZoneElMin]) &&
                (RZone[Count][RZoneElMax] + buffer >= WZone[WZoneElMax])) {

                if (IsInPath(direction, RZone[Count], TraCmd, TraPos)) {
                    return 1;  // found = true
                }
            } else {
                // Explicit else clause from MATLAB is not required — found stays 0
            }
        }

        if (WZoneTrvEn) {
            // Create a pseudo RZone row from WZone
            real32_T pseudoRZone[5] = {0};
            pseudoRZone[RZoneTrMin] = WZone[WZoneTrMin];
            pseudoRZone[RZoneTrMax] = WZone[WZoneTrMax];
            pseudoRZone[RZoneElMin] = WZone[WZoneElMin];
            pseudoRZone[RZoneElMax] = WZone[WZoneElMax];
            // NOTE: pseudoRZone[RZoneEn] = 0 (not used)

            if (IsInPath(direction, pseudoRZone, TraCmd, TraPos)) {
                return 1;
            }
        }
    }

    return 0;  // Not found
}



uint16_T findDirection(real32_T TraCmd, real32_T TraPos, real32_T ElvCmd, real32_T ElvPos)
{
    uint16_T direction = 0;

    if (TraCmd <= TraPos) {
        if ((TraPos - TraCmd) < 180.0f) {
            if (ElvPos <= ElvCmd) {
                direction = DIR_LEFT_UP;
            } else {
                direction = DIR_LEFT_DOWN ;//DIR_LEFT_DOWN;
            }
        } else {
            if (ElvPos <= ElvCmd) {
                direction = DIR_RIGHT_UP;
            } else {
                direction = DIR_RIGHT_DOWN;
            }
        }
    } else if (TraCmd > TraPos) {
        if ((TraCmd - TraPos) < 180.0f) {
            if (ElvPos <= ElvCmd) {
                direction = DIR_RIGHT_UP;
            } else {
                direction = DIR_RIGHT_DOWN;
            }
        } else {
            if (ElvPos <= ElvCmd) {
                direction = DIR_LEFT_UP;
            } else {
                direction = DIR_LEFT_DOWN;
            }
        }
    } else {
        direction = 75;  // Optional fallback, already initialized
    }

    return direction;
}

void manipulateTargetPos(
    real32_T *manTraPos,
    real32_T *manElvPos,
    real32_T TraCmd,
    real32_T ElvCmd,
    real32_T TraPos,
    real32_T ElvPos,
    const real32_T WZone[4],
    real32_T WZoneTrvEn,
    real32_T WZoneElvEn,
    real32_T Axis1EndDampingBuffer,
    real32_T Axis2EndDampingBuffer,
    const real32_T RZone[inRow][inCol],
    uint16_T NumRZones,
    real32_T buffer)
    {
    // === WZone Elv Adjustment ===
     
    if (WZoneElvEn) {
        if (ElvCmd < WZone[WZoneElMin]) {
            ElvCmd = WZone[WZoneElMin] + Axis2EndDampingBuffer;
        }
        if (ElvCmd > WZone[WZoneElMax]) {
            ElvCmd = WZone[WZoneElMax] - Axis2EndDampingBuffer;
        }
    }
    
    // === WZone Tra Adjustment ===
    if (WZoneTrvEn) {
        if (WZone[WZoneTrMin] < WZone[WZoneTrMax]) {
            if (TraCmd < WZone[WZoneTrMin]) {
                TraCmd = WZone[WZoneTrMin] + Axis1EndDampingBuffer;
            } else if (TraCmd > WZone[WZoneTrMax]) {
                TraCmd = WZone[WZoneTrMax] - Axis1EndDampingBuffer;
            }
        } else {
            float mid = (WZone[WZoneTrMin] + WZone[WZoneTrMax]) / 2.0f;
            if (TraCmd > mid && TraCmd < WZone[WZoneTrMin]) {
                TraCmd = WZone[WZoneTrMin] + Axis1EndDampingBuffer;
            } else if (TraCmd <= mid && TraCmd > WZone[WZoneTrMax]) {
                TraCmd = WZone[WZoneTrMax] - Axis1EndDampingBuffer;
            }
        }
    }
    

    // === Find direction (placeholder function) ===
    uint16_T init_direction = findDirection(TraCmd, TraPos, ElvCmd, ElvPos);
     
    // === Loop through RZones ===
    uint16_T Count;
    for (Count = 0; Count < NumRZones; Count++) {
        if (RZone[Count][RZoneEn]) {
            uint16_T insideTra = IsInside(TraCmd, RZone[Count][RZoneTrMin], RZone[Count][RZoneTrMax]);
            uint16_T insideElv = IsInside(ElvCmd, RZone[Count][RZoneElMin], RZone[Count][RZoneElMax]);

            if (insideTra && insideElv) {
                if (RZone[Count][RZoneElMax] + buffer < WZone[WZoneElMax]) {
                    ElvCmd = RZone[Count][RZoneElMax] + buffer;
                } else if (RZone[Count][RZoneElMin] - buffer > WZone[WZoneElMin]) {
                    ElvCmd = RZone[Count][RZoneElMin] - buffer;
                } else {
                    if (init_direction == DIR_LEFT_UP || init_direction == DIR_LEFT_DOWN) {
                        TraCmd = RZone[Count][RZoneTrMax] + buffer;
                    } else if (init_direction == DIR_RIGHT_UP || init_direction == DIR_RIGHT_DOWN) {
                        TraCmd = RZone[Count][RZoneTrMin] - buffer;
                    }
                }
            }
        }
    }
    *manTraPos = TraCmd;//TraCmd;
    *manElvPos = ElvCmd;//ElvPos;
    
}
uint16_T findCmdZone(
    uint16_T direction,
    real32_T TraCmd,
    real32_T ElvCmd,
    const real32_T CurRZone[5],
    uint16_T foundDeadEnd
)
{
    uint16_T cmdZone = ZONE_NONE;

    uint16_T insideElv = IsInside(ElvCmd, CurRZone[RZoneElMin], CurRZone[RZoneElMax]);
    uint16_T insideTra = IsInside(TraCmd, CurRZone[RZoneTrMin], CurRZone[RZoneTrMax]);

    if (insideElv) {
        if (insideTra) {
            cmdZone = ZONE_NONE;
        } else if (((TraCmd - CurRZone[RZoneTrMax]) < (CurRZone[RZoneTrMin] - (TraCmd - 360.0f)) &&
                    IsInside(180.0f, TraCmd, CurRZone[RZoneTrMin]) && !foundDeadEnd) ||
                   (direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN)) {
            cmdZone = RIGHT;
        } else if (((CurRZone[RZoneTrMin] - TraCmd) < ((TraCmd + 360.0f) - CurRZone[RZoneTrMax]) &&
                     IsInside(-180.0f, CurRZone[RZoneTrMin], TraCmd) && foundDeadEnd) ||
                   (direction == DIR_LEFT_UP || direction == DIR_LEFT_DOWN)) {
            cmdZone = LEFT;
        }
    } else if (ElvCmd > CurRZone[RZoneElMax]) {
        if (((TraCmd - CurRZone[RZoneTrMax]) < (CurRZone[RZoneTrMin] - (TraCmd - 360.0f)) &&
             IsInside(180.0f, TraCmd, CurRZone[RZoneTrMin])) ||
            ((direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN) &&
             IsInside(180.0f, CurRZone[RZoneTrMax], TraCmd))) {
            cmdZone = RIGHT_TOP_CORNER;
        } else if (((CurRZone[RZoneTrMin] - TraCmd) < ((TraCmd + 360.0f) - CurRZone[RZoneTrMax]) &&
                    IsInside(-180.0f, CurRZone[RZoneTrMin], TraCmd)) ||
                   ((direction == DIR_LEFT_UP || direction == DIR_LEFT_DOWN) &&
                    IsInside(-180.0f, TraCmd, CurRZone[RZoneTrMax]))) {
            cmdZone = LEFT_TOP_CORNER;
        }

        if (insideTra || ((direction == DIR_LEFT_UP || direction == DIR_LEFT_DOWN) &&
                          IsInside(-180.0f, TraCmd, CurRZone[RZoneTrMax]))) {
            cmdZone = TOP;
        }
    } else if (ElvCmd < CurRZone[RZoneElMin]) {
        if (((TraCmd - CurRZone[RZoneTrMax]) < (CurRZone[RZoneTrMin] - (TraCmd - 360.0f)) &&
             IsInside(180.0f, TraCmd, CurRZone[RZoneTrMin])) ||
            ((direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN) &&
             IsInside(180.0f, CurRZone[RZoneTrMax], TraCmd))) {
            cmdZone = RIGHT_BOTTOM_CORNER;
        } else if (((CurRZone[RZoneTrMin] - TraCmd) < ((TraCmd + 360.0f) - CurRZone[RZoneTrMax]) &&
                    IsInside(-180.0f, CurRZone[RZoneTrMin], TraCmd)) ||
                   ((direction == DIR_LEFT_UP || direction == DIR_LEFT_DOWN) &&
                    IsInside(-180.0f, TraCmd, CurRZone[RZoneTrMax]))) {
            cmdZone = LEFT_BOTTOM_CORNER;
        }

        if (insideTra) {
            cmdZone = BOTTOM;
        }
    }

    return cmdZone;
}

// Function to compute total distance to current restricted zone
real32_T distCurRZone(
    uint16_T direction,
    uint16_T myZone,
    real32_T TraPos,
    real32_T TraCmd,
    real32_T ElvPos,
    real32_T ElvCmd,
    const real32_T RZone[5],
    uint16_T foundDeadEnd,
    real32_T BUFFER
)
{
    real32_T trDist = 0.0f;
    real32_T elDist = 0.0f;
    real32_T totalDist = 0.0f;

    uint16_T tempCmdZone = findCmdZone(direction, TraCmd, ElvCmd, RZone, foundDeadEnd);

    // If both current and command zones are on top or bottom, deprioritize this RZone
    uint16_T bothTop =
        (myZone == RIGHT_TOP_CORNER || myZone == LEFT_TOP_CORNER || myZone == TOP) &&
        (tempCmdZone == RIGHT_TOP_CORNER || tempCmdZone == LEFT_TOP_CORNER || tempCmdZone == TOP);

    uint16_T bothBottom =
        (myZone == RIGHT_BOTTOM_CORNER || myZone == LEFT_BOTTOM_CORNER || myZone == BOTTOM) &&
        (tempCmdZone == RIGHT_BOTTOM_CORNER || tempCmdZone == LEFT_BOTTOM_CORNER || tempCmdZone == BOTTOM);

    if (bothTop || bothBottom) {
        totalDist = 9999.0f;
    } else {
        // Elevation axis distance to zone
        if (myZone == LEFT || myZone == RIGHT) {
            elDist = BUFFER + 0.1f;
        } else {
            real32_T d1 = fabsf(ElvPos - RZone[RZoneElMax]);
            real32_T d2 = fabsf(ElvPos - RZone[RZoneElMin]);
            elDist = fminf(d1, d2);
        }

        // Travel axis distance to zone
        if (myZone == TOP || myZone == BOTTOM) {
            trDist = BUFFER + 0.1f;
        } else {
            if (direction == DIR_LEFT_UP || direction == DIR_LEFT_DOWN) {
                trDist = fmodf(TraPos - RZone[RZoneTrMax] + 360.0f, 360.0f);
            } else if (direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN) {
                trDist = fmodf(RZone[RZoneTrMin] - TraPos + 360.0f, 360.0f);
            }
        }

        totalDist = trDist + elDist;
    }

    return totalDist;
}

// Updated findCurRZone with ZONE_NONE handling to distinguish valid zones
void findCurRZone(
    uint16_T direction,
    real32_T TraCmd,
    real32_T ElvCmd,
    real32_T TraPos,
    real32_T ElvPos,
    const real32_T RZone[inRow][inCol],
    uint16_T NumRZones,
    uint16_T foundDeadEnd,
    real32_T CurRZone[5],
    uint16_T *myZone,
    real32_T buffer
)
{
    uint16_T myZones[inRow],i;
    for (i = 0; i < inRow; i++) myZones[i] = ZONE_NONE;

    uint16_T index = -1;
    real32_T myZoneDist = 0.0f;
    real32_T totPrevDist = 0.0f;
    uint16_T Count;
    for ( Count = 0; Count < NumRZones; Count++) {
        if (RZone[Count][RZoneEn]) {
            if (TraPos >= RZone[Count][RZoneTrMin] && TraPos <= RZone[Count][RZoneTrMax]) {
                if (ElvPos > RZone[Count][RZoneElMax])
                    myZones[Count] = TOP;
                else if (ElvPos < RZone[Count][RZoneElMin])
                    myZones[Count] = BOTTOM;
            }

            if (IsInPath(direction, RZone[Count], TraCmd, TraPos)) {
                if (IsInside(ElvPos, RZone[Count][RZoneElMin], RZone[Count][RZoneElMax])) {
                    if (direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN)
                        myZones[Count] = LEFT;
                    else
                        myZones[Count] = RIGHT;
                } else if (TraPos < RZone[Count][RZoneTrMin] && ElvPos < RZone[Count][RZoneElMin]) {
                    if (IsInside(-180.0f, RZone[Count][RZoneTrMax], TraPos) &&
                        (direction == DIR_LEFT_UP || direction == DIR_LEFT_DOWN))
                        myZones[Count] = RIGHT_BOTTOM_CORNER;
                    else
                        myZones[Count] = LEFT_BOTTOM_CORNER;
                } else if (TraPos < RZone[Count][RZoneTrMin] && ElvPos > RZone[Count][RZoneElMax]) {
                    if (IsInside(-180.0f, RZone[Count][RZoneTrMax], TraPos) &&
                        (direction == DIR_LEFT_UP || direction == DIR_LEFT_DOWN))
                        myZones[Count] = RIGHT_TOP_CORNER;
                    else
                        myZones[Count] = LEFT_TOP_CORNER;
                } else if (TraPos > RZone[Count][RZoneTrMax] && ElvPos < RZone[Count][RZoneElMin]) {
                    if (IsInside(180.0f, TraPos, RZone[Count][RZoneTrMin]) &&
                        (direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN))
                        myZones[Count] = LEFT_BOTTOM_CORNER;
                    else
                        myZones[Count] = RIGHT_BOTTOM_CORNER;
                } else if (TraPos > RZone[Count][RZoneTrMax] && ElvPos > RZone[Count][RZoneElMax]) {
                    if (IsInside(180.0f, TraPos, RZone[Count][RZoneTrMin]) &&
                        (direction == DIR_RIGHT_UP || direction == DIR_RIGHT_DOWN))
                        myZones[Count] = LEFT_TOP_CORNER;
                    else
                        myZones[Count] = RIGHT_TOP_CORNER;
                }
            }

            if (myZones[Count] != ZONE_NONE) {
                real32_T dist = distCurRZone(direction, myZones[Count], TraPos, TraCmd, ElvPos, ElvCmd, RZone[Count], foundDeadEnd, buffer);
                if (dist > 0 && (totPrevDist == 0 || dist <= totPrevDist)) {
                    totPrevDist = dist;
                    index = Count;
                }
            }
        }
    }

    for ( i = 0; i < 5; i++) CurRZone[i] = 0.0f;
    *myZone = ZONE_NONE;

    if (index != -1) {
        for ( i = 0; i < 5; i++) {
            CurRZone[i] = RZone[index][i];
        }
        *myZone = myZones[index];
    }
}


// Function to compute manipulated position commands based on zone transitions
void manipulatePosCmds(
    real32_T TraCmd,
    real32_T ElvCmd,
    uint16_T myZone,
    uint16_T cmdZone,
    const real32_T WZone[4],
    const real32_T CurRZone[5],
    real32_T buffer,
    real32_T *TraDmd,
    real32_T *ElvDmd,
    uint16_T *nextZoneResul
)
{
    *TraDmd = TraCmd;
    *ElvDmd = ElvCmd;
    uint16_T nextZone = 75; 
        

    switch (cmdZone) {
        case BOTTOM:
            switch (myZone) {
                case TOP:               nextZone = LEFT_TOP_CORNER; break;
                case LEFT_TOP_CORNER:   nextZone = LEFT; break;
                case LEFT:              nextZone = LEFT_BOTTOM_CORNER; break;
                case LEFT_BOTTOM_CORNER:
                case RIGHT_TOP_CORNER:  nextZone = RIGHT; break;
                case RIGHT:             nextZone = RIGHT_BOTTOM_CORNER; break;
            }
            break;
        case LEFT_BOTTOM_CORNER:
            switch (myZone) {
                case TOP:               nextZone = LEFT_TOP_CORNER; break;
                case RIGHT:             nextZone = RIGHT_BOTTOM_CORNER; break;
                case RIGHT_TOP_CORNER:
                case LEFT:
                case BOTTOM:            nextZone = RIGHT; break;
            }
            break;
        case LEFT:
            switch (myZone) {
                case TOP:               nextZone = LEFT_TOP_CORNER; break;
                case LEFT_TOP_CORNER:
                case LEFT_BOTTOM_CORNER:
                case BOTTOM:            nextZone = LEFT_BOTTOM_CORNER; break;
                case RIGHT_TOP_CORNER:  nextZone = TOP; break;
                case RIGHT:
                    if ((CurRZone[RZoneElMax] + buffer) >= WZone[WZoneElMax])
                        nextZone = RIGHT_BOTTOM_CORNER;
                    else
                        nextZone = RIGHT_TOP_CORNER;
                    break;
                case RIGHT_BOTTOM_CORNER: nextZone = BOTTOM; break;
            }
            break;
        case LEFT_TOP_CORNER:
            switch (myZone) {
                case LEFT:
                case LEFT_BOTTOM_CORNER:
                case BOTTOM:            nextZone = LEFT_BOTTOM_CORNER; break;
                case RIGHT:
                case RIGHT_TOP_CORNER:  nextZone = LEFT_TOP_CORNER; break;
                case RIGHT_BOTTOM_CORNER: nextZone = BOTTOM; break;
            }
            break;
        case TOP:
            switch (myZone) {
                case LEFT:              nextZone = LEFT_TOP_CORNER; break;
                case LEFT_BOTTOM_CORNER:nextZone = LEFT; break;
                case BOTTOM:            nextZone = LEFT_BOTTOM_CORNER; break;
                case RIGHT:             nextZone = RIGHT_TOP_CORNER; break;
                case RIGHT_BOTTOM_CORNER:nextZone = RIGHT; break;
                default:                nextZone = 75; break;
            }
            break;
        case RIGHT_TOP_CORNER:
            switch (myZone) {
                case LEFT:              nextZone = LEFT_TOP_CORNER; break;
                case LEFT_BOTTOM_CORNER:nextZone = LEFT; break;
                case BOTTOM:            nextZone = RIGHT_BOTTOM_CORNER; break;
            }
            break;
        case RIGHT:
            switch (myZone) {
                case LEFT_TOP_CORNER:   nextZone = TOP; break;
                case LEFT:
                    if ((CurRZone[RZoneElMax] + buffer) >= WZone[WZoneElMax])
                        nextZone = LEFT_BOTTOM_CORNER;
                    else
                        nextZone = LEFT_TOP_CORNER;
                    break;
                case LEFT_BOTTOM_CORNER:nextZone = BOTTOM; break;
                case BOTTOM:            nextZone = RIGHT_BOTTOM_CORNER; break;
                case TOP:               nextZone = RIGHT_TOP_CORNER; break;
            }
            break;
        case RIGHT_BOTTOM_CORNER:
            switch (myZone) {
                case LEFT_TOP_CORNER:   nextZone = TOP; break;
                case LEFT:              nextZone = LEFT_BOTTOM_CORNER; break;
                case TOP:               nextZone = RIGHT_TOP_CORNER; break;
            }
            break;
    }
    
    *nextZoneResul = nextZone;
    switch (nextZone) {
        case TOP:
            *TraDmd = (CurRZone[RZoneTrMax] + CurRZone[RZoneTrMin]) / 2.0f;
            *ElvDmd = CurRZone[RZoneElMax] + buffer;
            break;
        case LEFT_TOP_CORNER:
            *TraDmd = CurRZone[RZoneTrMin] - buffer;
            *ElvDmd = CurRZone[RZoneElMax] + buffer;
            break;
        case LEFT:
            *TraDmd = CurRZone[RZoneTrMin] - buffer;
            *ElvDmd = (CurRZone[RZoneElMax] + CurRZone[RZoneElMin]) / 2.0f;
            break;
        case LEFT_BOTTOM_CORNER:
            *TraDmd = CurRZone[RZoneTrMin] - buffer;
            *ElvDmd = CurRZone[RZoneElMin] - buffer;
            break;
        case BOTTOM:
            *TraDmd = (CurRZone[RZoneTrMax] + CurRZone[RZoneTrMin]) / 2.0f;
            *ElvDmd = CurRZone[RZoneElMin] - buffer;
            break;
        case RIGHT_TOP_CORNER:
            *TraDmd = CurRZone[RZoneTrMax] + buffer;
            *ElvDmd = CurRZone[RZoneElMax] + buffer;
            break;
        case RIGHT:
            *TraDmd = CurRZone[RZoneTrMax] + buffer;
            *ElvDmd = (CurRZone[RZoneElMax] + CurRZone[RZoneElMin]) / 2.0f;
            break;
        case RIGHT_BOTTOM_CORNER:
            *TraDmd = CurRZone[RZoneTrMax] + buffer;
            *ElvDmd = CurRZone[RZoneElMin] - buffer;
            break;
    }
}









// Function to determine the command zone based on command position and direction





static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port,
                                        const DimsInfo_T *dimsInfo)
{
    ssSetOutputPortMatrixDimensions(S, 0, inRow, inCol);
    
}


  
static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port,
                                        const DimsInfo_T *dimsInfo)
{
    if (!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
    
}

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);

    // === Input Ports ===
    ssSetNumInputPorts(S, 11);

    // Port 0: WZone [1x4]
    ssSetInputPortMatrixDimensions(S, 0, inRow, inCol);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_SINGLE);

    // Port 1: RZone [32x5]
    ssSetInputPortMatrixDimensions(S, 1, 1, 4);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDataType(S, 1, SS_SINGLE);

    // Ports 2–10: Scalar singles
    uint16_T i;
    for ( i = 2; i <= 10; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortDataType(S, i, SS_SINGLE);
    }

    // === Output Ports ===
    ssSetNumOutputPorts(S, 4);

    // Port 0: TraDmd [1x1]
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_SINGLE);

    // Port 1: ElvDmd [1x1]
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType(S, 1, SS_SINGLE);
    // Port 2: Debug [1x1]
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortDataType(S, 2, SS_SINGLE);
    // Port 3: Debug [1x1]
    ssSetOutputPortWidth(S, 3, 1);
    ssSetOutputPortDataType(S, 3, SS_SINGLE);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);  // 1 ms sample time
    ssSetOffsetTime(S, 0, 0.0);
}





/* Function: mdlOutputs */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // === Read inputs ===
    //int_T rowdim = ssGetInputPortDimensions(S, 0)[0];
    //int_T columdim = ssGetInputPortDimensions(S, 0)[1];
    
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    
    uint16_T i, j, idx;
    uint16_T NumRZones=0;
    real32_T RZones[inRow][inCol];

    for (i = 0; i < inRow; i++) {
        for (j = 0; j < inCol; j++) {
            idx = i + j * inRow;
            RZones[i][j] = *((const real32_T *)uPtrs[idx]);
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
    
   

    InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S, 1);
    real32_T WZones[5];
    for (i = 0; i < 5; i++) {
        WZones[i] = *((const real32_T *)uPtrs1[i]);
    }

    
    // Scalar ports
    // === Get scalar input pointers ===
    InputRealPtrsType uPtrs2  = ssGetInputPortRealSignalPtrs(S, 2);
    InputRealPtrsType uPtrs3  = ssGetInputPortRealSignalPtrs(S, 3);
    InputRealPtrsType uPtrs4  = ssGetInputPortRealSignalPtrs(S, 4);
    InputRealPtrsType uPtrs5  = ssGetInputPortRealSignalPtrs(S, 5);
    InputRealPtrsType uPtrs6  = ssGetInputPortRealSignalPtrs(S, 6);
    InputRealPtrsType uPtrs7  = ssGetInputPortRealSignalPtrs(S, 7);
    InputRealPtrsType uPtrs8  = ssGetInputPortRealSignalPtrs(S, 8);
    InputRealPtrsType uPtrs9  = ssGetInputPortRealSignalPtrs(S, 9);
    InputRealPtrsType uPtrs10 = ssGetInputPortRealSignalPtrs(S, 10);
    
    // === Dereference scalar inputs ===
    real32_T TraPos                = *((const real32_T *) uPtrs2[0]);
    real32_T ElvPos                = *((const real32_T *) uPtrs3[0]);
    real32_T TraCmd                = *((const real32_T *) uPtrs4[0]);
    real32_T ElvCmd                = *((const real32_T *) uPtrs5[0]);
    real32_T BufferDeg             = *((const real32_T *) uPtrs6[0]);
    real32_T WZoneTrvEn            = *((const real32_T *) uPtrs7[0]);
    real32_T WZoneElvEn            = *((const real32_T *) uPtrs8[0]);
    real32_T Axis1EndDampingBuffer = *((const real32_T *) uPtrs9[0]);
    real32_T Axis2EndDampingBuffer = *((const real32_T *) uPtrs10[0]);
    real32_T CurRZone[5] = {0};  

    real32_T manTraPos = 0.0f;
    real32_T manElvPos = 0.0f;
    manipulateTargetPos(
            &manTraPos,                     // Output Tra value
            &manElvPos,                     // Output Elv value
            TraCmd,                         // Current command
            ElvCmd,
            TraPos,                         // Current position
            ElvPos,
            WZones,                         // WZone[4] — already read from input port 0
            WZoneTrvEn,
            WZoneElvEn,
            Axis1EndDampingBuffer,
            Axis2EndDampingBuffer,
            RZones,                         // [32][5] matrix from input port 1
            NumRZones,                          // NumRZones = 32
            BufferDeg                       // Buffer (single)
        );

    TraCmd = manTraPos;
    ElvCmd = manElvPos;

    uint16_T direction = findDirection(TraCmd, TraPos, ElvCmd, ElvPos);

    uint16_T foundDeadEnd = findDeadEnd(direction,TraCmd,TraPos,WZones,WZoneTrvEn,RZones,     
                NumRZones,
                BufferDeg );

    direction =changeDirectionIfDeadEnd( direction, foundDeadEnd);

    uint16_T myZone = 0;              // output scalar

    findCurRZone(
            direction,         // current direction
            TraCmd,
            ElvCmd,
            TraPos,
            ElvPos,
            RZones,            // assume declared as real32_T RZones[32][5]
            NumRZones,
            foundDeadEnd,      // result of findDeadEnd(...)
            CurRZone,
            &myZone,           // pass address to receive value
            BufferDeg          // buffer
        );
    uint16_T cmdZone = findCmdZone(
            direction,
            TraCmd,
            ElvCmd,
            CurRZone,      // already declared as real32_T CurRZone[5];
            foundDeadEnd
        );
    real32_T TraDmdVal = 0.0;
    real32_T ElvDmdVal = 0.0;
    uint16_T nextZone ;
    manipulatePosCmds(
            TraCmd,
            ElvCmd,
            myZone,
            cmdZone,
            WZones,       // WZones[4]
            CurRZone,     // CurRZone[5]
            BufferDeg,
            &TraDmdVal,
            &ElvDmdVal,
            &nextZone
        );
















    // === Write outputs ===

    // Port 0: TraDmd
    real32_T *TraDmd = (real32_T *) ssGetOutputPortSignal(S, 0);

    // Port 1: ElvDmd
    real32_T *ElvDmd = (real32_T *) ssGetOutputPortSignal(S, 1);
    // Port 2: Debug 
    real32_T *debug = (real32_T *) ssGetOutputPortSignal(S, 2);
    // Port 3: Debug 
    real32_T *debug2 = (real32_T *) ssGetOutputPortSignal(S, 3);
    // === Simple example logic (replace with your real logic) ===
    *TraDmd = TraDmdVal;               // dummy computation
    *ElvDmd = ElvDmdVal;              // dummy computation
    *debug  = myZone;
    *debug2 = cmdZone ;
}




static void mdlTerminate(SimStruct *S)
{
    // No actions needed on termination
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
