#define S_FUNCTION_NAME  checkzone
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
#define RZONE_EN     0
#define RZONE_TRMIN  1
#define RZONE_ELMIN  2
#define RZONE_TRMAX  3
#define RZONE_ELMAX  4

#define WZONE_TRSTART 0
#define WZONE_ELSTART 1
#define WZONE_TREND   2
#define WZONE_ELEND   3






/*====================*
 * S-function methods *
 *====================*/
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO



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

    ssSetNumInputPorts(S, 1);
    ssSetInputPortMatrixDimensions(S, 0, DYNAMICALLY_SIZED, DYNAMICALLY_SIZED);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_SINGLE);

    //ssSetInputPortWidth(S, 1, 4);
    //ssSetInputPortDirectFeedThrough(S, 1, 1);
    //ssSetInputPortDataType(S, 1, SS_SINGLE);

    ssSetNumOutputPorts(S, 2);
    ssSetOutputPortMatrixDimensions(S, 0, inRow, inCol);
    ssSetOutputPortDataType(S, 0, SS_SINGLE);
    
    ssSetOutputPortMatrixDimensions(S, 1, 1, 1);
    ssSetOutputPortDataType(S, 1, SS_INT16);

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
    int_T rowdim = ssGetInputPortDimensions(S, 0)[0];
    int_T columdim = ssGetInputPortDimensions(S, 0)[1];
    real32_T res_count = 0; 

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    uint16_T i, j, idx;
    real32_T RZones[inRow][inCol];

    for (i = 0; i < rowdim; i++) {
        for (j = 0; j < columdim; j++) {
            idx = i + j * rowdim;
            RZones[i][j] = *((const real32_T *)uPtrs[idx]);
        }
    }
    
   

    //InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S, 1);
    //real32_T WZones[5];
    //for (i = 0; i < 5; i++) {
    //    WZones[i] = *((const real32_T *)uPtrs2[i]);
    //}

    real32_T parsedRZones[inRow][inCol];  // output matrix with extra rows

     // Step 1: copy original zones into top half
    for ( i = 0; i < rowdim; i++) {
        for ( j = 0; j < 5; j++) {
            parsedRZones[i][j] = RZones[i][j];
        }
    }
     for (i = rowdim; i < inRow; i++) {
        for (j = 0; j < inCol; j++) {
            parsedRZones[i][j] = 0;
        }
    }
    real32_T trMin,trMax;
    uint16_T counter=0;
    // Step 2: logic to split if trMin > trMax
    
    for ( i = 0; i < rowdim; i++) {
        if (RZones[i][RZONE_EN] == 1.0f) {
            trMin = RZones[i][RZONE_TRMIN];
            trMax = RZones[i][RZONE_TRMAX];
            res_count +=1;
    
            if (trMin > trMax) {
                // First part: override trMax to 180
                parsedRZones[i][RZONE_TRMAX] = 180.0f;
    
                // Second part: add a new row in bottom half
                uint16_T newIdx = rowdim + counter;
                parsedRZones[newIdx][RZONE_EN]    = 1.0f;
                parsedRZones[newIdx][RZONE_TRMIN] = -180.0f;
                parsedRZones[newIdx][RZONE_TRMAX] = trMax;
                parsedRZones[newIdx][RZONE_ELMIN] = RZones[i][RZONE_ELMIN];
                parsedRZones[newIdx][RZONE_ELMAX] = RZones[i][RZONE_ELMAX];
                counter ++;
            }
        }
    }

    // Step 3: if Zone not define but back
    int writtenIdx = 0;
    for ( i = 0; i < inRow; i++) {
            if (parsedRZones[i][RZONE_EN] == 1.0f) {
            
            RZones[writtenIdx][RZONE_EN]    = 1.0f;
            RZones[writtenIdx][RZONE_TRMIN] = parsedRZones[i][RZONE_TRMIN];
            RZones[writtenIdx][RZONE_TRMAX] = parsedRZones[i][RZONE_TRMAX];
            RZones[writtenIdx][RZONE_ELMIN] = parsedRZones[i][RZONE_TRMAX];
            RZones[writtenIdx][RZONE_ELMAX] = parsedRZones[i][RZONE_TRMAX];
            writtenIdx++;
            }
    }
     for (i = writtenIdx; i < inRow; i++) {
        for (j = 0; j < inCol; j++) {
            RZones[i][j] = 0;
        }
    }
    
   
    //output

    
    real32_T *y = (real32_T *) ssGetOutputPortSignal(S, 0);

    
    int16_T *y_count = (int16_T*) ssGetOutputPortSignal(S, 1);
    *y_count = (int16_T)res_count;   /* or whatever count you want */

    for (i = 0; i < inRow; i++) {
        for (j = 0; j < inCol; j++) {
            idx = i + j * inRow;
            y[idx] = parsedRZones[i][j];
        }
    }
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
