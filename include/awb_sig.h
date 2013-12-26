/*
 *  Copyright 2007 by Texas Instruments Incorporated.
 *
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 */
#ifndef AWB_SIG_
#define AWB_SIG_

typedef struct AWB_SIG_Obj {
    IALG_Obj   alg;            /* MUST be first field of all XDAS algs */
    XDAS_Int32 numRanges;
    //IAE_Range exposureTimeRange[IAE_MAX_RANGES];
    //XDAS_UInt32 exposureTimeStepSize;
    //IAE_Range apertureLevelRange[IAE_MAX_RANGES];
    //IAE_Range sensorGainRange[IAE_MAX_RANGES];
    //IAE_Range ipipeGainRange[IAE_MAX_RANGES];
    //IAE_Range targetBrightnessRange;
    //XDAS_UInt32 targetBrightness;
    //XDAS_UInt32 thrld;
    //XDAS_UInt32 avgY;
    XDAS_UInt32 numHistory;
    XDAS_UInt32 numSmoothSteps;
    //XDAS_Int32 *historyBrightness;
    //XDAS_UInt32 curBrightness;
    //XDAS_Int32  curExposureTime;
    //XDAS_Int32  curApertureLevel;
    XDAS_Int32  curSensorGain;
    //XDAS_Int32  curIpipeGain;
    //XDAS_Bool   locked;
} AWB_SIG_Obj;

extern Int AWB_SIG_alloc(const IALG_Params *algParams, IALG_Fxns **pf,
                        IALG_MemRec memTab[]);

extern Int AWB_SIG_free(IALG_Handle handle, IALG_MemRec memTab[]);

extern Int AWB_SIG_init(IALG_Handle handle,
                       const IALG_MemRec memTab[], IALG_Handle parent,
                       const IALG_Params *algParams);

extern XDAS_Int32 AWB_SIG_process(IAWB_Handle h, IAWB_InArgs *inArgs, IAWB_OutArgs *outArgs,
                                 IAEWB_Rgb *rgbData, void *customData);

extern XDAS_Int32 AWB_SIG_control(IAWB_Handle handle,
                                 IAWB_Cmd id, IAWB_DynamicParams *params, IAWB_Status *status);


#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== AWB_SIG_VIDENCCOPY ========
 *  Our implementation of the IAWB interface
 */
extern IAWB_Fxns AWB_SIG_AWB;

#ifdef __cplusplus
}
#endif

#endif

