/*
 *  ======== ae_sig.h ========
 */
#ifndef AF_SIG_PRIV_
#define AF_SIG_PRIV_

#define RY    0x4d
#define GY    0x96
#define BY    0x1d

typedef struct AF_SIG_Obj {
    IALG_Obj   alg;            /* MUST be first field of all XDAS algs */
    XDAS_Int32 numRanges;
    IAE_Range exposureTimeRange[IAE_MAX_RANGES];
    XDAS_UInt32 exposureTimeStepSize;
    IAE_Range apertureLevelRange[IAE_MAX_RANGES];
    IAE_Range sensorGainRange[IAE_MAX_RANGES];
    IAE_Range ipipeGainRange[IAE_MAX_RANGES];
    IAE_Range targetBrightnessRange;
    XDAS_UInt32 targetBrightness;
    XDAS_UInt32 thrld;
    XDAS_UInt32 avgY;
    XDAS_UInt32 numHistory;
    XDAS_UInt32 numSmoothSteps;
    XDAS_Int32 *historyBrightness;
    XDAS_UInt32 curBrightness;
    XDAS_Int32  curExposureTime;
    XDAS_Int32  curApertureLevel;
    XDAS_Int32  curSensorGain;
    XDAS_Int32  curIpipeGain;
    XDAS_Bool   locked;
} AF_SIG_Obj;

extern Int AF_SIG_alloc(const IALG_Params *algParams, IALG_Fxns **pf,
                        IALG_MemRec memTab[]);

extern Int AF_SIG_free(IALG_Handle handle, IALG_MemRec memTab[]);

extern Int AF_SIG_init(IALG_Handle handle,
                       const IALG_MemRec memTab[], IALG_Handle parent,
                       const IALG_Params *algParams);

extern XDAS_Int32 AF_SIG_process(IAE_Handle h, IAE_InArgs *inArgs, IAE_OutArgs *outArgs,
                                 IAEWB_Rgb *rgbData, XDAS_UInt8 *weight, int *customData);

extern XDAS_Int32 AF_SIG_control(IAE_Handle handle,
                                 IAE_Cmd id, IAE_DynamicParams *params, IAE_Status *status);

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== AE_TI_VIDENCCOPY ========
 *  Our implementation of the IAE interface
 */
extern IAE_Fxns AF_SIG_AE;

#ifdef __cplusplus
}
#endif

#endif

