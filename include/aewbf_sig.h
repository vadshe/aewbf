/*
 *
 *  ======== ae_sig.h ========
 */
#include "iaewbf_sig.h"

#ifndef AE_SIG_PRIV_
#define AE_SIG_PRIV_

#define RY    0x4d
#define GY    0x96
#define BY    0x1d
#define ALG_SENSOR_BITS (1<<9)
#define HISTORY 8

typedef struct ALG_AewbfObj {
    IAEWBF_InArgs   InArgs;
    IAEWBF_OutArgs  OutArgs;

    IALG_Handle   handle_aewbf;
    IALG_MemRec   memTab_aewbf[2];

    unsigned char *weight;

    int vsEnable;
    int vnfDemoCfg;
    int aewbType;
    int aewbVendor;
    int aewbPriority;
    int reduceShutter;
    int saldre;
    int flgAWBCalcDataUpdate;
    int ALTM;
    int AGainEnable;
    int DGainEnable;
    int sensorFps;
    int numEncodes;
    int sensorMode;
    int afEnable;

} ALG_AewbfObj;

typedef struct IAEWBF_Param{
    XDAS_Int32 Old;
    XDAS_Int32 New;
    XDAS_Int32 Step;
    XDAS_Int32 Min;
    XDAS_Int32 Max;
    XDAS_Int32 Sat;
    XDAS_Int32 Diff;
    XDAS_Int32 Th;
    XDAS_Int32 Avrg;
    XDAS_Int32 Avrs;
    XDAS_Int32 Change;
    XDAS_Int32 Hist[HISTORY];
    XDAS_Int32 HistC;
    IAEWBF_Range Range;
}IAEWBF_Param;


typedef struct IAEWBF_SIG_Obj {
    IALG_Obj   alg;            /* MUST be first field of all XDAS algs */

    XDAS_UInt32 w, h;               //The width and height of BoxCar image
    XDAS_UInt32 hsz;                //Size on BoxCar Histogram
    XDAS_UInt16 *box;               //Pointer to BoxCar image
    IAEWBF_Param Exp;               //Exposure parameters
    IAEWBF_Param Offset;            //Offset
    IAEWBF_Param GIFIF;             //IFIF gain
    IAEWBF_Param Grgb2rgb;          //Grgb2rgb gain
    IAEWBF_Param Y;                 //The average Y
    IAEWBF_Param Rgain;             //The ISIF gain for R color
    IAEWBF_Param Bgain;             //The ISIF gain for B color
    IAEWBF_Param Hmin;              //The minimum of histogram
    IAEWBF_Param Hmax;              //The maximum of histogram
    XDAS_UInt32 HmaxTh;             //The max of histogram threshold
    XDAS_UInt32 SatTh;              //Saturation Threshold for histogram max and min
    XDAS_UInt32 FPSmax;             //Maximum FPS
    XDAS_UInt32 FPScur;             //Current FPS

    //Dynamic parametar
    int gAePriorityMode, gBWMode, gDayNight, gIRCut, defaultFPS, gFlicker;
    int IRcutClose; //IR-cut 1-close, 0 - open
    int FPShigh; //FPS 1-high, 0 - low
} IAEWBF_SIG_Obj;

extern Int IAEWBF_SIG_alloc(const IALG_Params *algParams, IALG_Fxns **pf, IALG_MemRec memTab[]);

extern Int IAEWBF_SIG_free(IALG_Handle handle, IALG_MemRec memTab[]);

extern Int IAEWBF_SIG_init(IALG_Handle handle, const IALG_MemRec memTab[], IALG_Handle parent, const IALG_Params *algParams);

extern XDAS_Int32 IAEWBF_SIG_process(IAEWBF_Handle handle, IAEWBF_InArgs *inArgs, IAEWBF_OutArgs *outArgs);

extern XDAS_Int32 IAEWBF_SIG_control(IAEWBF_Handle handle, IAEWBF_Cmd id, IAEWBF_DynamicParams *params, IAEWBF_Status *status);

int Get_BoxCar(IALG_Handle handle);
void ALG_SIG_config(IALG_Handle handle);
void SIG2A_applySettings(void);
int SIG_2A_config(IALG_Handle handle);
#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== AE_TI_VIDENCCOPY ========
 *  Our implementation of the IAE interface
 */
extern IAEWBF_Fxns IAEWBF_SIG;

#ifdef __cplusplus
}
#endif

#endif

