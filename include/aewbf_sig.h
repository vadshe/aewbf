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

typedef struct ALG_AewbfObj {
    IAEWBF_InArgs   InArgs;
    IAEWBF_OutArgs  OutArgs;

    IALG_Handle   handle_aewbf;
    IALG_MemRec   memTab_aewbf[4];

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

typedef struct Hist_Param{      //Histogram parameters
    XDAS_UInt32 hist[ALG_SENSOR_BITS];
    XDAS_UInt32 min, max;   //Min and Max of histogram
    XDAS_UInt32 mins, maxs; //Saturation of Min amd Max
    XDAS_UInt32 MaxTh;      //Maximum threshould
} Hist_Param;

typedef struct IAEWBF_SIG_Obj {
    IALG_Obj   alg;            /* MUST be first field of all XDAS algs */
    XDAS_Int32 numRanges;
    //XDAS_UInt32 expStep;
    //IAEWBF_Range expRange[IAES_MAX_RANGES];
    //IAEWBF_Range sensorGainRange[IAES_MAX_RANGES];
    //IAEWBF_Range ipipeGainRange[IAES_MAX_RANGES];
    //IAEWBF_Range isifGainRange[IAES_MAX_RANGES];
    //IAEWBF_Range YRange;
    //XDAS_UInt32 maxDiffY;
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

    //Dinamyc parameters
    XDAS_Int32 expUp, expDown;      //Change Exposure
    XDAS_Int32 wbR, wbB;            //Change white balance

    XDAS_UInt32 w, h;               //The width and height of BoxCar image
    //XDAS_UInt32 R, G, B;            //Avarage of R, G, B color
    XDAS_UInt32 GR[3], GB[3];             //Avarage of G-R and G-B
    XDAS_UInt32 hsz;                //Size on BoxCar Histogram
    XDAS_UInt32 lut[512];           //Gamma correction table
    XDAS_UInt16 *box;               //Pointer to BoxCar image
    XDAS_Int32 RGBgain[3];          //R and G and B ISIF gains for white balansing
    //XDAS_UInt32 maxi;               //Maximum gain 0 - R ,1 - G, 2 - B
    IAEWBF_Param Exp;               //Exposure parameters
    IAEWBF_Param Offset;            //Offset
    IAEWBF_Param GISIF;             //ISIF gain
    IAEWBF_Param GIFIF;             //IFIF gain
    IAEWBF_Param Grgb2rgb;          //Grgb2rgb gain
    IAEWBF_Param Y;
    IAEWBF_Param Hunif;             //Histogram uniformity
    IAEWBF_Param Hmax;              //maximum of histogram
    Hist_Param  RGB[3];             //Histogram of BoxCar image
    XDAS_UInt32 Hmin[2];            //min[0] minimum of histogram, min[1] the number of pixels below min[0]
    XDAS_UInt32 Hhalf;              //The median value of histogram
    XDAS_UInt32 HmaxTh;             //The max of histogram threshold
    XDAS_UInt32 HminTh;             //The min of histogram threshold
    XDAS_UInt32 HhalfTh;            //The Half of Histogram threshold
    XDAS_UInt32 SatTh;              //Saturation Threshold for histogram max and min

    //Dynamic parametar
    int gAePriorityMode, gBWMode, gDayNight, gIRCut, defaultFPS;
    int IRcutClose; //IR-cut 1-close, 0 - open
    int FPShigh; //FPS 1-high, 0 - low
} IAEWBF_SIG_Obj;

extern Int IAEWBF_SIG_alloc(const IALG_Params *algParams, IALG_Fxns **pf, IALG_MemRec memTab[]);

extern Int IAEWBF_SIG_free(IALG_Handle handle, IALG_MemRec memTab[]);

extern Int IAEWBF_SIG_init(IALG_Handle handle, const IALG_MemRec memTab[], IALG_Handle parent, const IALG_Params *algParams);

extern XDAS_Int32 IAEWBF_SIG_process(IAEWBF_Handle handle, IAEWBF_InArgs *inArgs, IAEWBF_OutArgs *outArgs);

extern XDAS_Int32 IAEWBF_SIG_control(IAEWBF_Handle handle, IAEWBF_Cmd id, IAEWBF_DynamicParams *params, IAEWBF_Status *status);

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

