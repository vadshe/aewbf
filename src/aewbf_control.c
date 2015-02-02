#include "alg_aewb_priv.h"
#include <drv_gpio.h>
#include <drv_ipipe.h>
#include <drv_isif.h>

#include "aewbf_sig.h"
#include "iaewbf_sig.h"

#include <sc_env.h> //scam

#define __DEBUG
#ifdef __DEBUG
#define AE_DEBUG_PRINTS
#define dprintf printf
#else
#define dprintf
#endif

////////////////////////////////

#define SRESHOLD_DIFF 120
#define GET_SHUTTER_STEP( flickerMode )  ( (flickerMode) == SCFdDisabled ? 1 : ( (flickerMode) == SCFd60hz ? 8333 : 10000 ) )

////////////////////////////////

ALG_AewbObj		gALG_aewbObj;
ALG_AewbfObj	gSIG_Obj;

extern DRV_IpipeObj gDRV_ipipeObj;    //For boxcar
extern CSL_IpipeObj gCSL_ipipeHndl;   //For gamma and rgb2rgb

extern int OSA_fileWriteFile(const char *fileName, const void *addr, size_t size);

extern int gAePriorityMode, gBWMode, gIRCut, defaultFPS;
extern int IRcutClose, FPShigh;
extern int Threshold_IR_cut_open;
//extern int Threshold_low_fps, Threshold_IR_cut;
//extern int gHDR;
extern Uint32  gamma01[], gamma005[], gamma0005[], gamma001[], gamma002[], gamma003[], gamma004[], SIG_YEE_TABLE[], gamma_ti1[], gamma_ti2[];
extern Uint32 gam03[], gam04[], gam05[];
extern Int32 leave_frames;
extern int  frame_count;

int DEBUG = 1;


static void smooth_change( IAEWBF_Param *par, int fr)
{
    if(!fr) {
        par->Change = (par->New - par->Old)/leave_frames;
        par->Old += par->Change;
    } else if (fr == (leave_frames-1)){
        par->Old = par->New;
    } else {
        par->Old += par->Change;
    }
}

int Get_BoxCar(IALG_Handle handle)
{
    int status;
    int bufId=-1;
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    OSA_BufInfo *pBufInfo;

    status = DRV_ipipeGetBoxcarBuf(&bufId, 1<<30); //OSA_TIMEOUT_NONE
    if(status!= OSA_SOK) {
        OSA_ERROR("ERROR: DRV_ipipeGetBoxcarBuf()\n");
        return status;
    }
    pBufInfo = DRV_ipipeGetBoxcarBufInfo(bufId);
    DRV_ipipePutBoxcarBuf(bufId);

    //Fill AEWBF filds
    hn->box = pBufInfo->virtAddr;
    hn->w = gDRV_ipipeObj.boxcarInfo.width;
    hn->h = gDRV_ipipeObj.boxcarInfo.height;

    return OSA_SOK;
}
/*
void setup_color_matrix_sensor(CSL_IpipeRgb2RgbConfig *rgb2rgb2, const char *sensor)
{
    if (strcmp(sensor, "MICRON_MT9M034_720P") == 0)
    {
        rgb2rgb2->matrix[0][0] = 427;
        rgb2rgb2->matrix[0][1] = -105;
        rgb2rgb2->matrix[0][2] = -66;

        rgb2rgb2->matrix[1][0] = -99;
        rgb2rgb2->matrix[1][1] = 422;
        rgb2rgb2->matrix[1][2] = -67;

        rgb2rgb2->matrix[2][0] = -8;
        rgb2rgb2->matrix[2][1] = -78;
        rgb2rgb2->matrix[2][2] = 342;
    }
    else if (strcmp(sensor, "MICRON_AR0331_1080P") == 0)
    {
        rgb2rgb2->matrix[0][0] = 380;
        rgb2rgb2->matrix[0][1] = -59;
        rgb2rgb2->matrix[0][2] = -66;

        rgb2rgb2->matrix[1][0] = -89;
        rgb2rgb2->matrix[1][1] = 402;
        rgb2rgb2->matrix[1][2] = -57;

        rgb2rgb2->matrix[2][0] = -8;
        rgb2rgb2->matrix[2][1] = -98;
        rgb2rgb2->matrix[2][2] = 362;
    }
    else if (strcmp(sensor, "MICRON_MT9P031_5MP") == 0)
    {
        rgb2rgb2->matrix[0][0] = 380;
        rgb2rgb2->matrix[0][1] = -59;
        rgb2rgb2->matrix[0][2] = -66;

        rgb2rgb2->matrix[1][0] = -89;
        rgb2rgb2->matrix[1][1] = 402;
        rgb2rgb2->matrix[1][2] = -57;

        rgb2rgb2->matrix[2][0] = -8;
        rgb2rgb2->matrix[2][1] = -168;
        rgb2rgb2->matrix[2][2] = 432;
    }
    else if(strcmp(sensor, "SONY_IMX136_3MP") == 0)
    {

        rgb2rgb2->matrix[0][0] = 360;
        rgb2rgb2->matrix[0][1] = -153;
        rgb2rgb2->matrix[0][2] = 49;

        rgb2rgb2->matrix[1][0] = -92;
        rgb2rgb2->matrix[1][1] = 312;
        rgb2rgb2->matrix[1][2] = 36;

        rgb2rgb2->matrix[2][0] = 37;
        rgb2rgb2->matrix[2][1] = -338;
        rgb2rgb2->matrix[2][2] = 557;

    }
    rgb2rgb2->offset[0]    = 0;
    rgb2rgb2->offset[1]    = 0;
    rgb2rgb2->offset[2]    = 0;

    if(DRV_ipipeSetRgb2Rgb2(rgb2rgb2) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
}

void setup_color_matrix_value(CSL_IpipeRgb2RgbConfig *rgb2rgb2, int value)
{
    rgb2rgb2->matrix[0][0] = value;
    rgb2rgb2->matrix[0][1] = 0;
    rgb2rgb2->matrix[0][2] = 0;

    rgb2rgb2->matrix[1][0] = 0;
    rgb2rgb2->matrix[1][1] = value;
    rgb2rgb2->matrix[1][2] = 0;

    rgb2rgb2->matrix[2][0] = 0;
    rgb2rgb2->matrix[2][1] = 0;
    rgb2rgb2->matrix[2][2] = value;

    rgb2rgb2->offset[0]    = 0;
    rgb2rgb2->offset[1]    = 0;
    rgb2rgb2->offset[2]    = 0;

    if(DRV_ipipeSetRgb2Rgb2(rgb2rgb2) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
}
*/

#if 1 // unused
static void print_debug(int frames, int leave_frames, IAEWBF_SIG_Obj *hn){
    int i = 0, fr = frames%leave_frames, all = frames%150;
    //all = fr;
    //all = 0; fr = 0;

    if(DEBUG && (!fr)){
        printf("Threshold = %i Threshold_FPS_up = %i Threshold_IR_cut_close = %i FPShigh = %i IRcutClose = %i defaultFPS = %i\n",
               hn->Threshold_IR_cut[0], hn->Threshold_IR_cut[1], hn->Threshold_IR_cut[2], FPShigh, IRcutClose, defaultFPS);
        i++;

        if (hn->Exp.New != hn->Exp.Old || !all){
            printf("%6d   EXP          : Exp.New  = %6ld Exp.Old = %6ld Exp.Max = %6ld\n", frames, hn->Exp.New, hn->Exp.Old, hn->Exp.Range.max);
            i++;
        }

        if (hn->Offset.New != hn->Offset.Old || !all){
            printf("%6d   OFFSET       : Off.New = %4ld Off.Old = %4ld \n", frames, hn->Offset.New, hn->Offset.Old);
            i++;
        }
        if (hn->Rgain.New != hn->Rgain.Old || hn->Bgain.New != hn->Bgain.Old || !all){
            printf("%6d   GAIN WB      : Rgain.NewA = %4ld Rgain.New = %4ld Rgain.Old = %4ld Bgain.NewA = %4ld Bgain.New = %4ld Bgain.Old = %4ld \n",
                    frames, hn->Rgain.NewA, hn->Rgain.New, hn->Rgain.Old, hn->Bgain.NewA, hn->Bgain.New, hn->Bgain.Old);
            i++;
        }
        if (hn->GIFIF.New !=  hn->GIFIF.Old || !all) {
            printf("%6d   GAIN IFIF    : GIFIF.New = %4ld GIFIF.Old = %4ld \n", frames, hn->GIFIF.New, hn->GIFIF.Old);
            i++;
        }

        if (i) printf("Y.NewA = %4ld Y.New = %4ld Y.Old = %4ld Hmin = %4ld Hmax = %4ld HminA = %4ld HmaxA = %4ld\n",
               hn->Y.NewA, hn->Y.New, hn->Y.Old, hn->Hmin.New, hn->Hmax.New, hn->Hmin.NewA, hn->Hmax.NewA);

    }
}
#else
static void print_debug(int frames, int leave_frames, IAEWBF_SIG_Obj *hn)
{
    (void)frames; (void)leave_frames; (void)hn;
}
#endif

void check_range(IAEWBF_Param *val)
{
    val->New = val->New > val->Range.max ? val->Range.max : val->New;
    val->New = val->New < val->Range.min ? val->Range.min : val->New;
}

int SIG_2A_config(IALG_Handle handle)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    //int i, stepSize, sensorExposureMax;
    char zoomvalue[4];

    const scImgParams_t* pScParams = &(Aew_ext_parameter.scImgParams[SCCamModeDay]); //scam

    //Zoom in 0 position
    sprintf(zoomvalue, "%04d", 0);
    if(OSA_fileWriteFile("/var/run/zoom", zoomvalue, sizeof(zoomvalue)) !=OSA_SOK) {
        OSA_printf("AF: error write in file\n");
    }


    //sensorExposureMax = 1000000 / 30; //scam
    //stepSize = 1;

    //Exposure setup
    hn->Exp.Step = 1;
    hn->Exp.Old = 1000000/defaultFPS-1;
    hn->Exp.New = 1000000/defaultFPS;
    hn->Exp.Range.max = pScParams->shutterScope.max;
    hn->Exp.Range.min = pScParams->shutterScope.min;
    DRV_imgsSetEshutter(hn->Exp.New, 0);
    DRV_imgsSetFramerate(defaultFPS);

    //ISIF R gain setup
    hn->Rgain.Old = 511;
    hn->Rgain.New = 980;
    hn->Rgain.Range.min = pScParams->RgainScope.min; //50;
    hn->Rgain.Range.max = pScParams->RgainScope.max; //4095;
    //ISIF B gain setup
    hn->Bgain.Old = 511;
    hn->Bgain.New = 860;
    hn->Bgain.Range.min = pScParams->BgainScope.min; //50;
    hn->Bgain.Range.max = pScParams->BgainScope.max; //50;
    //Setup isif white balance gain

    //Setup ipipe global gains and offset
    DRV_IpipeWb ipipeWb;
    hn->GIFIF.Step = 1;
    hn->GIFIF.Old = 511;
    hn->GIFIF.New = 512;
    hn->GIFIF.Range.min = pScParams->gainScope.min; //512;
    hn->GIFIF.Range.max = pScParams->gainScope.max; //8191;

    //ISIF offset setup
    hn->Offset.Step = 1;
    hn->Offset.Old = 1;
    hn->Offset.New = 0;
    hn->Offset.Range.min = 1;
    hn->Offset.Range.max = 4095;

    //Y setup
    hn->Y.Step = 1;
    hn->Y.New = 1;
    hn->Y.Old = 1;
    hn->Y.Range.min = 0;
    hn->Y.Range.max = 4095;
    hn->Y.HistC = 0;
    hn->YAE = 800; //800
    hn->SatTh = hn->w*hn->h*3/100;

    //Setup IR-cut and BW mode
    //hn->gIRCut			= gIRCut;
    //hn->gBWMode			= gBWMode;
    hn->gIRCut			= gIRCut; //pScParams->ircutOpen;
    hn->gBWMode			= gBWMode; //pScParams->bwMode;
    hn->IRcutClose		= IRcutClose;   //Signal for close or open IR-cut
    DRV_imgsNDShutter(gIRCut, gBWMode);

    hn->Threshold_IR_cut[0] = 30; //Threshold_IR_cut_open;
    hn->Threshold_IR_cut[1] = 30; //Threshold_IR_cut_open;

    //Get parameters form Boxcar
    BRT_CRT_PARAM brtCtrParam;
    hn->w = gALG_aewbObj.IAEWB_StatMatdata.winCtHorz;
    hn->h = gALG_aewbObj.IAEWB_StatMatdata.winCtVert;
    hn->pix = gALG_aewbObj.IAEWB_StatMatdata.pixCtWin;
    hn->SatTh = hn->w*hn->h*3/100;

    //Default brightness setup
    brtCtrParam.yuv_adj_ctr = Aew_ext_parameter.contrast >> 3;
    brtCtrParam.yuv_adj_brt = Aew_ext_parameter.brightness;

    //Global offset for Sony IMX136
    DRV_isifSetDcSub(-ZERO);

    //Setup default gamma correction tables
    CSL_IpipeGammaConfig dataG;
    dataG.tableSize = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SIZE_512;
    dataG.tableSrc  = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SELECT_RAM;
    dataG.bypassR = 0;
    dataG.bypassG = dataG.bypassR;
    dataG.bypassB = dataG.bypassR;
    dataG.tableR = gamma002;
    dataG.tableG = dataG.tableR;
    dataG.tableB = dataG.tableR;
    if(CSL_ipipeSetGammaConfig(&gCSL_ipipeHndl, &dataG) != CSL_SOK)
        OSA_ERROR("Fail CSL_ipipeSetGammaConfig!!!\n");

    CSL_IpipeRgb2RgbConfig rgb2rgb1, rgb2rgb2;
    //Setup first RGB2RGB matrix
    rgb2rgb1.matrix[0][0] = 256;
    rgb2rgb1.matrix[0][1] = 0;
    rgb2rgb1.matrix[0][2] = 0;

    rgb2rgb1.matrix[1][0] = 0;
    rgb2rgb1.matrix[1][1] = 256;
    rgb2rgb1.matrix[1][2] = 0;

    rgb2rgb1.matrix[2][0] = 0;
    rgb2rgb1.matrix[2][1] = 0;
    rgb2rgb1.matrix[2][2] = 256;

    rgb2rgb1.offset[0]    = 0;
    rgb2rgb1.offset[1]    = 0;
    rgb2rgb1.offset[2]    = 0;
    if(DRV_ipipeSetRgb2Rgb(&rgb2rgb1) != CSL_SOK)
         OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");

    //Setup second RGB2RGB matrix for SONY IMX136
    rgb2rgb2.matrix[0][0] = 360;
    rgb2rgb2.matrix[0][1] = -153;
    rgb2rgb2.matrix[0][2] = 49;

    rgb2rgb2.matrix[1][0] = -92;
    rgb2rgb2.matrix[1][1] = 312;
    rgb2rgb2.matrix[1][2] = 36;

    rgb2rgb2.matrix[2][0] = 37;
    rgb2rgb2.matrix[2][1] = -338;
    rgb2rgb2.matrix[2][2] = 557;

    rgb2rgb2.offset[0]    = 0;
    rgb2rgb2.offset[1]    = 0;
    rgb2rgb2.offset[2]    = 0;
    if(DRV_ipipeSetRgb2Rgb2(&rgb2rgb2) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");



    //hn->Exp.Step = stepSize;
    //hn->Exp.Old = (sensorExposureMax/stepSize)*stepSize;
    //hn->Exp.New = hn->Exp.Old;
    //hn->Exp.Range.max = sensorExposureMax;
    //hn->Exp.Range.min = stepSize;
    /*
    for(i=0; i < HISTORY; i++) hn->Y.Hist[i] = 0;
    hn->Y.Avrg = 0;

    hn->Hmax.HistC = 0;
    for(i=0; i < HISTORY; i++) hn->Hmax.Hist[i] = 0;
    hn->Hmax.Avrg = 0;

    hn->Hmin.HistC = 0;
    for(i=0; i < HISTORY; i++) hn->Hmin.Hist[i] = 0;
    hn->Hmin.Avrg = 0;
    */
    //hn->HmaxTh = 3500;
    //hn->FPSmax			= defaultFPS;
    //hn->FPScur			= defaultFPS;

    //size_t i;
    //Uint32 hsz = ALG_SENSOR_BITS;
    //if(defaultFPS != DRV_imgsGetFramerate()) {
    //    defaultFPS = DRV_imgsGetFramerate();
    //    hn->FPScur = defaultFPS;
    //    hn->FPSmax = defaultFPS;
    //

   // setup_color_matrix_sensor(&rgb2rgb2, DRV_imgsGetImagerName());

   //SCAM
   // if(DRV_ipipeSetRgb2Rgb(&rgb2rgb1) != CSL_SOK)
   //     OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");



    // Save prev values
    //memcpy(&(hn->scImgParams), pScParams, sizeof(scImgParams_t));
    //hn->scIsAutoCamMode = Aew_ext_parameter.scIsAutoCamMode;

    return 0;
}

////////////////////////////////
//SCAM
static short MUL12Q8( short a, short b )
{
    const int Q = 8;
    signed long int tmp = (((long int)a * (long int)b) +(1 << (Q-1)));
    return (tmp >> Q) & 0xFFF;
}

void SCSetSaturation( unsigned char saturation )
{
    CSL_IpipeRgb2RgbConfig rgb2rgb;
    short sat = saturation << 1;

    rgb2rgb.matrix[0][0] = 77 + MUL12Q8(179 , sat);
    rgb2rgb.matrix[0][1] = MUL12Q8(150 , (256 - sat));
    rgb2rgb.matrix[0][2] = MUL12Q8(29  , (256 - sat));

    rgb2rgb.matrix[1][0] = MUL12Q8(77  , (256 - sat));
    rgb2rgb.matrix[1][1] = 150 + MUL12Q8(106 , sat);
    rgb2rgb.matrix[1][2] = MUL12Q8(29  , (256 - sat));

    rgb2rgb.matrix[2][0] = MUL12Q8(77  , (256 - sat));
    rgb2rgb.matrix[2][1] = MUL12Q8(150 , (256 - sat));
    rgb2rgb.matrix[2][2] = 29  + MUL12Q8(227 , sat);

    rgb2rgb.offset[0]    = 0;
    rgb2rgb.offset[1]    = 0;
    rgb2rgb.offset[2]    = 0;

    if(DRV_ipipeSetRgb2Rgb(&rgb2rgb) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
}

void SCSetSharpness( unsigned char sharpness )
{
    CSL_IpipeEdgeEnhanceConfig config;

    config.enable                   = TRUE;
    config.haloReduceEnable         = FALSE;
    config.mergeMethod              = CSL_IPIPE_YEE_EE_ES_MERGING_METHOD_SUMMATION;
    config.hpfShift                 = 0;//shift_val;
    config.hpfCoeff[0][0]           = 8;// 48;
    config.hpfCoeff[0][1]           = 2;//12;
    config.hpfCoeff[0][2]           = -2;//1024-10;
    config.hpfCoeff[1][0]           = 2;//12;
    config.hpfCoeff[1][1]           = 0;
    config.hpfCoeff[1][2]           = -1;//1024-6;
    config.hpfCoeff[2][0]           = -2;//1024-10;
    config.hpfCoeff[2][1]           = -1;//1024-6;
    config.hpfCoeff[2][2]           = 0;//1024-2;
    config.lowerThres               = 5;//32;//8; //changed based on Gang Comment
    config.edgeSharpGain            = sharpness;
    config.edgeSharpHpValLowThres   = ((int)48) & 0x3F;//128;
    config.edgeSharpHpValUpLimit    = ((int)-30) & 0x3F;//768;
    config.edgeSharpGainGradient    = 32;
    config.edgeSharpOffsetGradient  = 0;
    config.table                    = SIG_YEE_TABLE;//g_scYeeTables;

    DRV_ipipeSetEdgeEnhance(&config);
}

void copy_parameters(scImgParams_t* pScParams, IAEWBF_SIG_Obj *hn)
{
    pScParams->shutterScope.min	=	1;
    pScParams->shutterScope.max	=	1000000/defaultFPS;
    pScParams->gainScope.min		=	512;
    pScParams->gainScope.max		=	8191;
    pScParams->RgainScope.min      =   50;
    pScParams->RgainScope.max      =   4095;
    pScParams->BgainScope.min      =   50;
    pScParams->BgainScope.max      =   4095;


    hn->GIFIF.Range.min = pScParams->gainScope.min;
    hn->GIFIF.Range.max = pScParams->gainScope.max;
    hn->Rgain.Range.min = pScParams->RgainScope.min;
    hn->Rgain.Range.max = pScParams->RgainScope.max;
    hn->Bgain.Range.min = pScParams->BgainScope.min;
    hn->Bgain.Range.max = pScParams->BgainScope.max;
    hn->Exp.Range.min =  pScParams->shutterScope.min;
    hn->Exp.Range.max =  pScParams->shutterScope.max;

    check_range(&hn->Offset);
    check_range(&hn->GIFIF);
    check_range(&hn->Rgain);
    check_range(&hn->Bgain);
    check_range(&hn->Exp);
}


void SC2A_applySettings(void)
{
    static int sFrames = 0;
	IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)gSIG_Obj.handle_aewbf;
	DRV_IpipeWb ipipeWb;
    CSL_IpipeRgb2RgbConfig rgb2rgb, rgb2rgb2;
    scImgParams_t* pScParams;

  
	int fr = sFrames % leave_frames;
    //int shutterValToSet = -1;


    Aew_ext_parameter.scCurrentCamMode = SCCamModeNight;

    //Change day/night auto
    if( Aew_ext_parameter.scIsAutoCamMode && Aew_ext_parameter.scCurrentCamMode != SCCamModePreview)
    {
        if( FPShigh == 0 )  Aew_ext_parameter.scCurrentCamMode = SCCamModeNight;
        else                Aew_ext_parameter.scCurrentCamMode = SCCamModeDay;
        pScParams = &(Aew_ext_parameter.scImgParams[Aew_ext_parameter.scCurrentCamMode]);
        copy_parameters(pScParams, hn);
    }

    pScParams = &(Aew_ext_parameter.scImgParams[Aew_ext_parameter.scCurrentCamMode]);
    copy_parameters(pScParams, hn);

    print_debug( sFrames, leave_frames, hn );

    //Change offset
    if( hn->Offset.New != hn->Offset.Old ) {
        smooth_change(&hn->Offset, fr);
        DRV_ipipeSetWbOffset(-hn->Offset.Old);
    }

    //Change gain
    if( hn->GIFIF.New !=  hn->GIFIF.Old ) {
        smooth_change(&hn->GIFIF, fr);
        ipipeWb.gainR  = hn->GIFIF.Old;
        ipipeWb.gainGr = hn->GIFIF.Old;
        ipipeWb.gainGb = hn->GIFIF.Old;
        ipipeWb.gainB  = hn->GIFIF.Old;
        DRV_ipipeSetWb(&ipipeWb);
    }

    //Change WB
    if( hn->Rgain.New != hn->Rgain.Old || hn->Bgain.New != hn->Bgain.Old ) {
        smooth_change(&hn->Rgain, fr);
        smooth_change(&hn->Bgain, fr);
        DRV_isifSetDgain(512 , hn->Rgain.Old, hn->Bgain.Old, 512, 0);
    }

    //Change Expouse
    if(pScParams->fdMode == SCFdDisabled){
        if( hn->Exp.New != hn->Exp.Old )
        {
            smooth_change(&hn->Exp, fr);
            int fpsNew = 1000000/hn->Exp.Old;
            if(fpsNew < defaultFPS)
            {
                DRV_imgsSetFramerate(fpsNew);
                defaultFPS = fpsNew;
            }
            DRV_imgsSetEshutter(hn->Exp.Old, 0);
        }
    }
    else if(pScParams->fdMode == SCFd50hz)
    {
        hn->Exp.New = (50/defaultFPS)*20000;
        hn->Exp.Old = hn->Exp.New;
        DRV_imgsSetEshutter(hn->Exp.Old, 0);
    }
    else if(pScParams->fdMode == SCFd50hz)
    {
        hn->Exp.New = (60/defaultFPS)*16667;
        hn->Exp.Old = hn->Exp.New;
        DRV_imgsSetEshutter(hn->Exp.Old, 0);
    }

    //Thresholds dynamic change
    if( Threshold_IR_cut_open != hn->Threshold_IR_cut[0] ) {
        hn->Threshold_IR_cut[0] = 30; //Threshold_IR_cut_open;
        hn->Threshold_IR_cut[1] = 30; //Threshold_IR_cut_open;
    }

    //BW and color night mode dynamic change and IR-cut dynamic change
    if(IRcutClose != hn->IRcutClose)
    {
        if(IRcutClose == 0)
        {
            if( pScParams->bwMode != hn->gBWMode ||  pScParams->ircutOpen != hn->gIRCut)
            {
                DRV_imgsNDShutter( pScParams->ircutOpen,  pScParams->bwMode);
                hn->gBWMode = pScParams->bwMode;
                hn->gIRCut = pScParams->ircutOpen;
            }
        }
        else
        {
            DRV_imgsNDShutter(0, 0);
        }
        hn->IRcutClose = IRcutClose;
    }
    /*
	if( pScParams->expMode != SCExpManual ) //scam
	{
		//IR-cut dynamic change
		if( gIRCut != hn->gIRCut ) {
		    if( gIRCut == ALG_IRCUT_OPEN ) {
				IRcutClose = 0;  // Open
			}
		    else if( gIRCut == ALG_IRCUT_CLOSE ) {
				IRcutClose = 1;  // Close
			}
		    hn->gIRCut = gIRCut;
		}

		//BW and color night mode dynamic change
		if( gBWMode != hn->gBWMode ) {
		    if( gIRCut == ALG_IRCUT_AUTO && !IRcutClose ) {
		        DRV_imgsNDShutter( gIRCut, gBWMode );
		    }
		    hn->gBWMode = gBWMode;
		}

		//Change IR-cut
		if( IRcutClose != hn->IRcutClose ) {
		    DRV_imgsNDShutter(IRcutClose, gBWMode);
		    //Change color matrix in night mode
            //if( IRcutClose == 0 ) {
            //    setup_color_matrix_value(&rgb2rgb2, 255);
            //} else {
            //    setup_color_matrix_sensor(&rgb2rgb2, DRV_imgsGetImagerName());
            //}
		    hn->IRcutClose = IRcutClose;
		}
	}

	//////////////////////////////////////////
    // Exp min max set
	if( (pScParams->expMode != hn->scImgParams.expMode) || (pScParams->shutter != hn->scImgParams.shutter) )
	{
		OSA_printf(">>>=== SCAM: EXPMODE || SHUTTER changed (EXP: %d->%d, SHUTTER: %d->%d)\n", 
				pScParams->expMode, hn->scImgParams.expMode,
				pScParams->shutter, hn->scImgParams.shutter );

		hn->Exp.New = (hn->Exp.Range.max / hn->Exp.Step) * hn->Exp.Step;
		hn->Exp.Old = hn->Exp.New - 1;

		hn->GIFIF.Old = 511;
		hn->GIFIF.New = 512;

        //hn->Grgb2rgb.Old = 255;
        //hn->Grgb2rgb.New = 256;

		if( (pScParams->expMode == SCExpManual) && (pScParams->shutter == 0) )
		{
			hn->Exp.Range.min = pScParams->shutterScope.min;
			hn->Exp.Range.max = pScParams->shutterScope.max;
		}
		else
		{
			hn->Exp.Range.min = hn->Exp.Step;
			hn->Exp.Range.max = 33333; // 1/30
		}
    }
	//////////////////////////////////////////
	//////////////////////////////////////////

    //Flicker
    if( pScParams->fdMode != hn->scImgParams.fdMode ) {

		OSA_printf(">>>=== SCAM: FDMODE changed\n");

		hn->Exp.Step = GET_SHUTTER_STEP( pScParams->fdMode ); 	// Exposure stepsize
		hn->Exp.New = (hn->Exp.Range.max / hn->Exp.Step) * hn->Exp.Step;
	}

	//Seting Expouse
	if( (pScParams->expMode != SCExpManual) || (pScParams->shutter == 0) ) //scam
	{
		if( hn->Exp.New != hn->Exp.Old ) {
			smooth_change(&hn->Exp, fr);
			shutterValToSet = hn->Exp.Old;
		}
	}

	if( (pScParams->expMode != hn->scImgParams.expMode) || (pScParams->shutter != hn->scImgParams.shutter) )
	{
		if( (pScParams->expMode == SCExpManual) && (pScParams->shutter != 0) )
		{
			shutterValToSet = pScParams->shutter;
		}
	}

    if( shutterValToSet != -1 || FPShigh != hn->FPShigh )
	{	
		OSA_printf(">>>=== SCAM: NEW SHUTTER value\n");

		int fpsNew = 30;
		if(shutterValToSet > (1000000 / fpsNew)) //if greater max -> calculate FPS
		{
			fpsNew = 1000000 / shutterValToSet;
			if(fpsNew < 5)  fpsNew = 5;
			if(fpsNew > 30) fpsNew = 30;
			shutterValToSet = 1000000 / fpsNew; 
		}

		if(defaultFPS != fpsNew)
		{
            //hn->FPScur = fpsNew;
            //hn->FPSmax = fpsNew;
			DRV_imgsSetFramerate( fpsNew );
		}

		shutterValToSet = ((int)(shutterValToSet / hn->Exp.Step)) * hn->Exp.Step; //calc value for flicker

		DRV_imgsSetEshutter(shutterValToSet, 0);
        hn->FPShigh = FPShigh;
	}
    */

    //Config RGB2RGB matrix for more gain
    /*
    if( hn->Grgb2rgb.New !=  hn->Grgb2rgb.Old ) {
        smooth_change(&hn->Grgb2rgb, fr);
     
		rgb2rgb.matrix[0][0] = hn->Grgb2rgb.Old;
        rgb2rgb.matrix[0][1] = 0;
        rgb2rgb.matrix[0][2] = 0;

        rgb2rgb.matrix[1][0] = 0;
        rgb2rgb.matrix[1][1] = hn->Grgb2rgb.Old;
        rgb2rgb.matrix[1][2] = 0;

        rgb2rgb.matrix[2][0] = 0;
        rgb2rgb.matrix[2][1] = 0;
        rgb2rgb.matrix[2][2] = hn->Grgb2rgb.Old;

        rgb2rgb.offset[0]    = 0;
        rgb2rgb.offset[1]    = 0;
        rgb2rgb.offset[2]    = 0;

        if(DRV_ipipeSetRgb2Rgb(&rgb2rgb) != CSL_SOK)
            OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
    }
    */
	//////////////////////////////////////////
	// brightness, contrast
	if( pScParams->brightness != hn->scImgParams.brightness || pScParams->contrast != hn->scImgParams.contrast )
	{
		OSA_printf(">>>=== SCAM: NEW BR CONTRAST value\n");

	    BRT_CRT_PARAM brtCtrParam;
	    brtCtrParam.yuv_adj_ctr = pScParams->contrast >> 3;
	    brtCtrParam.yuv_adj_brt = pScParams->brightness;
	    ALG_aewbSetContrastBrightness(&brtCtrParam);
	}
	//////////////////////////////////////////
	//////////////////////////////////////////

	//////////////////////////////////////////
	// saturation, sharpness
	if( pScParams->saturation != hn->scImgParams.saturation )
   		SCSetSaturation(pScParams->saturation);

	if( pScParams->sharpness != hn->scImgParams.sharpness )
   		SCSetSharpness(pScParams->sharpness);
	//////////////////////////////////////////
	//////////////////////////////////////////
		

	//////////////////////////////////////////
	// Save prev values
	memcpy(&(hn->scImgParams), pScParams, sizeof(scImgParams_t));
	hn->scIsAutoCamMode = Aew_ext_parameter.scIsAutoCamMode;
	//////////////////////////////////////////

	///////////////////
    sFrames++;
	///////////////////
}



