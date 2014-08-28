#include "alg_aewb_priv.h"
#include <drv_gpio.h>
#include <drv_ipipe.h>
#include <drv_isif.h>

#include "aewbf_sig.h"
#include "iaewbf_sig.h"

#define __DEBUG
#ifdef __DEBUG
#define AE_DEBUG_PRINTS
#define dprintf printf
#else
#define dprintf
#endif

ALG_AewbObj gALG_aewbObj;
ALG_AewbfObj gSIG_Obj;

#define SRESHOLD_DIFF 120

extern DRV_IpipeObj gDRV_ipipeObj;    //For boxcar
extern CSL_IpipeObj gCSL_ipipeHndl;   //For gamma and rgb2rgb
extern int OSA_fileWriteFile(const char *fileName, const void *addr, size_t size);
//extern short TI_2A_SetEEValues(int shift_val);
//extern int DRV_ipipeSetEdgeEnhance(CSL_IpipeEdgeEnhanceConfig *config);

extern int gAePriorityMode, gBWMode, gDayNight, gIRCut, defaultFPS, gFlicker;
extern int IRcutClose, FPShigh;
extern int Threshold_IR_cut_open;
//extern int gHDR;
extern Uint32  gamma01[], gamma005[], gamma001[], gamma003[], SIG_YEE_TABLE[];
extern Int32 leave_frames;
extern int  frame_count;
extern int d_th_count;


int DEBUG = 1;

short SIG_2A_SetEEValues(int shift_val) //shift=3 for 1080P/720P, shift=4 for D1
{
    CSL_IpipeEdgeEnhanceConfig config;

    config.enable                   = TRUE;
    config.haloReduceEnable         = FALSE;
    config.mergeMethod              = CSL_IPIPE_YEE_EE_ES_MERGING_METHOD_ABSOLUTE_MAX;//CSL_IPIPE_YEE_EE_ES_MERGING_METHOD_SUMMATION; //
    config.hpfShift                 = shift_val;
    config.hpfCoeff[0][0]           = 48;
    config.hpfCoeff[0][1]           = 12;
    config.hpfCoeff[0][2]           = 1024-10;
    config.hpfCoeff[1][0]           = 12;
    config.hpfCoeff[1][1]           = 0;
    config.hpfCoeff[1][2]           = 1024-6;
    config.hpfCoeff[2][0]           = 1024-10;
    config.hpfCoeff[2][1]           = 1024-6;
    config.hpfCoeff[2][2]           = 1024-2;
    config.lowerThres               = 32;//8; //changed based on Gang Comment
    config.edgeSharpGain            = 0;
    config.edgeSharpHpValLowThres   = 128;
    config.edgeSharpHpValUpLimit    = 768;
    config.edgeSharpGainGradient    = 32;
    config.edgeSharpOffsetGradient  = 0;
    config.table                    = SIG_YEE_TABLE;

    DRV_ipipeSetEdgeEnhance(&config);

    return 0;
}


void smooth_change( IAEWBF_Param *par, int fr)
{
    if(!fr) {
        par->Change = (par->New - par->Old)/leave_frames;
        par->Old += par->Change;
    } else if (fr == (leave_frames-1)){
        par->Old = par->New;
    } else {
        par->Old += par->Change;
    }
    //OSA_printf("fr = %d hn->Exp.New = %d hn->Exp.Old = %d Change = %d diff = %d\n", fr, par->New, par->Old, par->Change, par->New - par->Old);
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

    //For Hight resolution we can get only half size of boxcar
    if(hn->w == 162 && hn->h == 120) hn->h /= 2;
    if(hn->w == 160 && hn->h == 90)  hn->h /= 2;
    if(hn->w == 144 && hn->h == 82)  hn->h /= 2;

    hn->SatTh = hn->w*hn->h*3/300;
    //printf("box = %p w = %d h = %d SatTh = %d\n", hn->box, hn->w, hn->h, hn->SatTh);

    return OSA_SOK;
}

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


void ALG_SIG_config(IALG_Handle handle)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    char zoomvalue[4];
    int j;
    size_t i;
    Uint32 hsz = ALG_SENSOR_BITS;

    //Uint32 tables[512];
    CSL_IpipeGammaConfig dataG;
    CSL_IpipeRgb2RgbConfig rgb2rgb1, rgb2rgb2;
    DRV_IpipeWb ipipeWb;

    if(defaultFPS != DRV_imgsGetFramerate()) {
        defaultFPS = DRV_imgsGetFramerate();
        hn->FPScur = defaultFPS;
        hn->FPSmax = defaultFPS;
    }

    hn->HISTTH = 20;

    //Zoom in 0 position
    sprintf(zoomvalue, "%04d", 0);
    if(OSA_fileWriteFile("/var/run/zoom", zoomvalue, sizeof(zoomvalue)) !=OSA_SOK) {
        OSA_printf("AF: error write in file\n");
    }

    //if(raw) ALG_aewbSetSensorDcsub(0); //176
    //else ALG_aewbSetSensorDcsub(ZERO);
    DRV_isifSetDcSub(-ZERO);
    SIG_2A_SetEEValues(3);

    //Config contrast and Brightness
    //DRV_ipipeSetYoffet((pParm->yuv_adj_brt-128));
    //DRV_ipipeSetContrastBrightness(pParm->yuv_adj_ctr, 0x0);

    //Config gamma correction tables
    dataG.tableSize = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SIZE_512;
    dataG.tableSrc  = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SELECT_RAM;
    dataG.bypassR = 0;
    dataG.bypassG = dataG.bypassR;
    dataG.bypassB = dataG.bypassR;
    dataG.tableR = gamma003; //gamma005//gamma42; //gamma_hdr011; //gamma_hdr01; //gamma01; //gamma00520
    dataG.tableG = dataG.tableR;
    dataG.tableB = dataG.tableR;

    //Liner gamma tables
    /*
    vl0 = 0;
    for(i=0; i < 512; i++){
        vl1 = i<<1;
        tables[i] = (vl0<<10) | (vl1 - vl0);
        vl0 = vl1;
    }
    */
    if(CSL_ipipeSetGammaConfig(&gCSL_ipipeHndl, &dataG) != CSL_SOK)
        OSA_ERROR("Fail CSL_ipipeSetGammaConfig!!!\n");


    //Config RGB2RGB matrix
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

    setup_color_matrix_sensor(&rgb2rgb2, DRV_imgsGetImagerName());

    //rgb2rgb2.offset[0]    = 0;
    //rgb2rgb2.offset[1]    = 0;
    //rgb2rgb2.offset[2]    = 0;

    //if(DRV_ipipeSetRgb2Rgb(&rgb2rgb1) != CSL_SOK)
    //    OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
    if(DRV_ipipeSetRgb2Rgb2(&rgb2rgb2) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");

    //Config isif white balance gain
    DRV_isifSetDgain(512, hn->Rgain.New, hn->Bgain.New, 512, 0);

    //Config ipipe gains and offset
    ipipeWb.gainR  = hn->GIFIF.New;
    ipipeWb.gainGr = hn->GIFIF.New;
    ipipeWb.gainGb = hn->GIFIF.New;
    ipipeWb.gainB  = hn->GIFIF.New;

    DRV_ipipeSetWbOffset(0);
    DRV_ipipeSetWb(&ipipeWb);
}

#if 1 // unused
static void print_debug(int frames, int leave_frames, IAEWBF_SIG_Obj *hn){
    int i = 0, fr = frames%leave_frames, all = frames%150;
    //all = fr;
    //all = 0; fr = 0;

    if(DEBUG && (!fr)){
        if (gIRCut != hn->gIRCut || gBWMode != hn->gBWMode || FPShigh != hn->FPShigh ||
                gAePriorityMode != hn->gAePriorityMode || IRcutClose != hn->IRcutClose ||
                gFlicker != hn->gFlicker || !all){
            dprintf("%6d   IRflick = %d AEPrior = %d hAEPrior = %d BWMod = %d DayNight = %d IRCut = %d hIRCut = %d defaultFPS = %d FPScur = %lu IRClose = %d hIRClose = %d FPShigh = %d hFPShigh = %d\n",
                    frames, gFlicker, gAePriorityMode, hn->gAePriorityMode, gBWMode, gDayNight, gIRCut, hn->gIRCut,
                    defaultFPS, hn->FPScur, IRcutClose, hn->IRcutClose, FPShigh, hn->FPShigh);
            i++;
        }
        if (hn->Exp.New != hn->Exp.Old || !all){
            dprintf("%6d   EXP          : Exp.New  = %6ld Exp.Old = %6ld Exp.Max = %6ld\n", frames, hn->Exp.New, hn->Exp.Old, hn->Exp.Range.max);
            i++;
        }

        if (hn->Offset.New != hn->Offset.Old || !all){
            dprintf("%6d   OFFSET       : Off.New = %4ld Off.Old = %4ld \n", frames, hn->Offset.New, hn->Offset.Old);
            i++;
        }
        if (hn->Rgain.New != hn->Rgain.Old || hn->Bgain.New != hn->Bgain.Old || !all){
            dprintf("%6d   GAIN WB      : Rgain.NewA = %4ld Rgain.New = %4ld Rgain.Old = %4ld Bgain.NewA = %4ld Bgain.New = %4ld Bgain.Old = %4ld \n",
                    frames, hn->Rgain.NewA, hn->Rgain.New, hn->Rgain.Old, hn->Bgain.NewA, hn->Bgain.New, hn->Bgain.Old);
            i++;
        }
        if (hn->GIFIF.New !=  hn->GIFIF.Old || !all) {
            dprintf("%6d   GAIN IFIF    : GIFIF.New = %4ld GIFIF.Old = %4ld \n", frames, hn->GIFIF.New, hn->GIFIF.Old);
            i++;
        }
        if (hn->Grgb2rgb.New !=  hn->Grgb2rgb.Old || !all) {
            dprintf("%6d   GAIN RGB2RGB : RGB2gain.New = %4ld RGB2gain.Old = %4ld \n", frames, hn->Grgb2rgb.New, hn->Grgb2rgb.Old);
            i++;
        }
         dprintf("Threshold_IR_cut_open = %i Threshold_IR_cut_close = %i d_th_count = %i\n", hn->Threshold_IR_cut[0], hn->Threshold_IR_cut[1], d_th_count);


        if (i) dprintf("Y.NewA = %4ld Y.New = %4ld Y.Old = %4ld Hmin = %4ld Hmax = %4ld HminA = %4ld HmaxA = %4ld\n",
		       hn->Y.NewA, hn->Y.New, hn->Y.Old, hn->Hmin.New, hn->Hmax.New, hn->Hmin.NewA, hn->Hmax.NewA);

    }
}
#endif

void SIG2A_applySettings(void)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)gSIG_Obj.handle_aewbf;
    CSL_IpipeGammaConfig dataG;
    DRV_IpipeWb ipipeWb;
    CSL_IpipeRgb2RgbConfig rgb2rgb, rgb2rgb2;
    static int frames = 0;
    int fr = frames%leave_frames;

    print_debug(frames, leave_frames, hn);

    //IR-cut dynamic change
    if(gIRCut != hn->gIRCut) {
        if      (gIRCut == ALG_IRCUT_OPEN)  IRcutClose = 0;  // Open
        else if (gIRCut == ALG_IRCUT_CLOSE) IRcutClose = 1;  // Close
        hn->gIRCut = gIRCut;
    }

    //BW and color night mode dynamic change
    if(gBWMode != hn->gBWMode) {
        if (gIRCut == ALG_IRCUT_AUTO && !IRcutClose) {
            DRV_imgsNDShutter(gIRCut, gBWMode);
            //DRV_imgsSetBWmode(gBWMode);
        }
        hn->gBWMode = gBWMode;
    }

    //Thresholds dynamic change
    if(Threshold_IR_cut_open != hn->Threshold_IR_cut[0]){
        hn->Threshold_IR_cut[0] = Threshold_IR_cut_open;
        hn->Threshold_IR_cut[1] = Threshold_IR_cut_open;
    }

    //Change FPS
    if (FPShigh != hn->FPShigh || gAePriorityMode != hn->gAePriorityMode || defaultFPS != hn->FPScur) {
        if(defaultFPS != hn->FPScur){
            hn->FPScur = defaultFPS;
            hn->FPSmax = defaultFPS;
        }
        if(FPShigh == 0 ){
            //Go to low FPS
            if (gAePriorityMode == ALG_FPS_LOW) {
                hn->FPScur = hn->FPSmax>>1;
                DRV_imgsSetFramerate(hn->FPScur);
                hn->Exp.Range.max = 1000000/hn->FPScur;
                //hn->Exp.New  = (hn->Exp.Range.max/hn->Exp.Step)*hn->Exp.Step;
            }
            else if (gAePriorityMode == ALG_FPS_5FPS){
                hn->FPScur = 5;
                DRV_imgsSetFramerate(hn->FPScur);
                hn->Exp.Range.max = 200000;
                //hn->Exp.New  = (hn->Exp.Range.max/hn->Exp.Step)*hn->Exp.Step;
            } else {
                hn->FPScur = hn->FPSmax;
                DRV_imgsSetFramerate(hn->FPScur);
                hn->Exp.Range.max = 1000000/hn->FPScur;
                //hn->Exp.New  = (hn->Exp.Range.max/hn->Exp.Step)*hn->Exp.Step;
            }
        } else {
            //Go to high FPS
            hn->FPScur = hn->FPSmax;
            DRV_imgsSetFramerate(hn->FPScur);
            hn->Exp.Range.max = 1000000/hn->FPScur;
            //hn->Exp.New  = (hn->Exp.Range.max/hn->Exp.Step)*hn->Exp.Step;
        }

        hn->Exp.New  = (hn->Exp.Range.max/hn->Exp.Step)*hn->Exp.Step;
        hn->Exp.Old = hn->Exp.New;
        hn->FPShigh = FPShigh;
        hn->gAePriorityMode = gAePriorityMode;
        DRV_imgsSetEshutter(hn->Exp.New, 0);
        OSA_printf("defaultFPS = %d hn->Exp.New = %ld hn->Exp.Range.max = %ld hn->Exp.Step = %ld\n",
                   defaultFPS, hn->Exp.New, hn->Exp.Range.max, hn->Exp.Step);
    }

    //Change IR-cut
    if (IRcutClose != hn->IRcutClose) {
        DRV_imgsNDShutter(IRcutClose, gBWMode);
        //Change color matrix in night mode
        if(IRcutClose == 0){
            setup_color_matrix_value(&rgb2rgb2, 255);
        } else {
            setup_color_matrix_sensor(&rgb2rgb2, DRV_imgsGetImagerName());
        }
        hn->IRcutClose = IRcutClose;
    }

    //Flicker
    if(gFlicker != hn->gFlicker) {
        if(gFlicker == VIDEO_NTSC) {		// 60 Hz flicker
            hn->Exp.Step = 8333; 	// Exposure stepsize
        } else if(gFlicker == VIDEO_PAL) {	// 50 Hz flicker
            hn->Exp.Step = 10000; 	// Exposure stepsize
        } else {
            hn->Exp.Step = 1;
        }
        //hn->Exp.Max = 1000000/hn->FPSmax;
        hn->Exp.New = (hn->Exp.Old/hn->Exp.Step)*hn->Exp.Step;
        hn->gFlicker = gFlicker;
    }

    //Seting Expouse
    if(hn->Exp.New != hn->Exp.Old) {
        smooth_change(&hn->Exp, fr);
        //hn->Exp.Old = hn->Exp.New;
        DRV_imgsSetEshutter(hn->Exp.Old, 0);
    }
    //ISIF gain seting
    if(hn->Rgain.New != hn->Rgain.Old || hn->Bgain.New != hn->Bgain.Old){
        smooth_change(&hn->Rgain, fr);
        smooth_change(&hn->Bgain, fr);
        //hn->Rgain.Old = hn->Rgain.New;
        //hn->Bgain.Old = hn->Bgain.New;
        DRV_isifSetDgain(512 , hn->Rgain.Old, hn->Bgain.Old, 512, 0);
    }

    if(hn->Offset.New != hn->Offset.Old) {
        smooth_change(&hn->Offset, fr);
        //hn->Offset.Old = hn->Offset.New;
        DRV_ipipeSetWbOffset(-hn->Offset.Old);
    }

    if(hn->GIFIF.New !=  hn->GIFIF.Old){
        smooth_change(&hn->GIFIF, fr);
        //hn->GIFIF.Old = hn->GIFIF.New;
        ipipeWb.gainR  = hn->GIFIF.Old;
        ipipeWb.gainGr = hn->GIFIF.Old;
        ipipeWb.gainGb = hn->GIFIF.Old;
        ipipeWb.gainB  = hn->GIFIF.Old;
        DRV_ipipeSetWb(&ipipeWb);
    }

    //Config RGB2RGB matrix for more gain
    if(hn->Grgb2rgb.New !=  hn->Grgb2rgb.Old){
        smooth_change(&hn->Grgb2rgb, fr);
        //hn->Grgb2rgb.Old = hn->Grgb2rgb.New;
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
    frames++;
}


int SIG_2A_config(IALG_Handle handle)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    int i, stepSize, sensorExposureMax;

    if(gFlicker == VIDEO_NTSC) {		// 60 Hz flicker
        stepSize = 8333; 	// Exposure stepsize
    } else if(gFlicker == VIDEO_PAL) {	// 50 Hz flicker
        stepSize = 10000; 	// Exposure stepsize
    } else {
        stepSize = 1;
    }

    sensorExposureMax = 1000000/defaultFPS;

    //Exposure setup
    hn->Exp.Step = stepSize;
    hn->Exp.Old = (sensorExposureMax/stepSize)*stepSize;
    hn->Exp.New = hn->Exp.Old;
    hn->Exp.Range.max = sensorExposureMax;
    hn->Exp.Range.min = stepSize;

    //ISIF R gain setup
    hn->Rgain.Old = 511;
    hn->Rgain.New = 512;
    hn->Rgain.Range.min = 50;
    hn->Rgain.Range.max = 4095;

    //ISIF B gain setup
    hn->Bgain.Old = 511;
    hn->Bgain.New = 512;
    hn->Bgain.Range.min = 50;
    hn->Bgain.Range.max = 4095;

    //IFIF gain setup
    hn->GIFIF.Step = 16;
    hn->GIFIF.Old = 511;
    hn->GIFIF.New = 512;
    hn->GIFIF.Range.min = 1;
    hn->GIFIF.Range.max = 8180;

    //ISIF offset setup
    hn->Offset.Step = 1;
    hn->Offset.Old = 0;
    hn->Offset.New = 0;
    hn->Offset.Range.min = 1;
    hn->Offset.Range.max = 4095;

    hn->Grgb2rgb.Old = 256;
    hn->Grgb2rgb.New = 256;
    hn->Grgb2rgb.Range.min = 1;
    hn->Grgb2rgb.Range.max = 256; //385;
    //if(strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0)
    //    hn->Grgb2rgb.Range.max = 512;
    //else hn->Grgb2rgb.Range.max = 356;

    //Y setup
    hn->Y.Step = 1;
    hn->Y.New = 1;
    hn->Y.Old = 1;
    hn->Y.Range.min = 0;
    hn->Y.Range.max = 4095;
    hn->Y.HistC = 0;
    for(i=0; i < HISTORY; i++) hn->Y.Hist[i] = 0;
    hn->Y.Avrg = 0;

    hn->Hmax.HistC = 0;
    for(i=0; i < HISTORY; i++) hn->Hmax.Hist[i] = 0;
    hn->Hmax.Avrg = 0;

    hn->Hmin.HistC = 0;
    for(i=0; i < HISTORY; i++) hn->Hmin.Hist[i] = 0;
    hn->Hmin.Avrg = 0;

    hn->HmaxTh = 3500;
    hn->YAE = 500;
    hn->SatTh = hn->w*hn->h*3/100;

    //First value of dymanic parameters
    hn->gAePriorityMode = 4;    //gAePriorityMode;
    hn->gIRCut = gIRCut;
    hn->gBWMode = gBWMode;
    hn->gFlicker = 4;
    hn->IRcutClose = IRcutClose; //IR-cut 1-open, 0 - close
    hn->FPShigh = FPShigh;      //FPS 1-high, 0 - low
    hn->FPSmax = defaultFPS;
    hn->FPScur = defaultFPS;
    hn->Threshold_IR_cut[0] = Threshold_IR_cut_open;
    hn->Threshold_IR_cut[1] = Threshold_IR_cut_open;

    ALG_SIG_config(gSIG_Obj.handle_aewbf);

    return 0;
}

