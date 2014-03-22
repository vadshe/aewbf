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

extern DRV_IpipeObj gDRV_ipipeObj;    //For boxcar
extern CSL_IpipeObj gCSL_ipipeHndl;   //For gamma and rgb2rgb
extern int OSA_fileWriteFile(const char *fileName, const void *addr, size_t size);
//extern short TI_2A_SetEEValues(int shift_val);
//extern int DRV_ipipeSetEdgeEnhance(CSL_IpipeEdgeEnhanceConfig *config);

extern int gAePriorityMode, gBWMode, gDayNight, gIRCut, defaultFPS, gFlicker;
extern int IRcutClose, FPShigh;
extern int gHDR;
extern Uint32  gamma01[], gamma005[], gamma001[], SIG_YEE_TABLE[];
extern Int32 leave_frames;

int DEBUG = 1, raw = 0;

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
    //OSA_printf("frames = %d fr = %d hn->Exp.New = %d hn->Exp.Old = %d diff = %d\n", frames, fr, hn->Exp.New, hn->Exp.Old, (hn->Exp.New - hn->Exp.Old)/leave_frames);
    if(!fr) {
        par->Change = (par->New - par->Old)/leave_frames;
        par->Old += par->Change;
    } else if (fr == (leave_frames-1)){
        par->Old = par->New;
    } else {
        par->Old += par->Change;
    }
    //OSA_printf("hn->Exp.New = %d hn->Exp.Old = %d hn->Exp.Change = %d leave_frames = %d\n",
    //           hn->Exp.New, hn->Exp.Old, hn->Exp.Change, leave_frames);
}


int Get_BoxCar(IALG_Handle handle)
{
    int status;
    int bufId=-1;
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    OSA_BufInfo *pBufInfo;

    status = DRV_ipipeGetBoxcarBuf(&bufId, 1<<31); //OSA_TIMEOUT_NONE);
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
    hn->SatTh = hn->w*hn->h*3/300;

    return OSA_SOK;
}

void ALG_SIG_config(IALG_Handle handle)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    char zoomvalue[4];

    //Uint32 tables[512];
    CSL_IpipeGammaConfig dataG;
    CSL_IpipeRgb2RgbConfig rgb2rgb1, rgb2rgb2;
    DRV_IpipeWb ipipeWb;

    if(defaultFPS != DRV_imgsGetFramerate()) {
        defaultFPS = DRV_imgsGetFramerate();
        hn->FPScur = defaultFPS;
        hn->FPSmax = defaultFPS;
    }

    hn->HISTTH = 40;

    //Zoom in 0 position
    sprintf(zoomvalue, "%04d", 0);
    if(OSA_fileWriteFile("/var/run/zoom", zoomvalue, sizeof(zoomvalue)) !=OSA_SOK) {
        OSA_printf("AF: error write in file\n");
    }

    //if(raw) ALG_aewbSetSensorDcsub(0); //176
    //else ALG_aewbSetSensorDcsub(ZERO);
    if(raw) DRV_isifSetDcSub(0);
    else DRV_isifSetDcSub(-ZERO);
    SIG_2A_SetEEValues(3);

    //Config contrast and Brightness
    //DRV_ipipeSetYoffet((pParm->yuv_adj_brt-128));
    //DRV_ipipeSetContrastBrightness(pParm->yuv_adj_ctr, 0x0);

    //Config gamma correction tables
    dataG.tableSize = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SIZE_512;
    dataG.tableSrc  = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SELECT_RAM;
    if(raw) dataG.bypassR = 1;
    else dataG.bypassR = 0;
    dataG.bypassG = dataG.bypassR;
    dataG.bypassB = dataG.bypassR;
    dataG.tableR = gamma005; //gamma005//gamma42; //gamma_hdr011; //gamma_hdr01; //gamma01; //gamma00520
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

    rgb2rgb2.matrix[0][0] = 256;
    rgb2rgb2.matrix[0][1] = 0;
    rgb2rgb2.matrix[0][2] = 0;

    rgb2rgb2.matrix[1][0] = 0;
    rgb2rgb2.matrix[1][1] = 256;
    rgb2rgb2.matrix[1][2] = 0;

    rgb2rgb2.matrix[2][0] = 0;
    rgb2rgb2.matrix[2][1] = 0;
    rgb2rgb2.matrix[2][2] = 256;

    if(!raw){
        if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9M034_720P") == 0) {
            rgb2rgb2.matrix[0][0] = 427;
            rgb2rgb2.matrix[0][1] = -105;
            rgb2rgb2.matrix[0][2] = -66;

            rgb2rgb2.matrix[1][0] = -99;
            rgb2rgb2.matrix[1][1] = 422;
            rgb2rgb2.matrix[1][2] = -67;

            rgb2rgb2.matrix[2][0] = -8;
            rgb2rgb2.matrix[2][1] = -78;
            rgb2rgb2.matrix[2][2] = 342;
        } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0) {
            rgb2rgb2.matrix[0][0] = 380;
            rgb2rgb2.matrix[0][1] = -59;
            rgb2rgb2.matrix[0][2] = -66;

            rgb2rgb2.matrix[1][0] = -89;
            rgb2rgb2.matrix[1][1] = 402;
            rgb2rgb2.matrix[1][2] = -57;

            rgb2rgb2.matrix[2][0] = -8;
            rgb2rgb2.matrix[2][1] = -98;
            rgb2rgb2.matrix[2][2] = 362;
        } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0) {
            hn->HISTTH = 30; //Reduse threshold to remove nonliniarity

            rgb2rgb2.matrix[0][0] = 380;
            rgb2rgb2.matrix[0][1] = -59;
            rgb2rgb2.matrix[0][2] = -66;

            rgb2rgb2.matrix[1][0] = -89;
            rgb2rgb2.matrix[1][1] = 402;
            rgb2rgb2.matrix[1][2] = -57;

            rgb2rgb2.matrix[2][0] = -8;
            rgb2rgb2.matrix[2][1] = -168;
            rgb2rgb2.matrix[2][2] = 432;
        } else if(strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0){
            rgb2rgb2.matrix[0][0] = 360;
            rgb2rgb2.matrix[0][1] = -153;
            rgb2rgb2.matrix[0][2] = 49;

            rgb2rgb2.matrix[1][0] = -92;
            rgb2rgb2.matrix[1][1] = 312;
            rgb2rgb2.matrix[1][2] = 36;

            rgb2rgb2.matrix[2][0] = 37;
            rgb2rgb2.matrix[2][1] = -338;
            rgb2rgb2.matrix[2][2] = 557;
        }
    }
    rgb2rgb2.offset[0]    = 0;
    rgb2rgb2.offset[1]    = 0;
    rgb2rgb2.offset[2]    = 0;

    if(DRV_ipipeSetRgb2Rgb(&rgb2rgb1) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
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

void print_debug(int frames, int leave_frames, IAEWBF_SIG_Obj *hn){
    int i = 0, fr = frames%leave_frames, all = frames%150;
    //all = fr;

    if(DEBUG && (!fr)){
        if(gIRCut != hn->gIRCut || gBWMode != hn->gBWMode || FPShigh != hn->FPShigh ||
                gAePriorityMode != hn->gAePriorityMode || IRcutClose != hn->IRcutClose ||
                gFlicker != hn->gFlicker || !all){
            dprintf("%6d   IRflick = %d AEPrior = %d hAEPrior = %d BWMod = %d DayNight = %d IRCut = %d hIRCut = %d defaultFPS = %d FPScur = %d IRClose = %d hIRClose = %d FPShigh = %d hFPShigh = %d\n",
                    frames, gFlicker, gAePriorityMode, hn->gAePriorityMode, gBWMode, gDayNight, gIRCut, hn->gIRCut,
                    defaultFPS, hn->FPScur, IRcutClose, hn->IRcutClose, FPShigh, hn->FPShigh);
            i++;
        }
        if(hn->Exp.New != hn->Exp.Old || !all){
            dprintf("%6d   EXP          : Exp.New  = %6d Exp.Old = %6d Exp.Max = %6d\n", frames, hn->Exp.New, hn->Exp.Old, hn->Exp.Range.max);
            i++;
        }
        if(hn->Offset.New != hn->Offset.Old || !all){
            dprintf("%6d   OFFSET       : Off.New = %4d Off.Old = %4d \n", frames, hn->Offset.New, hn->Offset.Old);
            i++;
        }
        if(hn->Rgain.New != hn->Rgain.Old || hn->Bgain.New != hn->Bgain.Old || !all){
            dprintf("%6d   GAIN WB      : Rgain.New = %4d Rgain.Old = %4d Bgain.New = %4d Bgain.Old = %4d \n",
                    frames, hn->Rgain.New, hn->Rgain.Old, hn->Bgain.New, hn->Bgain.Old);
            i++;
        }
        if(hn->GIFIF.New !=  hn->GIFIF.Old || !all) {
            dprintf("%6d   GAIN IFIF    : GIFIF.New = %4d GIFIF.Old = %4d \n", frames, hn->GIFIF.New, hn->GIFIF.Old);
            i++;
        }
        if(hn->Grgb2rgb.New !=  hn->Grgb2rgb.Old || !all) {
            dprintf("%6d   GAIN RGB2RGB : RGB2gain.New = %4d RGB2gain.Old = %4d \n", frames, hn->Grgb2rgb.New, hn->Grgb2rgb.Old);
            i++;
        }
        if(i ) dprintf("Y.New = %4d Y.Old = %4d Y.Min = %4d Y.Max = %4d Y.Diff = %4d Hmin = %4d Hmax = %4d HminA = %4d HmaxA = %4d\n",
                                   hn->Y.New, hn->Y.Old, hn->Y.Min, hn->Y.Max, hn->Y.Diff, hn->Hmin.New, hn->Hmax.New, hn->Hmin.NewA, hn->Hmax.NewA);

    }
}

void SIG2A_applySettings(void)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)gSIG_Obj.handle_aewbf;
    CSL_IpipeGammaConfig dataG;
    DRV_IpipeWb ipipeWb;
    CSL_IpipeRgb2RgbConfig rgb2rgb;
    static int frames = 0;
    int fr = frames%leave_frames;

    print_debug(frames, leave_frames, hn);

    //IR-cut dynamic change
    if(gIRCut != hn->gIRCut) {
        if      (gIRCut == ALG_IRCUT_OPEN)  IRcutClose = 0;  // Open
        else if (gIRCut == ALG_IRCUT_CLOSE) IRcutClose = 1;  // Close
        hn->gIRCut = gIRCut;
    }

    //Black and color night mode dynamic change
    if(gBWMode != hn->gBWMode) {
        if (gIRCut == ALG_IRCUT_AUTO && !IRcutClose) {
            DRV_imgsNDShutter(gIRCut, gBWMode);
            //DRV_imgsSetBWmode(gBWMode);
        }
        hn->gBWMode = gBWMode;
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
        hn->FPShigh = FPShigh;
        hn->gAePriorityMode = gAePriorityMode;
        printf("defaultFPS = %d\n", defaultFPS);
    }

    //Change IR-cut
    if (IRcutClose != hn->IRcutClose) {
        DRV_imgsNDShutter(IRcutClose, gBWMode);
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
        hn->Exp.Max = 1000000/hn->FPSmax;
        hn->Exp.New = (hn->Exp.Old/hn->Exp.Step)*hn->Exp.Step;
        hn->gFlicker = gFlicker;
    }

    //Seting Expouse
    if(hn->Exp.New != hn->Exp.Old) {
        smooth_change(&hn->Exp, fr);
        DRV_imgsSetEshutter(hn->Exp.Old, 0);
        //hn->Exp.Old = hn->Exp.New;
    }

    if(gHDR){
        //Config gamma correction tables
        if(hn->Rgain.New != hn->Rgain.Old || hn->Bgain.New != hn->Bgain.Old){
            dataG.tableSize = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SIZE_512;
            dataG.tableSrc  = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SELECT_RAM;
            dataG.bypassR = 0;
            dataG.bypassG = dataG.bypassR;
            dataG.bypassB = dataG.bypassR;
            dataG.tableR = hn->RGB[0];
            dataG.tableG = hn->RGB[1];
            dataG.tableB = hn->RGB[2];
            //Setup gamma tables
            if(CSL_ipipeSetGammaConfig(&gCSL_ipipeHndl, &dataG) != CSL_SOK)
                OSA_ERROR("Fail CSL_ipipeSetGammaConfig!!!\n");
        }
    } else {
        if(!raw){
            //ISIF gain seting
            if(hn->Rgain.New != hn->Rgain.Old || hn->Bgain.New != hn->Bgain.Old){
                smooth_change(&hn->Rgain, fr);
                smooth_change(&hn->Bgain, fr);
                DRV_isifSetDgain(512 , hn->Rgain.Old, hn->Bgain.Old, 512, 0);
                //hn->Rgain.Old = hn->Rgain.New;
                //hn->Bgain.Old = hn->Bgain.New;
            }

            if(hn->Offset.New != hn->Offset.Old) {
                smooth_change(&hn->Offset, fr);
                DRV_ipipeSetWbOffset(-hn->Offset.Old);
                //hn->Offset.Old = hn->Offset.New;
            }

            if(hn->GIFIF.New !=  hn->GIFIF.Old){
                smooth_change(&hn->GIFIF, fr);
                ipipeWb.gainR  = hn->GIFIF.Old;
                ipipeWb.gainGr = hn->GIFIF.Old;
                ipipeWb.gainGb = hn->GIFIF.Old;
                ipipeWb.gainB  = hn->GIFIF.Old;
                DRV_ipipeSetWb(&ipipeWb);
                //hn->GIFIF.Old = hn->GIFIF.New;
            }

            //Config RGB2RGB matrix for more gain
            if(hn->Grgb2rgb.New !=  hn->Grgb2rgb.Old){
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
                //if(DRV_ipipeSetRgb2Rgb2(&rgb2rgb) != CSL_SOK)
                //    OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");

                //hn->Grgb2rgb.Old = hn->Grgb2rgb.New;
            }
        }
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
    hn->Exp.Max = sensorExposureMax;
    hn->Exp.Min = sensorExposureMax;
    hn->Exp.Range.max = sensorExposureMax;
    hn->Exp.Range.min = stepSize;
    hn->Exp.Th = 10; //10%
    hn->Exp.Diff = 0;

    //printf("SIG_2A_config: hn->Exp.New = %d hn->Exp.Old = %d sensorExposureMax = %d gFlicker = %d\n",
    //       hn->Exp.New, hn->Exp.Old, sensorExposureMax, gFlicker);


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
    hn->GIFIF.Max = 512;
    hn->GIFIF.Min = 512;
    hn->GIFIF.Range.min = 512;
    hn->GIFIF.Range.max = 8180;
    hn->GIFIF.Th = 10; //10%
    hn->GIFIF.Diff = 0;

    //ISIF offset setup
    hn->Offset.Step = 1;
    hn->Offset.Old = 0;
    hn->Offset.New = 0;
    hn->Offset.Max = 0;
    hn->Offset.Min = 2000;
    hn->Offset.Range.min = 1;
    hn->Offset.Range.max = 4095;
    hn->Offset.Th = 10; //10%
    hn->Offset.Diff = 0;

    hn->Grgb2rgb.Old = 256;
    hn->Grgb2rgb.New = 256;
    hn->Grgb2rgb.Range.min = 1;
    hn->Grgb2rgb.Range.max = 512;

    //Y setup
    hn->Y.Step = 1;
    hn->Y.New = 1;
    hn->Y.Old = 1;
    hn->Y.Max = 1;
    hn->Y.Min = 4095;
    hn->Y.Range.min = 0;
    hn->Y.Range.max = 4095;
    hn->Y.Th = 10; //10%
    hn->Y.Diff = 0;

    hn->HmaxTh = 3200;
    hn->SatTh = hn->w*hn->h*3/100;

    hn->Hmax.HistC = 0;
    hn->Hmin.HistC = 0;
    for(i=0; i < HISTORY; i++) hn->Hmax.Hist[i] = 0;
    hn->Hmax.Avrg = 0;

    for(i=0; i < HISTORY; i++) hn->Hmin.Hist[i] = 0;
    hn->Hmin.Avrg = 0;

    //First value of dymanic parameters
    hn->gAePriorityMode = 4; //gAePriorityMode;
    hn->gIRCut = gIRCut;
    hn->gBWMode = gBWMode;
    hn->gFlicker = 4;
    hn->IRcutClose = IRcutClose; //IR-cut 1-open, 0 - close
    hn->FPShigh = FPShigh; //FPS 1-high, 0 - low
    hn->FPSmax = defaultFPS;
    hn->FPScur = defaultFPS;
    ALG_SIG_config(gSIG_Obj.handle_aewbf);

    return 0;
}

