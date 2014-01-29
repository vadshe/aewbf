#include "alg_aewb_priv.h"
#include "alg_ti_aewb_priv.h"
#include "alg_ti_flicker_detect.h"
#include "ae_ti.h"
#include "awb_ti.h"
#include "ae_appro.h"
#include "awb_appro.h"

#include "Appro_aewb.h"
#include "TI_aewb.h"
#include "imageTune.h"
#include "imageTuneCmdHandler.h"

#include <drv_gpio.h>
#include <drv_ipipe.h>
#include <drv_isif.h>

#include "aewbf_sig.h"
#include "iaewbf_sig.h"
#include "alg_aewbf.h"

ALG_AewbObj gALG_aewbObj;
ALG_AewbfObj gSIG_Obj;

AEW_EXT_PARAM Aew_ext_parameter;
extern AWB_OUTPUT_DATA      gOutAWBData;
extern DRV_IpipeObj gDRV_ipipeObj;      //For boxcar
extern CSL_IpipeObj gCSL_ipipeHndl;   //For gamma and rgb2rgb
extern int gFlicker;

extern int gAePriorityMode, gBWMode, gDayNight, gIRCut, defaultFPS;
extern int IRcutClose, FPShigh;



#define GIO_AUTO_IRIS	(83)

#define FD_DEBUG_MSG
#define ALG_AEWB_DEBUG

typedef struct {
  Uint32 awbNumWinH;
  Uint32 awbNumWinV;
  Uint32 aewbNumSPCInWin;
  Uint32 awbMiscData[16];
  IAEWB_Rgb *awbRgbData;
  OSA_MutexHndl statusLock;

}ALG_AewbData_AwbSrc;

static ALG_AewbData_AwbSrc gITTAwb;

typedef struct {
	int y;
	int u;
	int v;
}IAWB_Yuv;

void ALG_aewbConvert_RGB_YUV(IAEWB_Rgb *rgbData, int pix_in_pax, int awb_h3a_paxels, IAEWB_Rgb *norm_rgbData, IAWB_Yuv *yuvData);

static int aewbFrames = 0;
static int flicker_detect_complete = 0;
static int *g_flickerMem = NULL; //algorithm persistent memory
static IAEWB_Rgb *rgbData = NULL;
//--- for Y buffer - temorary
static unsigned int *YDataBuff = NULL;
static unsigned int YDataBuffV;
static unsigned int YDataBuffH;
//---
static aewDataEntry *aew_data = NULL;
static int sensorGain = 1000;
static int sensorExposure = 8333;
static int env_50_60Hz = -1;
static AWB_PARAM ipipe_awb_gain = {
    1024 , //1536,
    1024,
    1024,
    1024, //1536,
    256,
    0
};

static unsigned int HISTgain_mid = 256;
static unsigned int HISTgain_minmax = 256;
static unsigned int HISTmin = 0;
static unsigned int HISTmode = 2;
static int lowlight = 0;
static int sensorExposureMax = 33333;
static int HighGain = 0;
static int OV271X_gain = 0;

unsigned int * GetYBuffDATA(unsigned int *Vsize, unsigned int *Hsize)
{
    if(YDataBuff)
    {
        *Vsize = YDataBuffV;
        *Hsize = YDataBuffH;
        return YDataBuff;
    } else
        return NULL;
}

int ALG_aewbPlatformCheck( void )
{
	FILE *pfile = NULL;
	char tempbuff[100];
	char filename[]="/proc/version";
	int	 ret = 0;
	char *pStr = NULL;

	pfile = fopen(filename,"r");
	if( pfile == NULL )
	{
		ret = -1;
		goto CHECK_END;
	}

	if( fread(tempbuff, sizeof(tempbuff),1,pfile) <= 0 )
	{
		ret = -1;
		goto CHECK_END;
	}

	tempbuff[sizeof(tempbuff)-1]='\0';

	/*Linux version 2.6.10_mvl401_IPNC_1.0.0.0 (root@localhost.localdomain) (gcc ve.....*/

	pStr = strstr(tempbuff,"IPNC");
	if( pStr != NULL )
	{
		ret = 1;
		goto CHECK_END;
	}
	pStr = strstr(tempbuff,"ipnc");
	if( pStr != NULL )
	{
		ret = 1;
		goto CHECK_END;
	}

	pStr = strstr(tempbuff,"EVM");
	if( pStr != NULL )
	{
		ret = 0;
		goto CHECK_END;
	}
	pStr = strstr(tempbuff,"evm");
	if( pStr != NULL )
	{
		ret = 0;
		goto CHECK_END;
	}


CHECK_END:

	if( pStr )
	{
		fprintf(stderr,"%s \n",pStr);
	}
	if( pfile )
	{
		fclose(pfile);
	}
	return ret;
}

int ALG_aewbCheckAutoIris(void)
{
	int ret = 0;

	DRV_gpioSetMode(GIO_AUTO_IRIS, CSL_GPIO_INPUT);

	if( ALG_aewbPlatformCheck() == 1 )
	{
		ret = DRV_gpioGet(GIO_AUTO_IRIS);
		OSA_printf("IPNC AUTO_IRIS = %d \n",!ret);
		return !ret;
	}
	else {
		ret = MANUAL_IRIS;
		OSA_printf("EVM AUTO_IRIS = %d \n",ret);
		return ret;
	}
}

static void ALG_SIG_config(IALG_Handle handle)
{
    int i, vl0, vl1;
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;

    Uint32 tables[512];
    CSL_IpipeGammaConfig dataG;
    CSL_IpipeRgb2RgbConfig rgb2rgb;
    DRV_IpipeWb ipipeWb;

    DRV_imgsSetFramerate(defaultFPS); //Max FPS frame rate
    DRV_imgsSetEshutter(hn->Exp.Range.max, 0); //Max expouse
    ALG_aewbSetSensorDcsub(190); //Offset for SONY IMX136
    //ALG_aewbSetSensorDcsub(170); //170 Offset for Aptina MT9P006
    //DRV_imgsNDShutterInit();
    //DRV_imgsNDShutter(1, gBWMode); //Close IR-cut

    //Config contrast and Brightness
    //DRV_ipipeSetYoffet((pParm->yuv_adj_brt-128));
    //DRV_ipipeSetContrastBrightness(pParm->yuv_adj_ctr, 0x0);

    //Config gamma correction tables

    dataG.tableSize = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SIZE_512;
    dataG.tableSrc  = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SELECT_RAM;
    dataG.bypassR = 0;
    dataG.bypassG = 0;
    dataG.bypassB = 0;
    dataG.tableR = tables;
    dataG.tableG = tables;
    dataG.tableB = tables;

    //Liner gamma tables
    vl0 = 0;
    for(i=0; i < 512; i++){
        vl1 = i<<1;
        tables[i] = (vl0<<10) | (vl1 - vl0);
        vl0 = vl1;
    }

    if(CSL_ipipeSetGammaConfig(&gCSL_ipipeHndl, &dataG) != CSL_SOK)
        OSA_ERROR("Fail CSL_ipipeSetGammaConfig!!!\n");



    //Config RGB2RGB matrix
    rgb2rgb.matrix[0][0] = 256;
    rgb2rgb.matrix[0][1] = 0;
    rgb2rgb.matrix[0][2] = 0;

    rgb2rgb.matrix[1][0] = 0;
    rgb2rgb.matrix[1][1] = 256;
    rgb2rgb.matrix[1][2] = 0;

    rgb2rgb.matrix[2][0] = 0;
    rgb2rgb.matrix[2][1] = 0;
    rgb2rgb.matrix[2][2] = 256;

    rgb2rgb.offset[0]    = 0;
    rgb2rgb.offset[1]    = 0;
    rgb2rgb.offset[2]    = 0;

    if(DRV_ipipeSetRgb2Rgb(&rgb2rgb) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
    if(DRV_ipipeSetRgb2Rgb2(&rgb2rgb) != CSL_SOK)
        OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");


    //Config isif white balance gain
    //DRV_isifSetDgain(512, 512, 512, 512, 0);
    DRV_isifSetDgain(hn->RGBgain[1], hn->RGBgain[0], hn->RGBgain[2], hn->RGBgain[1], 0);


    //Config ipipe gains and offset

    ipipeWb.gainR  = hn->GIFIF.New;
    ipipeWb.gainGr = hn->GIFIF.New;
    ipipeWb.gainGb = hn->GIFIF.New;
    ipipeWb.gainB  = hn->GIFIF.New;

    DRV_ipipeSetWbOffset(0);
    DRV_ipipeSetWb(&ipipeWb);

    //DRV_imgsSetAEPriority(0);
    //ALG_aewbSetNDShutterOnOff(1);
    //ALG_aewbSetNDShutterOnOff(0);
    //ALG_aewbSetNDShutterOnOff(1);

}

void SIG2A_applySettings(void)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)gSIG_Obj.handle_aewbf;
    CSL_IpipeGammaConfig dataG;
    DRV_IpipeWb ipipeWb;
    CSL_IpipeRgb2RgbConfig rgb2rgb;

    OSA_printf("SIG2A_apply: gAePriorityMode = %d gBWMode = %d gDayNight =%d gIRCut = %d hn->gIRCut = %d defaultFPS = %d IRcutClose = %d hn->IRcutClose = %d FPShigh = %d hn->FPShigh = %d\n",
               gAePriorityMode, gBWMode, gDayNight, gIRCut, hn->gIRCut, defaultFPS, IRcutClose, hn->IRcutClose, FPShigh, hn->FPShigh);

    //IR-cut dynamic change
    if(gIRCut != hn->gIRCut) {
        if      (gIRCut == ALG_IRCUT_OPEN)  IRcutClose = 0;  // Open
        else if (gIRCut == ALG_IRCUT_CLOSE) IRcutClose = 1;  // Close
        hn->gIRCut = gIRCut;
    }

    //Black and color night mode dynamic change
    if(gBWMode != hn->gBWMode) {
        if (gIRCut == ALG_IRCUT_AUTO || gIRCut == ALG_IRCUT_OPEN) {
            DRV_imgsNDShutter(gIRCut, gBWMode);
            //DRV_imgsSetBWmode(gBWMode);
        }
        hn->gBWMode = gBWMode;
    }

    //Change FPS
    if (FPShigh != hn->FPShigh || gAePriorityMode != hn->gAePriorityMode) {
        if(FPShigh == 0 ){
            //Go to low FPS
            if (gAePriorityMode == ALG_FPS_LOW) {
                DRV_imgsSetFramerate(defaultFPS>>1);
                hn->Exp.Range.max = 1000000/(defaultFPS>>1);
                hn->Exp.New  = hn->Exp.Range.max;
            }
            else if (gAePriorityMode == ALG_FPS_5FPS){
                DRV_imgsSetFramerate(5);
                hn->Exp.Range.max = 200000;
                hn->Exp.New  = hn->Exp.Range.max;
            } else {
                DRV_imgsSetFramerate(defaultFPS);
                hn->Exp.Range.max = 1000000/defaultFPS;
                hn->Exp.New  = hn->Exp.Range.max;
            }
            //else if (gAePriorityMode == ALG_FPS_NONE)   DRV_imgsSetFramerate(defaultFPS);
        } else {
            //Go to high FPS
            DRV_imgsSetFramerate(defaultFPS);
            hn->Exp.Range.max = 1000000/defaultFPS;
            hn->Exp.New  = hn->Exp.Range.max;
        }
        hn->FPShigh = FPShigh;
        hn->gAePriorityMode = gAePriorityMode;
    }

    //Change IR-cut
    if (IRcutClose != hn->IRcutClose) {
        DRV_imgsNDShutter(IRcutClose, gBWMode);
        hn->IRcutClose = IRcutClose;
    }

    //Config gamma correction tables
    if(hn->GISIF.New != hn->GISIF.Old || hn->Exp.New != hn->Exp.Old){
        dataG.tableSize = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SIZE_512;
        dataG.tableSrc  = CSL_IPIPE_GAMMA_CORRECTION_TABLE_SELECT_RAM;
        dataG.bypassR = 0;
        dataG.bypassG = 0;
        dataG.bypassB = 0;
        dataG.tableR = hn->RGB[0].hist;
        dataG.tableG = hn->RGB[1].hist;
        dataG.tableB = hn->RGB[2].hist;
        //Setup gamma tables
        if(CSL_ipipeSetGammaConfig(&gCSL_ipipeHndl, &dataG) != CSL_SOK)
            OSA_ERROR("Fail CSL_ipipeSetGammaConfig!!!\n");
    }

    //Seting Expouse
    if(hn->Exp.New != hn->Exp.Old) {
        DRV_imgsSetEshutter(hn->Exp.New, 0);
        hn->Exp.Old = hn->Exp.New;
    }

    //ISIF gain seting
    if(hn->GISIF.New != hn->GISIF.Old) {
        if(hn->RGBgain[0] != hn->GISIF.Range.max && hn->RGBgain[1] != hn->GISIF.Range.max && hn->RGBgain[2] != hn->GISIF.Range.max ){
            hn->RGBgain[1] = hn->GISIF.New;
            hn->RGBgain[0] = hn->GISIF.New*hn->RGBgain[0]/hn->GISIF.Old;
            hn->RGBgain[2] = hn->GISIF.New*hn->RGBgain[2]/hn->GISIF.Old;
        }
        hn->GISIF.Old = hn->GISIF.New;
        //OSA_printf("SIG2A_applySettings: new = %d old = %d Rgain = %d Ggain = %d Bgain = %d\n",
        //           hn->GISIF.New, hn->GISIF.Old, hn->RGBgain[0], hn->RGBgain[1], hn->RGBgain[2]);
    }
    DRV_isifSetDgain(hn->RGBgain[1] , hn->RGBgain[0], hn->RGBgain[2], hn->RGBgain[1], 0);

    if(hn->Offset.New != hn->Offset.Old) {
        //ALG_aewbSetSensorDcsub(hn->Offset.New);
        //ALG_aewbSetSensorDcsub(170);
        hn->Offset.Old = hn->Offset.New;
    }

    //DRV_isifSetDgain(512, 512, 512, 512, 0);
    //gain = hn->Gain.New - hn->RGBgain[hn->maxi];
    //DRV_isifSetDgain(gain + hn->RGBgain[1] , gain + hn->RGBgain[0], gain + hn->RGBgain[2], gain + hn->RGBgain[1], 0);
    //gain = hn->Gain.New - hn->RGBgain[hn->maxi];




    //gain = (4000<<9)/(hn->Hmax[0] - hn->Hmin[0]);
    if(hn->GIFIF.New !=  hn->GIFIF.Old){
        hn->GIFIF.Old = hn->GIFIF.New;
        ipipeWb.gainR  = hn->GIFIF.New;
        ipipeWb.gainGr = hn->GIFIF.New;
        ipipeWb.gainGb = hn->GIFIF.New;
        ipipeWb.gainB  = hn->GIFIF.New;
        DRV_ipipeSetWb(&ipipeWb);
        hn->GIFIF.Old = hn->GIFIF.New;
    }

    //offset = hn->Hmin[0] > 2047 ? 2047 : hn->Hmin[0];
    //OSA_printf("SIG2A_applySettings: ofset = %d gain = %d\n", -offset, gain);
    //DRV_ipipeSetWbOffset(-offset);


    //Config RGB2RGB matrix
    if(0){
        rgb2rgb.matrix[0][0] = 256;
        rgb2rgb.matrix[0][1] = 0;
        rgb2rgb.matrix[0][2] = 0;

        rgb2rgb.matrix[1][0] = 0;
        rgb2rgb.matrix[1][1] = 256;
        rgb2rgb.matrix[1][2] = 0;

        rgb2rgb.matrix[2][0] = 0;
        rgb2rgb.matrix[2][1] = 0;
        rgb2rgb.matrix[2][2] = 256;

        rgb2rgb.offset[0]    = 0;
        rgb2rgb.offset[1]    = 0;
        rgb2rgb.offset[2]    = 0;

        if(DRV_ipipeSetRgb2Rgb(&rgb2rgb) != CSL_SOK)
            OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
        if(DRV_ipipeSetRgb2Rgb2(&rgb2rgb) != CSL_SOK)
            OSA_ERROR("Fail DRV_ipipeSetRgb2Rgb2!!!\n");
    }

}


int SIG_2A_config(IALG_Handle handle)
{
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    int i, stepSize;

    i = 0; aewbFrames = 0; stepSize = 1;

    OSA_printf("SIG_2A_config: start\n");
    /*
    if (gSIG_Obj.sensorFps == 25) {
        ALG_aewbSetSensor50_60Hz(1); // 25FPS
    } else {
        ALG_aewbSetSensor50_60Hz(0); // 30FPS
    }
    */
    if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0) 	// AR0331 sensor
    {
        if(gFlicker == VIDEO_NTSC)			// 60 Hz flicker
        {
            if (gSIG_Obj.sensorFps == 20)
            {
                stepSize = 6000; 	// Exposure stepsize
                //if (sensorExposureMax > 33333)
                sensorExposureMax = 48000;
            }
            else if (gSIG_Obj.sensorFps == 25){
                stepSize = 7500; 	// Exposure stepsize
                sensorExposureMax = 40000;
            } else if (gSIG_Obj.sensorFps == 30){
                stepSize = 9000; 	// Exposure stepsize
                sensorExposureMax = 33333;
            }
        } else if(gFlicker == VIDEO_PAL) { // 50 Hz flicker
            if (gSIG_Obj.sensorFps == 20)
            {
                stepSize = 7200; 	// Exposure stepsize
                //if (sensorExposureMax > 33333)
                sensorExposureMax = 43200;
            }
            else if (gSIG_Obj.sensorFps == 25) {
                stepSize = 9000; 	// Exposure stepsize
                sensorExposureMax = 40000;
            } else if (gSIG_Obj.sensorFps == 30) {
                stepSize = 10800; 	// Exposure stepsize
                sensorExposureMax = 33333;
            }
        } else {
            stepSize = 1;
        }
    } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9M034_720P") == 0) // MT9M034 sensor
    {
        if(gFlicker == VIDEO_NTSC)			// 60 Hz flicker
        {
            if (gSIG_Obj.sensorFps == 25){
                stepSize = 7084; 	// Exposure stepsize
                sensorExposureMax = 40000;
            } else if (gSIG_Obj.sensorFps == 30){
                stepSize = 8500; 	// Exposure stepsize
                sensorExposureMax = 33333;
            }
        } else if(gFlicker == VIDEO_PAL)	// 50 Hz flicker
        {
            if (gSIG_Obj.sensorFps == 25) {
                stepSize = 8500; 	// Exposure stepsize
                sensorExposureMax = 40000;
            } else if (gSIG_Obj.sensorFps == 30) {
                stepSize = 10200; 	// Exposure stepsize
                sensorExposureMax = 33333;
            }
        } else
        {
            stepSize = 1;
        }
    } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0) // MT9M031 sensor
    {
        if(gFlicker == VIDEO_NTSC)	{		// 60 Hz flicker
            stepSize = 8333; 	// Exposure stepsize
            sensorExposureMax = 33333;
        } else if(gFlicker == VIDEO_PAL) {	// 50 Hz flicker
            stepSize = 10000; 	// Exposure stepsize
            sensorExposureMax = 40000;
        } else {
            stepSize = 1;
        }
    } else if (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0) // IMX136 sensor
    {
        if(gFlicker == VIDEO_NTSC) {		// 60 Hz flicker
            stepSize = 8333; 	// Exposure stepsize
            sensorExposureMax = 33333;
        } else if(gFlicker == VIDEO_PAL) {	// 50 Hz flicker
            stepSize = 10000; 	// Exposure stepsize
            sensorExposureMax = 40000;
        } else {
            stepSize = 1;
        }
    } else
    {
        stepSize = 1;
    }
    /*
    DP.numRanges ++;

    DP.sensorGainRange[i].min = 1000;
    DP.sensorGainRange[i].max = 1000;
    DP.ipipeGainRange[i].min = 0;
    DP.ipipeGainRange[i].max = 8191;
    //DP.isifGainRange[i].min = 0;
    //DP.isifGainRange[i].max = 4095;
    DP.YRange.min = 0;
    DP.YRange.max = 0;
    */

    //DP.Ythresh = 10;
    //DP.ExpStep = stepSize;

    sensorGain = 1000;
    lowlight = DRV_imgsGetAEPriority();


    //Exposure setup
    hn->Exp.Step = stepSize;
    hn->Exp.Old = sensorExposureMax-1;
    hn->Exp.New = sensorExposureMax;
    hn->Exp.Max = sensorExposureMax;
    hn->Exp.Min = sensorExposureMax;
    hn->Exp.Range.max = sensorExposureMax;
    hn->Exp.Range.min = stepSize;
    hn->Exp.Th = 10; //10%
    hn->Exp.Diff = 0;

    //ISIF gain setup
    hn->GISIF.Step = 16;
    hn->GISIF.Old = 511;
    hn->GISIF.New = 512;
    hn->GISIF.Max = 512;
    hn->GISIF.Min = 512;
    hn->GISIF.Range.min = 100;
    hn->GISIF.Range.max = 4095;
    hn->GISIF.Th = 10; //10%
    hn->GISIF.Diff = 0;

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
    hn->Offset.New = 1;
    hn->Offset.Max = 1;
    hn->Offset.Min = 1;
    hn->Offset.Range.min = 1;
    hn->Offset.Range.max = 4095;
    hn->Offset.Th = 10; //10%
    hn->Offset.Diff = 0;

    //Y setup
    hn->Y.Step = 1;
    hn->Y.New = 1;
    hn->Y.Old = 1;
    hn->Y.Max = 1;
    hn->Y.Min = 1;
    hn->Y.Range.min = 0;
    hn->Y.Range.max = 4095;
    hn->Y.Th = 10; //10%
    hn->Y.Diff = 0;

    hn->RGBgain[0] = 512;
    hn->RGBgain[1] = 512;
    hn->RGBgain[2] = 512;

    //For Aptina MT9P006 5 mpix
    hn->HmaxTh = 3800;
    hn->HminTh = 0;
    hn->HhalfTh = 100;
    hn->Hhalf = 0;

    hn->RGB[0].MaxTh = 3700;
    hn->RGB[1].MaxTh = 3700;
    hn->RGB[2].MaxTh = 3700;

    //First value of dymanic parameters
    hn->gAePriorityMode = gAePriorityMode;
    hn->gIRCut = gIRCut;
    hn->gBWMode = gBWMode;
    hn->IRcutClose = IRcutClose; //IR-cut 1-open, 0 - close
    hn->FPShigh = FPShigh; //FPS 1-high, 0 - low
    /*
    retval = IAEWBF_SIG.control((IAEWBF_Handle)gSIG_Obj.handle_aewbf, IAEWBF_CMD_SET_CONFIG, &DP, NULL);
    if(retval == -1) {
        OSA_ERROR("IAEWBF_SIG.control\n");
        return retval;
    }
    */
    ALG_SIG_config(gSIG_Obj.handle_aewbf);

    return 0;
}

void *ALG_aewbCreate(ALG_AewbCreate *create)
{
  IAE_Params aeParams;
  IAWB_Params awbParams;
  IAEWBF_Params aewbfParams;

  int numMem;
  int retval;

  memset(&gALG_aewbObj, 0, sizeof(gALG_aewbObj));
  memset(&Aew_ext_parameter, 0, sizeof(Aew_ext_parameter));

  rgbData = calloc(sizeof(IAEWB_Rgb), create->pH3aInfo->aewbNumWinH * create->pH3aInfo->aewbNumWinV);
  //----
  YDataBuffH = create->pH3aInfo->aewbNumWinH;
  YDataBuffV = create->pH3aInfo->aewbNumWinV;
  YDataBuff = calloc(sizeof(unsigned int), YDataBuffH * YDataBuffV);
  //OSA_printf("YDataBuffH %d : YDataBuffV %d: %x \n", YDataBuffH, YDataBuffV, YDataBuff);
  //----
  aew_data = calloc(sizeof(aewDataEntry), (create->pH3aInfo->aewbNumWinH * create->pH3aInfo->aewbNumWinV + 7) >> 3);
  //g_flickerMem = calloc(sizeof(int), 6*1024);

  gALG_aewbObj.sensorMode 		= (create->sensorMode & 0xFF);

  gALG_aewbObj.vsEnable 		= (create->sensorMode & DRV_IMGS_SENSOR_MODE_VSTAB) ? 1 : 0;
  gALG_aewbObj.aewbVendor   	= create->aewbVendor;
  gALG_aewbObj.reduceShutter   	= create->reduceShutter;
  gALG_aewbObj.saldre   		= create->saldre;
  gALG_aewbObj.AGainEnable      = create->AGainEnable;
  gALG_aewbObj.DGainEnable      = create->DGainEnable;
  gALG_aewbObj.ALTM             = create->ALTM;
  gALG_aewbObj.sensorFps		= create->sensorFps;
  gALG_aewbObj.numEncodes		= create->numEncodes;
  gALG_aewbObj.afEnable         = create->afEnable;

  OSA_printf("AEWB: sensor FPS = %d %d\n",create->sensorFps, create->numEncodes);

  if (gALG_aewbObj.ALTM == 1)
  {
      if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
      {
          HISTmode = 8; // ALTM mode
      }
  }

  Aew_ext_parameter.GAIN_SETUP            = ALG_aewbSetSensorGain        ;
  Aew_ext_parameter.SHUTTER_SETUP         = ALG_aewbSetSensorExposure    ;
  Aew_ext_parameter.AWB_SETUP             = ALG_aewbSetIpipeWb           ;
  Aew_ext_parameter.DCSUB_SETUP           = ALG_aewbSetSensorDcsub       ;
  Aew_ext_parameter.BIN_SETUP             = ALG_aewbSetSensorBin         ;
  Aew_ext_parameter.RGB2RGB_SETUP         = ALG_aewbSetRgb2Rgb           ;
  Aew_ext_parameter.MEDIAN_SETUP          = ALG_aewbSetOtfCorrect        ;
  Aew_ext_parameter.EDGE_SETUP            = ALG_aewbSetEdgeEnhancement   ;
  Aew_ext_parameter.BRT_CTR_SET           = ALG_aewbSetContrastBrightness;
  Aew_ext_parameter.BINNING_SKIP_SETUP    = ALG_aewbSetSensorBinSkip     ;
  Aew_ext_parameter.ENV_50_60HZ_SETUP     = ALG_aewbSetSensor50_60Hz     ;
  Aew_ext_parameter.SENSOR_FRM_RATE_SETUP = ALG_aewbSetSensorFrameRate   ;
  Aew_ext_parameter.SENSOR_GET_FPS		  = ALG_aewbGetSensorFrameRate   ;
  Aew_ext_parameter.SENSOR_AE_PRIORITY    = ALG_aewbSetAEPriorityMode    ;
  Aew_ext_parameter.ND_SETUP              = ALG_aewbSetNDShutterOnOff    ;
  Aew_ext_parameter.SENSOR_WB_MODE        = ALG_aewbSetWBMode            ;
  Aew_ext_parameter.AF_ENABLE             = ALG_aewbAFEnable             ;
  Aew_ext_parameter.H3ABuffer         = NULL;
  Aew_ext_parameter.sensor_frame_rate = 0;
  Aew_ext_parameter.ipipe_dgain_base  = 1;
  Aew_ext_parameter.special_mode      = 0;
  Aew_ext_parameter.auto_iris         = ALG_aewbCheckAutoIris();
  Aew_ext_parameter.day_night         = AE_DAY;
  Aew_ext_parameter.awb_mode          = AWB_AUTO;
  Aew_ext_parameter.saturation        = 128;
  Aew_ext_parameter.blc               = BACKLIGHT_NORMAL;
  Aew_ext_parameter.sharpness         = 128;
  Aew_ext_parameter.brightness        = 128;
  Aew_ext_parameter.contrast          = 128;
  Aew_ext_parameter.aew_enable        = AEW_ENABLE;
  Aew_ext_parameter.binning_mode      = create->aewbBinEnable?SENSOR_BINNING:SENSOR_SKIP;
  Aew_ext_parameter.NFGain            = 0;

  gITTAwb.awbNumWinH = create->pH3aInfo->aewbNumWinH;
  gITTAwb.awbNumWinV = create->pH3aInfo->aewbNumWinV;
  gITTAwb.aewbNumSPCInWin= create->pH3aInfo->aewbNumSamplesPerColorInWin;
  gITTAwb.awbRgbData = rgbData;
  OSA_mutexCreate(&gITTAwb.statusLock);

  if(create->aewbVendor == ALG_AEWB_ID_APPRO) {
	  //Initial AE
	  gALG_aewbObj.weight = APPRO_WEIGHTING_MATRIX;
	  aeParams.size = sizeof(aeParams);
	  aeParams.numHistory = 0;
	  aeParams.numSmoothSteps = 1;
	  numMem = AE_APPRO_AE.ialg.algAlloc((IALG_Params *)&aeParams, NULL, gALG_aewbObj.memTab_ae);
	  while(numMem > 0){
		gALG_aewbObj.memTab_ae[numMem-1].base = malloc(gALG_aewbObj.memTab_ae[numMem-1].size);
		numMem --;
	  }

	  gALG_aewbObj.handle_ae = (IALG_Handle)gALG_aewbObj.memTab_ae[0].base;
	  retval = AE_APPRO_AE.ialg.algInit(gALG_aewbObj.handle_ae, gALG_aewbObj.memTab_ae, NULL, (IALG_Params *)&aeParams);
	  if(retval == -1) {
		OSA_ERROR("AE_APPRO_AE.ialg.algInit()\n");
		return NULL;
	  }

	  //Initial AWB
	  awbParams.size = sizeof(awbParams);
	  awbParams.numHistory = 0;
	  awbParams.numSmoothSteps = 1;
	  numMem = AWB_APPRO_AWB.ialg.algAlloc((IALG_Params *)&awbParams, NULL, gALG_aewbObj.memTab_awb);
	  while(numMem > 0){
		gALG_aewbObj.memTab_awb[numMem-1].base = malloc(gALG_aewbObj.memTab_awb[numMem-1].size);
		numMem --;
	  }

	  gALG_aewbObj.handle_awb = (IALG_Handle)gALG_aewbObj.memTab_awb[0].base;
	  retval = AWB_APPRO_AWB.ialg.algInit(gALG_aewbObj.handle_awb, gALG_aewbObj.memTab_awb, NULL, (IALG_Params *)&awbParams);
	  if(retval == -1) {
		OSA_ERROR("AWB_APPRO_AWB.ialg.algInit()\n");
		return NULL;
	  }

	  gALG_aewbObj.IAEWB_StatMatdata.winCtVert  = create->pH3aInfo->aewbNumWinV;
	  gALG_aewbObj.IAEWB_StatMatdata.winCtHorz  = create->pH3aInfo->aewbNumWinH;
	  gALG_aewbObj.IAEWB_StatMatdata.pixCtWin   = create->pH3aInfo->aewbNumSamplesPerColorInWin;

	  if(create->vnfDemoCfg)
	  {
		IAE_DynamicParam.exposureTimeStepSize = 20;
		IAE_DynamicParam.targetBrightness = 100;
		IAE_DynamicParam.targetBrightnessRange.min = 90;
		IAE_DynamicParam.targetBrightnessRange.max = 110;
		memcpy( (void *)&IAE_DynamicParam.exposureTimeRange,
		  (void *)&APPRO_shutter_List_NF,
		  sizeof(IAE_DynamicParam.exposureTimeRange) );

		memcpy( (void *)&IAE_DynamicParam.sensorGainRange,
		  (void *)&APPRO_agc_List_NF,
		  sizeof(IAE_DynamicParam.sensorGainRange) );

		memcpy( (void *)&IAE_DynamicParam.ipipeGainRange,
		  (void *)&APPRO_dgain_List_NF,
		  sizeof(IAE_DynamicParam.ipipeGainRange) );

	  }else if(gALG_aewbObj.sensorMode <= DRV_IMGS_SENSOR_MODE_800x600) {

		memcpy( (void *)&IAE_DynamicParam.exposureTimeRange,
		  (void *)&APPRO_shutter_List_480P_Bin,
		  sizeof(IAE_DynamicParam.exposureTimeRange) );

		memcpy( (void *)&IAE_DynamicParam.sensorGainRange,
		  (void *)&APPRO_agc_List_480P,
		  sizeof(IAE_DynamicParam.sensorGainRange) );

		memcpy( (void *)&IAE_DynamicParam.ipipeGainRange,
		  (void *)&APPRO_dgain_List_480P,
		  sizeof(IAE_DynamicParam.ipipeGainRange) );

	  } else {

		memcpy( (void *)&IAE_DynamicParam.exposureTimeRange,
		  (void *)&APPRO_shutter_List_720P,
		  sizeof(APPRO_shutter_List_720P) );

		memcpy( (void *)&IAE_DynamicParam.sensorGainRange,
		  (void *)&APPRO_agc_List_720P,
		  sizeof(APPRO_agc_List_720P) );

		memcpy( (void *)&IAE_DynamicParam.ipipeGainRange,
		  (void *)&APPRO_dgain_List_720P,
		  sizeof(APPRO_dgain_List_720P) );
	  }

	  memcpy( (void *)&gALG_aewbObj.AE_InArgs.statMat,
		(void *)&gALG_aewbObj.IAEWB_StatMatdata,
		sizeof(IAEWB_StatMat) );

	  memcpy( (void *)&gALG_aewbObj.AWB_InArgs.statMat,
		(void *)&gALG_aewbObj.IAEWB_StatMatdata,
		sizeof(IAEWB_StatMat) );


	  IAE_DynamicParam.size = sizeof(IAE_DynamicParams);
	  retval = AE_APPRO_AE.control((IAE_Handle)gALG_aewbObj.handle_ae, IAE_CMD_SET_CONFIG, &IAE_DynamicParam, NULL);
	  if(retval == -1) {
		OSA_ERROR("AE_APPRO_AE.control()\n");
		return NULL;
	  }

	  IAWB_DynamicParam.size = sizeof(IAWB_DynamicParams);
	  retval = AWB_APPRO_AWB.control((IAWB_Handle)gALG_aewbObj.handle_awb, IAWB_CMD_SET_CONFIG, &IAWB_DynamicParam, NULL);
	  if(retval == -1) {
		OSA_ERROR("AWB_APPRO_AWB.control()\n");
		return NULL;
	  }

	  CONTROL_DRIVER_initial(
			(IAE_Handle)gALG_aewbObj.handle_ae,
			(IAWB_Handle)gALG_aewbObj.handle_awb,
			&gALG_aewbObj.AE_OutArgs,
			&gALG_aewbObj.AWB_OutArgs
			);
  }
  else if(create->aewbVendor == ALG_AEWB_ID_TI) {
      TI_2A_init_tables(create->pH3aInfo->aewbNumWinH, create->pH3aInfo->aewbNumWinV);
	  //Initial AE
	  gALG_aewbObj.weight = TI_WEIGHTING_MATRIX;
	  aeParams.size = sizeof(aeParams);
	  aeParams.numHistory = 10;
	  aeParams.numSmoothSteps = 1;
	  numMem = AE_TI_AE.ialg.algAlloc((IALG_Params *)&aeParams, NULL, gALG_aewbObj.memTab_ae);
	  while(numMem > 0){
		gALG_aewbObj.memTab_ae[numMem-1].base = malloc(gALG_aewbObj.memTab_ae[numMem-1].size);
		numMem --;
	  }

	  gALG_aewbObj.handle_ae = (IALG_Handle)gALG_aewbObj.memTab_ae[0].base;
	  retval = AE_TI_AE.ialg.algInit(gALG_aewbObj.handle_ae, gALG_aewbObj.memTab_ae, NULL, (IALG_Params *)&aeParams);
	  if(retval == -1) {
		OSA_ERROR("AE_TI_AE.ialg.algInit()\n");
		return NULL;
	  }

	  //Initial AWB
	  awbParams.size = sizeof(awbParams);
	  awbParams.numHistory = 6;
	  numMem = AWB_TI_AWB.ialg.algAlloc((IALG_Params *)&awbParams, NULL, gALG_aewbObj.memTab_awb);
	  while(numMem > 0){
		gALG_aewbObj.memTab_awb[numMem-1].base = malloc(gALG_aewbObj.memTab_awb[numMem-1].size);
		numMem --;
	  }

	  gALG_aewbObj.handle_awb = (IALG_Handle)gALG_aewbObj.memTab_awb[0].base;
	  retval = AWB_TI_AWB.ialg.algInit(gALG_aewbObj.handle_awb, gALG_aewbObj.memTab_awb, NULL, (IALG_Params *)&awbParams);
	  if(retval == -1) {
		OSA_ERROR("AWB_TI_AWB.ialg.algInit()\n");
		return NULL;
	  }

	  gALG_aewbObj.IAEWB_StatMatdata.winCtVert  = create->pH3aInfo->aewbNumWinV;
	  gALG_aewbObj.IAEWB_StatMatdata.winCtHorz  = create->pH3aInfo->aewbNumWinH;
	  gALG_aewbObj.IAEWB_StatMatdata.pixCtWin   = create->pH3aInfo->aewbNumSamplesPerColorInWin;

	  retval = TI_2A_config(1, create->saldre);
	  if(retval == -1) {
		  return NULL;
	  }

	  /* setup initial ipipe gains */
	  ALG_aewbSetIpipeWb(&ipipe_awb_gain, gALG_aewbObj.DGainEnable, lowlight);
	  ALG_aewbSetSensorDcsub(172);
	  ALG_aewbSetSensorGain(sensorGain);
	  TI_2A_SetEEValues(create->shiftValue);
  }
  else if(create->aewbVendor == ALG_AEWB_ID_SIG) {
      OSA_printf("ALG_aewbCreate: start\n");
      memset(&gSIG_Obj, 0, sizeof(gSIG_Obj));


      gSIG_Obj.sensorMode 		= (create->sensorMode & 0xFF);
      gSIG_Obj.vsEnable 		= (create->sensorMode & DRV_IMGS_SENSOR_MODE_VSTAB) ? 1 : 0;
      gSIG_Obj.aewbVendor   	= create->aewbVendor;
      gSIG_Obj.reduceShutter   	= create->reduceShutter;
      gSIG_Obj.saldre   		= create->saldre;
      gSIG_Obj.AGainEnable      = create->AGainEnable;
      gSIG_Obj.DGainEnable      = create->DGainEnable;
      gSIG_Obj.ALTM             = create->ALTM;
      gSIG_Obj.sensorFps		= create->sensorFps;
      gSIG_Obj.numEncodes		= create->numEncodes;
      gSIG_Obj.afEnable         = create->afEnable;

      aewbfParams.size = sizeof(aewbfParams);
      aewbfParams.numHistory = 10;
      aewbfParams.numSmoothSteps = 1;


      numMem = IAEWBF_SIG.ialg.algAlloc((IALG_Params *)&aewbfParams, NULL, gSIG_Obj.memTab_aewbf);
      while(numMem > 0){
          gSIG_Obj.memTab_aewbf[numMem-1].base = malloc(gSIG_Obj.memTab_aewbf[numMem-1].size);
          numMem --;
      }

      gSIG_Obj.handle_aewbf = (IALG_Handle)gSIG_Obj.memTab_aewbf[0].base;
      retval = IAEWBF_SIG.ialg.algInit(gSIG_Obj.handle_aewbf, gSIG_Obj.memTab_aewbf, NULL, (IALG_Params *)&aewbfParams);
      if(retval == -1) {
          OSA_ERROR("AE_SIG_AE.ialg.algInit()\n");
          return NULL;
      }

      retval = SIG_2A_config(gSIG_Obj.handle_aewbf);
      if(retval == -1) {
          OSA_ERROR("ERROR: SIG_2A_config\n");
          return NULL;
      }

      /* setup initial ipipe gains */
      //ALG_aewbSetIpipeWb(&ipipe_awb_gain, gALG_aewbObj.DGainEnable, lowlight);
      //ALG_aewbSetSensorDcsub(0);
      //ALG_aewbSetSensorGain(sensorGain);
      //TI_2A_SetEEValues(create->shiftValue);
      return &gSIG_Obj;
  }

  return &gALG_aewbObj;
}

int TI_2A_config(int flicker_detection, int saldre)
{
    IAE_DynamicParams aeDynamicParams;
    int i, stepSize;
    int retval;
    aeDynamicParams.size = sizeof(aeDynamicParams);
    aeDynamicParams.numRanges = 0;

    OSA_printf("TI_2A_config\n");

    i = 0; aewbFrames = 0; stepSize = 1;

#ifdef FD_DEBUG_MSG
    OSA_printf("\n\nTI_2A_config: flicker detection = %d\n\n", flicker_detection);
    //OSA_printf("gFlicker = %d\n", gFlicker);
    OSA_printf("Exposure maximum = %d, HighGain = %d\n", sensorExposureMax, HighGain);
#endif

    if (gALG_aewbObj.sensorFps == 25)
    {
    ALG_aewbSetSensor50_60Hz(1); // 25FPS
    } else
    {
    ALG_aewbSetSensor50_60Hz(0); // 30FPS
    }

    sensorGain = 1000;

    /* set stepSize based on input from Flicker detectiom and PAL/NTSC environment */
    /*if(flicker_detection == 1)
    {
    if(Aew_ext_parameter.env_50_60Hz == VIDEO_NTSC)
        stepSize = 8333;
    else
        stepSize = 10000;
    }
    else
    {
    stepSize = 1;
#ifdef FD_DEBUG_MSG
    OSA_printf("stepSize = 1\n");
#endif
    }

    if(gFlicker == VIDEO_NTSC && flicker_detection == 3){
    stepSize = (8333*gALG_aewbObj.reduceShutter)/100;
    }
    else if(gFlicker == VIDEO_PAL && flicker_detection == 2){
    stepSize = 10000;
    }*/

    lowlight = DRV_imgsGetAEPriority();

    extern int gFlicker;

    if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)	// AR0331 sensor
    {
    if(gFlicker == VIDEO_NTSC)			// 60 Hz flicker
    {
        if (gALG_aewbObj.sensorFps == 20)
        {
        stepSize = 6000; 	// Exposure stepsize
        if (sensorExposureMax > 33333)
            sensorExposureMax = 48000;
        }
        else if (gALG_aewbObj.sensorFps == 25)
        stepSize = 7500; 	// Exposure stepsize
        else if (gALG_aewbObj.sensorFps == 30)
        stepSize = 9000; 	// Exposure stepsize
    } else if(gFlicker == VIDEO_PAL)	// 50 Hz flicker
    {
        if (gALG_aewbObj.sensorFps == 20)
        {
        stepSize = 7200; 	// Exposure stepsize
        if (sensorExposureMax > 33333)
            sensorExposureMax = 43200;
        }
        else if (gALG_aewbObj.sensorFps == 25)
        stepSize = 9000; 	// Exposure stepsize
        else if (gALG_aewbObj.sensorFps == 30)
        stepSize = 10800; 	// Exposure stepsize
    } else
    {
        stepSize = 1;
    }
    } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9M034_720P") == 0) // MT9M034 sensor
    {
    if(gFlicker == VIDEO_NTSC)			// 60 Hz flicker
    {
        if (gALG_aewbObj.sensorFps == 25)
        stepSize = 7084; 	// Exposure stepsize
        else if (gALG_aewbObj.sensorFps == 30)
        stepSize = 8500; 	// Exposure stepsize
    } else if(gFlicker == VIDEO_PAL)	// 50 Hz flicker
    {
        if (gALG_aewbObj.sensorFps == 25)
        stepSize = 8500; 	// Exposure stepsize
        else if (gALG_aewbObj.sensorFps == 30)
        stepSize = 10200; 	// Exposure stepsize
    } else
    {
        stepSize = 1;
    }
    } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0) // MT9M031 sensor
    {
    if(gFlicker == VIDEO_NTSC)			// 60 Hz flicker
    {
        stepSize = 8333; 	// Exposure stepsize
    } else if(gFlicker == VIDEO_PAL)	// 50 Hz flicker
    {
        stepSize = 10000; 	// Exposure stepsize
    } else
    {
        stepSize = 1;
    }
    } else if (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0) // IMX136 sensor
    {
    if(gFlicker == VIDEO_NTSC)			// 60 Hz flicker
    {
        stepSize = 8333; 	// Exposure stepsize
    } else if(gFlicker == VIDEO_PAL)	// 50 Hz flicker
    {
        stepSize = 10000; 	// Exposure stepsize
    } else
    {
        stepSize = 1;
    }
    } else if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0) // OV271X sensor
    {
    if(gFlicker == VIDEO_NTSC)			// 60 Hz flicker
    {
        if (gALG_aewbObj.sensorFps == 25)
        DRV_imgsSetFlicker(0x12);
        else if (gALG_aewbObj.sensorFps == 30)
        DRV_imgsSetFlicker(0x22);
    } else if(gFlicker == VIDEO_PAL)	// 50 Hz flicker
    {
        if (gALG_aewbObj.sensorFps == 25)
        DRV_imgsSetFlicker(0x11);
        else if (gALG_aewbObj.sensorFps == 30)
        DRV_imgsSetFlicker(0x21);
    } else
    {
        DRV_imgsSetFlicker(0);;
    }
    } else
    {
    stepSize = 1;
    }

#ifdef FD_DEBUG_MSG
    OSA_printf("TI_2A_config: stepSize = %d final\n", stepSize);
#endif

    aeDynamicParams.numRanges ++;

    if (HISTmode == 8) // ALTM enable
    {
    aeDynamicParams.exposureTimeRange[i].min = 0x20*32;
    aeDynamicParams.exposureTimeRange[i].max = sensorExposureMax;
    } else
    {
    if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
    {
        aeDynamicParams.exposureTimeRange[i].min = 0x10*32;
    } else
    {
        aeDynamicParams.exposureTimeRange[i].min = stepSize;
    }
    aeDynamicParams.exposureTimeRange[i].max = sensorExposureMax;
    }

    aeDynamicParams.apertureLevelRange[i].min = 0;
    aeDynamicParams.apertureLevelRange[i].max = 0;
    aeDynamicParams.sensorGainRange[i].min = 1000;
    aeDynamicParams.sensorGainRange[i].max = 1000;
    aeDynamicParams.ipipeGainRange[i].min = 1024;
    aeDynamicParams.ipipeGainRange[i].max = 1024;
    i++;

    aeDynamicParams.numRanges ++;
    aeDynamicParams.exposureTimeRange[i].min = 0;
    aeDynamicParams.exposureTimeRange[i].max = 0;
    aeDynamicParams.apertureLevelRange[i].min = 0;
    aeDynamicParams.apertureLevelRange[i].max = 0;
    aeDynamicParams.sensorGainRange[i].min = 1000;

    if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
    {
    if (HighGain)
        aeDynamicParams.sensorGainRange[i].max = 5001;
    else if (lowlight)
        aeDynamicParams.sensorGainRange[i].max = 3001;
    else
        aeDynamicParams.sensorGainRange[i].max = 2001;
    } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0)
    {
    extern int gIRCut, gBWMode;
    if (HighGain)
        aeDynamicParams.sensorGainRange[i].max = 2000;
    else if (lowlight && (!gBWMode || gIRCut != 0))
        aeDynamicParams.sensorGainRange[i].max = 2000;
    else if (lowlight)
        aeDynamicParams.sensorGainRange[i].max = 2000;
    else
        aeDynamicParams.sensorGainRange[i].max = 2000;
    } else if (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0)
    {
        if (HighGain)
        aeDynamicParams.sensorGainRange[i].max = 100000; // logarithmic
    else if (lowlight)
        aeDynamicParams.sensorGainRange[i].max = 15800;
    else
        aeDynamicParams.sensorGainRange[i].max = 3000;
    } else
    {
    if (HighGain)
        aeDynamicParams.sensorGainRange[i].max = 2000;
    else if (lowlight)
        aeDynamicParams.sensorGainRange[i].max = 2000;
    else
        aeDynamicParams.sensorGainRange[i].max = 1500;
    }
    aeDynamicParams.ipipeGainRange[i].min = 0;
    aeDynamicParams.ipipeGainRange[i].max = 0;
    i++;
    aeDynamicParams.numRanges ++ ;
    aeDynamicParams.exposureTimeRange[i].min = 0;
    aeDynamicParams.exposureTimeRange[i].max = 0;
    aeDynamicParams.apertureLevelRange[i].min = 0;
    aeDynamicParams.apertureLevelRange[i].max = 0;
    aeDynamicParams.sensorGainRange[i].min = 0;
    aeDynamicParams.sensorGainRange[i].max = 0;
    aeDynamicParams.ipipeGainRange[i].min = 4;

    if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0)
    {
    if (lowlight)
        aeDynamicParams.ipipeGainRange[i].max = 4096;
    else
        aeDynamicParams.ipipeGainRange[i].max = 6144;
    }
    if (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0)
    {
    aeDynamicParams.ipipeGainRange[i].max = 4096;
    }
    else
    aeDynamicParams.ipipeGainRange[i].max = 6144;

    if(gFlicker == VIDEO_NONE) // More sensitive then not use flicker control
    {
    aeDynamicParams.targetBrightnessRange.min = 38;
    aeDynamicParams.targetBrightnessRange.max = 42;
    aeDynamicParams.targetBrightness = 40;
    aeDynamicParams.thrld = 4;
    } else
    {
    aeDynamicParams.targetBrightnessRange.min = 33;
    aeDynamicParams.targetBrightnessRange.max = 47;
    aeDynamicParams.targetBrightness = 40;
    aeDynamicParams.thrld = 14;
    }

    aeDynamicParams.exposureTimeStepSize = stepSize;

    memcpy((void *)&gALG_aewbObj.AE_InArgs.statMat,
       (void *)&gALG_aewbObj.IAEWB_StatMatdata,
       sizeof(IAEWB_StatMat) );

    memcpy( (void *)&gALG_aewbObj.AWB_InArgs.statMat,
       (void *)&gALG_aewbObj.IAEWB_StatMatdata,
       sizeof(IAEWB_StatMat) );

    retval = AE_TI_AE.control((IAE_Handle)gALG_aewbObj.handle_ae, IAE_CMD_SET_CONFIG, &aeDynamicParams, NULL);
    if(retval == -1) {
    OSA_ERROR("AE_TI_AE.control()\n");
    return retval;
    }

    if(flicker_detection == 1) sensorExposure = stepSize;
    ALG_aewbSetSensorExposure(sensorExposure);

    /* Pass calibration data to TI AWB */
    //retval = IMAGE_TUNE_GetAwbParams(&awb_calc_data);
    retval = AWB_TI_AWB.control((IAWB_Handle)gALG_aewbObj.handle_awb, TIAWB_CMD_CALIBRATION, &awb_calc_data, NULL);
    if(retval == -1) {
    OSA_ERROR("AWB_TI_AWB.control()\n");
    return retval;
    }

    return 0;
}

int Get_BoxCar(IALG_Handle handle)
{
    int status;
    int bufId=-1;
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    OSA_BufInfo *pBufInfo;

    status = DRV_ipipeGetBoxcarBuf(&bufId, OSA_TIMEOUT_NONE);
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
    hn->SatTh = hn->w*hn->h/200;

    return OSA_SOK;
}

static void GETTING_RGB_BLOCK_VALUE(unsigned short * BLOCK_DATA_ADDR,IAEWB_Rgb *rgbData, aewDataEntry *aew_data, int shift)
{
  unsigned short i,j,k, numWin, idx1, idx2;
  Uint8 *curAewbAddr;
  CSL_H3aAewbOutUnsatBlkCntOverlay *pAewbUnsatBlk;
  CSL_H3aAewbOutSumModeOverlay *pAewbWinData;
  int accValue[4];
  int aew_win_vt_cnt = gALG_aewbObj.IAEWB_StatMatdata.winCtVert;
  int aew_win_hz_cnt = gALG_aewbObj.IAEWB_StatMatdata.winCtHorz;

  curAewbAddr = (Uint8*)BLOCK_DATA_ADDR;
  numWin=0;

  accValue[0]=accValue[1]=accValue[2]=accValue[3]=0;

  for(i=0;i<aew_win_vt_cnt; i++) {
    for(j=0;j<aew_win_hz_cnt; j++) {

      pAewbWinData = (CSL_H3aAewbOutSumModeOverlay *)curAewbAddr;

      idx1 = numWin/8;
      idx2 = numWin%8;

      aew_data[idx1].window_data[idx2][0] = pAewbWinData->subSampleAcc[0];
      aew_data[idx1].window_data[idx2][1] = pAewbWinData->subSampleAcc[1];
      aew_data[idx1].window_data[idx2][2] = pAewbWinData->subSampleAcc[2];
      aew_data[idx1].window_data[idx2][3] = pAewbWinData->subSampleAcc[3];

      accValue[0] += pAewbWinData->subSampleAcc[0];
      accValue[1] += pAewbWinData->subSampleAcc[1];
      accValue[2] += pAewbWinData->subSampleAcc[2];
      accValue[3] += pAewbWinData->subSampleAcc[3];

      curAewbAddr += sizeof(CSL_H3aAewbOutSumModeOverlay);

      numWin++;

      if(numWin%8==0) {
        pAewbUnsatBlk = (CSL_H3aAewbOutUnsatBlkCntOverlay*)curAewbAddr;

        for(k=0; k<8;k++)
          aew_data[idx1].unsat_block_ct[k] = pAewbUnsatBlk->unsatCount[k];

        curAewbAddr += sizeof(CSL_H3aAewbOutUnsatBlkCntOverlay);
      }
    }
    curAewbAddr = (Uint8*)OSA_align( (Uint32)curAewbAddr, 32);
  }

  OSA_mutexLock(&gITTAwb.statusLock);  //for ITT - rgb data cpy yo ITT
  for(i = 0; i < (aew_win_hz_cnt * aew_win_vt_cnt)>>3;i ++){
    for(j = 0; j < 8; j ++){
      rgbData[i * 8 + j].r = aew_data[i].window_data[j][1] >> shift;
      rgbData[i * 8 + j].b = aew_data[i].window_data[j][2] >> shift;
      rgbData[i * 8 + j].g = (aew_data[i].window_data[j][0]
        + aew_data[i].window_data[j][3]+ 1) >> (1 + shift) ;
    }
  }
  OSA_mutexUnlock(&gITTAwb.statusLock);

  accValue[0] /= numWin*gALG_aewbObj.IAEWB_StatMatdata.pixCtWin;
  accValue[1] /= numWin*gALG_aewbObj.IAEWB_StatMatdata.pixCtWin;
  accValue[2] /= numWin*gALG_aewbObj.IAEWB_StatMatdata.pixCtWin;
  accValue[3] /= numWin*gALG_aewbObj.IAEWB_StatMatdata.pixCtWin;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Avg Color: %5d, %5d, %5d, %5d\n", accValue[0], accValue[1], accValue[2], accValue[3]);
  #endif
}

void AEW_SETUP_CONTROL( CONTROL3AS *CONTROL3A )
{
    CONTROL3A->IMAGE_SHARPNESS  = Aew_ext_parameter.sharpness;
    CONTROL3A->IMAGE_CONTRAST   = Aew_ext_parameter.contrast;
    CONTROL3A->IMAGE_BRIGHTNESS = Aew_ext_parameter.brightness;
    CONTROL3A->IMAGE_SATURATION = Aew_ext_parameter.saturation;
    CONTROL3A->IMAGE_BACKLIGHT  = Aew_ext_parameter.blc;
    CONTROL3A->INDOUTDOOR       = Aew_ext_parameter.awb_mode;
    CONTROL3A->VIDEO_MODE       = Aew_ext_parameter.env_50_60Hz;
    CONTROL3A->AUTO_IRIS        = Aew_ext_parameter.auto_iris;
    CONTROL3A->DAY_NIGHT        = Aew_ext_parameter.day_night;

    gOutAWBData.NFGain = Aew_ext_parameter.NFGain;

    if ( Aew_ext_parameter.binning_mode == SENSOR_BINNING )
        CONTROL3A->SKIP_BINNING_MODE = 0;
    else
        CONTROL3A->SKIP_BINNING_MODE = 1;

    if ( Aew_ext_parameter.aew_enable == AEW_ENABLE )
        CONTROL3A->PAUSE_AWWB = 0;
    else
        CONTROL3A->PAUSE_AWWB = 1;

    if (CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_LOW ||
            CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_LOW2 )
    {
        if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
            gALG_aewbObj.weight= APPRO_WEIGHTING_MATRIX;
        }
        else if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
            gALG_aewbObj.weight= TI_WEIGHTING_MATRIX;
        }
    }
    else if(CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_HIGH ||
            CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_HIGH2 )
    {
        if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
            gALG_aewbObj.weight=APPRO_WEIGHTING_SPOT;
        }
        else if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
            gALG_aewbObj.weight=TI_WEIGHTING_SPOT;
        }
    }
    else
    {
        if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
            gALG_aewbObj.weight=APPRO_WEIGHTING_CENTER;
        }
        else   if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
            gALG_aewbObj.weight=TI_WEIGHTING_CENTER;
        }
    }

    /* 50/60Hz switch & brightness & contrast support for TI 2A */
    if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
        BRT_CRT_PARAM brtCtrParam;
        brtCtrParam.yuv_adj_ctr = CONTROL3A->IMAGE_CONTRAST >> 3;
        brtCtrParam.yuv_adj_brt = CONTROL3A->IMAGE_BRIGHTNESS;
        ALG_aewbSetContrastBrightness(&brtCtrParam);
        if(env_50_60Hz != CONTROL3A->VIDEO_MODE) {
            env_50_60Hz = CONTROL3A->VIDEO_MODE;
            TI_2A_config(1, gALG_aewbObj.saldre);
        }
    }
}

void AEW_SETUP_SIG( CONTROL3AS *CONTROL3A )
{
    CONTROL3A->IMAGE_SHARPNESS  = Aew_ext_parameter.sharpness;
    CONTROL3A->IMAGE_CONTRAST   = Aew_ext_parameter.contrast;
    CONTROL3A->IMAGE_BRIGHTNESS = Aew_ext_parameter.brightness;
    CONTROL3A->IMAGE_SATURATION = Aew_ext_parameter.saturation;
    CONTROL3A->IMAGE_BACKLIGHT  = Aew_ext_parameter.blc;
    CONTROL3A->INDOUTDOOR       = Aew_ext_parameter.awb_mode;
    CONTROL3A->VIDEO_MODE       = Aew_ext_parameter.env_50_60Hz;
    CONTROL3A->AUTO_IRIS        = Aew_ext_parameter.auto_iris;
    CONTROL3A->DAY_NIGHT        = Aew_ext_parameter.day_night;

    gOutAWBData.NFGain = Aew_ext_parameter.NFGain;

    if ( Aew_ext_parameter.binning_mode == SENSOR_BINNING )
        CONTROL3A->SKIP_BINNING_MODE = 0;
    else
        CONTROL3A->SKIP_BINNING_MODE = 1;

    if ( Aew_ext_parameter.aew_enable == AEW_ENABLE )
        CONTROL3A->PAUSE_AWWB = 0;
    else
        CONTROL3A->PAUSE_AWWB = 1;

    if (CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_LOW ||
            CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_LOW2 )
    {
        if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
            gALG_aewbObj.weight= APPRO_WEIGHTING_MATRIX;
        }
        else if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
            gALG_aewbObj.weight= TI_WEIGHTING_MATRIX;
        }
    }
    else if(CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_HIGH ||
            CONTROL3A->IMAGE_BACKLIGHT==BACKLIGHT_HIGH2 )
    {
        if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
            gALG_aewbObj.weight=APPRO_WEIGHTING_SPOT;
        }
        else if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
            gALG_aewbObj.weight=TI_WEIGHTING_SPOT;
        }
    }
    else
    {
        if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
            gALG_aewbObj.weight=APPRO_WEIGHTING_CENTER;
        }
        else   if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
            gALG_aewbObj.weight=TI_WEIGHTING_CENTER;
        }
    }

    /* 50/60Hz switch & brightness & contrast support for TI 2A */
    if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_TI || gALG_aewbObj.aewbVendor==ALG_AEWB_ID_SIG) {
        BRT_CRT_PARAM brtCtrParam;
        brtCtrParam.yuv_adj_ctr = CONTROL3A->IMAGE_CONTRAST >> 3;
        brtCtrParam.yuv_adj_brt = CONTROL3A->IMAGE_BRIGHTNESS;
        ALG_aewbSetContrastBrightness(&brtCtrParam);
        if(env_50_60Hz != CONTROL3A->VIDEO_MODE) {
            env_50_60Hz = CONTROL3A->VIDEO_MODE;
            TI_2A_config(1, gALG_aewbObj.saldre);
        }
    }
}

#define RY    0x4d
#define GY    0x96
#define BY    0x1d

int AWB_Sigrand_process(IAWB_Handle handle, IAWB_InArgs *inArgs, IAWB_OutArgs *outArgs,
			IAEWB_Rgb *rgbData, XDAS_UInt8 *weight, void *customData)
{
    int width = inArgs->statMat.winCtHorz;
    int height = inArgs->statMat.winCtVert;
    int i;
    int j;
    int currpos;
    unsigned int redSum = 0;
    unsigned int blueSum = 0;
    unsigned int greenSum = 0;
    Uint32 redp = 0;
    Uint32 bluep = 0;
    Uint32 greenp = 0;
    unsigned int totalY;
    unsigned int weightSum = 0;
    unsigned int dU, dV;
    int b_sign;
    int r_sign;
    int b_step, r_step;
    int ColorSum = 0;
    unsigned int ColorSum_temp;
    int max_color = 0;
    int min_color = 65536;

    for(i = 0; i < height; i ++)
    {
	for(j = 0; j < width; j ++)
	{
                currpos = i * width;
                redp = (Uint32)rgbData[currpos + j].r;
                greenp = (Uint32)rgbData[currpos + j].g;
                bluep = (Uint32)rgbData[currpos + j].b;
                redSum += redp;
                greenSum += greenp;
                blueSum += bluep;
                //----
                YDataBuff[currpos + j] = (unsigned int)((redp * RY + greenp * GY + bluep * BY)>>8);
	}
    }

    weightSum = height * width;
    totalY = ((redSum * RY) + (greenSum * GY) + (blueSum * BY)) >> 8;

    if (blueSum >= totalY)
    {
	dU = blueSum - totalY;
	b_sign = -1;
    } else
    {
	dU = totalY - blueSum;
	b_sign = 1;
    }

    if (redSum >= totalY)
    {
	dV = redSum - totalY;
	r_sign = -1;
    } else
    {
	dV = totalY - redSum;
	r_sign = 1;
    }

	if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0)
	{
		/*dU = (dU * 2) /(weightSum * 3);
		dV = (dV * 2) /(weightSum * 3);*/
        dU = dU /(weightSum * 2);
		dV = dV /(weightSum * 2);
	} else
	{
		dU = dU/(weightSum);
		dV = dV/(weightSum);
	}

    totalY = totalY/weightSum;

#ifdef ALG_AEWB_DEBUG
    OSA_printf(" ALG: Sigrand AWB: dU = %d dV = %d totalY = %d\n", dU, dV, totalY);
#endif

    b_step = 0;
    if (dU > 31)
    {
	if (dU < 256)
	    b_step = dU >> 5;
	else
	    b_step = 8;
    }

    r_step = 0;
    if (dV > 31)
    {
	if (dV < 256)
	    r_step = dV >> 5;
	else
	    r_step = 8;
    }

	if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0)
	{
		outArgs->nextWb.rGain = inArgs->curWb.rGain + (r_sign * r_step * 2);
		if (outArgs->nextWb.rGain > outArgs->nextWb.gGain * 3)
			outArgs->nextWb.rGain = outArgs->nextWb.gGain * 3;
		outArgs->nextWb.bGain = inArgs->curWb.bGain + (b_sign * b_step * 2);
		if (outArgs->nextWb.bGain > outArgs->nextWb.gGain * 3)
			outArgs->nextWb.bGain = outArgs->nextWb.gGain * 3;
	} else
	{
		outArgs->nextWb.rGain = inArgs->curWb.rGain + (r_sign * r_step * 8);
		if (outArgs->nextWb.rGain > outArgs->nextWb.gGain * 3)
			outArgs->nextWb.rGain = outArgs->nextWb.gGain * 3;
		outArgs->nextWb.bGain = inArgs->curWb.bGain + (b_sign * b_step * 8);
		if (outArgs->nextWb.bGain > outArgs->nextWb.gGain * 3)
			outArgs->nextWb.bGain = outArgs->nextWb.gGain * 3;
	}

    /* Histogram AE algorithm */
    for(i = 0; i < height; i ++)
    {
	for(j = 0; j < width; j ++)
	{
	    ColorSum_temp = ((unsigned int)rgbData[i * width + j].r * RY + (unsigned int)rgbData[i * width + j].g * GY + (unsigned int)rgbData[i * width + j].b * BY + 128)>>8;

	    max_color = (ColorSum_temp > max_color) ? ColorSum_temp : max_color;
	    min_color = (ColorSum_temp < min_color) ? ColorSum_temp : min_color;

	    if (ColorSum_temp > 3500)
		ColorSum_temp = 3500;

	    ColorSum += ColorSum_temp;
	}
    }

    ColorSum = ColorSum / (height*width);       // Middle histogram value
    ColorSum = ColorSum - min_color;

    if (ColorSum <= 0)                          // Too dark or too light
	ColorSum = 2048;

    HISTgain_mid = (128 * 8192)/ColorSum;       // Deviation from middle value
    if (HISTgain_mid >= 4095)
    {
	HISTgain_mid = 4095;
    }

    if (max_color > min_color)
    {
	HISTgain_minmax = (512 * 16384)/ (max_color - min_color);
    } else
    {
	HISTgain_minmax = 512;
    }

    if (HISTgain_minmax >= 4095)
    {
	HISTgain_minmax = 4095;
    }

    HISTmin = min_color >> 2;                   // 14bit to 12bit value

#ifdef ALG_AEWB_DEBUG
    OSA_printf(" ALG: Sigrand AWB: ColorSum = %d min_color = %d max_color = %d HISTgain_mid = %d HISTgain_minmax = %d\n", ColorSum, min_color, max_color, HISTgain_mid, HISTgain_minmax);
#endif

    return 0;
}

int AE_OV271X_Night_Gain(void)
{
	int step = 0;
	int sign;
	int d_avg_image;
	int avg_image;
	int avg_image_point = 0x26; // best average image data at night

	// Check night mode
	if (DRV_imgsReadReg(0x3503) != 0x07)
	{
		return 0;
	}

	// Read average image data
	avg_image = DRV_imgsReadReg(0x5690);

	if ((avg_image <= (avg_image_point + 2)) && (avg_image >= (avg_image_point - 2)))
	{
		return 0;
	}

    if (avg_image >= avg_image_point)
    {
		d_avg_image = avg_image - avg_image_point;
		sign = -1;
    } else
    {
		d_avg_image = avg_image_point - avg_image;
		sign = 1;
    }

	if (d_avg_image > 20)
		step = 0x10;
	else if (d_avg_image > 10)
		step = 2;
	else
		step = 1;

	// Set new gain
	OV271X_gain = OV271X_gain + (sign * step);

	// Check diapason
	if (OV271X_gain < 0)
		OV271X_gain = 0;
	if (OV271X_gain > 0x5F)
		OV271X_gain = 0x5F;

    return 0;
}

void TI2AFunc(void *pAddr)
{
    int i = 0, retval = OSA_SOK ;
    CONTROL3AS TI_Control3A;
    int rgbMatrixIndex = 0;
    int diff, next_diff;
    float FD_brightness_cur;
    float FD_brightness_next;
    int AE_customdata;
    Uint16 *box;

    GETTING_RGB_BLOCK_VALUE(pAddr, rgbData, aew_data, 2);

    /* Xiangdong: we need a flag from the tuning serser to signal to the AWB thread that a new set of
     calibration data has been created by the tuning tool and need to be used,
     the following code needs to be enabled for to pass new tuning data in */
    if (IMAGE_TUNE_CmdGetAwbPrmStatus(&i) ){
    retval = IMAGE_TUNE_GetAwbParams(&awb_calc_data);
    retval = AWB_TI_AWB.control((IAWB_Handle)gALG_aewbObj.handle_awb, TIAWB_CMD_CALIBRATION, &awb_calc_data, NULL);
    IMAGE_TUNE_CmdSetAwbPrmStatus(0); //reset flag
    }
    if (Aew_ext_parameter.aew_enable == AEW_ENABLE && !(aewbFrames % NUM_STEPS) )
    {
    gALG_aewbObj.AE_InArgs.curAe.exposureTime = sensorExposure;
    gALG_aewbObj.AE_InArgs.curAe.sensorGain = sensorGain;
    gALG_aewbObj.AE_InArgs.curAe.ipipeGain = ipipe_awb_gain.dGain << 2;
    gALG_aewbObj.AE_InArgs.curWb.rGain = ipipe_awb_gain.rGain;
    gALG_aewbObj.AE_InArgs.curWb.gGain = ipipe_awb_gain.grGain;
    gALG_aewbObj.AE_InArgs.curWb.bGain = ipipe_awb_gain.bGain;
    AE_customdata = HISTmode;

    if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
    {
        AE_customdata += 10;
    }

    //OSA_printf("pixCtWin= %d AE_customdata = %d\n", gALG_aewbObj.AE_InArgs.statMat.pixCtWin, AE_customdata);

    if(gALG_aewbObj.aewbType == ALG_AEWB_AE || gALG_aewbObj.aewbType == ALG_AEWB_AEWB){
        AE_TI_AE.process(
                 (IAE_Handle)gALG_aewbObj.handle_ae,
                 &gALG_aewbObj.AE_InArgs,
                 &gALG_aewbObj.AE_OutArgs,
                 rgbData,
                 gALG_aewbObj.weight,
                 AE_customdata
                );
        if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0)
        {
            AE_OV271X_Night_Gain();
        }
    }
    else {
        gALG_aewbObj.AE_OutArgs.nextAe = gALG_aewbObj.AE_InArgs.curAe;
    }

    FD_brightness_cur = ((float)gALG_aewbObj.AE_InArgs.curAe.exposureTime) * gALG_aewbObj.AE_InArgs.curAe.sensorGain * gALG_aewbObj.AE_InArgs.curAe.ipipeGain;
    FD_brightness_next = ((float)gALG_aewbObj.AE_OutArgs.nextAe.exposureTime) * gALG_aewbObj.AE_OutArgs.nextAe.sensorGain * gALG_aewbObj.AE_OutArgs.nextAe.ipipeGain;

    /* Trigger Flicker detection process based on brightness threshold being crossed */
    if(FD_brightness_next < FD_BRIGHTNESS_THRESHHOLD && FD_brightness_cur >= FD_BRIGHTNESS_THRESHHOLD)
    {
        flicker_detect_complete =0;
    }

    if(FD_brightness_next > FD_BRIGHTNESS_THRESHHOLD && FD_brightness_cur <= FD_BRIGHTNESS_THRESHHOLD)
    {
        TI_2A_config(1, gALG_aewbObj.saldre);
    }

    if(gALG_aewbObj.AE_OutArgs.nextAe.exposureTime == gALG_aewbObj.AE_InArgs.curAe.exposureTime &&
       gALG_aewbObj.AE_OutArgs.nextAe.sensorGain == gALG_aewbObj.AE_InArgs.curAe.sensorGain &&
       (gALG_aewbObj.aewbType == ALG_AEWB_AWB || gALG_aewbObj.aewbType == ALG_AEWB_AEWB) ||
       (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0))
        //   gALG_aewbObj.AE_OutArgs.nextAe.ipipeGain == gALG_aewbObj.AE_InArgs.curAe.ipipeGain)
    {
        /* calling awb only we AE has converged */
        gALG_aewbObj.AWB_InArgs.curWb = gALG_aewbObj.AE_InArgs.curWb;
        gALG_aewbObj.AWB_InArgs.curAe = gALG_aewbObj.AE_InArgs.curAe;

        AWB_TI_AWB.process(
                   (IAWB_Handle)gALG_aewbObj.handle_awb,
                   &gALG_aewbObj.AWB_InArgs,
                   &gALG_aewbObj.AWB_OutArgs,
                   rgbData,
                   NULL
                  );

        if (gALG_aewbObj.aewbVendor == ALG_AEWB_ID_SIG)
        {
        // Sigrand AWB algorithm
        AWB_Sigrand_process(
                    (IAWB_Handle)gALG_aewbObj.handle_awb,
                    &gALG_aewbObj.AWB_InArgs,
                    &gALG_aewbObj.AWB_OutArgs,
                    rgbData,
                    gALG_aewbObj.weight,
                    NULL
                   );
        }
        ipipe_awb_gain.rGain = gALG_aewbObj.AWB_OutArgs.nextWb.rGain;
        ipipe_awb_gain.grGain = gALG_aewbObj.AWB_OutArgs.nextWb.gGain;
        ipipe_awb_gain.gbGain = gALG_aewbObj.AWB_OutArgs.nextWb.gGain;
        ipipe_awb_gain.bGain = gALG_aewbObj.AWB_OutArgs.nextWb.bGain;

        for(i = 0; i < NUM_RGB2RGB_MATRIXES-1; i ++){
        diff = gALG_aewbObj.AWB_OutArgs.nextWb.colorTemp - rgb_maxtrixes[i].color_temp;
        next_diff = rgb_maxtrixes[i+1].color_temp - gALG_aewbObj.AWB_OutArgs.nextWb.colorTemp;
        if((next_diff >= 0 && diff >= 0) || diff < 0){
            if(next_diff < diff) i++;
            break;
        }
        }

        if (i >= NUM_RGB2RGB_MATRIXES) i = NUM_RGB2RGB_MATRIXES - 1;


        rgbMatrixIndex = RGB2RGB_stab(i);

        ALG_aewbSetRgb2Rgb(&rgb_maxtrixes[rgbMatrixIndex].rgb2rgbparam);
        ALG_aewbSetRgb2Rgb2(&rgb_maxtrixes[rgbMatrixIndex].rgb2rgb2param);
        TI2A_applySettings(&gALG_aewbObj.AE_InArgs.curAe,
                   &gALG_aewbObj.AE_OutArgs.nextAe, NUM_STEPS-1, 0);

    }
    AEW_SETUP_CONTROL( &TI_Control3A );
    }
    else if(Aew_ext_parameter.aew_enable == AEW_ENABLE && (gALG_aewbObj.aewbType == ALG_AEWB_AE || gALG_aewbObj.aewbType == ALG_AEWB_AEWB)){
    TI2A_applySettings(&gALG_aewbObj.AE_InArgs.curAe, &gALG_aewbObj.AE_OutArgs.nextAe, NUM_STEPS-1, (aewbFrames % NUM_STEPS));
    }

    /* remove the count and put it into the process */
    aewbFrames ++;

}


void SIG2AFunc(void *pAddr)
{

    Get_BoxCar(gSIG_Obj.handle_aewbf);

    if ((Aew_ext_parameter.aew_enable == AEW_ENABLE) ) {

        if(gSIG_Obj.aewbType == ALG_AEWB_AE || gSIG_Obj.aewbType == ALG_AEWB_AEWB){
            IAEWBF_SIG.process((IAEWBF_Handle)gSIG_Obj.handle_aewbf, &gSIG_Obj.InArgs, &gSIG_Obj.OutArgs);
            SIG2A_applySettings();
        } else {
            gSIG_Obj.OutArgs.nextAe = gSIG_Obj.InArgs.curAe;
        }


    }
    aewbFrames++;
}

void TI2A_applySettings(IAEWB_Ae *curAe, IAEWB_Ae *nextAe, int numSmoothSteps, int step)
{
  if (gALG_aewbObj.afEnable == 1)
      return;

  int delta_sensorgain = ((int)nextAe->sensorGain - (int)curAe->sensorGain)/numSmoothSteps;
  int delta_exposure = ((int)nextAe->exposureTime - (int)curAe->exposureTime)/numSmoothSteps;
  int delta_ipipe = ((int)nextAe->ipipeGain - (int)curAe->ipipeGain)/numSmoothSteps;

  step ++;

  sensorGain = delta_sensorgain * step + curAe->sensorGain;
  sensorExposure = delta_exposure * step + curAe->exposureTime;
  ipipe_awb_gain.dGain = (delta_ipipe * step +curAe->ipipeGain) >> 2;

  if(step >= numSmoothSteps) {
    sensorGain = nextAe->sensorGain;
    sensorExposure = nextAe->exposureTime;
    ipipe_awb_gain.dGain = nextAe->ipipeGain>> 2;
  }

  if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0)
  {
      sensorGain = OV271X_gain;
  }

  ipipe_awb_gain.hGain_minmax = HISTgain_minmax;
  ipipe_awb_gain.hGain_mid = HISTgain_mid;
  ipipe_awb_gain.hMin = HISTmin;
  ipipe_awb_gain.hMode = HISTmode;

  ALG_aewbSetIpipeWb(&ipipe_awb_gain, gALG_aewbObj.DGainEnable, lowlight);
  ALG_aewbSetSensorExposure(sensorExposure);
  if (gALG_aewbObj.AGainEnable)
  {
      ALG_aewbSetSensorGain(sensorGain);
  }
}

short ALG_aewbDummy(int setval)
{
	return 0;
}

void Appro3AFunc(void *pAddr)
{
  CONTROL3AS Appro_Control3A;
  static int firstApproflg = 1;
  static IAWB_InArgs   AWB_InArgs;
  static IAWB_OutArgs  AWB_OutArgs;

  if( firstApproflg )
  {
	firstApproflg = 0;
	AWB_InArgs	= gALG_aewbObj.AWB_InArgs;
  }

  GETTING_RGB_BLOCK_VALUE(pAddr, rgbData, aew_data,0);

  /* remove the count and put it into the process */
  if(Aew_ext_parameter.aew_enable == AEW_ENABLE )
  {

    gALG_aewbObj.AE_InArgs.curWb = gALG_aewbObj.AWB_OutArgs.nextWb;
    gALG_aewbObj.AWB_InArgs.curWb = gALG_aewbObj.AWB_OutArgs.nextWb;

	  AE_APPRO_AE.process(
        (IAE_Handle)gALG_aewbObj.handle_ae,
        &gALG_aewbObj.AE_InArgs,
        &gALG_aewbObj.AE_OutArgs,
        rgbData,
        gALG_aewbObj.weight,
        NULL
        );

    gALG_aewbObj.AE_InArgs.curAe = gALG_aewbObj.AE_OutArgs.nextAe;
    gALG_aewbObj.AWB_InArgs.curAe = gALG_aewbObj.AE_OutArgs.nextAe;

    AWB_APPRO_AWB.process(
        (IAWB_Handle)gALG_aewbObj.handle_awb,
        &gALG_aewbObj.AWB_InArgs,
        &gALG_aewbObj.AWB_OutArgs,
        rgbData,
        aew_data
        );

    #ifdef ALG_AEWB_DEBUG
    OSA_printf(" ALG: Aewb: Exposure = %ld, Gain = %ld\n", gALG_aewbObj.AE_OutArgs.nextAe.exposureTime, gALG_aewbObj.AE_OutArgs.nextAe.sensorGain);
    #endif

  // If 2A algo is replacd, recommednd to replace this control of shutter also
  // replacement ths function name
    AEW_SETUP_CONTROL( &Appro_Control3A );

    CONTROL_DRIVER_process(
        (IAE_Handle)gALG_aewbObj.handle_ae,
        (IAWB_Handle)gALG_aewbObj.handle_awb,
        &gALG_aewbObj.AE_OutArgs,
        &gALG_aewbObj.AWB_OutArgs,
        &Appro_Control3A
        );
  }
}

//convert H3A RGB data into the luma image (int16) the FD algorithm needed
static void GETTING_RGB_BLOCK_VALUE_Y(unsigned short * BLOCK_DATA_ADDR, short *y, int shift)
{
  unsigned short i,j, numWin;
  Uint8 *curAewbAddr;
  CSL_H3aAewbOutSumModeOverlay *pAewbWinData;
  int aew_win_vt_cnt = gALG_aewbObj.IAEWB_StatMatdata.winCtVert;
  int aew_win_hz_cnt = gALG_aewbObj.IAEWB_StatMatdata.winCtHorz;
  int r, g, b;

  curAewbAddr = (Uint8*)BLOCK_DATA_ADDR;
  numWin=0;

  for(i=0;i<aew_win_vt_cnt; i++)
  {
    for(j=0;j<aew_win_hz_cnt; j++)
    {

      pAewbWinData = (CSL_H3aAewbOutSumModeOverlay *)curAewbAddr;

      g = (pAewbWinData->subSampleAcc[0] + pAewbWinData->subSampleAcc[3]) >> (1+shift);
      r = pAewbWinData->subSampleAcc[1] >> shift;
      b = pAewbWinData->subSampleAcc[2] >> shift;
      *y++ = ((0x4D * r) + (0x96 * g) + (0x1D * b) + 128 ) / 256;

      curAewbAddr += sizeof(CSL_H3aAewbOutSumModeOverlay);

      numWin++;

      if(numWin%8==0) {
        curAewbAddr += sizeof(CSL_H3aAewbOutUnsatBlkCntOverlay);
      }
    }
    curAewbAddr = (Uint8*)OSA_align( (Uint32)curAewbAddr, 32);
  }
}

int ALG_aewbRun(void *hndl, ALG_AewbRunPrm *prm, ALG_AewbStatus *status)
{
    memset(status, 0, sizeof(*status));

    gALG_aewbObj.vnfDemoCfg   	= prm->vnfDemoCfg;
    gALG_aewbObj.aewbType     	= prm->aewbType;
    gALG_aewbObj.aewbVendor   	= prm->aewbVendor;
    gALG_aewbObj.aewbPriority   	= prm->aewbPriority;
    gALG_aewbObj.reduceShutter   	= prm->reduceShutter;
    gALG_aewbObj.saldre   		= prm->saldre;

    if(prm->aewbVendor == ALG_AEWB_ID_APPRO) {
	if(Aew_ext_parameter.aew_enable == AEW_ENABLE )
	{
	    ApproSend3A(
			(IAE_Handle)gALG_aewbObj.handle_ae,
			(IAWB_Handle)gALG_aewbObj.handle_awb,
			&gALG_aewbObj.AE_OutArgs,
			&gALG_aewbObj.AWB_OutArgs
		       );
	}

	Aew_ext_parameter.H3ABuffer = (void*)prm->h3aDataVirtAddr;
	Appro3AFunc( Aew_ext_parameter.H3ABuffer );
    }
    else if(prm->aewbVendor == ALG_AEWB_ID_TI) {
        TI2AFunc( (void *)prm->h3aDataVirtAddr );
    }
    else if(prm->aewbVendor == ALG_AEWB_ID_SIG) {
        gSIG_Obj.vnfDemoCfg   	= prm->vnfDemoCfg;
        gSIG_Obj.aewbType     	= prm->aewbType;
        gSIG_Obj.aewbVendor   	= prm->aewbVendor;
        gSIG_Obj.aewbPriority   	= prm->aewbPriority;
        gSIG_Obj.reduceShutter   	= prm->reduceShutter;
        gSIG_Obj.saldre   		= prm->saldre;
        SIG2AFunc( (void *)prm->h3aDataVirtAddr );
    }
    return 0;
}

int ALG_aewbDelete(void *hndl)
{
    int numMem;

    if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
	numMem = AE_APPRO_AE.ialg.algFree(gALG_aewbObj.handle_ae, gALG_aewbObj.memTab_ae);
	while(numMem > 0){
	    free( gALG_aewbObj.memTab_ae[numMem-1].base );
	    numMem --;
	}

	numMem = AWB_APPRO_AWB.ialg.algFree(gALG_aewbObj.handle_awb, gALG_aewbObj.memTab_awb);
	while(numMem > 0){
	    free( gALG_aewbObj.memTab_awb[numMem-1].base );
	    numMem --;
	}
    }
    else if(gALG_aewbObj.aewbVendor == ALG_AEWB_ID_TI) {

	numMem = AE_TI_AE.ialg.algFree(gALG_aewbObj.handle_ae, gALG_aewbObj.memTab_ae);
	while(numMem > 0){
	    free( gALG_aewbObj.memTab_ae[numMem-1].base );
	    numMem --;
	}

	numMem = AWB_TI_AWB.ialg.algFree(gALG_aewbObj.handle_awb, gALG_aewbObj.memTab_awb);
	while(numMem > 0){
	    free( gALG_aewbObj.memTab_awb[numMem-1].base );
	    numMem --;
	}
    }
    else if(gALG_aewbObj.aewbVendor == ALG_AEWB_ID_SIG) {
        while(numMem > 0){
            free( gALG_aewbObj.memTab_ae[numMem-1].base );
            numMem --;
        }

        numMem = AWB_TI_AWB.ialg.algFree(gALG_aewbObj.handle_awb, gALG_aewbObj.memTab_awb);
        while(numMem > 0){
            free( gALG_aewbObj.memTab_awb[numMem-1].base );
            numMem --;
        }
    }
    free(rgbData);
  //----
    free(YDataBuff);
  //----
    free(aew_data);
    //free(g_flickerMem);

    OSA_mutexDelete(&gITTAwb.statusLock);

    return 0;
}

int ALG_aewbSetTTawb( ALG_AewbData_ITTAwb ipipe_awb_gain){

  //TBD

  return 0;
}

/* get AWB data for Image Tuning tool */
int ALG_aewbGetTTawb( ALG_AewbData_ITTAwb *itt_AwbData)
{
  IAEWB_Rgb *tRgb;
  Uint32 awbDataSize = 0;

  OSA_mutexLock(&gITTAwb.statusLock);


  itt_AwbData->awbNumWinH = gITTAwb.awbNumWinH;
  itt_AwbData->awbNumWinV = gITTAwb.awbNumWinV;
  memset(itt_AwbData->awbMiscData ,0x0, sizeof(gITTAwb.awbMiscData));  //Temp

  awbDataSize = (itt_AwbData->awbNumWinH * itt_AwbData->awbNumWinV);
  tRgb = gITTAwb.awbRgbData;

  if(awbDataSize > (IMAGE_TUNE_AWB_RGB_SIZE))
  	awbDataSize = (IMAGE_TUNE_AWB_RGB_SIZE);
  if( tRgb != NULL) {
  	ALG_aewbConvert_RGB_YUV(tRgb, (int)gITTAwb.aewbNumSPCInWin, (int)awbDataSize, (IAEWB_Rgb *)itt_AwbData->awbRgbData, (IAWB_Yuv *)itt_AwbData->awbYuvData);
  }
  else{
      memset(itt_AwbData->awbRgbData ,0x0, (IMAGE_TUNE_AWB_RGB_SIZE*3*4));
      memset(itt_AwbData->awbYuvData ,0x0, (IMAGE_TUNE_AWB_RGB_SIZE*3*4));
  }

  OSA_mutexUnlock(&gITTAwb.statusLock);

  return 0;
}

/*********************************************************************************************************************************
 * README
 *
 * ALG_aewbConvert_RGB_YUV - CONVERT_RGB_YUV(IAWB_RGB *rgbData, IAWB_RGB *norm_rgbData, IAWB_YUV *yuvData, int pix_in_pax, int awb_h3a_paxels)
 *
 * IAWB_RGB *rgbData: pointer to the RGB H3A data passed to AWB library
 * IAWB_RGB *norm_rgbData: pointer to the normalized RGB H3A data, that is, rgbData divided by pix_in_pax (number of pixels per H3A paxel)
 * IAWB_YUV *yuvData: pointer to the converted H3A data in Y, Cb, Cr format
 * int pix_in_pax: number of accumulated pixels per H3A paxel
 * int awb_h3a_paxels: total number of H3A paxels (i.e., total number of H3A windows)
 *
 * Note:
 * 1. IAWB_RGB *rgbData should points to the "IAEWB_Rgb *rgbData" produced by function
 *			static void GETTING_RGB_BLOCK_VALUE(unsigned short * BLOCK_DATA_ADDR,IAEWB_Rgb *rgbData, aewDataEntry *aew_data, int shift)
 * 2. IAWB_RGB *norm_rgbData and IAWB_YUV *yuvData are the RGB data and YUV data needed by IPNC tuning tool for AWB tuning
 *
 * Source - Buyue
 *********************************************************************************************************************************/

void ALG_aewbConvert_RGB_YUV(IAEWB_Rgb *rgbData, int pix_in_pax, int awb_h3a_paxels, IAEWB_Rgb *norm_rgbData, IAWB_Yuv *yuvData)
{
    int i;
    int current_R, current_G, current_B;
    int current_Y, current_Cb, current_Cr;
    int temp;


    for ( i = 0; i < awb_h3a_paxels; i++ )
    {
	current_R = rgbData[i].r;
	current_G = rgbData[i].g;
	current_B = rgbData[i].b;

	current_R = (current_R + pix_in_pax / 2) / pix_in_pax;
	current_G = (current_G + pix_in_pax / 2) / pix_in_pax;
	current_B = (current_B + pix_in_pax / 2) / pix_in_pax;

	current_Y = (( 0x4D * current_R ) + ( 0x96 * current_G ) + ( 0x1D * current_B ) + 128 ) / 256;

	temp = -0x2B * current_R - 0x55 * current_G + 0x80 * current_B;
	if ( temp > 0 ) temp += ( current_Y + 1 ) / 2;
	else if (temp < 0 ) temp -= ( current_Y + 1 ) / 2;
	current_Cb = temp / ( current_Y + 1 );

	temp = 0x80 * current_R - 0x6B * current_G - 0x15 * current_B;
	if ( temp > 0 ) temp += ( current_Y + 1 ) / 2;
	else if ( temp < 0 ) temp -= ( current_Y + 1 ) / 2;
	current_Cr = temp / ( current_Y + 1 );

	norm_rgbData[i].r = current_R;
	norm_rgbData[i].g = current_G;
	norm_rgbData[i].b = current_B;

	yuvData[i].y = current_Y;
	yuvData[i].u = current_Cb;
	yuvData[i].v = current_Cr;

    }
}

awb_calc_data_t *ImageTune_imgsGetAwbConfig(int mod)
{
  //Revisit
  //OSA_printf("IT AlgAwb: GetAwbConfig done \n");

  return &awb_calc_data;

}

void ALG_SetExposureMax(int exposure_max)
{
  sensorExposureMax = exposure_max;
  ALG_aewbSetSensorExposure(sensorExposureMax);
  //TI_2A_config(1, gALG_aewbObj.saldre);
}

void ALG_SetHighGain(int sensorHighGain)
{
  HighGain = sensorHighGain;
  //TI_2A_config(1, gALG_aewbObj.saldre);
}

short ALG_aewbSetWBMode(int WBMode)
{
  if (HISTmode == 8) // ALTM enable
  {
      OSA_printf(" AEWB: ALTM mode\n");
      return 0;
  }

  //TI_2A_config(1, gALG_aewbObj.saldre);
  //DRV_imgsSetEshutter(2, 0);
  //DRV_ipipeSetYoffet(0);
  HISTmode = WBMode;

  if (WBMode == 0)
  {
      OSA_printf(" AEWB: Middle histogram gain mode\n");
  }
  else if (WBMode == 1)
  {
      OSA_printf(" AEWB: MinMax histogram gain mode\n");
  }
  else
  {
      OSA_printf(" AEWB: No histogram gain mode\n");
  }

  return 0;
}
