
#include "alg_aewb_priv.h"
#include "imageTunePriv.h"
#include <drv_capture.h>
#include <drv_display.h>
#include "alg_aewbf.h"

#define ALG_AEWB_DEBUG

extern IMAGE_TUNE_Ctrl gIMAGE_TUNE_ctrl;

static Uint32 yee_table[] = {
	#include "alg_aewb_ee_table.txt"
};

static int rgb2rgb_gain = 0;    // Digital Gain in RGB2RGB module
static int ALTM_enable = 0;     // ALTM mode
static int sensor_exposure = 33333;

#define ENABLE_COMPENSATION  (200) // 100 = 1.00x compensation i.e none, 150 = 1.5x compensation

AWB_OUTPUT_DATA gOutAWBData;
extern int DRV_imgsFocus(int IsFocus);
extern int DRV_imgsZoom(int IsZoom);
extern int DRV_imgsNDShutter(int bIsDay, int BWMode);

short ALG_aewbSetSensorGain(int gain)
{
  static int prevValue = -1;
  int gain32;

  if(prevValue==gain)
    return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Sensor Gain = %d\n", gain);
  #endif

  prevValue = gain;
  gOutAWBData.sensorGain = (int)gain;

  gain32 = (int)gain*1;

  if(gALG_aewbObj.vsEnable)
    gain32 = (int)(gain*ENABLE_COMPENSATION)/100;
  else if(gALG_aewbObj.vnfDemoCfg)
  	gain32 = (int)(gain*2);

  DRV_imgsSetAgain(gain32, 0);

  return 0;
}

short ALG_aewbSetSensorExposure(int shutter)
{
  static int prevValue = -1;
  int shutter32;

  //if(prevValue==shutter)
  //  return 0;


  prevValue = shutter;

  shutter32 = (int)(shutter*100)/gALG_aewbObj.reduceShutter;

  if(gALG_aewbObj.vsEnable)
    shutter32 = (shutter32*100)/ENABLE_COMPENSATION;
  /*else if(gALG_aewbObj.vnfDemoCfg)
    shutter32 = (shutter32*200)/ENABLE_COMPENSATION;*/

#ifdef ALG_AEWB_DEBUG
  OSA_printf("ALG_aewbSetSensorExposure: Exposure = %d\n", shutter32);
#endif
   DRV_imgsSetEshutter(shutter32, 0);

  sensor_exposure = shutter32;

  return 0;
}

short ALG_aewbSetDayNight(IALG_Handle handle, int lowlight)
{
    DRV_IpipeWb ipipeWb;
    static AWB_PARAM PreAwb_Data;
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    static int darkframe = 0;
    static int frame_cnt = 0;
    static int frame_cnt_ircut = 0;
    extern int gDayNight;

    if (lowlight) {
        if ((hn->Y < 150) && gDayNight) { // Open IRCut if too dark
            frame_cnt++;
            if (frame_cnt >= 150) {
                gDayNight = 0; // Night
                ALG_aewbSetNDShutterOnOff(gDayNight);
                frame_cnt = 0;
            }
        } else {
            frame_cnt = 0;
        }
    }

    if (gDayNight == 0) {
        if (hn->Y > 240) {  // Close IRCut
            frame_cnt_ircut++;
            if (frame_cnt_ircut >= 150){
                gDayNight = 1; // Day
                ALG_aewbSetNDShutterOnOff(gDayNight);
                frame_cnt_ircut = 0;
            }
        }
    }

    // Go to low FPS mode if 150 frames dark
    if (!lowlight) {
        if (hn->Y < 130) {
            darkframe++;
            if (darkframe > 150) {
                DRV_imgsSetAEPriority(1);
                darkframe = 0;
            }
        } else {
            darkframe = 0;
        }
    }

    // Go to High FPS mode if 60 frames light
    if (lowlight) {
        if (hn->Y > 150) {
            darkframe++;
            if (darkframe > 60) {
                DRV_imgsSetAEPriority(0);
                darkframe = 0;
            }
        } else {
            darkframe = 0;
        }
    }
}

short ALG_aewbSetIpipeWb(AWB_PARAM  *pAwb_Data, int DGainEnable, int lowlight)
{
    DRV_IpipeWb ipipeWb;
    static AWB_PARAM PreAwb_Data;
    static int darkframe = 0;
    static int frame_cnt = 0;
    static int frame_cnt_ircut = 0;
    extern int gDayNight;
    int dGain;
    int Y_offset = 0;

    if (lowlight)
    {
    if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0)
    {
        if ((DRV_imgsReadReg(0x5690) < 0x10) && gDayNight)
        {
        frame_cnt++;
        if (frame_cnt >= 150)
        {
            gDayNight = 0; // Night
            ALG_aewbSetNDShutterOnOff(gDayNight);
            DRV_imgsSetND(gDayNight);
            frame_cnt = 0;
        }
        } else
        {
        frame_cnt = 0;
        }
    } else if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0)
    {
        if ((pAwb_Data->hGain_mid >= 3000) && gDayNight) // Open IRCut if too dark
        {
        frame_cnt++;
        if (frame_cnt >= 150)
        {
            gDayNight = 0; // Night
            ALG_aewbSetNDShutterOnOff(gDayNight);
            frame_cnt = 0;
        }
        } else
        {
        frame_cnt = 0;
        }
    } else
    {
        if ((pAwb_Data->hGain_mid >= 3000) && gDayNight) // Open IRCut if too dark
        {
        frame_cnt++;
        if (frame_cnt >= 150)
        {
            gDayNight = 0; // Night
            ALG_aewbSetNDShutterOnOff(gDayNight);
            frame_cnt = 0;
        }
        } else
        {
        frame_cnt = 0;
        }
    }
    }

    if (gDayNight == 0)
    {
    if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0)
    {
        if ((DRV_imgsReadReg(0x350A) == 0) && (DRV_imgsReadReg(0x350B) < 0x18)) // low light, medium gain
        {
        frame_cnt_ircut++;
        if (frame_cnt_ircut >= 60)
        {
            gDayNight = 1; // Day
            ALG_aewbSetNDShutterOnOff(gDayNight);
            DRV_imgsSetND(gDayNight);
            frame_cnt_ircut = 0;
        }
        } else
        {
        frame_cnt_ircut = 0;
        }
    } else if ((strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0) ||
           (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP"   ) == 0))
    {
        if ((pAwb_Data->dGain <= 256))  // Close IRCut
        {
        frame_cnt_ircut++;
        if (frame_cnt_ircut >= 30)
        {
            gDayNight = 1; // Day
            ALG_aewbSetNDShutterOnOff(gDayNight);
            frame_cnt_ircut = 0;
        }
        }
    } else
    {
        if ((sensor_exposure < 43000))  // Close IRCut
        {
        frame_cnt_ircut++;
        if (frame_cnt_ircut >= 3)
        {
            gDayNight = 1; // Day
            ALG_aewbSetNDShutterOnOff(gDayNight);
            frame_cnt_ircut = 0;
        }
        }
    }
    }

    // Disable ISIF digital gain
    if (DGainEnable != 1)
    pAwb_Data->dGain = 256;

    if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
    {
    if (pAwb_Data->dGain < 128)
    {
        pAwb_Data->dGain = 128;
    }
    }
    else
    {
    if (pAwb_Data->dGain < 256)
    {
        pAwb_Data->dGain = 256;
    }
    }

    // Go to low FPS mode if 150 frames dark
    if (!lowlight)
    {
    if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0)
    {
        if (DRV_imgsReadReg(0x5690) < 0x1C)
        {
        darkframe++;
        if (darkframe > 150)
        {
            DRV_imgsSetAEPriority(1);
            darkframe = 0;
        }
        } else
        {
        darkframe = 0;
        }
    } else if (pAwb_Data->hMode == 8)    // ALTM Enable
    {
        if (pAwb_Data->dGain >= 512 && DRV_imgsReadReg(0x315E) < 0x40)
        {
        darkframe++;
        if (darkframe > 150)
        {
            DRV_imgsSetAEPriority(1);
            darkframe = 0;
        }
        } else
        {
        darkframe = 0;
        }
    } else if (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0)
    {
        if (pAwb_Data->hGain_mid >= 1023 && pAwb_Data->dGain >= 1023)
        {
        darkframe++;
        if (darkframe > 150)
        {
            DRV_imgsSetAEPriority(1);
            darkframe = 0;
        }
        } else
        {
        darkframe = 0;
        }
    } else
    {
        if (pAwb_Data->hGain_mid >= 1023 && pAwb_Data->dGain >= 1535)
        {
        darkframe++;
        if (darkframe > 150)
        {
            DRV_imgsSetAEPriority(1);
            darkframe = 0;
        }
        } else
        {
        darkframe = 0;
        }
    }
    }

    // Go to High FPS mode if 60 frames light
    if (lowlight)
    {
    if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0)
    {
        if ((DRV_imgsReadReg(0x5690) > 0x2F) && (DRV_imgsReadReg(0x350B) < 0x20) && (gDayNight == 1))
        {
        darkframe++;
        if (darkframe > 60)
        {
            DRV_imgsSetAEPriority(0);
            darkframe = 0;
        }
        } else
        {
        darkframe = 0;
        }
    } else if (pAwb_Data->hMode == 8)    // ALTM Enable
    {
        if (pAwb_Data->hGain_mid < 512)
        {
        darkframe++;
        if (darkframe > 60)
        {
            DRV_imgsSetAEPriority(0);
            darkframe = 0;
        }
        } else
        {
        darkframe = 0;
        }
    }
    }

    if( memcmp( &PreAwb_Data, pAwb_Data, sizeof(AWB_PARAM))== 0 )
    return 0;

    PreAwb_Data = *pAwb_Data;

#ifdef ALG_AEWB_DEBUG
    OSA_printf(" AEWB: R Gr Gb B = (%d, %d, %d, %d) DGAIN = %d HistGAIN_mid = %d HistGAIN_mm = %d HistMin = %d HMode = %d\n",
           pAwb_Data->rGain, pAwb_Data->grGain, pAwb_Data->gbGain, pAwb_Data->bGain, pAwb_Data->dGain, pAwb_Data->hGain_mid, pAwb_Data->hGain_minmax, pAwb_Data->hMin, pAwb_Data->hMode
          );
#endif

    if (gALG_aewbObj.aewbVendor == ALG_AEWB_ID_TI)
    {
    ipipeWb.gainR  = pAwb_Data->rGain >> 1;
    ipipeWb.gainGr = pAwb_Data->grGain >> 1;
    ipipeWb.gainGb = pAwb_Data->gbGain >> 1;
    ipipeWb.gainB  = pAwb_Data->bGain >> 1;
    } else
    {
    DRV_imgsSetWB(4, pAwb_Data->rGain >> 3, pAwb_Data->bGain >> 3);

    if (lowlight)
    {
        /*if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
        {
        rgb2rgb_gain = (3*pAwb_Data->hGain_mid)/2;
        ALTM_enable = 0;
        } else
        {
        rgb2rgb_gain = 2*pAwb_Data->hGain_mid;
        }*/

        rgb2rgb_gain = 0x400;
        ALTM_enable = 0;

        ipipeWb.gainR  = 4*128;
        ipipeWb.gainGr = 4*128;
        ipipeWb.gainGb = 4*128;
        ipipeWb.gainB  = 4*128;
    } else
    {
        if (pAwb_Data->hMode == 8)     // ALTM Enable
        {
        /*rgb2rgb_gain = pAwb_Data->hGain_mid + pAwb_Data->hGain_minmax;

        Y_offset = pAwb_Data->hMin/8;
        if (Y_offset >= 126)
            Y_offset = 126;
        DRV_ipipeSetYoffet(-Y_offset);*/

        rgb2rgb_gain = 0x400;
        ALTM_enable = 1;

        if (pAwb_Data->dGain < 512)
        {
            pAwb_Data->dGain = (pAwb_Data->dGain * 3) / 2;
        }

#ifdef ALG_AEWB_DEBUG
        OSA_printf(" AEWB: ALTM hGain_mid = %d rgb2rgb_gain = %d Y_offset = %d\n", pAwb_Data->hGain_mid, rgb2rgb_gain, Y_offset);
#endif
        } else
        {
        /*if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
        {
            rgb2rgb_gain = pAwb_Data->hGain_mid + (pAwb_Data->hGain_minmax / 3);
        } else
        {
            rgb2rgb_gain = pAwb_Data->hGain_mid + pAwb_Data->hGain_minmax;
        }

        Y_offset = pAwb_Data->hMin/4;
        if (Y_offset >= 126)
            Y_offset = 126;
        DRV_ipipeSetYoffet(-Y_offset);*/

        rgb2rgb_gain = 0x400;

#ifdef ALG_AEWB_DEBUG
        OSA_printf(" AEWB: Middle rgb2rgb_gain = %d Y_offset = %d\n", rgb2rgb_gain, Y_offset);
#endif
        }

        ipipeWb.gainR  = 4*128;
        ipipeWb.gainGr = 4*128;
        ipipeWb.gainGb = 4*128;
        ipipeWb.gainB  = 4*128;
    }
    }

    DRV_ipipeSetWb(&ipipeWb);

    dGain = pAwb_Data->dGain*2;

    if (gALG_aewbObj.vnfDemoCfg)
    dGain = (512*8)-1;

    if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0)
    {
        if (lowlight)
            DRV_isifSetDgain(dGain, OSA_min(4095, (dGain * pAwb_Data->rGain) / 1024), OSA_min(4095, (dGain * pAwb_Data->bGain) / 1024), dGain, 0);
        else
            DRV_isifSetDgain(dGain, dGain + ((pAwb_Data->rGain % 64) * dGain / 1024), dGain + ((pAwb_Data->bGain % 64) * dGain / 1024), dGain, 0);
    } else if (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0)
    {
        if (pAwb_Data->rGain > pAwb_Data->bGain)
        {
            if (((dGain * pAwb_Data->rGain) / 1024) > 4095)
            {
                dGain = (4096 * 1024) / pAwb_Data->rGain;
            }
        } else
        {
            if (((dGain * pAwb_Data->bGain) / 1024) > 4095)
            {
                dGain = (4096 * 1024) / pAwb_Data->bGain;
            }
        }
        DRV_isifSetDgain(dGain, OSA_min(4095, (dGain * pAwb_Data->rGain) / 1024), OSA_min(4095, (dGain * pAwb_Data->bGain) / 1024), dGain, 0);
    } else
    {
        DRV_isifSetDgain(dGain, dGain, dGain, dGain, 0);
    }

    return 0;
}

short ALG_aewbSetIpipeWb2(AWB_PARAM  *pAwb_Data )
{
  DRV_IpipeWb ipipeWb;
  static AWB_PARAM PreAwb_Data;

  if( memcmp( &PreAwb_Data, pAwb_Data, sizeof(AWB_PARAM))== 0 )
      return 0;

  PreAwb_Data = *pAwb_Data;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB2: R Gr Gb B = (%d, %d, %d, %d) DGAIN = %d\n",
    pAwb_Data->rGain, pAwb_Data->grGain, pAwb_Data->gbGain, pAwb_Data->bGain, pAwb_Data->dGain
    );
  #endif

  ipipeWb.gainR  = pAwb_Data->rGain >> 1;
  ipipeWb.gainGr = pAwb_Data->grGain >> 1;
  ipipeWb.gainGb = pAwb_Data->gbGain >> 1;
  ipipeWb.gainB  = pAwb_Data->bGain >> 1;

  DRV_ipipeSetWb(&ipipeWb);

  return 0;
}


short ALG_aewbSetSensorDcsub(int dcsub)
{
  static int prevValue=-1;

  if( prevValue == dcsub )
      return 0;

  //#ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Sensor DCSUB = %d\n", dcsub);
  //#endif

  prevValue = dcsub;

  dcsub = -dcsub;

  DRV_isifSetDcSub(dcsub);

  return 0;
}

short ALG_aewbSetSensorBin(int bin)
{
  static int prevValue=-1;

  if(prevValue==bin)
    return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Sensor BIN = %d!!!!!!!!!!!!!!!!!Switch\n", bin);
  #endif

  prevValue = bin;

  if (gALG_aewbObj.vnfDemoCfg)
      DRV_imgsBinMode(bin=0x20);
  else
      DRV_imgsBinMode(bin);

  return 0;
}

short ALG_aewbSetFocus(int IsFocus)
{
	//Focus Control, 0:Stop, 1:Tele, 2:Wide
	static int prevValue=-1;

	if(prevValue == IsFocus)
		return 0;

#ifdef ALG_AEWB_DEBUG
	OSA_printf(" AEWB: Focus = %d\n", IsFocus);
#endif
	prevValue = IsFocus;
	DRV_imgsFocus(IsFocus);

	return 0;
}

short ALG_aewbSetZoom(int IsZoom)
{
	//Focus Control, 0:Stop, 1:Near, 2:Far
	static int prevValue=-1;

	if(prevValue == IsZoom)
		return 0;

#ifdef ALG_AEWB_DEBUG
	OSA_printf(" AEWB: Zoom = %d\n", IsZoom);
#endif
	prevValue = IsZoom;
	DRV_imgsZoom(IsZoom);

	return 0;
}

extern int gIRCut, gBWMode;

short ALG_aewbSetNDShutterOnOff(int bIsDay)
{
    //ND Shutter Control, 1:Day, 0:Night
    static int prevValue=-1;

#ifdef ALG_AEWB_DEBUG
    OSA_printf(" AEWB: NDShutter = %d Switch\n", bIsDay);
#endif
    prevValue = bIsDay;

    if (gIRCut == ALG_IRCUT_AUTO)
        DRV_imgsNDShutter(bIsDay, gBWMode);
    else if (gIRCut == ALG_IRCUT_OPEN)
        DRV_imgsNDShutter(0, 0);
    else if (gIRCut == ALG_IRCUT_CLOSE)
        DRV_imgsNDShutter(1, 0);
/*
    if (gIRCut == ALG_IRCUT_AUTO)
        ALG_SetHighGain(!bIsDay);

    if (gALG_aewbObj.aewbVendor == ALG_AEWB_ID_TI)
    {
	DRV_imgsSetWB(bIsDay, 0x20, 0x20);
    }
*/
    return 0;
}

short ALG_aewbSetRgb2Rgb(RGB2RGB_PARAM  *pRgb2Rgb )
{
  CSL_IpipeRgb2RgbConfig rgb2rgb;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: RGB2RGB \n");
  #endif

  if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9M034_720P") == 0)
  {
      // Need color correction for more best matrix
      rgb2rgb.matrix[0][0] = 427;
      rgb2rgb.matrix[0][1] = -105;
      rgb2rgb.matrix[0][2] = -66;

      rgb2rgb.matrix[1][0] = -99;
      rgb2rgb.matrix[1][1] = 422;
      rgb2rgb.matrix[1][2] = -67;

      rgb2rgb.matrix[2][0] = -8;
      rgb2rgb.matrix[2][1] = -78;
      rgb2rgb.matrix[2][2] = 342;
  } else
  if (strcmp(DRV_imgsGetImagerName(), "MICRON_AR0331_1080P") == 0)
  {
      // Need color correction for more best matrix
      rgb2rgb.matrix[0][0] = 380;
      rgb2rgb.matrix[0][1] = -59;
      rgb2rgb.matrix[0][2] = -66;

      rgb2rgb.matrix[1][0] = -89;
      rgb2rgb.matrix[1][1] = 402;
      rgb2rgb.matrix[1][2] = -57;

      rgb2rgb.matrix[2][0] = -8;
      rgb2rgb.matrix[2][1] = -98;
      rgb2rgb.matrix[2][2] = 362;
  } else
  if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") == 0)
  {
      // Need color correction for more best matrix
      rgb2rgb.matrix[0][0] = 380;
      rgb2rgb.matrix[0][1] = -59;
      rgb2rgb.matrix[0][2] = -66;

      rgb2rgb.matrix[1][0] = -89;
      rgb2rgb.matrix[1][1] = 402;
      rgb2rgb.matrix[1][2] = -57;

      rgb2rgb.matrix[2][0] = -8;
      rgb2rgb.matrix[2][1] = -168;
      rgb2rgb.matrix[2][2] = 432;
  } else
  if (strcmp(DRV_imgsGetImagerName(), "OMNIVISION_OV271X_1080P") == 0)
  {
      rgb2rgb.matrix[0][0] = 392;
      rgb2rgb.matrix[0][1] = -69;
      rgb2rgb.matrix[0][2] = -67;

      rgb2rgb.matrix[1][0] = -72;
      rgb2rgb.matrix[1][1] = 432;
      rgb2rgb.matrix[1][2] = -104;

      rgb2rgb.matrix[2][0] = -8;
      rgb2rgb.matrix[2][1] = -230;
      rgb2rgb.matrix[2][2] = 494;
  } else
  if (strcmp(DRV_imgsGetImagerName(), "SONY_IMX136_3MP") == 0)
  {
      // Need color correction for more best matrix
      rgb2rgb.matrix[0][0] = 380;
      rgb2rgb.matrix[0][1] = -59;
      rgb2rgb.matrix[0][2] = -66;

      rgb2rgb.matrix[1][0] = -89;
      rgb2rgb.matrix[1][1] = 402;
      rgb2rgb.matrix[1][2] = -57;

      rgb2rgb.matrix[2][0] = -8;
      rgb2rgb.matrix[2][1] = -168;
      rgb2rgb.matrix[2][2] = 432;
  } else
  {
  rgb2rgb.matrix[0][0] = pRgb2Rgb->rgb_mul_rr;
  rgb2rgb.matrix[0][1] = pRgb2Rgb->rgb_mul_gr;
  rgb2rgb.matrix[0][2] = pRgb2Rgb->rgb_mul_br;

  rgb2rgb.matrix[1][0] = pRgb2Rgb->rgb_mul_rg;
  rgb2rgb.matrix[1][1] = pRgb2Rgb->rgb_mul_gg;
  rgb2rgb.matrix[1][2] = pRgb2Rgb->rgb_mul_bg;

  rgb2rgb.matrix[2][0] = pRgb2Rgb->rgb_mul_rb;
  rgb2rgb.matrix[2][1] = pRgb2Rgb->rgb_mul_gb;
  rgb2rgb.matrix[2][2] = pRgb2Rgb->rgb_mul_bb;
  }

  if (ALTM_enable == 1)
  {
      // Need calibration
      rgb2rgb.matrix[0][0] = 256;
      rgb2rgb.matrix[0][1] = 0;
      rgb2rgb.matrix[0][2] = 0;

      rgb2rgb.matrix[1][0] = 0;
      rgb2rgb.matrix[1][1] = 256;
      rgb2rgb.matrix[1][2] = 0;

      rgb2rgb.matrix[2][0] = 0;
      rgb2rgb.matrix[2][1] = 0;
      rgb2rgb.matrix[2][2] = 256;
  }

  // Histogram Gain
  if (rgb2rgb_gain < 0x200)
      rgb2rgb_gain = 0x200;

  if (rgb2rgb_gain > 0xFFF)
      rgb2rgb_gain = 0xFFF;

  rgb2rgb.matrix[0][0] = (rgb2rgb.matrix[0][0]*rgb2rgb_gain) >> 10;
  rgb2rgb.matrix[0][1] = (rgb2rgb.matrix[0][1]*rgb2rgb_gain) >> 10;
  rgb2rgb.matrix[0][2] = (rgb2rgb.matrix[0][2]*rgb2rgb_gain) >> 10;

  rgb2rgb.matrix[1][0] = (rgb2rgb.matrix[1][0]*rgb2rgb_gain) >> 10;
  rgb2rgb.matrix[1][1] = (rgb2rgb.matrix[1][1]*rgb2rgb_gain) >> 10;
  rgb2rgb.matrix[1][2] = (rgb2rgb.matrix[1][2]*rgb2rgb_gain) >> 10;

  rgb2rgb.matrix[2][0] = (rgb2rgb.matrix[2][0]*rgb2rgb_gain) >> 10;
  rgb2rgb.matrix[2][1] = (rgb2rgb.matrix[2][1]*rgb2rgb_gain) >> 10;
  rgb2rgb.matrix[2][2] = (rgb2rgb.matrix[2][2]*rgb2rgb_gain) >> 10;

  rgb2rgb.offset[0]    = pRgb2Rgb->rgb_oft_or;
  rgb2rgb.offset[1]    = pRgb2Rgb->rgb_oft_og;
  rgb2rgb.offset[2]    = pRgb2Rgb->rgb_oft_ob;

  if(gALG_aewbObj.vnfDemoCfg == 0)
	DRV_ipipeSetRgb2Rgb(&rgb2rgb);

  return 0;
}
short ALG_aewbSetRgb2Rgb2(RGB2RGB_PARAM  *pRgb2Rgb )
{
  CSL_IpipeRgb2RgbConfig rgb2rgb;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: RGB2RGB2\n");
  #endif

  rgb2rgb.matrix[0][0] = pRgb2Rgb->rgb_mul_rr;
  rgb2rgb.matrix[0][1] = pRgb2Rgb->rgb_mul_gr;
  rgb2rgb.matrix[0][2] = pRgb2Rgb->rgb_mul_br;

  rgb2rgb.matrix[1][0] = pRgb2Rgb->rgb_mul_rg;
  rgb2rgb.matrix[1][1] = pRgb2Rgb->rgb_mul_gg;
  rgb2rgb.matrix[1][2] = pRgb2Rgb->rgb_mul_bg;

  rgb2rgb.matrix[2][0] = pRgb2Rgb->rgb_mul_rb;
  rgb2rgb.matrix[2][1] = pRgb2Rgb->rgb_mul_gb;
  rgb2rgb.matrix[2][2] = pRgb2Rgb->rgb_mul_bb;

  rgb2rgb.offset[0]    = pRgb2Rgb->rgb_oft_or;
  rgb2rgb.offset[1]    = pRgb2Rgb->rgb_oft_og;
  rgb2rgb.offset[2]    = pRgb2Rgb->rgb_oft_ob;

  DRV_ipipeSetRgb2Rgb2(&rgb2rgb);

  return 0;
}
short ALG_aewbSetOtfCorrect( int level )
{
    static int prevalue = -1;
    CSL_IpipeDpcConfig config;
    int levelD = 64;
    int levelC = 255;

    if( level == prevalue )
	return 0;

    prevalue = level;

#ifdef ALG_AEWB_DEBUG
    OSA_printf(" AEWB: Sensor OTF Level = %d\n", level);
#endif

    config.lutEnable		= 0;
    config.lutType		= 0;
    config.lutOption0CorMethod	= 0;
    config.lutStartAddr		= 0;
    config.lutNumEntries	= 0;
    config.lutAddr		= 0;

    config.otfEnable		= 1;
    config.otfType		= 0;
    config.otfAlg		= 0;
    config.otf2DetThres[0]	= level*levelD;
    config.otf2DetThres[1]	= level*levelD;
    config.otf2DetThres[2]	= level*levelD;
    config.otf2DetThres[3]	= level*levelD;

    config.otf2CorThres[0]	= level*levelC;
    config.otf2CorThres[1]	= level*levelC;
    config.otf2CorThres[2]	= level*levelC;
    config.otf2CorThres[3]	= level*levelC;


    config.otf3ActAdjust	= 0;
    config.otf3DetThres		= 0;
    config.otf3DetThresSlope	= 0;
    config.otf3DetThresMin	= 0;
    config.otf3DetThresMax	= 0;
    config.otf3CorThres		= 0;
    config.otf3CorThresSlope	= 0;
    config.otf3CorThresMin	= 0;
    config.otf3CorThresMax	= 0;

    DRV_ipipeSetDpcConfig(&config);
    return 0;
}

short ALG_aewbSetEdgeEnhancement(EDGE_PARAM  *pParm )
{
  CSL_IpipeEdgeEnhanceConfig config;
  static CSL_IpipeEdgeEnhanceConfig prev_config = {0};



  config.enable                   = pParm->yee_en;
  config.haloReduceEnable         = pParm->en_halo_red;
  config.mergeMethod              = pParm->merge_meth;
  config.hpfShift                 = pParm->yee_shf;
  config.hpfCoeff[0][0]           = pParm->yee_mul_00;
  config.hpfCoeff[0][1]           = pParm->yee_mul_01;
  config.hpfCoeff[0][2]           = pParm->yee_mul_02;
  config.hpfCoeff[1][0]           = pParm->yee_mul_10;
  config.hpfCoeff[1][1]           = pParm->yee_mul_11;
  config.hpfCoeff[1][2]           = pParm->yee_mul_12;
  config.hpfCoeff[2][0]           = pParm->yee_mul_20;
  config.hpfCoeff[2][1]           = pParm->yee_mul_21;
  config.hpfCoeff[2][2]           = pParm->yee_mul_22;
  config.lowerThres               = pParm->yee_thr;
  config.edgeSharpGain            = pParm->es_gain;
  config.edgeSharpHpValLowThres   = (pParm->es_thr1&0x3F);
  config.edgeSharpHpValUpLimit    = (pParm->es_thr2&0x3F);
  config.edgeSharpGainGradient    = pParm->es_gain_grad;
  config.edgeSharpOffsetGradient  = pParm->es_ofst_grad;
  config.table                    = yee_table;

  if(memcmp(&prev_config, &config, sizeof(config))==0)
    return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: EDGE ENHANCEMENT \n");
  #endif

  memcpy(&prev_config, &config, sizeof(config));

  if(gALG_aewbObj.vnfDemoCfg == 0)
	DRV_ipipeSetEdgeEnhance(&config);

  return 0;
}

short ALG_aewbSetContrastBrightness(BRT_CRT_PARAM  *pParm )
{
  static BRT_CRT_PARAM  ParmSet;

  if( memcmp(&ParmSet, pParm, sizeof(BRT_CRT_PARAM))== 0 )
      return 0;

  ParmSet = *pParm;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Contrast = %5d, Brightness = %5d\n", pParm->yuv_adj_ctr, pParm->yuv_adj_brt);
  #endif
  DRV_ipipeSetYoffet((pParm->yuv_adj_brt-128));

  pParm->yuv_adj_ctr += 2; // Increase contrast
  DRV_ipipeSetContrastBrightness(pParm->yuv_adj_ctr, 0x0);

  return 0;
}

short ALG_aewbSetSensorBinSkip(int Is_binning)
{
  static int prevValue=-1;

  if (prevValue==Is_binning)
      return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Sensor BinOrSkip = %d\n", Is_binning);
  #endif

  prevValue = Is_binning;

  DRV_imgsBinEnable(Is_binning);

  return 0;
}

short ALG_aewbSetSensor50_60Hz(int Is50Hz)
{
  static int prevValue = -1;



  if(prevValue==Is50Hz)
    return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" ALG_aewbSetSensor50_60Hz: Sensor 50_60Hz = %d\n", Is50Hz);
  #endif

  prevValue = Is50Hz;

  DRV_imgsSet50_60Hz(Is50Hz);
  OSA_printf(" ALG_aewbSetSensor50_60Hz: DRV_imgsSet50_60Hz\n");

  if( Is50Hz )
  {
      DRV_displaySetMode(DRV_DISPLAY_MODE_PAL);
  }else{
      DRV_displaySetMode(DRV_DISPLAY_MODE_NTSC);
  }
  OSA_printf(" ALG_aewbSetSensor50_60Hz: finish\n");

  return 0;
}

short ALG_aewbSetSensorFrameRate(int frame_rate_mode)
{
  static int prevValue=-1;
  int maxframerate;

  if (strcmp(DRV_imgsGetImagerName(), "MICRON_MT9P031_5MP") != 0)
      return 0;

  frame_rate_mode = OSA_max(frame_rate_mode, 5);

  if (gALG_aewbObj.numEncodes > 1)
  {
      switch (gALG_aewbObj.sensorMode)
      {
        case DRV_IMGS_SENSOR_MODE_2048x1536:
            maxframerate = 20;
            break;
        case DRV_IMGS_SENSOR_MODE_2304x1296:
            maxframerate = 14;
            break;
        case DRV_IMGS_SENSOR_MODE_2560x1440:
            maxframerate = 12;
            break;
        case DRV_IMGS_SENSOR_MODE_2592x1920:
            maxframerate = 9;
            break;
        default:
            maxframerate = 30;
            break;
      }
  } else
  {
      switch (gALG_aewbObj.sensorMode)
      {
        case DRV_IMGS_SENSOR_MODE_2048x1536:
            maxframerate = 20;
            break;
        case DRV_IMGS_SENSOR_MODE_2304x1296:
            maxframerate = 15;
            break;
        case DRV_IMGS_SENSOR_MODE_2560x1440:
            maxframerate = 13;
            break;
        case DRV_IMGS_SENSOR_MODE_2592x1920:
            maxframerate = 10;
            break;
        default:
            maxframerate = 30;
            break;
      }
  }
  frame_rate_mode = OSA_min(frame_rate_mode, maxframerate);

  if (prevValue==frame_rate_mode)
      return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Sensor frame-rate = %d\n", frame_rate_mode);
  #endif

  prevValue = frame_rate_mode;

  DRV_imgsSetFramerate(frame_rate_mode);

  return 0;
}

short ALG_aewbGetSensorFrameRate(int frame_rate)
{
  int FPS = 0;

  FPS = DRV_imgsGetFramerate();

  return FPS;
}

short ALG_aewbSetAEPriority (int ae_priority)
{
  static int prevValue=-1;

  if(prevValue==ae_priority)
    return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Sensor ae_priority = %d\n", ae_priority);
  #endif

  prevValue = ae_priority;

  DRV_imgsSetAEPriority(ae_priority);

  return 0;
}

short ALG_aewbSetAEPriorityMode (int ae_priority_mode)
{
  static int prevValue=-1;

  if(prevValue==ae_priority_mode)
    return 0;

  #ifdef ALG_AEWB_DEBUG
  OSA_printf(" AEWB: Sensor ae_priority_mode = %d\n", ae_priority_mode);
  #endif

  prevValue = ae_priority_mode;

  extern int gAePriorityMode;
  DRV_imgsSetAEPriorityMode(gAePriorityMode);

  return 0;
}

short ALG_aewbAFEnable  (int af_enable)
{
  gALG_aewbObj.afEnable = af_enable;
  return 0;
}

void ALG_aewbGetAEValues(Int32 *exposureTime, Int32 *apertureLevel, Int32 *sensorGain, Int32 *ipipeGain)
{
    *exposureTime 	= gALG_aewbObj.AE_OutArgs.nextAe.exposureTime;
    *apertureLevel 	= gALG_aewbObj.AE_OutArgs.nextAe.apertureLevel;
    *sensorGain 	= gALG_aewbObj.AE_OutArgs.nextAe.sensorGain;
    *ipipeGain 		= gALG_aewbObj.AE_OutArgs.nextAe.ipipeGain;
}

void ALG_aewbGetAWBGains(Uint16 *rGain, Uint16 *grGain, Uint16 *gbGain, Uint16 *bGain)
{
	if(gALG_aewbObj.aewbVendor == ALG_AEWB_ID_APPRO) {
		*rGain= gALG_aewbObj.AWB_OutArgs.nextWb.rGain;
		*grGain= gALG_aewbObj.AWB_OutArgs.nextWb.gGain;
		*gbGain= gALG_aewbObj.AWB_OutArgs.nextWb.gGain;
		*bGain= gALG_aewbObj.AWB_OutArgs.nextWb.bGain;
	}
	else {
		*rGain= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.wb.gain[0];
		*grGain= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.wb.gain[1];
		*gbGain= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.wb.gain[2];
		*bGain= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.wb.gain[3];
	}
}

void ALG_aewbGetRgb2Rgb(Int16*matrix)
{
    Uint32 i;
    static Int16 approMatrix[9] = {
	551, -324, 29,
	-39, 383, -88,
	9, -207, 454
    };

    if(gALG_aewbObj.aewbVendor==ALG_AEWB_ID_APPRO) {
	for (i=0; i<9; i++)
	    matrix[i] = approMatrix[i];
    }
    else {
	matrix[0]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[0][0];
	matrix[1]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[0][1];
	matrix[2]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[0][2];
	matrix[3]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[1][0];
	matrix[4]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[1][1];
	matrix[5]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[1][2];
	matrix[6]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[2][0];
	matrix[7]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[2][1];
	matrix[8]= gIMAGE_TUNE_ctrl.curPrm.ipipePrm.rgb2rgb1.matrix[2][2];
    }

}
