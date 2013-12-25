
#ifndef _DRV_IMGS_H_
#define _DRV_IMGS_H_

#include <drv.h>
#include <drv_csl.h>
#include <imageTuneParams.h>

typedef enum{
	DRV_IMGS_SENSOR_MODE_640x480 = 0,
	DRV_IMGS_SENSOR_MODE_720x480,
	DRV_IMGS_SENSOR_MODE_800x600,
	DRV_IMGS_SENSOR_MODE_1024x768,
	DRV_IMGS_SENSOR_MODE_1280x720,
	DRV_IMGS_SENSOR_MODE_1280x960,
	DRV_IMGS_SENSOR_MODE_1280x1024,
	DRV_IMGS_SENSOR_MODE_1600x1200,
	DRV_IMGS_SENSOR_MODE_1620x1080,
	DRV_IMGS_SENSOR_MODE_1920x1080,
	DRV_IMGS_SENSOR_MODE_2048x1536,
	DRV_IMGS_SENSOR_MODE_2304x1296,
	DRV_IMGS_SENSOR_MODE_2560x1440,
	DRV_IMGS_SENSOR_MODE_2592x1920
} DRV_IMGS_SENSOR_MODE;

#define DRV_IMGS_VNF_PAD_VALUE          (32)      // pads 16 extra pixels on all four sides, this is needed Katana NF
#define DRV_IMGS_SENSOR_MODE_PIXEL_PAD  (0x0200)  // pads 16 extra pixels on all four sides, this is needed Katana NF
#define DRV_IMGS_SENSOR_MODE_VSTAB      (0x0100)  // pads 10% extra pixels on all fours side, this is needed for video stabilization

#define DRV_IMGS_SENSOR_MODE_DEFAULT    (DRV_IMGS_SENSOR_MODE_1280x720)

// GPIO
#define IMGS_ZOOM_CTRL1	(38)
#define IMGS_ZOOM_CTRL2	(39)
#define IMGS_FOCUS_CTRL1	(40)
#define IMGS_FOCUS_CTRL2	(41)
#define IMGS_ND_CTRL1		(46)
#define IMGS_ND_CTRL2		(49)
#define IMGS_LIGHT_SENSOR	(45)

typedef struct {

  int  sensorMode;
  int  fps;
  Bool binEnable;
  Bool hdr;
  Bool hdr2DMotComp;
  Bool ALTM;

} DRV_ImgsConfig;

typedef struct {

  IMAGE_TUNE_CcdcParams ccdcParams;
  CSL_CcdcSyncConfig syncConfig;

} DRV_ImgsIsifConfig;

typedef struct {

  IMAGE_TUNE_IpipeifParams ipipeifParams;
  IMAGE_TUNE_IpipeParams   ipipeParams;

} DRV_ImgsIpipeConfig;

typedef struct {

  IMAGE_TUNE_LdcParams ldcParams;

} DRV_ImgsLdcConfig;

typedef struct {

  Uint16  medFiltThreshold;

  Bool    aewbMedFiltEnable;
  Uint16  aewbSatLimit;
  Uint16  aewbWinStartX;
  Uint16  aewbWinStartY;
  Uint16  aewbWinNumH;
  Uint16  aewbWinNumV;
  Uint8   aewbOutFormat;
  Uint8   aewbShift;

  Bool    afVfEnable;
  Bool    afMedFiltEnable;
  Uint8   afRgbPos;
  Uint16  afFvAccMode;
  Uint16  afPaxStartX;
  Uint16  afPaxStartY;
  Uint16  afPaxNumH;
  Uint16  afPaxNumV;
  Int32   afIirCoeff0[11];
  Int32   afIirCoeff1[11];
  Int32   afVfvFir1Coeff[5];
  Int32   afVfvFir2Coeff[5];
  Uint16  afVfvFir1Threshold;
  Uint16  afHfvFir1Threshold;
  Uint16  afVfvFir2Threshold;
  Uint16  afHfvFir2Threshold;

} DRV_ImgsH3aConfig;

typedef struct {

  int  sensorDataWidth;
  int  sensorDataHeight;
  int  validStartX;
  int  validStartY;
  int  validWidth;
  int  validHeight;
  Bool binEnable;

} DRV_ImgsModeConfig;


int DRV_imgsOpen(DRV_ImgsConfig *config);
int DRV_imgsClose(void);

const char* DRV_imgsGetImagerName(void);
int DRV_imgsReadReg(int RegNum);
int DRV_imgsSetFlicker(int Flicker);
int DRV_imgsSetND(int setND);

int DRV_imgsSpecificSetting(void);
int DRV_imgsSetAgain(int again, int setRegDirect);
int DRV_imgsSetEshutter(Uint32 eshutterInUsec, int setRegDirect);
int DRV_imgsSetDcSub(Uint32 dcSub, int setRegDirect);
int DRV_imgsBinEnable(Bool enable);
int DRV_imgsBinMode(int binMode);
int DRV_imgsSetFramerate(int fps);
int DRV_imgsGetFramerate(void);
int DRV_imgsSet50_60Hz(Bool is50Hz);
int DRV_imgsSetWB(int awb_mode, int red_gain, int blue_gain);
int DRV_imgsSetAEPriority(int ae_priority);
int DRV_imgsGetAEPriority(void);
int DRV_imgsSetAVGFilter(int AVG_filter);
void DRV_imgsSetAVG(int AVG);

int DRV_imgsSetAEPriorityMode(int ae_priority_mode);

int DRV_imgsEnable(Bool enable);

int DRV_imgsDoSnapshot(int snapshot);
int DRV_imgsNDShutter(int bIsNight, int BWMode);
int DRV_imgsNDShutterInit(void);

DRV_ImgsModeConfig      *DRV_imgsGetModeConfig(int sensorMode);
DRV_ImgsIsifConfig      *DRV_imgsGetIsifConfig(int sensorMode);
DRV_ImgsH3aConfig       *DRV_imgsGetH3aConfig(int sensorMode, int aewbVendor);
DRV_ImgsIpipeConfig     *DRV_imgsGetIpipeConfig(int sensorMode, int vnfDemoCfg );
DRV_ImgsLdcConfig       *DRV_imgsGetLdcConfig(int sensorMode, Uint16 ldcInFrameWidth, Uint16 ldcInFrameHeight);

#endif
