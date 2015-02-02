/*
 *  ======== ae_sig.c ========
 *  Sigrand's implementation of the AE algorithm.
 *
 *  This file contains an implementation of the IALG interface
 *  required by xDAIS.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <aewb_xdm.h>

#include "alg_aewb_priv.h"

#include "iaewbf_sig.h"
#include "aewbf_sig.h"
#include "alg_aewb.h"

#include <drv_motor.h>

#include <drv_ipipe.h>

#include <sc_env.h> //scam

extern DRV_IpipeObj gDRV_ipipeObj;    //For boxcar

// <osa.h> conflicts with dvsdk:xdc.h
extern int OSA_fileReadFile(const char *fileName, void *addr, size_t readSize, size_t *actualReadSize);
extern int OSA_fileWriteFile(const char *fileName, const void *addr, size_t size);
extern void OSA_waitMsecs(Uint32 msecs);
extern void GETTING_RGB_BLOCK_VALUE(unsigned short * BLOCK_DATA_ADDR, IAEWB_Rgb *rgbData, aewDataEntry *aew_data, int shift);

extern IAEWB_Rgb *rgbData;
extern aewDataEntry *aew_data;

extern IAEWBF_Fxns IAEWBF_SIG_IALG;
extern int gIRCut;
int IRcutClose = 1, FPShigh = 1;  //IR-cut 1-open, 0 - close
int fps_count = 0, ir_count;  //Dynamic theshould counter
int Ymax = 0;
int GN[3] = { -8, 0, 8}, wbup = 0;
int RR=0, GG=0, BB=0;
int bw = 0;

extern int DEBUG;
extern ALG_AewbfObj gSIG_Obj;
//#define HISTTH 30

int  frame_count = 0, leave_frames = 5, exp_on = 1, history = 0;

extern Uint32 gm003[], gm005[];



#define __DEBUG
#ifdef __DEBUG
#define AE_DEBUG_PRINTS
#define dprintf printf
#else
#define dprintf
#endif

#define IALGFXNS  \
    &IAEWBF_SIG_IALG,    /* module ID */                         \
    NULL,                /* activate */                          \
    IAEWBF_SIG_alloc,    /* alloc */                             \
    NULL,                /* control (NULL => no control ops) */  \
    NULL,                /* deactivate */                        \
    IAEWBF_SIG_free,     /* free */                              \
    IAEWBF_SIG_init,     /* init */                              \
    NULL,                /* moved */                             \
    NULL                 /* numAlloc (NULL => IALG_MAXMEMRECS) */

/*
 *  ======== AE_SIG_IAE ========
 *  This structure defines Sigrand's implementation of the IAE interface
 *  for the AE_SIG module.
 */
IAEWBF_Fxns IAEWBF_SIG = {    /* module_vendor_interface */
                          {IALGFXNS},
                          IAEWBF_SIG_process,
                          IAEWBF_SIG_control,
};

/*
 *  ======== AE_SIG_IALG ========
 *  This structure defines Sigrand's implementation of the IALG interface
 *  for the AE_SIG module.
 */
IAEWBF_Fxns IAEWBF_SIG_IALG = {      /* module_vendor_interface */
    {IALGFXNS}, NULL, NULL };

/*
 *  ======== AE_SIG_alloc ========
 */
Int IAEWBF_SIG_alloc(const IALG_Params *algParams,
                 IALG_Fxns **pf, IALG_MemRec memTab[])
{
    (void)algParams;
    (void)pf;
    int numTabs = 1;

    /* Request memory for my object */
    memTab[0].size = sizeof(IAEWBF_SIG_Obj);
    memTab[0].alignment = 0;
    memTab[0].space = IALG_EXTERNAL;
    memTab[0].attrs = IALG_PERSIST;
    return (numTabs);
}

/*
 *  ======== AE_SIG_free ========
 */
Int IAEWBF_SIG_free(IALG_Handle handle, IALG_MemRec memTab[])
{
    (void)handle;
    int numTabs = 1;
    /* Request memory for my object */
    memTab[0].size = sizeof(IAEWBF_SIG_Obj);
    memTab[0].alignment = 0;
    memTab[0].space = IALG_EXTERNAL;
    memTab[0].attrs = IALG_PERSIST;
    return (numTabs);
}

/*
 *  ======== AE_SIG_initObj ========
 */
Int IAEWBF_SIG_init(IALG_Handle handle,
		    const IALG_MemRec memTab[], IALG_Handle p,
		    const IALG_Params *algParams)
{
    (void)handle; (void)memTab; (void)p; (void)algParams;
    return (IAES_EOK);
}

int add_history(IAEWBF_Param *p)
{
    int diff = 0;
    p->Avrg += p->New;
    p->Avrg -= p->Hist[p->HistC];
    if(p->New) diff = abs(p->Hist[p->HistC] - p->New)*100/p->New;
    p->Hist[p->HistC] = p->New;
    p->HistC = (p->HistC == (HISTORY - 1)) ? 0 : p->HistC + 1;
    p->NewA = (history < HISTORY) ? p->Avrg/history : p->Avrg/HISTORY;
    return diff;
}

/*
 *  ======== AE_SIG_process ========
 */

XDAS_Int32 IAEWBF_SIG_process(IAEWBF_Handle handle, IAEWBF_InArgs *inArgs, IAEWBF_OutArgs *outArgs)
{
    (void)inArgs; (void)outArgs;
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    //int width = gALG_aewbObj.IAEWB_StatMatdata.winCtHorz;
    //int height = gALG_aewbObj.IAEWB_StatMatdata.winCtVert;
    //int pix = gALG_aewbObj.IAEWB_StatMatdata.pixCtWin;


    int tmp=0;
    size_t i, j;
    //const Uint16 w = hn->w, h = hn->h;
    Int32 w, h, sz, sz4;
    Uint16 r=0, g=0, b=0;
    Uint32 Y=0; // must be 32 bits
    static const size_t ns = 3;
    Uint32 GR[ns], GB[ns];
    //const Uint16 *box = hn->box;
    static const size_t hsz = ALG_SENSOR_BITS; // 512
    Uint32 minr, minb, min, up, dc;
    Uint32 hist[hsz], rgb[3][hsz];
    static size_t frames = 0;
    size_t sum;

    if(hn->pix) dc = (1<<12)/hn->pix;

    memset(GR, 0, sizeof(GR));
    memset(GB, 0, sizeof(GB));
    memset(hist, 0, sizeof(hist));

    if( Get_BoxCar( gSIG_Obj.handle_aewbf ) == OSA_SOK )
    {
        IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)(IAEWBF_Handle)gSIG_Obj.handle_aewbf;
        w = gDRV_ipipeObj.boxcarInfo.width;
        h = gDRV_ipipeObj.boxcarInfo.height;
        sz = w*h;  sz4 = sz*4;
        Uint16 *box = hn->box;

        for(i=0; i < sz4; i+=4) {
            r = box[i+2];
            g = box[i+1];
            b = box[i  ];

            RR += r; GG += g; BB += b;
            //Y += ((117*b + 601*g + 306*r)>>10);

            Uint32 r_ = r*hn->Rgain.Old>>14, g_ = g>>5, b_ = b*hn->Bgain.Old>>14;
            r_ = r_ < hsz ? r_ : hsz - 1;
            g_ = g_ < hsz ? g_ : hsz - 1;
            b_ = b_ < hsz ? b_ : hsz - 1;
            hist[r_]++;
            hist[g_]++;
            hist[b_]++;

            Y += ((117*b_ + 601*g_ + 306*r_)>>7);

            //for(j=0; j < ns; j++) {
                //GB[j] += abs(g - (b*(512 + GN[j])>>9));
                //GR[j] += abs(g - (r*(512 + GN[j])>>9));
                //GR[j] += abs(g - (r*(hn->Rgain.Old + GN[j])>>9));
                //GB[j] += abs(g - (b*(hn->Bgain.Old + GN[j])>>9));
            //}
        }
    }
    else
    {
        return;
    }

    Y = Y/sz;
    hn->Y.New = (Y == 0) ? 1 : Y;


    RR = RR/sz; GG = GG/sz; BB = BB/sz;
    printf("RR = %d GG = %d BB = %d\n", RR, GG, BB);

    //Find histogram min
    sum = 0;
    for(i=0; i < hsz && sum < hn->SatTh; i++) sum += hist[i];
    //for(i=0; i < hsz && !hist[i]; i++);
    hn->Hmin.New = i-1;
    //Find histogram max
    sum = 0;
    for(i=hsz-1; i >= 0 && sum < hn->SatTh; i--) sum += hist[i];
    hn->Hmax.New = i+1;

    //Averaging
    history++;
    add_history(&hn->Hmax);
    hn->Hmax.New = hn->Hmax.New<<3;
    hn->Hmax.NewA = hn->Hmax.NewA<<3;

    add_history(&hn->Hmin);
    hn->Hmin.New = hn->Hmin.New<<3;
    hn->Hmin.NewA = hn->Hmin.NewA<<3;

    add_history(&hn->Y);

    //White balance old algorithm
    if(frames > 3) {
        if( !(bw%3) ) {
            //if(FPShigh){
            if(0){
                //Day WB algorithm
                /*
                min = GR[0]; minr = 0;
                for(j=1; j < ns; j++){
                    if(GR[j] < min) { min = GR[j]; minr = j; }
                }
                min = GB[0]; minb = 0;
                for(j=1; j < ns; j++){
                    if(GB[j] < min) { min = GB[j]; minb = j; }
                }

                if(minr != 1) hn->Rgain.New = hn->Rgain.Old + (GN[minr]*hn->Rgain.Old>>9);
                if(minb != 1) hn->Bgain.New = hn->Bgain.Old + (GN[minb]*hn->Bgain.Old>>9);
                */
            } else {
                //Night WB algoritm
                if(RR < GG) hn->Rgain.New = hn->Rgain.Old + (GN[2]*hn->Rgain.Old>>9);
                if(RR > GG*133/128) hn->Rgain.New = hn->Rgain.Old - (GN[2]*hn->Rgain.Old>>9);
                if(BB < GG*123/128) hn->Bgain.New = hn->Bgain.Old + (GN[2]*hn->Bgain.Old>>9);
                if(BB > GG) hn->Bgain.New = hn->Bgain.Old - (GN[2]*hn->Bgain.Old>>9);
            }
        }
        bw++;

        //Check range
        //hn->Rgain.New = hn->Rgain.New > hn->Rgain.Range.max ? hn->Rgain.Range.max : hn->Rgain.New;
        //hn->Rgain.New = hn->Rgain.New < hn->Rgain.Range.min ? hn->Rgain.Range.min : hn->Rgain.New;
        //hn->Bgain.New = hn->Bgain.New > hn->Bgain.Range.max ? hn->Bgain.Range.max : hn->Bgain.New;
        //hn->Bgain.New = hn->Bgain.New < hn->Bgain.Range.min ? hn->Bgain.Range.min : hn->Bgain.New;
    }

    //scImgParams_t* pScParams = &(Aew_ext_parameter.scImgParams[ Aew_ext_parameter.scCurrentCamMode ]); //scam

    //if( (pScParams->fdMode == SCFdDisabled) && (pScParams->expMode != SCExpManual || pScParams->shutter == 0)) {
		
        if(hn->Y.New) tmp = (hn->Y.New > hn->YAE) ? hn->Y.New*100/hn->YAE : hn->YAE*100/hn->Y.New;
        if(tmp > 200){
            if(hn->Y.New) hn->Exp.New = hn->Exp.Old*(hn->Y.New*2 + hn->YAE)/(hn->Y.New*3);
        } else if(tmp > 120){
            if(hn->Y.New > hn->YAE) hn->Exp.New = hn->Exp.Old*99/100;
            else hn->Exp.New = hn->Exp.Old*100/99;
        }

        //if(hn->Exp.New > hn->Exp.Range.max)  hn->Exp.New = hn->Exp.Range.max;
        //if(hn->Exp.New < hn->Exp.Range.min)  hn->Exp.New = hn->Exp.Range.min;
    //}

    //Change the offset and gain
    //hn->Offset.New = hn->Hmin.NewA - 50;
    //if(hn->Offset.New < 0) hn->Offset.New = 0;

    //hn->Offset.New = hn->Y.NewA/12;
    //hn->Offset.New = hn->Hmin.NewA - hn->Offset.New;
    //if(hn->Offset.New < 0) hn->Offset.New = 0;
    hn->Offset.New = hn->Hmin.New/2; //0;

    if(hn->Y.NewA - hn->Offset.New) { //scam
        hn->GIFIF.New = (hn->YAE*512)/(hn->Y.NewA - hn->Offset.New);
        /*
		if(pScParams->expMode == SCExpManual)
		{
			int valMin = (pScParams->gainScope.min * 11808)/100 + 512;
			int valMax = (pScParams->gainScope.max * 11808)/100 + 512;
			if(valMin < valMax)
			{
				if(hn->GIFIF.New < valMin) hn->GIFIF.New = valMin; // 512 MIN
				if(hn->GIFIF.New > valMax) hn->GIFIF.New = valMax; // 12320 MAX (8192 + 7 bit rgb gain)
			}
		}
        */
	}
    //up = hn->Hmax.NewA*hn->GIFIF.New>>9;
    //if((up < hn->HmaxTh) && (hn->Y.NewA - hn->Offset.New))
    //    hn->GIFIF.New = (((hn->HmaxTh*2 - up)>>2)*512)/(hn->Y.NewA - hn->Offset.New);


    //If not enough IFIF gain add rgb2rgb gain
    /*
    if(hn->GIFIF.New > hn->GIFIF.Range.max) {
        hn->Grgb2rgb.New = (hn->GIFIF.New*256/hn->GIFIF.Range.max);
    } else {
		hn->Grgb2rgb.New = 256;
	}
    */
    //Check gain range
    //hn->GIFIF.New = hn->GIFIF.New > hn->GIFIF.Range.max ? hn->GIFIF.Range.max : hn->GIFIF.New;
    //hn->GIFIF.New = hn->GIFIF.New < hn->GIFIF.Range.min ? hn->GIFIF.Range.min : hn->GIFIF.New;
    //hn->Grgb2rgb.New = hn->Grgb2rgb.New > hn->Grgb2rgb.Range.max ? hn->Grgb2rgb.Range.max : hn->Grgb2rgb.New;
    //hn->Grgb2rgb.New = hn->Grgb2rgb.New < hn->Grgb2rgb.Range.min ? hn->Grgb2rgb.Range.min : hn->Grgb2rgb.New;

	//scam
    //Check Low light condition 
    //Got to night mode
    //Down FPS
    if ( FPShigh == 1 && IRcutClose == 1 && Y < hn->Threshold_IR_cut[0]) { //20
        frame_count += leave_frames;
        if (frame_count > FRAMES_TO_CLOSE_IR) {
            FPShigh = 0;
            frame_count = 0;
            fps_count = 1;
        }
    }
    //Open IR-cut
    if ( FPShigh == 0 && IRcutClose == 1 && Y < hn->Threshold_IR_cut[0]) { //20
        frame_count += leave_frames;
        if (frame_count > FRAMES_TO_CLOSE_IR) {
            IRcutClose = 0;
            frame_count = 0;
            ir_count = 1;
        }
    }

    //Come back to day mode
    //Close IR-cut
    if (FPShigh == 0 && IRcutClose == 0 && Y > hn->Threshold_IR_cut[2] && !ir_count) { //130
        frame_count += leave_frames;
        if (frame_count > FRAMES_TO_CLOSE_IR) {
            IRcutClose = 1;
            frame_count = 0;
        }
    }

    //UP FPS
    if (FPShigh == 0 && IRcutClose == 1 && Y > hn->Threshold_IR_cut[1] && !fps_count) { //130
        frame_count += leave_frames;
        if (frame_count > FRAMES_TO_CLOSE_IR) {
            FPShigh = 1;
            frame_count = 0;
        }
    }

    //Calculate dynamic threshold for FPS
    if(fps_count){
        frame_count += leave_frames;
        if (Ymax < hn->Y.New) Ymax = hn->Y.New;
        if (frame_count > FRAMES_TO_CLOSE_IR) {
            frame_count = 0;
            hn->Threshold_IR_cut[1] = Ymax*110/100;
            if(hn->Threshold_IR_cut[1] < hn->Threshold_IR_cut[0]*2)
                hn->Threshold_IR_cut[1] = hn->Threshold_IR_cut[0]*2;
            Ymax = 0;
            fps_count = 0;
        }
    }

    //Calculate dynamic threshold for IR-cut
    if(ir_count){
        frame_count += leave_frames;
        if (Ymax < hn->Y.New) Ymax = hn->Y.New;
        if (frame_count > FRAMES_TO_CLOSE_IR) {
            frame_count = 0;
            hn->Threshold_IR_cut[2] = Ymax*110/100;
            if(hn->Threshold_IR_cut[2] < hn->Threshold_IR_cut[0]*2)
                hn->Threshold_IR_cut[2] = hn->Threshold_IR_cut[0]*2;
            Ymax = 0;
            ir_count = 0;
        }
    }

    hn->Y.Old = hn->Y.New;
    frames++;
    return(IAES_EOK);
}

/*
 *  ======== AE_SIG_control ========
 */
XDAS_Int32 IAEWBF_SIG_control(IAEWBF_Handle handle, IAEWBF_Cmd id,
                              IAEWBF_DynamicParams *params, IAEWBF_Status *status)
{
    (void)handle; (void)id; (void)params; (void)status;
    return IAES_EOK;
}

enum choise {autofocus, zoomchange, af_end};

void AF_SIG_process(int *afEnable, ALG_AewbRunPrm *runPrm)
{
    Int32 i, focus_val = 0;
    Int32 MAX_STEP = 250;
    int status;
    char zoomvalue[4];
    int readsize, zoom, shift = 0;
    static int frames = 0, numframes = 0;
    static int zoomstep = 0;

    static Bool zoomdir = 0;
    static Bool focusdir = 1;
    static Bool dir = 0;

    static int maxstep, maxN, maxO;
    static int step = autofocus;

    static int firststep = 0;
    static int stepcnt = 0;
    static int AFMax = 0;

	if( Get_BoxCar( gSIG_Obj.handle_aewbf ) == OSA_SOK )
	{
		IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)(IAEWBF_Handle)gSIG_Obj.handle_aewbf;
		Int32 w = gDRV_ipipeObj.boxcarInfo.width;
		Int32 h = gDRV_ipipeObj.boxcarInfo.height;
		Int32 sz = w*h,  sz4 = sz*4;
		Uint16 *box = hn->box;

		for(i=5; i < sz4; i++) {
			focus_val += abs(box[i] - box[i-4]);
		}   
	}
	else
	{
		return;
	}
   
   
    if (!numframes) {
        status = OSA_fileReadFile("/var/run/zoom", zoomvalue, sizeof(zoomvalue), (size_t*)&readsize);

        if(status!=OSA_SOK) {
            OSA_printf("AF: error read from file\n");
            status = 0;
            zoom = 0;
        } else {
            zoom = atoi(zoomvalue);
        }

        if (zoom > 0 && zoom < 2000) {
            if (zoom > 1000) {
                zoomdir = TRUE;
                zoomstep = zoom - 1000;
            } else {
                zoomdir = FALSE;
                zoomstep = zoom;
            }
            dir = zoomdir;
            if (zoomstep) {
                step = zoomchange;
                //if (zoomstep > 10) zoomstep += 10;
                //else zoomstep *= 2;
                DRV_imgsMotorStep(MOTOR_FOCUS, dir, zoomstep>>1);
                sprintf(zoomvalue, "%04d", 0);
                status = OSA_fileWriteFile("/var/run/zoom", zoomvalue, sizeof(zoomvalue));
                if(status!=OSA_SOK) {
                    OSA_printf("AF: error write in file\n");
                }
            }
        }
        //numframes = 0;
        if(step != zoomchange) step = autofocus;
        AFMax = 0;
        maxO = 0;
#ifdef ALG_DEBUG_AF
	OSA_printf("step  = %d zoomstep = %d \n",step, zoomstep);
#endif
    }
    //step = coarse;
    numframes++;

    switch(step) {
    case zoomchange:
        //if (!numframes) {
        //    DRV_imgsMotorStep(MOTOR_FOCUS, dir, zoomstep>>1);
        //    maxO = focus_val;
        //} else {
        stepcnt +=2;
        DRV_imgsMotorStep(MOTOR_FOCUS, dir, 2); // 2 is optimal step for AF
        if(stepcnt > shift){
            //maxO = 0;
            if(focus_val > AFMax){
                maxN = focus_val;
                if(maxN < maxO){
                    DRV_imgsMotorStep(MOTOR_FOCUS, !dir, 13); //change to 13
                    step = af_end;
                    stepcnt = 0;
                }
                maxO = maxN;
            }
        }
        #ifdef ALG_DEBUG_AF
        OSA_printf("zoomchange stepcnt = %d focus_val = %ld maxN = %d maxO = %d focusdir = %d step = %d\n",
                   stepcnt, focus_val, maxN, maxO, focusdir, step);
        #endif
        if ((stepcnt - shift) > MAX_STEP) {
            step = af_end;
            stepcnt = 0;
        }
        //}
        break;
    case autofocus:
        if (firststep < 20) { // set start position and stabilize video
            if (firststep == 0) { // set start position
                focusdir = MOTOR_BACKWARD;
                DRV_imgsMotorStep(MOTOR_FOCUS, focusdir, MAX_STEP);
                //if(!focusdir) {
                //    DRV_imgsMotorStep(MOTOR_FOCUS, MOTOR_FORWARD, maxstep-4);
                //}
            }
            stepcnt = 0;
            firststep++;
            AFMax = focus_val;
        } else {
            stepcnt +=2;
            DRV_imgsMotorStep(MOTOR_FOCUS, !focusdir, 2); // 2 is optimal step for AF
            if(stepcnt > shift){
                if(focus_val > AFMax) {  AFMax = focus_val; maxstep = stepcnt; }
                if ((stepcnt - shift) > MAX_STEP) {
                    stepcnt = 0;
                    AFMax = AFMax*97/100;
                    maxO = 0;
                    dir = focusdir;
                    step = zoomchange;
                }
                #ifdef ALG_DEBUG_AF
                OSA_printf("autofocus stepcnt = %d focus_val = %ld AFMax = %d maxstep = %d focusdir = %d step = %d\n",
                           stepcnt, focus_val, AFMax, maxstep, focusdir, step);
                #endif
            }
        }
        break;
    case af_end:
        *afEnable = 0;
        stepcnt = 0;
        numframes = 0;
        firststep = 0;
        DRV_imgsMotorStep(MOTOR_FOCUS, MOTOR_BACKWARD, 0); // turn off gpio
        break;
    }

    frames++;
}
