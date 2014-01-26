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
#include "iaewbf_sig.h"
#include "aewbf_sig.h"
#include "alg_aewb.h"


extern IAEWBF_Fxns IAEWBF_SIG_IALG;
extern int gAePriorityMode, gBWMode, gDayNight, gIRCut, defaultFPS;
int IRcutClose = 1; //IR-cut 1-open, 0 - close
int FPShigh = 1; //FPS 1-high, 0 - low


int up = 1, downExp = 1, wbR = 0, wbB = 0, wbS = 0, gnS = 0;
int Rstep = 10, Bstep = 10, Rchange = 0, Bchange = 0;
Uint32 wb_frames = 0, diffY = 0, WBth = 0, dec_exp = 0;
Uint32 frames = 0, frame_count = 0;

#define __DEBUG
#ifdef __DEBUG
#define AE_DEBUG_PRINTS
#define dprintf printf
#else
#define dprintf
#endif

#define IALGFXNS  \
    &IAEWBF_SIG_IALG,        /* module ID */                         \
    NULL,                /* activate */                          \
    IAEWBF_SIG_alloc,        /* alloc */                             \
    NULL,                /* control (NULL => no control ops) */  \
    NULL,                /* deactivate */                        \
    IAEWBF_SIG_free,         /* free */                              \
    IAEWBF_SIG_init,         /* init */                              \
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
                               IALGFXNS
                        };

/*
 *  ======== AE_SIG_alloc ========
 */
Int IAEWBF_SIG_alloc(const IALG_Params *algParams,
                 IALG_Fxns **pf, IALG_MemRec memTab[])
{
    IAEWBF_Params *params = (IAEWBF_Params *)algParams;
    int numTabs = 1;

    /* Request memory for my object */
    memTab[0].size = sizeof(IAEWBF_SIG_Obj);
    memTab[0].alignment = 0;
    memTab[0].space = IALG_EXTERNAL;
    memTab[0].attrs = IALG_PERSIST;

    memTab[1].size = sizeof(XDAS_UInt32) * (params->numHistory + 1);
    memTab[1].alignment = 0;
    memTab[1].space = IALG_EXTERNAL;
    memTab[1].attrs = IALG_PERSIST;
    numTabs++;
    return (numTabs);
}

/*
 *  ======== AE_SIG_free ========
 */
Int IAEWBF_SIG_free(IALG_Handle handle, IALG_MemRec memTab[])
{
    IAEWBF_SIG_Obj *h = (IAEWBF_SIG_Obj *)handle;
    int numTabs = 0;
    /* Request memory for my object */
    memTab[0].size = sizeof(IAEWBF_SIG_Obj);
    memTab[0].alignment = 0;
    memTab[0].space = IALG_EXTERNAL;
    memTab[0].attrs = IALG_PERSIST;

    memTab[1].size = sizeof(XDAS_UInt32) * h->numHistory + 1;
    memTab[1].alignment = 0;
    memTab[1].space = IALG_EXTERNAL;
    memTab[1].attrs = IALG_PERSIST;
    numTabs++;
    return (numTabs);
}

/*
 *  ======== AE_SIG_initObj ========
 */
Int IAEWBF_SIG_init(IALG_Handle handle,
                const IALG_MemRec memTab[], IALG_Handle p,
                const IALG_Params *algParams)
{
    IAEWBF_SIG_Obj *h = (IAEWBF_SIG_Obj *)handle;
    IAEWBF_Params *params = (IAEWBF_Params *)algParams;
    int i;

    if(handle == NULL) return (IAES_EFAIL);
    if(params == NULL) {
        /* no static parameters passed in, use default */
        h->numHistory = 1;
        h->numSmoothSteps = 1;
        h->historyBrightness = NULL;
    } else if(params->size != sizeof(IAEWBF_Params)){
        return (IAES_EUNSUPPORTED);
    }else{
        h->numHistory = params->numHistory + 1;
        h->numSmoothSteps = params->numSmoothSteps;
        h->historyBrightness = memTab[1].base;
        for( i = 0; i < h->numHistory; i ++){
            h->historyBrightness[i] = -1;
        }
        //h->avgY = -1;
    }
    h->numRanges = 0;
    //h->ExpStep = 1;
    //h->targetBrightness = 200;
    //h->targetBrightnessRange.min = h->targetBrightness - 20;
    //h->targetBrightnessRange.max = h->targetBrightness + 20;
    //h->YRange.min = h->targetBrightness - 10;
    //h->YRange.max = h->targetBrightness + 10;
    //h->thrld = 40;
    //h->locked = FALSE;

    return (IAES_EOK);
}

/*
 *  ======== AE_SIG_process ========
 */
#define SAT_Y              180

XDAS_Int32 IAEWBF_SIG_process(IAEWBF_Handle handle, IAEWBF_InArgs *inArgs, IAEWBF_OutArgs *outArgs)
{
    //int width = inArgs->statMat.winCtHorz;
    //int height = inArgs->statMat.winCtVert;
    int numPixels = 56;
    unsigned int redSum = 0;
    unsigned int blueSum = 0;
    unsigned int greenSum = 0;
    unsigned int redSum_unsat = 0;
    unsigned int blueSum_unsat = 0;
    unsigned int greenSum_unsat = 0;
    unsigned int totalY, totalY_unsat;
    unsigned int avgY;
    unsigned int weightSum = 0;
    unsigned int weightSum_unsat = 0;
    unsigned int rY, bY, gY;
    //unsigned int newExp = inArgs->curAe.exposureTime;
    int newExpStep = 0;
    unsigned int newSensorGain = inArgs->curAe.sensorGain;
    //unsigned int newApertureLevel = inArgs->curAe.apertureLevel;
    unsigned int newIpipeGain = inArgs->curAe.ipipeGain;
    unsigned int curY, curY_unsat, curY_sat;
    IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;
    long long  adjRatio;
    unsigned int y, max_y = 0, min_y = 255 * numPixels;
    unsigned int temp;
    int pixelY = 0;
    int cnt_light = 0;
    int hdr_enable = 0;
    //unsigned int newExp = hn->Exp;


    int i, i1, i2, j;
    Uint32 w = hn->w, h = hn->h;//, h = h->h;
    int sz = w*h,  sz4 = sz*4, sz3 = sz*3, sz2 = sz3>>1;
    //int hv = sz3, hv2 = sz3>>1;
    //Uint32 R=0, G=0, B=0;
    Uint32  Y=0, Y1=0, ns = 3, GR[ns], GB[ns], GN[ns];
    Uint16 *box = hn->box;
    Uint32 hsz = ALG_SENSOR_BITS, leave_frames = 5;
    Uint32 sum, mins, maxs, tmp, th = hn->SatTh, thh, hc = 0, min, mini;
    Uint32 hist[hsz], cn = 0;

    //GN[0] = 1536; GN[1] = 1280; GN[2] = 1024; GN[3] = 768; GN[4] = 512;
    //N[0][0] = 1536; GN[0][1] = 1024; GN[0][4] = 512;
    //GN[1][0] = 1280; GN[1][1] = 1024; GN[1][4] = 768;
    //GN[0] = 1152; GN[1] = 1024; GN[2] = 896;
    GN[0] = 1088; GN[1] = 1024; GN[2] = 960;

    dprintf("frames = %d\n", frames);
    if(!(frames%leave_frames)){
        //Clear histogram
        memset(hist, 0, sizeof(Uint32)*hsz);
        memset(hn->RGB[0].hist, 0, sizeof(Uint32)*hsz);
        memset(hn->RGB[1].hist, 0, sizeof(Uint32)*hsz);
        memset(hn->RGB[2].hist, 0, sizeof(Uint32)*hsz);
        for(j=0; j < ns; j++) { GR[j] = 0; GB[j] = 0; }

        for(i=0; i < sz4; i+=4) {
            i1 = i+1; i2 = i+2;
            Y1 = (117*box[i  ] + 601*box[i1] + 306*box[i2])>>12;
            //if(Y1 > hn->Hhalf) {
                //B += box[i ];
                //G += box[i1];
                //R += box[i2];
            //if(wbS){
                for(j=0; j < ns; j++) {
                    GB[j] += abs(box[i1] - (box[i ]*GN[j]>>10));
                    GR[j] += abs(box[i1] - (box[i2]*GN[j]>>10));
                }
            //}
            //hc++;
            //}
            Y += Y1;
            hn->RGB[0].hist[box[i2]>>5]++;
            hn->RGB[1].hist[box[i1]>>5]++;
            hn->RGB[2].hist[box[i ]>>5]++;
            //hist[Y1>>3]++;
            cn++;
        }
        /*
        if(hc) {
            R = R/hc>>2; G = G/hc>>2; B = B/hc>>2;
            GR = GR/hc; GB = GB/hc;
        } else {
            R = Y/sz; G = Y/sz; B = Y/sz;
            GR = 1; GB = 1;
        }
        */

        Y = Y/sz;
        for(j=0; j < ns; j++) { GR[j] = GR[j]/sz; GB[j] = GB[j]/sz; }

        for(i=0; i < hsz; i++) hist[i] = hn->RGB[0].hist[i] + hn->RGB[1].hist[i] + hn->RGB[2].hist[i];
        //for(i=0; i < hsz; i++) dprintf("%d   Y = %d R = %d G = %d B = %d\n", i, hist[i], hn->RGB[0].hist[i], hn->RGB[1].hist[i], hn->RGB[2].hist[i]);

        sum = 0;
        thh = hn->HminTh>>3;
        for(j=0; j <= thh; j++) sum += hist[j];
        if(!sum){
            for(i=j; sum < th; i++) sum += hist[i];
            hn->Hmin[0] = i<<3; hn->Hmin[1] = hist[j];
        } else { hn->Hmin[0] = hn->HminTh; hn->Hmin[1] = sum; }

        //Find half of histogram
        thh = sz3>>1;
        for(i=0; sum < thh; i++) sum += hist[i];
        hn->Hhalf = (i<<12)/hn->GISIF.New; //Half of histogram in real sensor value

        sum = 0;
        thh = hn->HmaxTh>>3;
        for(i=hsz-1; sum < hn->SatTh; i--) sum += hist[i];
        hn->Hmax[0] = i<<3; hn->Hmax[1] = sum;

        /*
        for(j=hsz-1; j > thh; j--) sum += hist[j];
        if(!sum){
            for(i=j; sum < th; i--) sum += hist[i];
            hn->Hmax[0] = i<<3; hn->Hmax[1] = 1;
        } else { hn->Hmax[0] = hn->HmaxTh; hn->Hmax[1] = sum; }
        */


        //Find max and min in histogram
        for(j=0; j < 3; j++){
            /*
            sum = 0;
            for(i=0; sum < th; i++) sum += hn->RGB[j].hist[i];
            hn->RGB[j].min = i<<3; hn->RGB[j].mins = sum;
            sum = 0;
            for(i=hsz-1; sum < th; i--) sum += hn->RGB[j].hist[i];
            hn->RGB[j].max = i<<3; hn->RGB[j].maxs = sum;
            */

            for(i=0; !hn->RGB[j].hist[i]; i++);
            hn->RGB[j].min = i<<3; hn->RGB[j].mins = hn->RGB[j].hist[i];
            sum = 0;
            thh = hn->RGB[j].MaxTh>>3;
            for(i=hsz-1; i > thh; i--) sum += hn->RGB[j].hist[i];
            hn->RGB[j].max = hn->RGB[j].MaxTh; hn->RGB[j].maxs = sum;
        }


#ifdef AE_DEBUG_PRINTS
        dprintf("sz = %u cn = %d Y = %u min = %u minv = %u max = %u maxv = %u SatTh = %u\n",
                sz, cn, Y, hn->Hmin[0], hn->Hmin[1], hn->Hmax[0], hn->Hmax[1],  hn->SatTh);
        //dprintf("GR[0] = %u GR[1] = %u GR[2] = %u GR[3] = %u GR[4] = %u \n", GR[0], GR[1], GR[2], GR[3], GR[4]);
        //dprintf("GB[0] = %u GB[1] = %u GB[2] = %u GB[3] = %u GB[4] = %u \n", GB[0], GB[1], GB[2], GB[3], GB[4]);
        dprintf("GR[0] = %u GR[1] = %u GR[2] = %u \n", GR[0], GR[1], GR[2]);
        dprintf("GB[0] = %u GB[1] = %u GB[2] = %u \n", GB[0], GB[1], GB[2]);
        dprintf("Rmin = %u Rmins = %u Rmax = %u Rmaxs = %u  \n", hn->RGB[0].min, hn->RGB[0].mins, hn->RGB[0].max, hn->RGB[0].maxs);
        dprintf("Gmin = %u Gmins = %u Gmax = %u Gmaxs = %u  \n", hn->RGB[1].min, hn->RGB[1].mins, hn->RGB[1].max, hn->RGB[1].maxs);
        dprintf("Bmin = %u Bmins = %u Bmax = %u Bmaxs = %u  \n", hn->RGB[2].min, hn->RGB[2].mins, hn->RGB[2].max, hn->RGB[2].maxs);
#endif
        //White balance algorithm
        min = GR[0]; mini = 0;
        for(j=1; j < ns; j++){
            if(GR[j] < min) { min = GR[j]; mini = j; }
        }
        if(mini != 1){
            hn->RGBgain[0] = hn->RGBgain[0]*GN[mini]>>10;
            hn->RGBgain[0] = hn->RGBgain[0] > hn->GISIF.Range.max ? hn->GISIF.Range.max : hn->RGBgain[0];
            hn->RGBgain[0] = hn->RGBgain[0] < hn->GISIF.Range.min ? hn->GISIF.Range.min : hn->RGBgain[0];

            dprintf("mini = %d GN[mini] = %d hn->RGBgain[0] = %d \n", mini, GN[mini], hn->RGBgain[0]);
        }

        min = GB[0]; mini = 0;
        for(j=1; j < ns; j++){
            if(GB[j] < min) { min = GB[j]; mini = j; }
        }
        if(mini != 1){
            hn->RGBgain[2] = hn->RGBgain[2]*GN[mini]>>10;
            hn->RGBgain[2] = hn->RGBgain[2] > hn->GISIF.Range.max ? hn->GISIF.Range.max : hn->RGBgain[2];
            hn->RGBgain[2] = hn->RGBgain[2] < hn->GISIF.Range.min ? hn->GISIF.Range.min : hn->RGBgain[2];

            dprintf("mini = %d GN[mini] = %d hn->RGBgain[2] = %d \n", mini, GN[mini], hn->RGBgain[2]);
        }


        //Check Y history of difference
        if(Y > hn->Y.Max) hn->Y.Max = Y;
        if(Y < hn->Y.Min) hn->Y.Min = Y;
        hn->Y.Diff = (hn->Y.Max - hn->Y.Min)*100/hn->Y.Max;

#ifdef AE_DEBUG_PRINTS
        dprintf("Y.Min = %d Y.Max = %d Y.Diff = %d Y.Th = %d\n", hn->Y.Min, hn->Y.Max, hn->Y.Diff, hn->Y.Th);
        dprintf("Hmin = %d Hhalf = %d Hmax = %d diff = %d\n", hn->Hmin[0], hn->Hhalf, hn->Hmax[0], hn->Hhalf*hn->GISIF.New>>9);
#endif

        //AE algorithm
        //if(!wbS && !wbR && !wbB) {
            //Offset
            /*
            if(hn->Hmin[0] != 0){
                //Increase offset
                hn->Offset.New = hn->Offset.Old + hn->Hmin[0]*512/hn->Gain.Old;
            } else {
                if(hn->Hmin[1] > hn->SatTh){
                    //Decrease offset
                    hn->Offset.New = hn->Offset.Old*(hv-hn->Hmin[1])/hv;
                }
            }
            */
            //Expouse and gain
            //if(hn->Hmax[1] > hn->SatTh || hn->RGB[0].maxs > hn->SatTh || hn->RGB[1].maxs > hn->SatTh || hn->RGB[2].maxs > hn->SatTh){
            if(hn->RGB[0].maxs > hn->SatTh || hn->RGB[1].maxs > hn->SatTh || hn->RGB[2].maxs > hn->SatTh){
                dprintf("DOWN!!!!!!!!!!!!!!!!!\n");
                //Find max step
                //mins = ((sz3 - hn->Hmax[1])<<10)/sz3;
                mins = ((sz - hn->RGB[0].maxs)<<10)/sz;
                for(j=1; j < 3; j++) {
                    tmp = ((sz - hn->RGB[j].maxs)<<10)/sz;
                    mins = tmp < mins ? tmp : mins;
                }
                dprintf("mins = %d g = %f \n", mins, (float)mins/1024.);

                if(hn->GISIF.Old  > 512){
                    //Decrease gain at first
                    //hn->Gain.New = hn->Gain.Old*(hv-hn->Hmax[1])/hv;
                    hn->GISIF.New = hn->GISIF.Old*mins>>10;
                    if(hn->GISIF.New < 512) hn->GISIF.New = 512;
                } else {
                    //Decrease expouse at second
                    //hn->Exp.New = hn->Exp.Old*(hv-hn->Hmax[1])/hv;
                    hn->Exp.New = hn->Exp.Old*mins>>10;
                    if(hn->Exp.New < hn->Exp.Range.min)  hn->Exp.New = hn->Exp.Range.min;
                }
                hn->Y.Diff = 0;
                hn->Y.Max = Y;
                hn->Y.Min = Y;
                up = 0;

            } else if (hn->Y.Diff > hn->Y.Th || up){
                dprintf("UP!!!!!!!!!!!!!!!!!\n");
                //Check max
                maxs = hn->Y.Diff > hn->Y.Th ? hn->Y.Diff : hn->Y.Th;
                //maxs = hn->Y.Th;
                //Increase expouse at first
                if(hn->Exp.Old < hn->Exp.Range.max ) {
                    //hn->Exp.New = hn->Exp.Old*hn->HmaxTh/hn->Hmax[0];
                    hn->Exp.New = hn->Exp.Old*(100 + maxs)/100;
                    if(hn->Exp.New > hn->Exp.Range.max){
                        hn->Exp.New = hn->Exp.Range.max;
                        //wbS = 1;
                    }
                } else {
                    //Increase gain at second
                    //hn->maxi = hn->RGBgain[0] > hn->RGBgain[2] ? 0 : 2;
                    //hn->maxi = hn->RGBgain[hn->maxi] > hn->RGBgain[1] ? hn->maxi : 1;
                    if(hn->Hmax[0]) hn->GISIF.New = hn->GISIF.Old*(100 + maxs)/100;
                    hn->GISIF.New = hn->GISIF.New > hn->GISIF.Range.max ? hn->GISIF.Range.max : hn->GISIF.New;
                }

                //Low light condition
                if(gIRCut == ALG_IRCUT_AUTO){
                    //Got to night mode
                    if ( IRcutClose == 1 && hn->Hhalf < 130) {
                        frame_count += leave_frames;
                        if (frame_count > 100) {
                            IRcutClose = 0;
                            frame_count = 0;
                        }
                    }
                    //Come back to day mode
                    if ( IRcutClose == 0 && hn->Hhalf > 180) {
                        frame_count += leave_frames;
                        if (frame_count > 100) {
                            IRcutClose = 1;
                            frame_count = 0;
                        }
                    }
                }
                if(gAePriorityMode == ALG_FPS_LOW || gAePriorityMode == ALG_FPS_5FPS){
                    /*
                    if ( FPShigh == 1 && hn->Hhalf < 128) {
                        frame_count += leave_frames;
                        if (frame_count > 100) {
                            FPShigh == 0;
                            frame_count = 0;
                        }
                    }
                    */
                }
            }

            //Remove gup when changes.
            if(hn->GISIF.New > hn->GISIF.Old){
                hn->GIFIF.New = hn->GIFIF.Old*100/(100 + maxs);
            } else if (hn->GISIF.New < hn->GISIF.Old){
                hn->GIFIF.New = hn->GIFIF.Old<<10/mins;
            } else {
                hn->GIFIF.New = (3800<<9)/hn->Hmax[0];
            }
            hn->GIFIF.New = hn->GIFIF.New >  hn->GIFIF.Range.max ? hn->GIFIF.Range.max : hn->GIFIF.New;
            hn->GIFIF.New = hn->GIFIF.New <  hn->GIFIF.Range.min ? hn->GIFIF.Range.min : hn->GIFIF.New;

            dprintf("lowlight = %d gain = %d\n", hn->lowlight, hn->GIFIF.New);

        //}


#ifdef AE_DEBUG_PRINTS
        dprintf("up = %d downExp = %d wbR = %d wbB = %d wbS = %d gnS = %d dec_exp = %d\n", up, downExp, wbR, wbB, wbS, gnS, dec_exp);
        dprintf("Exp.Old = %d Exp.New = %d Offset.Old = %d Offset.New = %d\n",
                hn->Exp.Old, hn->Exp.New, hn->Offset.Old, hn->Offset.New);
        dprintf("gain = %d Rgain = %d Ggain = %d Bgain = %d\n", hn->GISIF.New, hn->RGBgain[0], hn->RGBgain[1],  hn->RGBgain[2]);
#endif

        //Setup ipipe gains and offset
        //hn->Offset = -(hn->min[0]*6>>3);
        //hn->Offset = 0;
        //if(hn->max[0]) hn->Gain = (4095<<9)/(hn->max[0] + hn->Offset);
        //hn->Gain  = hn->Gain > 8190 ? 8190 : hn->Gain;

        //Find max gain
        //maxg = hn->RGBgain[0] > hn->RGBgain[2] ? hn->RGBgain[0] : hn->RGBgain[2];
        //maxg = maxg > hn->RGBgain[1] ? maxg : hn->RGBgain[1];


        int dm = 512, p[dm+1], d, a, b, hmax, hmin;
        int  vl0, vl1, st, sum1, step, lp;
        //Tone mapping

        hmax = hn->Hmax[0]>>3;
        hmin = hn->Hmin[0]>>3;
        d = hmax - hmin;
        a = (1<<16)/d, b = (1<<17)/sz;

        dprintf("Hmin = %d Hmax = %d d = %d \n", hmin, hmax, d);
        //Linear
        //for(i=0; i < hmin; i++) p[i] = 0;
        //for(i = hmin; i < hmax; i++) p[i] = ((i - hmin)<<10)*a>>16;
        //for(i=hmax; i <= dm; i++) p[i] = 1023;

        //Integral

        for(j=0; j < 3; j++){
            sum = 0;
            for(i=0; i < hsz; i++) {
                sum += hn->RGB[j].hist[i];
                hn->RGB[j].hist[i] = sum*b>>7;
            }
        }

        for(j=0; j < 3; j++){
            hn->RGB[j].hist[0] = 0;
            vl0 = 0;
            for(i=1; i < dm; i++){
                vl1 = hn->RGB[j].hist[i];
                //vl1 = i<<1;
                //hn->lut[i] = (vl0<<10) | (vl1 - vl0);
                hn->RGB[j].hist[i] = (vl0<<10) | (vl1 - vl0);
                //if(j==1) dprintf("%d  hist = %d  %d  %d\n", i, hn->RGBh[j][i], vl0, (vl1 - vl0));
                vl0 = vl1;
            }
        }
        /*
        for(st=1; st < 9; st++, d>>=1){
            for(j=0; j < dm; j+=d){
                for(i=p[j], sum1=0; i < p[j+d]; i++) sum1 += hist[i];
                //Remove holes from histogramm
                //sp = sum1/d;
                //for(i=p[j]; i < p[j+d]; i++) if(hist[i] > sp) { hist[i] = sp; sum1 -= (sp-hist[i]); }

                sum1>>=1;
                for(i=p[j], sum=0; sum < sum1; i++){
                    sum += hist[i];
                    //printf("i = %d sum = %d\n", i, sum);
                }
                lp = (p[j+d] + p[j])>>1;
                //p[j+(d>>1)] = lp + (i - lp)>>1;
                p[j+(d>>1)] = i;

                //printf("size = %d st = %d d = %d j = %d lp = %d ip = %d p[%d] = %d p[%d] = %d p[%d] = %d\n",
                //       size3>>st, st, d, j, lp, i, j+d, p[j+d], j, p[j], j+(d>>1), p[j+(d>>1)]);
            }
        }
        //for(i = 0; i <= 256; i++) printf("p[%d] = %d\n", i, p[i]);
        */

        //hn->R = R; hn->G = G; hn->B = B;
        hn->Y.New = Y;
        for(j=0; j < ns; j++) { hn->GR[j] = GR[j]; hn->GB[j] = GB[j]; }
        hn->Y.Old = Y;
    }

    frames++;

    return(IAES_EOK);
}

/*
 *  ======== AE_SIG_control ========
 */
XDAS_Int32 IAEWBF_SIG_control(IAEWBF_Handle handle, IAEWBF_Cmd id,
                              IAEWBF_DynamicParams *params, IAEWBF_Status *status)
{
    XDAS_Int32 retVal;
    IAEWBF_SIG_Obj *h = (IAEWBF_SIG_Obj *)handle;
    int i;
    /* validate arguments - this codec only supports "base" xDM. */


    dprintf("IAEWBF_SIG_control\n");

    if (params->size != sizeof(*params)){
        return (IAES_EUNSUPPORTED);
    }

    switch (id) {
    case IAEWBF_CMD_SET_CONFIG:
        if(params->numRanges > IAES_MAX_RANGES) {
            retVal = IAES_EFAIL;
        }else {
            h->numRanges = params->numRanges;
            for(i = 0; i < h->numRanges; i ++){
                //h->sensorGainRange[i] = params->sensorGainRange[i];
                //h->ipipeGainRange[i] = params->ipipeGainRange[i];
                //h->isifGainRange[i] = params->isifGainRange[i];
            }
            //h->ExpRange = params->ExpRange;
            //h->YRange = params->YRange;
            //h->maxDiffY = params->maxDiffY;
            h->thrld = params->thrld;
            h->targetBrightness = params->targetBrightness;
            //h->ExpStep = params->ExpStep;
            h->locked = FALSE;
            retVal = IAES_EOK;
        }
        break;
    case IAEWBF_CMD_GET_CONFIG:
        params->numRanges = h->numRanges;
        for(i = 0; i < h->numRanges; i ++){
            //params->sensorGainRange[i] = h->sensorGainRange[i];
            //params->ipipeGainRange[i] = h->ipipeGainRange[i];
            //params->isifGainRange[i] = h->isifGainRange[i];
        }
        //params->ExpRange = h->ExpRange;
        //params->YRange = h->YRange;
        //params->maxDiffY = h->maxDiffY;
        params->thrld = h->thrld;
        params->targetBrightness = h->targetBrightness;
        //params->ExpStep = h->ExpStep;
        retVal = IAES_EOK;
        break;

    default:
        /* unsupported cmd */
        retVal = IAES_EUNSUPPORTED;
        break;
    }
    return (retVal);
}

