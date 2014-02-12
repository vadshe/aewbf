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
extern int gIRCut;
//extern int gAePriorityMode, gBWMode, gDayNight, defaultFPS;
int IRcutClose = 1; //IR-cut 1-open, 0 - close
int FPShigh = 1; //FPS 1-high, 0 - low
extern int gHDR;


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
    return (numTabs);
}

/*
 *  ======== AE_SIG_free ========
 */
Int IAEWBF_SIG_free(IALG_Handle handle, IALG_MemRec memTab[])
{
    IAEWBF_SIG_Obj *h = (IAEWBF_SIG_Obj *)handle;
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
    return (IAES_EOK);
}

/*
 *  ======== AE_SIG_process ========
 */

XDAS_Int32 IAEWBF_SIG_process(IAEWBF_Handle handle, IAEWBF_InArgs *inArgs, IAEWBF_OutArgs *outArgs)
{
     IAEWBF_SIG_Obj *hn = (IAEWBF_SIG_Obj *)handle;

    int i, i1, i2, j;
    Uint32 w = hn->w, h = hn->h;//, h = h->h;
    int sz = w*h,  sz4 = sz*4, sz3 = sz*3, sz2 = sz3>>1;
    //int hv = sz3, hv2 = sz3>>1;
    Uint32 R=0, G=0, B=0;
    Uint32 r, g, b;
    Uint32  Y=0, Y1=0, ns = 3, GR[ns], GB[ns];
    Uint16 *box = hn->box;
    Uint32 hsz = ALG_SENSOR_BITS, leave_frames = 5;
    Uint32 sum, mins, minr, minb, maxs, maxg, max_rgb, tmp, th = hn->SatTh, thh, hc = 0, min, mini;
    Uint32 cn = 0, fp = 512, sp = 2304, fg = 1, sg = 7;
    Uint32 hist[hsz];
    Uint32 gr, gb;
    int GN[3];

    GN[0] = 16; GN[1] = 0; GN[2] = -16;

    dprintf("frames = %d\n", frames);

    gr = (1<<27)/hn->RGBgain[0];
    gb = (1<<27)/hn->RGBgain[2];

    if(!(frames%leave_frames) && frames > 6){
        //Clear histogram
        memset(hist, 0, sizeof(Uint32)*hsz);
        memset(hn->RGB[0].hist, 0, sizeof(Uint32)*hsz);
        memset(hn->RGB[1].hist, 0, sizeof(Uint32)*hsz);
        memset(hn->RGB[2].hist, 0, sizeof(Uint32)*hsz);
        for(j=0; j < ns; j++) { GR[j] = 0; GB[j] = 0; }

        for(i=0; i < sz4; i+=4) {
            //i1 = i+1; i2 = i+2;
            r = box[i+2]>>2;
            g = box[i+1]>>2;
            b = box[i  ]>>2;

            Y1 = (117*b + 601*g + 306*r)>>10;

            hn->RGB[0].hist[r>>3]++;
            hn->RGB[1].hist[g>>3]++;
            hn->RGB[2].hist[b>>3]++;

            if(1){ //HDR

                //R = r*gr>>18; G = g; B = b*gb>>18;
                if(r > sp) r = ((r - sp)<<sg) + sp;
                else if(r > fp) r = ((r - fp)<<fg) + fp;

                if(g > sp) g = ((g - sp)<<sg) + sp;
                else if(g > fp) g = ((g - fp)<<fg) + fp;

                if(b > sp) b = ((b - sp)<<sg) + sp;
                else if(b > fp) b = ((b - fp)<<fg) + fp;

                for(j=0; j < ns; j++) {
                    GB[j] += abs(g - (b*(hn->RGBgain[2] + GN[j])>>8));
                    GR[j] += abs(g - (r*(hn->RGBgain[0] + GN[j])>>8));
                }
            } else {
                //if(Y > Yold){
                    for(j=0; j < ns; j++) {
                        GB[j] += abs(g - (b*(512 + GN[j])>>9));
                        GR[j] += abs(g - (r*(512 + GN[j])>>9));
                    }
                //}
            }

            //r = r > 1024 ? (r<<2) - c : r;
            //g = g > 1024 ? (g<<2) - c : g;
            //b = b > 1024 ? (b<<2) - c : b;

            hist[Y1>>3]++;


            //B += b;
            //G += g;
            //R += r;
            Y += Y1;
            cn++;
        }

        Y = Y/sz;
        //R = R/sz; G = G/sz; B = B/sz;
        //Yold = Y;

        //for(j=0; j < ns; j++) { GR[j] = GR[j]/sz; GB[j] = GB[j]/sz; }

        //for(i=0; i < hsz; i++) hist[i] = hn->RGB[0].hist[i] + hn->RGB[1].hist[i] + hn->RGB[2].hist[i];
        //for(i=0; i < hsz; i++) dprintf("%d   Y = %d R = %d G = %d B = %d\n", i, hist[i], hn->RGB[0].hist[i], hn->RGB[1].hist[i], hn->RGB[2].hist[i]);

        /*
        //Find histogram minimum
        sum = 0;
        for(i=0; sum < hn->SatTh; i++) sum += hist[i];
        hn->Hmin[0] = (i-1)<<3; hn->Hmin[1] = sum;

         //Find histogram maximum
        sum = 0;
        for(i=hsz-1; sum < hn->SatTh; i--) sum += hist[i];
        hn->Hmax.New = (i+1)<<3;
        //for(i=hsz-1; !hist[i]; i--);
        //hn->Hmax.New = i<<3;


        //Find half of histogram
        thh = sz>>1;
        sum = 0;
        for(i=0; sum < thh; i++) sum += hist[i];
        hn->Hhalf = ((i-1)<<3); //Half of histogram
        //hn->Hhalf = (i<<12)/hn->GISIF.New; //Half of histogram in real sensor value
        */

        //Find max in color histogram
        for(j=0; j < 3; j++){
            sum = 0;
            for(i=0; sum < hn->SatTh; i++) sum += hn->RGB[j].hist[i];
            hn->RGB[j].min = (i-1)<<3; hn->RGB[j].mins = sum;
            //for(i=0; !hn->RGB[j].hist[i]; i++) ;
            //hn->RGB[j].min = i<<3; hn->RGB[j].mins = hn->RGB[j].hist[i];
            sum = 0;
            if(gHDR){ // gHDR
                for(i=hsz-1; sum < hn->SatTh; i--) sum += hn->RGB[j].hist[i];
                hn->RGB[j].max = (i+1)<<3; hn->RGB[j].maxs = sum;
                //for(i=hsz-1; !hn->RGB[j].hist[i]; i--);
                //hn->RGB[j].max = i<<3; hn->RGB[j].maxs = hn->RGB[j].hist[i];

            } else {
                thh = hn->RGB[j].MaxTh>>3;
                for(i=hsz-1; i > thh; i--) sum += hn->RGB[j].hist[i];
                hn->RGB[j].max = hn->RGB[j].MaxTh; hn->RGB[j].maxs = sum;
            }
        }

        //Check Y history of difference
        if(Y > hn->Y.Max) hn->Y.Max = Y;
        if(Y < hn->Y.Min) hn->Y.Min = Y;
        if(hn->Y.Max) hn->Y.Diff = (hn->Y.Max - hn->Y.Min)*100/hn->Y.Max;

#ifdef AE_DEBUG_PRINTS
        dprintf("sz = %u cn = %d Y = %u min = %u half = %d max= %u SatTh = %u gHDR = %d\n",
                sz, cn, Y, hn->Hmin.New, hn->Hhalf, hn->Hmax.New, hn->SatTh, gHDR);
        //dprintf("GB[0] = %u GB[1] = %u GB[2] = %u GB[3] = %u GB[4] = %u \n", GB[0], GB[1], GB[2], GB[3], GB[4]);
        dprintf("GR[0] = %u GR[1] = %u GR[2] = %u\n", GR[0], GR[1], GR[2]);
        dprintf("GB[0] = %u GB[1] = %u GB[2] = %u\n", GB[0], GB[1], GB[2]);
        dprintf("GN[0] = %d GN[1] = %d GN[2] = %d\n", GN[0], GN[1], GN[2]);
        dprintf("Rmin = %u Rmins = %u Rmax = %u Rmaxs = %u  \n", hn->RGB[0].min, hn->RGB[0].mins, hn->RGB[0].max, hn->RGB[0].maxs);
        dprintf("Gmin = %u Gmins = %u Gmax = %u Gmaxs = %u  \n", hn->RGB[1].min, hn->RGB[1].mins, hn->RGB[1].max, hn->RGB[1].maxs);
        dprintf("Bmin = %u Bmins = %u Bmax = %u Bmaxs = %u  \n", hn->RGB[2].min, hn->RGB[2].mins, hn->RGB[2].max, hn->RGB[2].maxs);
#endif
#ifdef AE_DEBUG_PRINTS
        dprintf("Y.Min = %d Y.Max = %d Y.Diff = %d Y.Th = %d\n", hn->Y.Min, hn->Y.Max, hn->Y.Diff, hn->Y.Th);
        dprintf("gain = %d Rgain = %d Ggain = %d Bgain = %d\n", hn->GISIF.New, hn->RGBgain[0], hn->RGBgain[1],  hn->RGBgain[2]);
        dprintf("Exp.Old = %d Exp.New = %d Offset.Old = %d Offset.New = %d\n",
                hn->Exp.Old, hn->Exp.New, hn->Offset.Old, hn->Offset.New);
#endif
        //AE algorithm
        //Expouse and gain
        if(1) { //HDR
            if(hn->Hmin.New > 20) {
                hn->Exp.New = hn->Exp.Old*20/hn->Hmin.New;
                hn->Y.Diff = 0;
                hn->Y.Max = Y;
                hn->Y.Min = Y;
            } else if (hn->Y.Diff > hn->Y.Th ){
                dprintf("UP!!!!!!!!!!!!!!!!!\n");
                //Check max
                //maxs = hn->Y.Diff > hn->Y.Th ? hn->Y.Diff : hn->Y.Th;
                maxs = hn->Y.Th;
                //Increase expouse at first
                if(hn->Exp.Old < hn->Exp.Range.max ) {
                    //hn->Exp.New = hn->Exp.Old*hn->HmaxTh/hn->Hmax[0];
                    hn->Exp.New = hn->Exp.Old*(100 + maxs)/100;
                    if(hn->Exp.New > hn->Exp.Range.max){
                        hn->Exp.New = hn->Exp.Range.max;
                    }
                }

                //down = 0;
            } //else down = 0;
        } else {
            if(hn->RGB[0].maxs > hn->SatTh || hn->RGB[1].maxs > hn->SatTh || hn->RGB[2].maxs > hn->SatTh ){
                dprintf("DOWN!!!!!!!!!!!!!!!!!\n");
                //Find max step
                //mins = ((sz3 - hn->Hmax[1])<<10)/sz3;
                mins = ((sz - hn->RGB[0].maxs)<<10)/sz;
                for(j=1; j < 3; j++) {
                    tmp = ((sz - hn->RGB[j].maxs)<<10)/sz;
                    mins = tmp < mins ? tmp : mins;
                }
                mins = mins < 512 ? 512 : mins;

                dprintf("mins = %d g = %f \n", mins, (float)mins/1024.);
                //Decrease expouse
                hn->Exp.New = hn->Exp.Old*mins>>10;
                if(hn->Exp.New < hn->Exp.Range.min)  hn->Exp.New = hn->Exp.Range.min;

                hn->Y.Diff = 0;
                hn->Y.Max = Y;
                hn->Y.Min = Y;
                //up = 0;
                //down = 1;
            } else if (hn->Y.Diff > hn->Y.Th ){
                dprintf("UP!!!!!!!!!!!!!!!!!\n");
                //Check max
                //maxs = hn->Y.Diff > hn->Y.Th ? hn->Y.Diff : hn->Y.Th;
                maxs = hn->Y.Th;
                //Increase expouse at first
                if(hn->Exp.Old < hn->Exp.Range.max ) {
                    //hn->Exp.New = hn->Exp.Old*hn->HmaxTh/hn->Hmax[0];
                    hn->Exp.New = hn->Exp.Old*(100 + maxs)/100;
                    if(hn->Exp.New > hn->Exp.Range.max){
                        hn->Exp.New = hn->Exp.Range.max;
                    }
                }
                //down = 0;
            } //else down = 0;
        }

        //White balance algorithm
        //if(hn->Exp.New == hn->Exp.Old){
        if(1){
            min = GR[0]; minr = 0;
            for(j=1; j < ns; j++){
                if(GR[j] < min) { min = GR[j]; minr = j; }
            }
            min = GB[0]; minb = 0;
            for(j=1; j < ns; j++){
                if(GB[j] < min) { min = GB[j]; minb = j; }
            }
            if(minr != 1){
                hn->RGBgain[0] = hn->RGBgain[0] + GN[minr];
                dprintf("mini = %d GN[mini] = %d hn->RGBgain[0] = %d \n", minr, GN[minr], hn->RGBgain[0]);
            }
            if(minb != 1){
                hn->RGBgain[2] = hn->RGBgain[2] + GN[minb];
                dprintf("mini = %d GN[mini] = %d hn->RGBgain[2] = %d \n", minb, GN[minb], hn->RGBgain[2]);
            }
            //}

            //for(j=0; j < ns; j++) GN[j] = GN[j]>>1;
        }
        //Check maximum and minimum
        hn->RGBgain[0] = hn->RGBgain[0] > hn->GISIF.Range.max ? hn->GISIF.Range.max : hn->RGBgain[0];
        hn->RGBgain[0] = hn->RGBgain[0] < hn->GISIF.Range.min ? hn->GISIF.Range.min : hn->RGBgain[0];
        hn->RGBgain[2] = hn->RGBgain[2] > hn->GISIF.Range.max ? hn->GISIF.Range.max : hn->RGBgain[2];
        hn->RGBgain[2] = hn->RGBgain[2] < hn->GISIF.Range.min ? hn->GISIF.Range.min : hn->RGBgain[2];

        //hn->RGBgain[0] = 1040;
        //Find histogram minimum and maximum
        hn->Hmin.New = hn->RGB[0].min*hn->RGBgain[0]>>8;
        hn->Hmax.New = hn->RGB[0].max*hn->RGBgain[0]>>8;
        for(j=1; j < 3; j++){
            if(hn->Hmax.New < hn->RGB[j].max*hn->RGBgain[j]>>8) hn->Hmax.New = hn->RGB[j].max*hn->RGBgain[j]>>8;
            if(hn->Hmin.New > hn->RGB[j].min*hn->RGBgain[j]>>8) hn->Hmin.New = hn->RGB[j].min*hn->RGBgain[j]>>8;
        }

        //hn->RGBgain[2] = 672;
        //Low light condition
        //First down fps

        if ( FPShigh == 1 && IRcutClose == 1 && Y < 100) {
            frame_count += leave_frames;
            if (frame_count > 100) {
                FPShigh = 0;
                frame_count = 0;
            }
        }
        if ( FPShigh == 0 && IRcutClose == 1 && Y > 180) {
            frame_count += leave_frames;
            if (frame_count > 100) {
                FPShigh = 1;
                frame_count = 0;
            }
        }
        //}

        //Second open IR-cut
        if(gIRCut == ALG_IRCUT_AUTO){
            //Got to night mode
            if ( FPShigh == 0 && IRcutClose == 1 && Y < 100) {
                frame_count += leave_frames;
                if (frame_count > 100) {
                    IRcutClose = 0;
                    frame_count = 0;
                }
            }
            //Come back to day mode
            if ( FPShigh == 0 && IRcutClose == 0 && Y > 160) {
                frame_count += leave_frames;
                if (frame_count > 100) {
                    IRcutClose = 1;
                    frame_count = 0;
                }
            }
        }

        if(hn->Y.Diff > hn->Y.Th){
            hn->Y.Diff = 0;
            hn->Y.Max = Y;
            hn->Y.Min = Y;
        }

        hn->Offset.New = hn->Hmin.New; // - (hn->Hmin.New>>2);
        //hn->Offset.New = 0;
        //if(Y) hn->GIFIF.New = ((1000)<<9)/(Y - hn->Offset.New);
        if(hn->Hmax.New) hn->GIFIF.New = ((hn->HmaxTh-1000)<<9)/(hn->Hmax.New - hn->Offset.New);
        //hn->GIFIF.New = (hn->GIFIF.New<<8)/hn->RGBgain[0];

        //hn->GIFIF.New = 512;
        //hn->Offset.New = 0;
        //if(hn->Hmax.New) hn->GIFIF.New = 512;

        //Find maximum gain
        maxg = hn->RGBgain[0] > hn->RGBgain[1] ? hn->RGBgain[0] : hn->RGBgain[1];
        maxg = maxg > hn->RGBgain[2] ? maxg : hn->RGBgain[2];
        max_rgb = hn->Grgb2rgb.Range.max/maxg<<8;

        if(hn->GIFIF.New > hn->GIFIF.Range.max){
            //hn->Grgb2rgb.New = (hn->GIFIF.New - hn->GIFIF.Range.max)>>1;
            hn->Grgb2rgb.New = (hn->GIFIF.New*256/hn->GIFIF.Range.max);
            //hn->Grgb2rgb.New = 512;
        } else {
            hn->Grgb2rgb.New = 256;
        }

        hn->GIFIF.New = hn->GIFIF.New > hn->GIFIF.Range.max ? hn->GIFIF.Range.max : hn->GIFIF.New;
        hn->GIFIF.New = hn->GIFIF.New < hn->GIFIF.Range.min ? hn->GIFIF.Range.min : hn->GIFIF.New;
        hn->Grgb2rgb.New = hn->Grgb2rgb.New > max_rgb ? max_rgb : hn->Grgb2rgb.New;
        hn->Grgb2rgb.New = hn->Grgb2rgb.New < hn->Grgb2rgb.Range.min ? hn->Grgb2rgb.Range.min : hn->Grgb2rgb.New;

#ifdef AE_DEBUG_PRINTS
        dprintf("hn->Hmin.New = %d hn->Hmin.Old = %d hn->Hmax.New  = %d hn->Hmax.Old = %d \n",
                hn->Hmin.New, hn->Hmin.Old, hn->Hmax.New, hn->Hmax.Old);
        dprintf("gain = %d grgb2rgb = %d offset = %d \n", hn->GIFIF.New, hn->Grgb2rgb.New, hn->Offset.New);
        //dprintf("up = %d  wbR = %d wbB = %d wbS = %d gnS = %d dec_exp = %d\n", up,  wbR, wbB, wbS, gnS, dec_exp);
        dprintf("Exp.Old = %d Exp.New = %d Offset.Old = %d Offset.New = %d\n",
                hn->Exp.Old, hn->Exp.New, hn->Offset.Old, hn->Offset.New);
#endif

        //Setup ipipe gains and offset
        //hn->Offset = -(hn->min[0]*6>>3);
        //hn->Offset = 0;
        //if(hn->max[0]) hn->Gain = (4095<<9)/(hn->max[0] + hn->Offset);
        //hn->Gain  = hn->Gain > 8190 ? 8190 : hn->Gain;

        //Find max gain
        //maxg = hn->RGBgain[0] > hn->RGBgain[2] ? hn->RGBgain[0] : hn->RGBgain[2];
        //maxg = maxg > hn->RGBgain[1] ? maxg : hn->RGBgain[1];


        int k, dm = 512, p[dm+1], d, a, b, hmax, hmin;
        int  vl0, vl1, st, sum1, step, lp;
        //Tone mapping

        //hmax = hn->Hmax.New>>3;
        //hmin = hn->Hmin[0]>>3;
        //d = hmax - hmin;
        d = 512;
        a = (1<<16)/d, b = (1000<<7)/sz;


        //dprintf("Hmin = %d Hmax = %d d = %d \n", hmin, hmax, d);
        //Linear
        //for(i=0; i < hmin; i++) p[i] = 0;
        //for(i = hmin; i < hmax; i++) p[i] = ((i - hmin)<<10)*a>>16;
        //for(i=hmax; i <= dm; i++) p[i] = 1023;

        //Integral

        for(j=0; j < 1; j++){

            sum = 0;
            for(i=0; i < hsz; i++) {
                //sum += hn->RGB[j].hist[i];
                sum += hist[i];
                //if(j==1)  dprintf("%d  hist = %d  %d  \n", i, hn->RGB[j].hist[i], sum*b>>7);
                //hn->RGB[j].hist[i] = sum*b>>7;
                hist[i] = sum*b>>7;
            }

            hn->RGB[j].hist[0] = 0;
            vl0 = 0;
            for(i=1; i < dm; i++){
                //vl1 = hn->RGB[j].hist[i];
                vl1 = hist[i];
                //vl1 = (hn->RGB[j].hist[i] + (i<<1))>>1;
                //vl1 = i<<1;
                //hn->lut[i] = (vl0<<10) | (vl1 - vl0);
                hn->RGB[j].hist[i] = (vl0<<10) | (vl1 - vl0);

                //if(j==1) dprintf("%d  hist = %d  %d  %d\n", i, hn->RGBh[j][i], vl0, (vl1 - vl0));
                vl0 = vl1;
            }
        }

        /*
        for(k=0; k < 3; k++){
            p[0] = 0; p[dm] = dm-1;
            for(st=1, d = dm; st < 10; st++, d>>=1){
                for(j=0; j < dm; j+=d){
                    for(i=p[j], sum1=0; i < p[j+d]; i++) sum1 += hn->RGB[k].hist[i];
                    //Remove holes from histogramm
                    //sp = sum1/d;
                    //for(i=p[j]; i < p[j+d]; i++) if(hist[i] > sp) { hist[i] = sp; sum1 -= (sp-hist[i]); }

                    sum1>>=1;
                    for(i=p[j], sum=0; sum < sum1; i++){
                        sum += hn->RGB[k].hist[i];
                        //printf("i = %d sum = %d\n", i, sum);
                    }
                    lp = (p[j+d] + p[j])>>1;
                    p[j+(d>>1)] = (i + lp)>>1;
                    //p[j+(d>>1)] = i;
                    //p[j+(d>>1)] = lp;

                    //printf("size = %d st = %d d = %d j = %d lp = %d ip = %d p[%d] = %d p[%d] = %d p[%d] = %d\n",
                    //       size3>>st, st, d, j, lp, i, j+d, p[j+d], j, p[j], j+(d>>1), p[j+(d>>1)]);
                }
            }

            //for(i = 0; i <= 512; i++) printf("%d  hist = %d p = %d\n", i, hn->RGB[k].hist[i], p[i]);
            int n = 0;
            hn->RGB[k].hist[0] = 0;
            vl0 = p[0];
            for(i=0; i < dm; i++){
                vl1 = p[i+1]<<1;
                //hn->lut[i] = (vl0<<10) | (vl1 - vl0);
                for(j=p[i]; j < p[i+1]; j++){
                    hn->RGB[k].hist[j] = (vl0<<10) | (vl1 - vl0);
                    //n++;
                    //dprintf("%d  vl0 = %d \n", n, vl0);
                 }
                //hn->RGB[k].hist[i] = (vl0<<10) | (vl1 - vl0);
                vl0 = vl1;
            }
            hn->RGB[k].hist[511] = 1023;
        }
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
    return IAES_EOK;
}

