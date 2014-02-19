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
#include "iaewbf_sig.h"
#include "aewbf_sig.h"
#include "alg_aewb.h"

extern IAEWBF_Fxns IAEWBF_SIG_IALG;
extern int gIRCut, gFlicker;
int IRcutClose = 1; //IR-cut 1-open, 0 - close
int FPShigh = 1; //FPS 1-high, 0 - low
extern int gHDR;
extern int DEBUG;

Int32 frames = 0, frame_count = 0, leave_frames = 5, down = 0;

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

    int i, j;
    Uint32 w = hn->w, h = hn->h, cn = 0;
    int sz = w*h,  sz4 = sz*4, sz3 = sz*3, sz2 = sz3>>1;
    Uint32 r, g, b;
    Uint32  Y=0, Y1=0, ns = 3, GR[ns], GB[ns];
    Uint16 *box = hn->box;
    Uint32 hsz = ALG_SENSOR_BITS;
    Uint32 minr, minb, min, max;
    //Uint32 fp = 512, sp = 2304, fg = 1, sg = 7;
    Uint32 hist[hsz];
    Uint32 upth = sz3/6, downth = sz3>>1,  mid, uphalf;
    int GN[3];

    GN[0] = 16; GN[1] = 0; GN[2] = -16;

    //dprintf("frames = %d\n", frames);
    if(!(frames%leave_frames) && frames){
        //Clear histogram
        memset(hist, 0, sizeof(Uint32)*hsz);
        for(j=0; j < ns; j++) { GR[j] = 0; GB[j] = 0; }

        for(i=0; i < sz4; i+=4) {
            //i1 = i+1; i2 = i+2;
            r = box[i+2]>>2;
            g = box[i+1]>>2;
            b = box[i  ]>>2;

            Y1 = (117*b + 601*g + 306*r)>>10;

            hist[r>>3]++;
            hist[g>>3]++;
            hist[b>>3]++;

            for(j=0; j < ns; j++) {
                GB[j] += abs(g - (b*(512 + GN[j])>>9));
                GR[j] += abs(g - (r*(512 + GN[j])>>9));
            }
            Y += Y1;
        }

        Y = Y/sz;
        hn->Y.New = Y;
        //Make integral histogram

        for(i=1; i < hsz; i++) hist[i] += hist[i-1];

        //Find max in color histogram
        for(i=0;  hist[i] < hn->SatTh; i++) ;
        hn->Hmin.New = i;

        for(i=hsz-1; (sz3 - hist[i]) < hn->SatTh; i--);
        hn->Hmax.New = i;
        /*
        //The middle of histogram
        mid = (hn->Hmax.New - hn->Hmin.New)>>1;
        //Check upper half
        uphalf = sz3 - hist[mid];
        //dprintf("hist[0] = %u hist[hsz-1] = %u min = %d mid = %d max = %d uphalf = %d upth = %d\n",
        //        hist[0], hist[hsz-1], hn->Hmin.New,  mid, hn->Hmax.New, uphalf, upth);

        while(uphalf < upth && hn->Hmax.New > 5){
            hn->Hmax.New--;
            mid = (hn->Hmax.New - hn->Hmin.New)>>1;
            uphalf = sz3 - hist[mid];
        }
        //dprintf("min = %d mid = %d max = %d \n", hn->Hmin.New,  mid, hn->Hmax.New);
        */

        //Avaraging
        hn->Hmax.Avrg += hn->Hmax.New;
        hn->Hmax.Avrg -= hn->Hmax.Hist[hn->Hmax.HistC];
        hn->Hmax.Hist[hn->Hmax.HistC] = hn->Hmax.New;
        hn->Hmax.HistC = hn->Hmax.HistC == HISTORY - 1 ? 0 : hn->Hmax.HistC + 1;

        hn->Hmin.Avrg += hn->Hmin.New;
        hn->Hmin.Avrg -= hn->Hmin.Hist[hn->Hmin.HistC];
        hn->Hmin.Hist[hn->Hmin.HistC] = hn->Hmin.New;
        hn->Hmin.HistC = (hn->Hmin.HistC == (HISTORY - 1)) ? 0 : hn->Hmin.HistC + 1;


        hn->Hmax.Avrg = (hn->Hmax.Avrg<<3)/HISTORY;
        hn->Hmin.Avrg = (hn->Hmin.Avrg<<3)/HISTORY;

        hn->Hmax.New = hn->Hmax.New<<3;
        hn->Hmin.New = hn->Hmin.New<<3;

        //Check Y history of difference
        if(Y > hn->Y.Max) hn->Y.Max = Y;
        if(Y < hn->Y.Min) hn->Y.Min = Y;
        if(hn->Y.Max) hn->Y.Diff = (hn->Y.Max - hn->Y.Min)*100/hn->Y.Max;

#ifdef AE_DEBUG_PRINTS
        /*
        dprintf("sz = %u cn = %d Y = %u min = %u max= %u SatTh = %u gHDR = %d \n",
                sz, cn, Y, hn->Hmin.New,  hn->Hmax.New, hn->SatTh, gHDR);
        //dprintf("GB[0] = %u GB[1] = %u GB[2] = %u GB[3] = %u GB[4] = %u \n", GB[0], GB[1], GB[2], GB[3], GB[4]);
        dprintf("GR[0] = %u GR[1] = %u GR[2] = %u\n", GR[0], GR[1], GR[2]);
        dprintf("GB[0] = %u GB[1] = %u GB[2] = %u\n", GB[0], GB[1], GB[2]);
        dprintf("GN[0] = %d GN[1] = %d GN[2] = %d\n", GN[0], GN[1], GN[2]);
        dprintf("Rgain = %d Ggain = %d Bgain = %d\n", hn->Rgain.New, 512,  hn->Bgain.New);
        */
#endif
        //White balance algorithm
        min = GR[0]; minr = 0;
        for(j=1; j < ns; j++){
            if(GR[j] < min) { min = GR[j]; minr = j; }
        }
        min = GB[0]; minb = 0;
        for(j=1; j < ns; j++){
            if(GB[j] < min) { min = GB[j]; minb = j; }
        }
        if(minr != 1){
            hn->Rgain.New = hn->Rgain.New + GN[minr];
            //if(DEBUG) dprintf("WB R : RgN %d RgO %d\n", hn->Rgain.New, hn->Rgain.Old);
        }
        if(minb != 1){
            hn->Bgain.New = hn->Bgain.New + GN[minb];
            //if(DEBUG) dprintf("WB B : BgN %d BgO %d\n", hn->Bgain.New, hn->Bgain.Old);
        }
        //Check range
        hn->Rgain.New = hn->Rgain.New > hn->Rgain.Range.max ? hn->Rgain.Range.max : hn->Rgain.New;
        hn->Rgain.New = hn->Rgain.New < hn->Rgain.Range.min ? hn->Rgain.Range.min : hn->Rgain.New;
        hn->Bgain.New = hn->Bgain.New > hn->Bgain.Range.max ? hn->Bgain.Range.max : hn->Bgain.New;
        hn->Bgain.New = hn->Bgain.New < hn->Bgain.Range.min ? hn->Bgain.Range.min : hn->Bgain.New;

        if(down){
            hn->Y.Diff = 0;
            hn->Y.Max = Y;
            hn->Y.Min = Y;
            down = 0;
        }

        //AE algorithm
        //Change expouse
        if(hn->Hmin.New > 60) { // || downth < uphalf){
            min = hn->Exp.Old*60/hn->Hmin.New;
            //min = min < hn->Exp.Old*downth/uphalf ? min : hn->Exp.Old*downth/uphalf;
            //Down expouse
            if(gFlicker == VIDEO_NONE){
                hn->Exp.New = min;
                //hn->Exp.New = hn->Exp.Old*100/hn->Hmin.New;
                //hn->Exp.New = hn->Exp.Old*downth/uphalf;
            } else {
                hn->Exp.New -= hn->Exp.Step;
            }
            if(hn->Exp.New < hn->Exp.Step) hn->Exp.New = hn->Exp.Step;
            down = 1;
            //if(DEBUG) dprintf("EXP DOWN : ExpN %d ExpO %d\n", hn->Exp.New, hn->Exp.Old);
        } else if (hn->Y.Diff > hn->Y.Th){
            //Up expouse
            if(gFlicker == VIDEO_NONE){
                hn->Exp.New = hn->Exp.Old*(100 + hn->Y.Th)/100;
                if(hn->Exp.New > hn->Exp.Range.max)  hn->Exp.New = hn->Exp.Range.max;
            } else  {
                hn->Exp.New += hn->Exp.Step;
                if(hn->Exp.New > hn->Exp.Range.max)  hn->Exp.New -= hn->Exp.Step;
            }
            //if(DEBUG) dprintf("EXP UP : ExpN %d ExpO %d YN %d YO %d YDiff %d\n", hn->Exp.New, hn->Exp.Old, hn->Y.New, hn->Y.Old, hn->Y.Diff);
            //if(DEBUG) dprintf("EXP UP : YN %d YO %d YDiff %d\n", hn->Y.New, hn->Y.Old, hn->Y.Diff);
        }
        //Check Low light condition
        //First down fps
        if ( FPShigh == 1 && IRcutClose == 1 && Y < 100) {
            frame_count += leave_frames;
            if (frame_count > 100) {
                FPShigh = 0;
                frame_count = 0;
                //if(DEBUG) dprintf("FPS DOWN : YN %d YO %d \n", hn->Y.New, hn->Y.Old);
            }
        }
        if ( FPShigh == 0 && IRcutClose == 1 && Y > 170) {
            frame_count += leave_frames;
            if (frame_count > 100) {
                FPShigh = 1;
                frame_count = 0;
                //if(DEBUG) dprintf("FPS UP : YN %d YO %d \n", hn->Y.New, hn->Y.Old);
            }
        }
        //Second open IR-cut
        if(gIRCut == ALG_IRCUT_AUTO){
            //Got to night mode
            if ( FPShigh == 0 && IRcutClose == 1 && Y < 100) {
                frame_count += leave_frames;
                if (frame_count > 100) {
                    IRcutClose = 0;
                    frame_count = 0;
                    //if(DEBUG) dprintf("IR OPEN : YN %d YO %d \n", hn->Y.New, hn->Y.Old);
                }
            }
            //Come back to day mode
            if ( FPShigh == 0 && IRcutClose == 0 && Y > 170) {
                frame_count += leave_frames;
                if (frame_count > 100) {
                    IRcutClose = 1;
                    frame_count = 0;
                    //if(DEBUG) dprintf("IR CLOSE : YN %d YO %d \n", hn->Y.New, hn->Y.Old);
                }
            }
        }

        //Change the offset
        //hn->Offset.New = hn->Hmin.New;
        hn->Offset.New = hn->Hmin.Avrg;
        //IFIF gain
        if(hn->Hmax.Avrg) hn->GIFIF.New = ((hn->HmaxTh)<<9)/(hn->Hmax.Avrg - hn->Offset.New);
        //If not enough IFIF gain add rgb2rgb gain

        if(hn->GIFIF.New > hn->GIFIF.Range.max){
            hn->Grgb2rgb.New = (hn->GIFIF.New*256/hn->GIFIF.Range.max);
        } else {
            hn->Grgb2rgb.New = 256;
        }

        //Check gain range
        hn->GIFIF.New = hn->GIFIF.New > hn->GIFIF.Range.max ? hn->GIFIF.Range.max : hn->GIFIF.New;
        hn->GIFIF.New = hn->GIFIF.New < hn->GIFIF.Range.min ? hn->GIFIF.Range.min : hn->GIFIF.New;
        hn->Grgb2rgb.New = hn->Grgb2rgb.New > hn->Grgb2rgb.Range.max ? hn->Grgb2rgb.Range.max : hn->Grgb2rgb.New;
        hn->Grgb2rgb.New = hn->Grgb2rgb.New < hn->Grgb2rgb.Range.min ? hn->Grgb2rgb.Range.min : hn->Grgb2rgb.New;

#ifdef AE_DEBUG_PRINTS
        /*
        dprintf("Y.Min = %d Y.Max = %d Y.Diff = %d Y.Th = %d\n", hn->Y.Min, hn->Y.Max, hn->Y.Diff, hn->Y.Th);
        dprintf("gain = %d grgb2rgb = %d offset = %d \n", hn->GIFIF.New, hn->Grgb2rgb.New, hn->Offset.New);
        dprintf("Exp.Old = %d Exp.New = %d Offset.Old = %d Offset.New = %d\n",
                hn->Exp.Old, hn->Exp.New, hn->Offset.Old, hn->Offset.New);
         */
#endif

        hn->Y.Old = hn->Y.New;
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

