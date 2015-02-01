
#ifndef _ALG_VNF_H_
#define _ALG_VNF_H_

#include <alg.h>

#define ALG_VNF_PIXEL_PAD          (32)

#define ALG_VNF_MODE_NO_NF         (0)
#define ALG_VNF_MODE_ONLY_TNF      (1)
#define ALG_VNF_MODE_ONLY_KNF      (2)
#define ALG_VNF_MODE_KNF_TNF       (3)
#define ALG_VNF_MODE_KTNF          (4)
#define ALG_VNF_MODE_FAST_TNF      (5)
#define ALG_VNF_MODE_TNF3          (6)
#define ALG_VNF_MODE_ONLY_TNF3     (7)

#ifdef TNF3_MODE_ON
#define DEFAULT_VNF_MODE           (ALG_VNF_MODE_TNF3)
#define DEFAULT_VNF_MODE_ONLY_TNF  (ALG_VNF_MODE_ONLY_TNF3)
#define DEFAULT_VNF_MODE_ONLY_SNF  (ALG_VNF_MODE_ONLY_KNF)
#define DEFAULT_VNF_MODE_OFF       (ALG_VNF_MODE_NO_NF)
#else
#define DEFAULT_VNF_MODE           (ALG_VNF_MODE_KTNF)
#define DEFAULT_VNF_MODE_ONLY_TNF  (ALG_VNF_MODE_ONLY_TNF)
#define DEFAULT_VNF_MODE_ONLY_SNF  (ALG_VNF_MODE_ONLY_KNF)
#define DEFAULT_VNF_MODE_OFF       (ALG_VNF_MODE_NO_NF)
#endif

typedef enum{
	SNF_DEFAULT = 0,
	SNF_CUSTOM
} SNF_STRENGTH_MODE;

typedef enum{
	TNF_AUTO = 0,
	TNF_LOW,
	TNF_MED,
	TNF_HIGH
} TNF_STRENGTH_MODE;

typedef struct {

  /* HWNF Parameters */
  Int32   nsf_thr00;            //color 0 level 1 offset
  Int32   nsf_thr01;            //color 0 level 1 slope
  Int32   nsf_thr02;            //color 0 level 2 offset
  Int32   nsf_thr03;            //color 0 level 2 slope
  Int32   nsf_thr04;            //color 0 level 3 offset
  Int32   nsf_thr05;            //color 0 level 3 slope
  Int32   nsf_thr10;            //color 1 level 1 offset
  Int32   nsf_thr11;            //color 1 level 1 slope
  Int32   nsf_thr12;            //color 1 level 2 offset
  Int32   nsf_thr13;            //color 1 level 2 slope
  Int32   nsf_thr14;            //color 1 level 3 offset
  Int32   nsf_thr15;            //color 1 level 3 slope
  Int32   nsf_thr20;            //color 2 level 1 offset
  Int32   nsf_thr21;            //color 2 level 1 slope
  Int32   nsf_thr22;            //color 2 level 2 offset
  Int32   nsf_thr23;            //color 2 level 2 slope
  Int32   nsf_thr24;            //color 2 level 3 offset
  Int32   nsf_thr25;            //color 2 level 3 slope
  Int32   nsf_sft_slope;        //nsf sft slope
  Int32   nsf_ee_l1_slope;      //nsf
  Int32   nsf_ee_l1_thr1;       //nsf
  Int32   nsf_ee_l1_thr2;       //nsf
  Int32   nsf_ee_l1_ofst2;      //nsf
  Int32   nsf_ee_l2_slope;      //nsf
  Int32   nsf_ee_l2_thr1;       //nsf
  Int32   nsf_ee_l2_thr2;       //nsf
  Int32   nsf_ee_l2_ofst2;      //nsf
  Int32   nsf_ee_l3_slope;      //nsf
  Int32   nsf_ee_l3_thr1;       //nsf
  Int32   nsf_ee_l3_thr2;       //nsf
  Int32   nsf_ee_l3_ofst2;      //nsf

  /* TNF Parameters */
  Uint32  TNF_A0;               /* Control Param: 0 to 255 */
  Uint32  TNF_TM;               /* Control Param: 1 to 255 */
  Uint32  TNFLuma;              /* 1: TNF applied on Luma Only, 0: TNF applied on both Luma and Chroma */

} ALG_vnfParams;

typedef struct
{
  //Place holder for the TNF3 Parameters
  Int32 tnf3TS;              // TS used by NFS2 operating on Diff Downsampled image in TNF3
  Int32 unDiffScaleFactor;   //[TNF3] Same variable is used for the boosting the differential image precision in case of TNF3.
                                      //This vairiable is same as variable q used in the Ref implementation/slides
  Int32 unMotionThreshold;   //Threshold for maximum allowed motion component, if motion exceeds this threshold then the
                                      //temporal filtered frame would be same as current frame
  Int32 unStrengthOfTNF3;    //values of the strength of Blending of TNF3
}ALG_tnf3Params;

typedef struct {

  Uint16  dataFormat;   // only YUV420 supported
  Uint16  width;
  Uint16  height;
  Uint16  offsetH;      // offsetH >= width, input and output offsetH are same
  Uint16  offsetV;      // offsetV >= height+16, , input and output offsetV are same
  Uint16  mode;
  Uint16 q_num;
  Uint16 askIMCOPRes;
  Uint16 snfStrength;

  ALG_vnfParams *pVnfParams;  // if NULL, default values will be used
  ALG_tnf3Params *pTnf3Params;
  Uint32  sysBaseAddr;

} ALG_VnfCreate;

typedef struct {

  Uint16  outWidth;
  Uint16  outHeight;
  Uint16  outStartX;
  Uint16  outStartY;
  Uint16  outPitch;

} ALG_VnfStatus;

typedef struct {

  Uint8  *inAddr;               //Virt Addr
  Uint8  *outAddr;              //Virt Addr

  Uint16  inStartX;
  Uint16  inStartY;

  Uint16  mode;
  Uint16  strength;
  Uint16  strUpdate;

  ALG_vnfParams *pVnfParams; // if NULL, previously set values are applied

} ALG_VnfRunPrm;

extern ALG_vnfParams gAlg_vnfPrm;

void   *ALG_vnfCreate(ALG_VnfCreate * create);
int     ALG_vnfRun(void *hndl, ALG_VnfRunPrm *prm, ALG_VnfStatus * status);
int     ALG_vnfDelete(void *hndl);

#endif
