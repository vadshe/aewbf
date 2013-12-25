#ifndef AEWB_TI_
#define AEWB_TI_

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_STEPS             6
#define NUM_RGB2RGB_MATRIXES  3

#define FD_FRAME_STEPS 				(5) 				//number of frames to wait between FD stages
#define FD_BRIGHTNESS_THRESHHOLD 	(8333.0*1000*1024) 	// threshold value to be crossed to trigger FD detection process

struct rgb2rgb_index {
    int color_temp;
    RGB2RGB_PARAM rgb2rgbparam;
    RGB2RGB_PARAM rgb2rgb2param;
};

extern unsigned char TI_WEIGHTING_SPOT[];
extern unsigned char TI_WEIGHTING_CENTER[];
extern unsigned char TI_WEIGHTING_MATRIX[];
extern Uint32 TI_YEE_TABLE[];

extern struct rgb2rgb_index rgb_maxtrixes[NUM_RGB2RGB_MATRIXES];

int TI_2A_config(int flicker_detection, int saldre);
void TI_2A_init_tables(int width, int height);
void TI2A_applySettings(IAEWB_Ae *curAe, IAEWB_Ae *nextAe, int numSmoothSteps, int step);
short TI_2A_SetEEValues(int shift_val);
int RGB2RGB_stab(int curr_RGB2RGBIndex);

#ifdef __cplusplus
}
#endif

#endif
