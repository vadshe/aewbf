#include <alg_motionDetect.h>
#include <ipnc/Msg_Def.h>

#include <df/log.h>
#include <df/error.h>

#include <unistd.h> // unlink

#define DFTRACE() dflog(LOG_INFO, "%s():%u", __func__, __LINE__)


char md_mask[SIGRAND_MOTION_MASK_CHAR_LENGTH];

sigrand_motion_t sig_md = {
    .mask_length = sizeof(md_mask),
    .adaptive_threshold = 1,
    .threshold = 25
};

void *ALG_motionDetectCreate(ALG_MotionDetectCreate *create, ALG_MotionDetectCreateStatus *status)
{
		ALG_MotionObj  *pObj = NULL;

		pObj = OSA_memAlloc(sizeof(ALG_MotionObj));

		if(pObj==NULL)
			return NULL;

		memset(pObj, 0, sizeof(*pObj));
		if( create )
			pObj->createPrm = *create;

		if( status )
			pObj->createStatus = *status;

		/*Start motion detection function after delay start_cnt frames*/
		pObj->frame_count 	= 0;
		pObj->start_cnt 	= 200;

		return pObj;
}


int ALG_motionDetectGetThres(ALG_MotionObj  *pObj)
{
	int block_size;
	if( pObj == NULL )
		return ALG_MOTION_S_FAIL;

	pObj->frame_width   = (pObj->runPrm.ImageWidth  >> 4); // Number of macroblocks in frame width
	pObj->frame_height  = (pObj->runPrm.ImageHeight >> 4); // Number of macroblocks in frame height

	/* Set the motion block size base on image size */
	pObj->win_width 	= pObj->runPrm.windowWidth >> 4; //Window width in macroblocks
	pObj->win_height 	= pObj->runPrm.windowHeight >> 4; //Window height in macroblocks

	block_size = pObj->win_width * pObj->win_height;

#if 0 //debug only
	dflog(LOG_INFO, "%s():%u Image: %u x %u, frame: %u x %u, window: %u x %u, win: %u x %u", __func__, __LINE__,
	      pObj->runPrm.ImageWidth,
	      pObj->runPrm.ImageHeight,

	      pObj->frame_width,
	      pObj->frame_height,

	      pObj->runPrm.windowWidth,
	      pObj->runPrm.windowHeight,

	      pObj->win_width,
	      pObj->win_height
	     );
#endif

	//+ Windows Motion Detected Maps
	size_t frame_square = pObj->frame_height * pObj->frame_width;

	if (pObj->Enabled == NULL)
	    pObj->Enabled = calloc(1, frame_square);

    if (pObj->Detected == NULL)
        pObj->Detected = malloc(frame_square);

    if (pObj->Ing == NULL)
        pObj->Ing = malloc(frame_square*(sizeof(*pObj->Ing)));

    memset(pObj->Detected, 0, frame_square);
	//- Windows Motion Detected Maps

	//+ convert boolean bit array md_mask to boolean char array pObj->Enabled
	unsigned char st = 1 << (CHAR_BIT-1);
	size_t i;
	for(i=0; i < frame_square; ++i)
		pObj->Enabled[i] = ( md_mask[i/CHAR_BIT] & (st >> (i & (CHAR_BIT-1))) ) ? 1 : 0;
	//- convert boolean bit array md_mask to boolean char array pObj->Enabled

	if(block_size >= 600) /* for 1080P */
	{
		pObj->SAD_THRESHOLD = 2500;
	}
	else if(block_size < 600 && block_size >= 300 ) /* for 720P */
	{
		pObj->SAD_THRESHOLD = 3000;
	}
	else if(block_size < 300 && block_size >= 100 ) /* for D1 */
	{
		pObj->SAD_THRESHOLD = 3500;
	}
	else if(block_size < 100 && block_size >= 20 ) /* for CIF */
	{
		pObj->SAD_THRESHOLD = 4000;
	}
	else /* less than CIF */
	{
		pObj->SAD_THRESHOLD = 4500;
	}

	pObj->SAD_THRESHOLD = 256 * sig_md.threshold;

	return ALG_MOTION_S_OK;
}

#if 0 // unused now
int ALG_motionDetectGetSensitive(ALG_MotionObj  *pObj)
{
	if( pObj == NULL )
		return ALG_MOTION_S_FAIL;

    pObj->runPrm.motionsens = 1; /* Intialize to avoid non-zero value */

	/* Get motion sensitivity from webpage according to user input*/
	if(pObj->runPrm.motioncenable == 0)
	{
			if(pObj->frame_width >= 120) /* 1080P = 675MB/zone */
			{
				if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_LOW)			//low level
				{
						pObj->runPrm.motionsens = 35;
				}
				else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_MEDIUM)			//medium level
				{
						pObj->runPrm.motionsens = 55;
				}
				else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_HIGH)			//high level
				{
						pObj->runPrm.motionsens = 95;
				}
			}
			else if(pObj->frame_width < 120 && pObj->frame_width >= 80) /* 720P = 300MB/zone */
			{
				if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_LOW)			//low level
				{
						pObj->runPrm.motionsens = 15;
				}
				else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_MEDIUM)			//medium level
				{
						pObj->runPrm.motionsens = 25;
				}
				else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_HIGH)			//high level
				{
						pObj->runPrm.motionsens = 50;
				}
			}
			else if(pObj->frame_width < 80 && pObj->frame_width >= 40) /* D1 = 112.5MB/zone */
			{
				if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_LOW)			//low level
				{
						pObj->runPrm.motionsens = 7;
				}
				else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_MEDIUM)			//medium level
				{
						pObj->runPrm.motionsens = 11;
				}
				else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_HIGH)			//high level
				{
						pObj->runPrm.motionsens = 20;
				}
			}
			else /* CIF = 20MB/zone */
			{
				if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_LOW)			//low level
				{
						pObj->runPrm.motionsens = 3;

				}else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_MEDIUM)			//medium level
				{
						pObj->runPrm.motionsens = 5;

				}else if(pObj->runPrm.motionlevel == ALG_MOTION_DETECT_SENSITIVITY_HIGH)			//high level
				{
						pObj->runPrm.motionsens = 10;

				}
			}
	}
	else
	{

		if(pObj->runPrm.motioncvalue == 0)
		{
			pObj->runPrm.motionsens = 20;

		}
		else
		{
			if(pObj->frame_width > 45)
			{
				if(pObj->runPrm.motioncvalue >= 20)
				{
					pObj->runPrm.motionsens = pObj->runPrm.motioncvalue;
				}else{
					pObj->runPrm.motionsens = 20;
				}

			}
			else
			{
				pObj->runPrm.motionsens = (pObj->runPrm.motioncvalue / 5);

				if(pObj->runPrm.motionsens < 10)
				{
					pObj->runPrm.motionsens = 10;
				}

			}
		}
	}

	if( pObj->runPrm.motionsens == 0)
		pObj->runPrm.motionsens = 1;

	/* Calculate the threshold value base on the motion sensitivity */
	pObj->threshold = (pObj->win_width * pObj->win_height)/ pObj->runPrm.motionsens;

	return ALG_MOTION_S_OK;
}
#endif // unused now

#if 0 // unused now
int ALG_motionDetectCalc(ALG_MotionObj  *pObj)
{
	ALG_MotionDetectMbMvInfo *mbMV_data;
	int i, j, status;

	mbMV_data = pObj->runPrm.mbMvInfo + pObj->MvDataOffset;

	pObj->warning_count = 0;

	for (i = 0; i < pObj->win_height; i++)
	{
		for(j = 0; j < pObj->win_width; j++)
		{
			if(mbMV_data->SAD > pObj->SAD_THRESHOLD)
			{
				pObj->warning_count++;
			}
			mbMV_data ++;
		}
		mbMV_data = mbMV_data + (pObj->frame_width - pObj->win_width);
	 }

	/* If the pObj->warning_count is bigger than pObj->threshold,
	the function will return a alarm signal*/

	status = ( pObj->warning_count >= pObj->threshold ) ? ALG_MOTION_S_DETECT : ALG_MOTION_S_NO_DETECT;

	return status;
}
#endif // unused now

#if 0
static void print_chars(const char *name, const char *ar, const size_t length)
{
	size_t i;
	dflog_(LOG_INFO, "%s: ", name);
	for(i=0; i < length; ++i)
		dflog_(LOG_INFO, "%1d ", ar[i]);
	dflog_flush(LOG_INFO);
}
#endif // unused now

#if 0
static void print_chars2d(const char *name, const char *ar, const size_t w, const size_t h)
{
    size_t y;
    for(y=0; y < h; ++y)
    {
	dflog_(LOG_INFO, "%s [%2zu]: ", name, y);

	size_t x;
	for(x=0; x < w; ++x)
	{
	    size_t yx = y * w + x;
	    dflog_(LOG_INFO, "%1d", ar[yx]);
	}
	dflog_flush(LOG_INFO);
    }
}
#endif // unused now

#if 0
static void print_md_mask(void)
{
    size_t i;
    dflog_(LOG_INFO, "md_mask[%zu]: ", SIGRAND_MOTION_MASK_CHAR_LENGTH);
    for (i=0; i < SIGRAND_MOTION_MASK_CHAR_LENGTH; ++i)
	dflog_(LOG_INFO, "%02x", md_mask[i]);
    dflog_flush(LOG_INFO);
}
#endif

static int out_motion_detected(const char *md, const size_t md_len)
{
    extern int   stream_get_cur_serial(void);
    int serial = stream_get_cur_serial();
    if (serial < 0)
	serial = 0;

    char mdf[NAME_MAX];
    sprintf(mdf, "%s.%d", SIGRAND_MOTION_DETECTED_FILE, serial);

    FILE *f = fopen(mdf, "w");
    if (f == NULL)
    {
	dferror(EXIT_SUCCESS, errno, "Can't create %s", mdf);
	return -1;
    }

    size_t in_idx, bit_num;
    static const unsigned char hibit = 1 << (CHAR_BIT-1);

    for (in_idx=0; in_idx < md_len; in_idx += 8)
    {
	unsigned char out = 0;

	for(bit_num=0; bit_num < CHAR_BIT; ++bit_num)
	    if ( md[in_idx + bit_num] )
		out |= (hibit >> bit_num);

	ssize_t rc = fprintf(f, "%02x", out);
	if (rc != 2)
	{
	    if (ferror(f))
		dferror(EXIT_SUCCESS, errno, "Error writing to %s",
			mdf);
	    else
		dferror(EXIT_SUCCESS, 0,
			"Error writing to %s, fprintf() returned %zd instead of 2",
			mdf, rc);

	    fclose(f);
	    unlink(mdf);
	    return -1;
	}
    }

    fprintf(f, "\n");

    if (fclose(f) != 0)
    {
	dferror(EXIT_SUCCESS, errno, "Error writing to %s",
		mdf);
	unlink(mdf);
	return -1;
    }

    return 0;
}

int ALG_motionDetectStart(ALG_MotionObj  *pObj)
{
	int detect_cnt = 0;
    int maxi = 0;
    size_t x, y, yx, yw, yx1, yw1;
    const size_t w = pObj->frame_width, h = pObj->frame_height, sz = w*h;
    size_t ecn = 0, MV_th, add;
	ALG_MotionDetectMbMvInfo *mbMV_data = pObj->runPrm.mbMvInfo;
	char *md = pObj->Detected;
	const char *en = pObj->Enabled;
    Uint32 *ing = pObj->Ing;

	int dr[8] = {-1, -1+w, -w, 1-w, 1, 1+w, w, -1+w};
	int hi[256]; //Histogram

	pObj->MvDataOffset = 0;


	// Find adaptive threshold
	if (sig_md.adaptive_threshold) {
		if (pObj->frame_count % 30 == 0) {
			memset(hi, 0, sizeof(hi)); //Clear histogram

			//Get noise statistics and calculate threshold
			mbMV_data = pObj->runPrm.mbMvInfo;
			for(y=0; y < h; y++) {
                yw = y*w;
				for(x=0; x < w; x++) {
                    yx = yw + x;
                    if(en[yx]) {
                        hi[mbMV_data[yx].SAD>>8]++; //Fill histogram
                        //avr += mbMV_data[yx].SAD;
                        //hi[mbMV_data->SAD>>8]++; //Fill histogram
                        //avr += mbMV_data->SAD;
                        //if      (max < mbMV_data->SAD) max = mbMV_data->SAD;
						//else if (min > mbMV_data->SAD) min = mbMV_data->SAD;
                        ecn++;
                    }
                    //mbMV_data++;
				}
			}

            //Find histogram threshold
            size_t i, sum = 0;
            MV_th = ecn*80/100;
            for(i=0; sum < MV_th; i++) {
                sum += hi[i];
                //printf("%  hi = %d sum = %d\n", i, hi[i], sum);
            }
            maxi = (i+1)*2;

            //avr = (avr >> 8) / sz;
            pObj->Sad_Threshold = maxi;
            pObj->Sad_Threshold = pObj->Sad_Threshold << 8;
		} // pObj->frame_count % 30 == 0
	} else {  //!sig_md.adaptive_threshold
		pObj->Sad_Threshold = sig_md.threshold << 8;
	}

    printf("pObj->Sad_Threshold = %d \n", pObj->Sad_Threshold>>8);

    //Integral matrix
    ing[x] = 0;
    for(x=1; x < w; x++) ing[x] = (en[x] ? mbMV_data[x].SAD + mbMV_data[x].SAD : mbMV_data[x].SAD)>>8;

    for(y=1; y < h; y++) {
        yw = y*w;
        yw1 = (y-1)*w;
        ing[yw] = ing[yw1];
        for(x=1; x < w; x++) {
            yx = yw + x;
            yx1 = yw1 + x;
            add = en[yx] ? (mbMV_data[yx].SAD>>8) : 0;
            ing[yx] = ing[yx-1] + ing[yx1] - ing[yx1-1] + add;
        }
    }

    printf("ing[sz-1] = %d \n", ing[sz-1]);

	mbMV_data = pObj->runPrm.mbMvInfo;
	for(y=0; y < h; y++) {
        yw = y*w;
		for(x=0; x < w; x++) {
            yx = yw + x;
			if (en[yx]) {
				if(mbMV_data->SAD > pObj->Sad_Threshold){
					md[yx] = 1; //Motion Detected
					detect_cnt++;
					//dflog(LOG_INFO, "%s():%u ALG_MOTION_S_DETECT detect = %d ", __func__, __LINE__, detect_cnt);
					//return ALG_MOTION_S_DETECT;
				} else {
					md[yx] = 0; //Motion Not Detected
				}
			}
			mbMV_data++;
		}
	}

    printf("ing[sz-1] = %d \n", ing[sz-1]);
	//if (pObj->frame_count % 5 == 0)
	{
	    //print_chars2d("Detected", md, w, h);
#if 0 //debug only
        if(ecn)
        dflog(LOG_INFO, "%s():%u  avr: %d  maxi: %d ing: %d ecn: %d Sad_Threshold: %d frame_count: %d",
          __func__, __LINE__, avr, maxi, ing[sz-1], ecn, pObj->Sad_Threshold>>8, pObj->frame_count);
#endif
	}
	//Check if MB have two or more neighbors
	for(y=1; y < (h-1); y++) {
        yw = y*w;
		for(x=1; x < (w-1); x++) {
            size_t cn = 0;
            yx = yw + x;
			if (md[yx]) {
				size_t i;
				for(i=0; i < 8; i++) {
					if (md[yx + dr[i]]) cn++;
				}
				if(cn > 2 && detect_cnt > 10) {
#if 1
                    dflog(LOG_INFO, "%s():%u detect_cnt: %d maxi: %d ing: %d Sad_Threshold: %d frame_count: %d",
                      __func__, __LINE__, detect_cnt, maxi, ing[sz-1], pObj->Sad_Threshold>>8, pObj->frame_count);
#endif
					out_motion_detected(md, sz);
					return ALG_MOTION_S_DETECT;
				}
			}
		}
	}

	return ALG_MOTION_S_NO_DETECT;
}

int ALG_motionDetectRun(void *hndl, ALG_MotionDetectRunPrm *prm, ALG_MotionDetectRunStatus *status)
{
	ALG_MotionObj  *pObj = (ALG_MotionObj*)hndl;

	if( pObj == NULL )
		return ALG_MOTION_S_FAIL;

	/*Parm tranfer*/
	pObj->runPrm 	= *prm;
	pObj->runStatus = *status;
	pObj->frame_count++;

	if((pObj->runPrm.isKeyFrame == TRUE) || (pObj->frame_count < pObj->start_cnt))
	{
		return ALG_MOTION_S_NO_DETECT;
	}

	ALG_motionDetectGetThres(pObj);
	//ALG_motionDetectGetSensitive(pObj);

	return ALG_motionDetectStart(pObj);
}

int ALG_motionDetectDelete(void *hndl)
{
	ALG_MotionObj *pObj=(ALG_MotionObj *)hndl;

	if(pObj==NULL)
		return ALG_MOTION_S_FAIL;

	//Free Windows Motion Enable and Detected Maps
	OSA_memFree(pObj->Detected);
	OSA_memFree(pObj->Enabled);

	OSA_memFree(pObj);

	return ALG_MOTION_S_OK;
}
