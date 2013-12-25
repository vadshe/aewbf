#ifndef _DRV_PWM_H_
#define _DRV_PWM_H_

#include <drv.h>
#include <drv_csl.h>
#include <osa.h>
#include <osa_thr.h>


typedef struct {
	int	PwmID;
	unsigned int Period;

	unsigned int PCR;
	unsigned int START;
	unsigned int CFG;
	unsigned int PER;
	unsigned int PH1D;
} DRV_PwmSetPrm;

extern int gPwmEnable;

int DRV_pwmInit(void);
int DRV_pwmExit(void);
int DRV_pwmSetup    (DRV_PwmSetPrm *prm);
int DRV_pwmSetPeriod(DRV_PwmSetPrm *prm);

#endif
