/*
 * pwm_setting.h
 *
 *  Created on: 2018. 6. 25.
 *      Author: kskwi
 */

#ifndef PWM_SETTING_H_
#define PWM_SETTING_H_

#include "..\F28346_Include\DSP28x_Project.h"

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  1515  // Period register
#define EPWM1_MAX_CMPA     1950
#define EPWM1_MIN_CMPA     757
#define EPWM1_MAX_CMPB     75
#define EPWM1_MIN_CMPB       1949

#define EPWM2_TIMER_TBPRD  303 // Period register
#define EPWM2_MAX_CMPA     9
#define EPWM2_MIN_CMPA     15
#define EPWM2_MAX_CMPB     1950
#define EPWM2_MIN_CMPB       21

#define EPWM3_TIMER_TBPRD  303  // Period register
#define EPWM3_MAX_CMPA      950
#define EPWM3_MIN_CMPA       40
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB     1050

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0


typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
}EPWM_INFO;


void update_compare(EPWM_INFO*);

//  BootMode();
void InitEPwm1();
void InitEPwm2();
void InitEPwm3();
void InitEPwm4();
void InitEPwm5();
void InitEPwm6();
void InitEPwm7();
void InitEPwm8();
void InitEPwm9();
void InitEPwm1_AB();
void InitEPwm3Event(void);



#endif /* PWM_SETTING_H_ */
