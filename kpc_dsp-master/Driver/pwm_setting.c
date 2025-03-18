/*
 * pwm_setting.c
 *
 *  Created on: 2018. 6. 25.
 *      Author: kskwi
 */

#include "pwm_setting.h"


// Global variables used in this example
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;

void update_compare(EPWM_INFO *epwm_info)
{


   // Every 10'th interrupt, change the CMPA/CMPB values
   if(epwm_info->EPwmTimerIntCount == 10)
   {
       epwm_info->EPwmTimerIntCount = 0;

       // If we were increasing CMPA, check to see if
       // we reached the max value.  If not, increase CMPA
       // else, change directions and decrease CMPA
       if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
       {
           if(epwm_info->EPwmRegHandle->CMPA.half.CMPA < epwm_info->EPwmMaxCMPA)
           {
              epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
           }
           else
           {
              epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
           }
       }

       // If we were decreasing CMPA, check to see if
       // we reached the min value.  If not, decrease CMPA
       // else, change directions and increase CMPA
       else
       {
           if(epwm_info->EPwmRegHandle->CMPA.half.CMPA == epwm_info->EPwmMinCMPA)
           {
              epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
              epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
           }
           else
           {
              epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
           }
       }

       // If we were increasing CMPB, check to see if
       // we reached the max value.  If not, increase CMPB
       // else, change directions and decrease CMPB
       if(epwm_info->EPwm_CMPB_Direction == EPWM_CMP_UP)
       {
           if(epwm_info->EPwmRegHandle->CMPB < epwm_info->EPwmMaxCMPB)
           {
              epwm_info->EPwmRegHandle->CMPB++;
           }
           else
           {
              epwm_info->EPwm_CMPB_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPB--;
           }
       }

       // If we were decreasing CMPB, check to see if
       // we reached the min value.  If not, decrease CMPB
       // else, change directions and increase CMPB

       else
       {
           if(epwm_info->EPwmRegHandle->CMPB == epwm_info->EPwmMinCMPB)
           {
              epwm_info->EPwm_CMPB_Direction = EPWM_CMP_UP;
              epwm_info->EPwmRegHandle->CMPB++;
           }
           else
           {
              epwm_info->EPwmRegHandle->CMPB--;
           }
       }
   }
   else
   {
      epwm_info->EPwmTimerIntCount++;
   }

   return;
}

void SetCh1OnPW(Uint16 OnPW)
{

}


void InitEPwm3Event(void)
{

   // Setup TBCLK

   EPwm3Regs.TBPRD = 37500;          // Set timer period
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
   EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;      // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;

   // Setup shadow register load on ZERO
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
   EPwm3Regs.CMPA.half.CMPA = 15;    // Set compare A value
//   EPwm3Regs.CMPB = 165;              // Set Compare B value

   // Set Actions
   EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM3A on period
   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM3A on event B, down count


//   EPwm3Regs.TZFRC.bit.OST = 1;
//   EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
//   EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW


//   EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM3A on period
//   EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Set PWM3A on event A, up count

   // Interrupt where we will change the Compare Values
   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;     // Select INT on Zero event
   EPwm3Regs.ETSEL.bit.INTEN = 0;                // Enable INT
   EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


}


void InitEPwm1_AB()
{

   // Setup TBCLK
   EPwm1Regs.TBPRD = 37500;           // Set timer period 801 TBCLKs
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 37500;     // Set compare A value
//   EPwm1Regs.CMPB =  EPWM2_MIN_CMPB;               // Set Compare B value

   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;             // Set PWM1A on event A, up count

   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
   EPwm1Regs.DBFED = 50;
   EPwm1Regs.DBRED = 50;
//   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count
//

//   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;             // Set PWM1B on event B, up count
//   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Interrupt where we will change the Compare Values
//   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
//   EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
//   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
//   epwm1_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm1_info.EPwmRegHandle = &EPwm1Regs;          // Set the pointer to the ePWM module
//   epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
//   epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
//   epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;

}


void InitEPwm1()
{

   // Setup TBCLK
   EPwm1Regs.TBPRD = 37500;           // Set timer period 801 TBCLKs
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 9;     // Set compare A value
//   EPwm1Regs.CMPB = EPWM2_MIN_CMPB;               // Set Compare B value

   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM1A on event A, up count
//   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count
//
   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
   EPwm1Regs.TZFRC.bit.OST = 1;
   EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
   EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW


//   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;             // Set PWM1B on event B, up count
//   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Interrupt where we will change the Compare Values
//   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
//   EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
//   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
//   epwm1_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm1_info.EPwmRegHandle = &EPwm1Regs;          // Set the pointer to the ePWM module
//   epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
//   epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
//   epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;

}


void InitEPwm2()
{


   // Setup TBCLK
   EPwm2Regs.TBPRD = 37500;           // Set timer period 801 TBCLKs
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = 75;     // Set compare A value
   EPwm2Regs.CMPB = 100;               // Set Compare B value

   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
   EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;


   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set actions
   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on event A, up count
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event B, down count

   EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM2B on zero
   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;

//   EALLOW;
//   EPwm2Regs.TZFRC.bit.OST = 1;
//   EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
//   EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW
//   EDIS;


   // Interrupt where we will change the Compare Values
//   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;   // increasing CMPB
//   epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the ePWM module
//   epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
//   epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
//   epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;

}

void InitEPwm3(void)
{

   // Setup TBCLK

   EPwm3Regs.TBPRD = 37500;          // Set timer period
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
   EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;      // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;

   // Setup shadow register load on ZERO
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
   EPwm3Regs.CMPA.half.CMPA = 15;    // Set compare A value
//   EPwm3Regs.CMPB = 165;              // Set Compare B value

   // Set Actions
   EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM3A on period
   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM3A on event B, down count



//   EPwm3Regs.TZFRC.bit.OST = 1;
//   EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
//   EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW


//   EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM3A on period
//   EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Set PWM3A on event A, up count

   // Interrupt where we will change the Compare Values
//   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
//   epwm3_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm3_info.EPwmRegHandle = &EPwm3Regs;          // Set the pointer to the ePWM module
//   epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
//   epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
//   epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;

}

void InitEPwm4()
{


   // Setup TBCLK
   EPwm4Regs.TBPRD = 37500;           // Set timer period 801 TBCLKs
   EPwm4Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm4Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm4Regs.CMPA.half.CMPA = 75;     // Set compare A value
   EPwm4Regs.CMPB = 100;               // Set Compare B value

   // Setup counter mode
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
   EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;


   // Setup shadowing
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set actions
   EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on event A, up count
   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event B, down count

   EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM2B on zero
   EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
   // Interrupt where we will change the Compare Values
//   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;   // increasing CMPB
//   epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the ePWM module
//   epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
//   epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
//   epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;

}

void InitEPwm5()
{


   // Setup TBCLK
   EPwm5Regs.TBPRD = 37500;           // Set timer period 801 TBCLKs
   EPwm5Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm5Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm5Regs.CMPA.half.CMPA = 15;     // Set compare A value
//   EPwm5Regs.CMPB = EPWM2_MIN_CMPB;               // Set Compare B value

   // Setup counter mode
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Disable phase loading
   EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

   // Setup shadowing
   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set actions
   EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on event A, up count
   EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event B, down count



   EPwm5Regs.TZFRC.bit.OST = 1;
   EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
   EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

//   EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;          // Clear PWM2B on zero
//   EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;
   // Interrupt where we will change the Compare Values
//   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;   // increasing CMPB
//   epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the ePWM module
//   epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
//   epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
//   epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;

}

void InitEPwm6(void)
{

   // Setup TBCLK

   EPwm6Regs.TBPRD = 37500;          // Set timer period
   EPwm6Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm6Regs.TBCTR = 0x0000;                     // Clear counter

   // Set Compare values
    EPwm6Regs.CMPA.half.CMPA = 75;    // Set compare A value
    EPwm6Regs.CMPB = 100;              // Set Compare B value

   EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
   EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;      // Clock ratio to SYSCLKOUT
   EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

   // Setup shadow register load on ZERO
   EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;



   // Set Actions
   EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM3A on period
   EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM3A on event B, down count

   EPwm6Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM3A on period
   EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Set PWM3A on event A, up count

   // Interrupt where we will change the Compare Values
//   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
//   epwm3_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm3_info.EPwmRegHandle = &EPwm3Regs;          // Set the pointer to the ePWM module
//   epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
//   epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
//   epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;

}

void InitEPwm7(void)
{

   // Setup TBCLK

   EPwm7Regs.TBPRD = 37500;          // Set timer period
   EPwm7Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

   // Set Compare values
    EPwm7Regs.CMPA.half.CMPA = 15;    // Set compare A value
 //   EPwm3Regs.CMPB = EPWM3_MAX_CMPB;              // Set Compare B value

   EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
   EPwm7Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;      // Clock ratio to SYSCLKOUT
   EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

   // Setup shadow register load on ZERO
   EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;



   // Set Actions
   EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM3A on period
   EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM3A on event B, down count


   EPwm7Regs.TZFRC.bit.OST = 1;
   EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
   EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

//   EPwm3Regs.AQCTLB.bit.PRD = AQ_CLEAR;          // Clear PWM3A on period
//   EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;            // Set PWM3A on event A, up count

   // Interrupt where we will change the Compare Values
//   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
//   epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
//   epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
//   epwm3_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
//   epwm3_info.EPwmRegHandle = &EPwm3Regs;          // Set the pointer to the ePWM module
//   epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;        // Setup min/max CMPA/CMPB values
//   epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
//   epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
//   epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;

}
void InitEPwm8(void)
{

   // Setup TBCLK

   EPwm8Regs.TBPRD = 37500;          // Set timer period
   EPwm8Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm8Regs.TBCTR = 0x0000;                     // Clear counter

   // Set Compare values
    EPwm8Regs.CMPA.half.CMPA = 75;    // Set compare A value
    EPwm8Regs.CMPB = 100;              // Set Compare B value

   EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
   EPwm8Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;      // Clock ratio to SYSCLKOUT
   EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

   // Setup shadow register load on ZERO
   EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;



   // Set Actions
   EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM3A on period
   EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM3A on event B, down count

   EPwm8Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM3A on period
   EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Set PWM3A on event A, up count

}

void InitEPwm9(void)
{
   // Setup TBCLK

   EPwm9Regs.TBPRD = 37500;          // Set timer period
   EPwm9Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm9Regs.TBCTR = 0x0000;                     // Clear counter

   // Set Compare values
    EPwm9Regs.CMPA.half.CMPA = 15;    // Set compare A value
 //   EPwm3Regs.CMPB = EPWM3_MAX_CMPB;              // Set Compare B value

   EPwm9Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Disable phase loading
   EPwm9Regs.TBCTL.bit.PRDLD = TB_SHADOW;        // Disable phase loading
   EPwm9Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;      // Clock ratio to SYSCLKOUT
   EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV2;
   EPwm9Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

   // Setup shadow register load on ZERO
   EPwm9Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm9Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm9Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm9Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;



   // Set Actions
   EPwm9Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM3A on period
   EPwm9Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM3A on event B, down count


   EPwm9Regs.TZFRC.bit.OST = 1;
   EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
   EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

}
