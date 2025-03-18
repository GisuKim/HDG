// TI File $Revision: /main/1 $
// Checkin $Date: February 1, 2008   09:59:27 $
//###########################################################################
//
// FILE:   DSP2834x_ECap.c
//
// TITLE:  DSP2834x eCAP Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File

// ECCTL1 ( ECAP Control Reg 1)
//==========================
// CAPxPOL bits
#define EC_RISING 0x0
#define EC_FALLING 0x1
// CTRRSTx bits
#define EC_ABS_MODE 0x0
#define EC_DELTA_MODE 0x1
// PRESCALE bits
#define EC_BYPASS 0x0
#define EC_DIV1 0x0
#define EC_DIV2 0x1
#define EC_DIV4 0x2
#define EC_DIV6 0x3
#define EC_DIV8 0x4
#define EC_DIV10 0x5
// ECCTL2 ( ECAP Control Reg 2)
//==========================
// CONT/ONESHOT bit
#define EC_CONTINUOUS 0x0
#define EC_ONESHOT 0x1
// STOPVALUE bit
#define EC_EVENT1 0x0
#define EC_EVENT2 0x1
#define EC_EVENT3 0x2
#define EC_EVENT4 0x3
// RE-ARM bit
#define EC_ARM 0x1
// TSCTRSTOP bit
#define EC_FREEZE 0x0
#define EC_RUN 0x1
// SYNCO_SEL bit
#define EC_SYNCIN 0x0
#define EC_CTR_PRD 0x1
#define EC_SYNCO_DIS 0x2
// CAP/APWM mode bit
#define EC_CAP_MODE 0x0
#define EC_APWM_MODE 0x1
// APWMPOL bit
#define EC_ACTV_HI 0x0
#define EC_ACTV_LO 0x1
// Generic
#define EC_DISABLE 0x0
#define EC_ENABLE 0x1
#define EC_FORCE 0x1


//---------------------------------------------------------------------------
// InitECap:
//---------------------------------------------------------------------------
// This function initializes the eCAP(s) to a known state.
//
void InitECap(void)
{
   // Initialize eCAP1/2/3

   //tbd...

}

//---------------------------------------------------------------------------
// Example: InitECapGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ECAP pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// For each eCAP peripheral
// Only one GPIO pin should be enabled for ECAP operation.
// Comment out other unwanted lines.

void InitECapGpio()
{

   InitECap1Gpio();
#if (DSP28_ECAP2)
   InitECap2Gpio();
#endif // endif DSP28_ECAP2
#if (DSP28_ECAP3)
   InitECap3Gpio();
#endif // endif DSP28_ECAP3
#if (DSP28_ECAP4)
   InitECap4Gpio();
#endif // endif DSP28_ECAP4
#if (DSP28_ECAP5)
   InitECap5Gpio();
#endif // endif DSP28_ECAP5
#if (DSP28_ECAP6)
   InitECap6Gpio();
#endif // endif DSP28_ECAP6
}

void InitECap1Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;      // Enable pull-up on GPIO5 (CAP1)
   GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;     // Enable pull-up on GPIO24 (CAP1)
// GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;     // Enable pull-up on GPIO34 (CAP1)


// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 0;    // Synch to SYSCLKOUT GPIO5 (CAP1)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;   // Synch to SYSCLKOUT GPIO24 (CAP1)
// GpioCtrlRegs.GPBQSEL1.bit.GPIO34 = 0;   // Synch to SYSCLKOUT GPIO34 (CAP1)

/* Configure eCAP-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP1 functional pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 3;     // Configure GPIO5 as CAP1
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;    // Configure GPIO24 as CAP1
// GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 1;    // Configure GPIO24 as CAP1

    EDIS;

	ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
	ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
	
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
	
	//________________________________________________________________________//
	// Configure peripheral registers
	ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING; 
	ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING; 

	ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;          // Enable capture units

	ECap1Regs.ECCTL1.bit.PRESCALE = EC_BYPASS;		// Event Prescaler Ny-Pass

	ECap1Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;          // Difference operation         
	ECap1Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;          // Difference operation         
	ECap1Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;          // Difference operation         
	ECap1Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;          // Difference operation         

	//________________________________________________________________________//
	//________________________________________________________________________//
//	ECap1Regs.ECCTL2.bit.STOP_WRAP = EC_EVENT2;        // Stop at 4 events
     
	ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_ONESHOT;
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS; // Pass through
	ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; 	// Allow TSCTR to run

	//________________________________________________________________________//
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
//	ECap1Regs.ECCTL2.bit.REARM = EC_ENABLE;    // arm one-shot
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
	ECap1Regs.ECEINT.bit.CEVT4 = 1;            // 4 events = interrupt

}

#if DSP28_ECAP2
void InitECap2Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;     // Enable pull-up on GPIO7 (CAP2)
// GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;    // Enable pull-up on GPIO25 (CAP2)
// GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0;    // Enable pull-up on GPIO37 (CAP2)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 0;    // Synch to SYSCLKOUT GPIO7 (CAP2)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;   // Synch to SYSCLKOUT GPIO25 (CAP2)
// GpioCtrlRegs.GPBQSEL1.bit.GPIO37 = 0;   // Synch to SYSCLKOUT GPIO37 (CAP2)

/* Configure eCAP-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP2 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 3;    // Configure GPIO7 as CAP2
// GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;   // Configure GPIO25 as CAP2
// GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;   // Configure GPIO37 as CAP2

    EDIS;
}
#endif // endif DSP28_ECAP2

#if DSP28_ECAP3
void InitECap3Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;      // Enable pull-up on GPIO9 (CAP3)
// GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;     // Enable pull-up on GPIO26 (CAP3)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO9 = 0;    // Synch to SYSCLKOUT GPIO9 (CAP3)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;   // Synch to SYSCLKOUT GPIO26 (CAP3)

/* Configure eCAP-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP3 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 3;     // Configure GPIO9 as CAP3
// GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1;    // Configure GPIO26 as CAP3

    EDIS;
}
#endif // endif DSP28_ECAP3


#if DSP28_ECAP4
void InitECap4Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pull-up on GPIO11 (CAP4)
// GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;   // Enable pull-up on GPIO27 (CAP4)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0; // Synch to SYSCLKOUT GPIO11 (CAP4)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0; // Synch to SYSCLKOUT GPIO27 (CAP4)

/* Configure eCAP-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP4 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 3;  // Configure GPIO11 as CAP4
// GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 1;  // Configure GPIO27 as CAP4

    EDIS;
}
#endif // endif DSP28_ECAP4


#if DSP28_ECAP5
void InitECap5Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;     // Enable pull-up on GPIO3 (CAP5)
// GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;    // Enable pull-up on GPIO48 (CAP5)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 0;  // Synch to SYSCLKOUT GPIO3 (CAP5)
// GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 0; // Synch to SYSCLKOUT GPIO48 (CAP5)

/* Configure eCAP-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP5 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2;   // Configure GPIO3 as CAP5
// GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 1;  // Configure GPIO48 as CAP5

    EDIS;
}
#endif // endif DSP28_ECAP5


#if DSP28_ECAP6
void InitECap6Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;     // Enable pull-up on GPIO1 (CAP6)
// GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;    // Enable pull-up on GPIO49 (CAP6)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 0;  // Synch to SYSCLKOUT GPIO1 (CAP6)
// GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 0; // Synch to SYSCLKOUT GPIO49 (CAP6)

/* Configure eCAP-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP6 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 2;   // Configure GPIO1 as CAP6
// GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 1;  // Configure GPIO49 as CAP6

    EDIS;
}
#endif // endif DSP28_ECAP6



//===========================================================================
// End of file.
//===========================================================================
