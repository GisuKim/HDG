// TI File $Revision: /main/1 $
// Checkin $Date: February 1, 2008   09:59:32 $
//###########################################################################
//
// FILE:   DSP2834x_EQep.c
//
// TITLE:  DSP2834x eQEP Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File

//---------------------------------------------------------------------------
// InitEQep:
//---------------------------------------------------------------------------
// This function initializes the eQEP(s) to a known state.
//
void InitEQep(void)
{
   // Initialize eQEP1/2/3

   //tbd...

}

//---------------------------------------------------------------------------
// Example: InitEQepGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as eQEP pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// For each eQEP peripheral
// Only one GPIO pin should be enabled for EQEPxA operation.
// Only one GPIO pin should be enabled for EQEPxB operation.
// Only one GPIO pin should be enabled for EQEPxS operation.
// Only one GPIO pin should be enabled for EQEPxI operation.
// Comment out other unwanted lines.

void InitEQepGpio()
{
#if DSP28_EQEP1
   InitEQep1Gpio();
#endif  // endif DSP28_EQEP1
#if DSP28_EQEP2
   InitEQep2Gpio();
#endif // endif DSP28_EQEP2
#if DSP28_EQEP3
   InitEQep3Gpio();
#endif // endif DSP28_EQEP3
}

#if DSP28_EQEP1
void InitEQep1Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pull-up on GPIO22 (EQEP1S)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pull-up on GPIO23 (EQEP1I)

//    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;   // Enable pull-up on GPIO50 (EQEP1A)
//   GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;   // Enable pull-up on GPIO51 (EQEP1B)
//    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;   // Enable pull-up on GPIO52 (EQEP1S)
//    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;   // Enable pull-up on GPIO53 (EQEP1I)


// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;   // Sync to SYSCLKOUT GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;   // Sync to SYSCLKOUT GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0;   // Sync to SYSCLKOUT GPIO22 (EQEP1S)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;   // Sync to SYSCLKOUT GPIO23 (EQEP1I)

//    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;   // Sync to SYSCLKOUT GPIO50 (EQEP1A)
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0;   // Sync to SYSCLKOUT GPIO51 (EQEP1B)
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 0;   // Sync to SYSCLKOUT GPIO52 (EQEP1S)
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 0;   // Sync to SYSCLKOUT GPIO53 (EQEP1I)

/* Configure eQEP-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eQEP1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EQEP1A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EQEP1B
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;   // Configure GPIO22 as EQEP1S
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO23 as EQEP1I

//    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 1;   // Configure GPIO50 as EQEP1A
//    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 1;   // Configure GPIO51 as EQEP1B
//    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 1;   // Configure GPIO52 as EQEP1S
//    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 1;   // Configure GPIO53 as EQEP1I


    EDIS;
}
#endif // if DSP28_EQEP1



#if DSP28_EQEP2
void InitEQep2Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;    // Enable pull-up on GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;    // Enable pull-up on GPIO25 (EQEP2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;    // Enable pull-up on GPIO26 (EQEP2I)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;    // Enable pull-up on GPIO27 (EQEP2S)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;  // Sync to SYSCLKOUT GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;  // Sync to SYSCLKOUT GPIO25 (EQEP2B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;  // Sync to SYSCLKOUT GPIO26 (EQEP2I)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0;  // Sync to SYSCLKOUT GPIO27 (EQEP2S)

/* Configure eQEP-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eQEP2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2;   // Configure GPIO24 as EQEP2A
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;   // Configure GPIO25 as EQEP2B
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2;   // Configure GPIO26 as EQEP2I
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 2;   // Configure GPIO27 as EQEP2S


    EDIS;
}
#endif // endif DSP28_EQEP2

#if DSP28_EQEP3
void InitEQep3Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;    // Enable pull-up on GPIO54 (EQEP3A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;    // Enable pull-up on GPIO55 (EQEP3B)
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;    // Enable pull-up on GPIO56 (EQEP3S)
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;    // Enable pull-up on GPIO57 (EQEP3I)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 0;  // Sync to SYSCLKOUT GPIO54 (EQEP3A)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 0;  // Sync to SYSCLKOUT GPIO55 (EQEP3B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 0;  // Sync to SYSCLKOUT GPIO56 (EQEP3S)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 0;  // Sync to SYSCLKOUT GPIO57 (EQEP3I)

/* Configure eQEP-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eQEP2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 3;   // Configure GPIO54 as EQEP3A
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 3;   // Configure GPIO55 as EQEP3B
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 3;   // Configure GPIO56 as EQEP3S
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 3;   // Configure GPIO57 as EQEP3I


    EDIS;
}
#endif // endif DSP28_EQEP3



//===========================================================================
// End of file.
//===========================================================================
