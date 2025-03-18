// TI File $Revision: /main/1 $
// Checkin $Date: February 1, 2008   09:59:35 $
//###########################################################################
//
// FILE:	DSP2834x_Gpio.c
//
// TITLE:	DSP2834x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example. 

void InitGpio(void)
{
   EALLOW;
   
   // Each GPIO pin can be: 
   // a) a GPIO input/output
   // b) peripheral function 1
   // c) peripheral function 2
   // d) peripheral function 3
   // By default, all are GPIO Inputs 
   GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
   GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
   GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO39
   GpioCtrlRegs.GPBMUX2.all = 0x0000;     // GPIO functionality GPIO48-GPIO63
   GpioCtrlRegs.GPCMUX1.all = 0x0000;     // GPIO functionality GPIO64-GPIO79
   GpioCtrlRegs.GPCMUX2.all = 0x0000;     // GPIO functionality GPIO80-GPIO95

   GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are inputs
   GpioCtrlRegs.GPBDIR.all = 0x0000;      // GPIO32-GPIO63 are inputs   
   GpioCtrlRegs.GPCDIR.all = 0x0000;      // GPI064-GPIO95 are inputs

   // Each input can have different qualification
   // a) input synchronized to SYSCLKOUT
   // b) input qualified by a sampling window
   // c) input sent asynchronously (valid for peripheral inputs only)
   GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
   GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO39 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPBQSEL2.all = 0x0000;    // GPIO48-GPIO63 Synch to SYSCLKOUT 

   // Pull-ups can be enabled or disabled. 
   GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO63
   GpioCtrlRegs.GPCPUD.all = 0x0000;      // Pullup's enabled GPIO64-GPIO79

   //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
   //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO34
   //GpioCtrlRegs.GPCPUD.all = 0xFFFF     // Pullup's disabled GPIO64-GPIO79


   EDIS;

}

void InitInput_Gpio(void)
{

    EALLOW;

    //C_OV
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;   // Enable pullup on GPIO7
    GpioDataRegs.GPASET.bit.GPIO31 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 0;   // GPIO5 = output

    //C_OC
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pullup on GPIO7
    GpioDataRegs.GPASET.bit.GPIO11 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;   // GPIO5 = output

    //C_OT
    GpioCtrlRegs.GPCPUD.bit.GPIO83 = 0;   // Enable pullup on GPIO7
    GpioDataRegs.GPCSET.bit.GPIO83 = 1;   // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPCDIR.bit.GPIO83 = 0;   // GPIO5 = output

    //P-Protection
    GpioCtrlRegs.GPCPUD.bit.GPIO82 = 0;   // Enable pullup on GPIO7
    GpioDataRegs.GPCSET.bit.GPIO82 = 1;   // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0;   // GPIO5 = output

    //SW_EMERGENCY
    GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0;   // Enable pullup on GPIO7
    GpioDataRegs.GPCSET.bit.GPIO81 = 1;   // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0;   // GPIO5 = output

    //SW_POWER_ON
    GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;   // Enable pullup on GPIO7
    GpioDataRegs.GPBSET.bit.GPIO46 = 1;   // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO46= 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPBDIR.bit.GPIO46 = 0;   // GPIO5 = output

    //SW_PULSE_ON
    GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;   // Enable pullup on GPIO47
    GpioDataRegs.GPBSET.bit.GPIO47 = 1;   // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPBDIR.bit.GPIO47 = 0;   // GPIO5 = output

   EDIS;


}
	
void InitLED_Gpio(void)
{

	   EALLOW;
	   //LED POWER
        GpioCtrlRegs.GPCPUD.bit.GPIO80 = 0;   // Enable pullup on GPIO7
        GpioDataRegs.GPCSET.bit.GPIO80 = 1;   // Load output latch
        GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0;  // GPIO5 = GPIO5
        GpioCtrlRegs.GPCDIR.bit.GPIO80 = 1;   // GPIO5 = output
        //LED PULSE
        GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;   // Enable pullup on GPIO7
        GpioDataRegs.GPBSET.bit.GPIO46 = 1;   // Load output latch
        GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;  // GPIO5 = GPIO5
        GpioCtrlRegs.GPBDIR.bit.GPIO46 = 1;   // GPIO5 = output

	   EDIS;
}

void InitOutput_Gpio(void)
{

       EALLOW;
       //LED POWER
        GpioCtrlRegs.GPCPUD.bit.GPIO67 = 0;   // Enable pullup on GPIO7
        GpioDataRegs.GPCSET.bit.GPIO67 = 1;   // Load output latch
        GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;  // GPIO5 = GPIO5
        GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;   // GPIO5 = output
        //LED PULSE
        GpioCtrlRegs.GPCPUD.bit.GPIO69 = 0;   // Enable pullup on GPIO7
        GpioDataRegs.GPCSET.bit.GPIO69 = 1;   // Load output latch
        GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 0;  // GPIO5 = GPIO5
        GpioCtrlRegs.GPCDIR.bit.GPIO69 = 1;   // GPIO5 = output

       EDIS;
}


// ADC ADS8343
void InitADS8343_Gpio(void)
{


    EALLOW;

    // ADC CS PIN (Active Low, Output) (GPIO 39, GPB)
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;     // Enable pullup on GPIO39
    GpioDataRegs.GPBSET.bit.GPIO39 = 1;     // Load output latch
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;    // GPIO39 = GPIO39
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;     //output



    // ADC BUSY PIN (Input) (GPIO 86, GPC)
    //ADC BUSY IN
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO7
    GpioDataRegs.GPASET.bit.GPIO20 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;  // GPIO5 = GPIO5
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;   // GPIO5 = output

    EDIS;


}


// DAC AD5664
void InitAD5664_Gpio(void)
{


    EALLOW;

    // DAC /SYNC PIN (Active Low, Output) (GPIO 87, GPC)
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pullup on GPIO87
    GpioDataRegs.GPASET.bit.GPIO30 = 1;     // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;    // GPIO87 = GPIO87
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;     //output


    EDIS;


}




void InitScicMUX_Gpio(void)
{

	   EALLOW;
	   //SEL SCIC EN	SET TO DISABLE- CLEAR TO ENABLE
	   // Enable an GPIO output on GPIO2, set it high
	   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO6
	   GpioDataRegs.GPASET.bit.GPIO2 = 1;   // Load output latch
	   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;  // GPIO2 = GPIO2
	   GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;   // GPIO2 = output
	   //SEL SEIC S0
	   // Enable an GPIO output on GPIO3, set it high
	   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO7
	   GpioDataRegs.GPASET.bit.GPIO3 = 1;   // Load output latch
	   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;  // GPIO3 = GPIO3
	   GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;   // GPIO3 = output
	   //SEL SEIC S1
	   // Enable an GPIO output on GPIO4, set it high
	   GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;   // Enable pullup on GPIO7
	   GpioDataRegs.GPASET.bit.GPIO4 = 1;   // Load output latch
	   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;  // GPIO4 = GPIO4
	   GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;   // GPIO5 = output
	   //SEL SEIC S2
	   // Enable an GPIO output on GPIO5, set it high
	   GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pullup on GPIO7
	   GpioDataRegs.GPASET.bit.GPIO5 = 1;   // Load output latch
	   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;  // GPIO5 = GPIO5
	   GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;   // GPIO5 = output

	   EDIS;

//	   MUX_EN_LO;
//	   MUX_S0_LO;
//	   MUX_S1_LO;
//	   MUX_S2_LO;

}
/*
 * Name of Function : InitRDC_Gpio
 * Return : void
 * Description : Initializing RDC GPIO
 */
void InitRDC_Gpio(void)
{

	   EALLOW;
	   // Enable an GPIO output on GPIO6, set it high
	   GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;   // Enable pullup on GPIO6
	   GpioDataRegs.GPASET.bit.GPIO6 = 1;   // Load output latch
	   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;  // GPIO6 = GPIO6
	   GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;   // GPIO6 = output

	   // Enable an GPIO output on GPIO7, set it high
	   GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;   // Enable pullup on GPIO7
	   GpioDataRegs.GPASET.bit.GPIO7 = 1;   // Load output latch
	   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;  // GPIO7 = GPIO7
	   GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;   // GPIO7 = output

	   EDIS;
}

//===========================================================================
// End of file.
//===========================================================================
