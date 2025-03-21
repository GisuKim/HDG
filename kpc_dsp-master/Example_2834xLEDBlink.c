// TI File $Revision: /main/3 $
// Checkin $Date: June 28, 2010   09:18:57 $
//###########################################################################
//
// FILE:    Example_2834xLedBlink.c
//
// TITLE:   DSP2834x eZdsp LED Blink Getting Started Program.
//
// ASSUMPTIONS:
//
//    This program requires the DSP2834x header files.
//
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The 2834x Boot Mode table is shown below.
//    For information on configuring the boot mode of an eZdsp,
//    please refer to the documentation included with the eZdsp,
//
//       $Boot_Table:
//
//         GPIO87   GPIO86     GPIO85   GPIO84
//          XA15     XA14       XA13     XA12
//           PU       PU         PU       PU
//        ==========================================
//            1        1          1        1    TI Test Only
//            1        1          1        0    SCI-A boot
//            1        1          0        1    SPI-A boot
//            1        1          0        0    I2C-A boot timing 1
//            1        0          1        1    eCAN-A boot timing 1
//            1        0          1        0    McBSP-A boot
//            1        0          0        1    Jump to XINTF x16
//            1        0          0        0    Jump to XINTF x32
//            0        1          1        1    eCAN-A boot timing 2
//            0        1          1        0    Parallel GPIO I/O boot
//            0        1          0        1    Parallel XINTF boot
//            0        1          0        0    Jump to SARAM	    <- "boot to SARAM"
//            0        0          1        1    Branch to check boot mode
//            0        0          1        0    I2C-A boot timing 2
//            0        0          0        1    Reserved
//            0        0          0        0    TI Test Only
//                                              Boot_Table_End$$
//
// DESCRIPTION:
//
//    This example configures CPU Timer0 for a 500 msec period, and toggles the GPIO34
//    LED on the 2834x eZdsp once per interrupt. For testing purposes, this example
//    also increments a counter each time the timer asserts an interrupt.
//
//       Watch Variables:
//          CpuTimer0.InterruptCount
//
//       Monitor the GPIO34 LED blink on (for 500 msec) and off (for 500 msec) on the 2834x eZdsp.
//
//###########################################################################
// $TI Release: 2834x Header Files V1.12 $
// $Release Date: March 2011 $
//###########################################################################


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);

void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2834x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2834x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2834x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2834x_DefaultIsr.c.
// This function is found in DSP2834x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in DSP2834x_CpuTimers.c
   InitCpuTimers();   // For this example, only initialize the Cpu Timers
#if (CPU_FRQ_300MHZ)
// Configure CPU-Timer 0 to interrupt every 500 milliseconds:
// 300MHz CPU Freq, 50 millisecond Period (in uSeconds)
   ConfigCpuTimer(&CpuTimer0, 300, 500000);
#endif
#if (CPU_FRQ_200MHZ)
// Configure CPU-Timer 0 to interrupt every 500 milliseconds:
// 200MHz CPU Freq, 50 millisecond Period (in uSeconds)
   ConfigCpuTimer(&CpuTimer0, 200, 500000);
#endif

// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2834x_CpuTimers.h), the
// below settings must also be updated.

   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0

// Step 5. User specific code, enable interrupts:

// Configure GPIO34 as a GPIO output pin
   EALLOW;
   GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;
   GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;
   EDIS;

// Enable CPU INT1 which is connected to CPU-Timer 0:
   IER |= M_INT1;

// Enable TINT0 in the PIE: Group 1 interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;);
}


interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
   GpioDataRegs.GPCTOGGLE.bit.GPIO66 = 1; // Toggle GPIO34 once per 500 milliseconds
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//===========================================================================
// No more.
//===========================================================================
