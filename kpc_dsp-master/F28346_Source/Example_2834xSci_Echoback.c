// TI File $Revision: /main/2 $
// Checkin $Date: July 31, 2009   09:56:56 $
//###########################################################################
//
// FILE:    Example_2834xSci_Echoback.c
//
// TITLE:   DSP2834x Device SCI Echoback.
//
// ASSUMPTIONS:
//
//    This program requires the DSP2834x header files.
//    As supplied, this project is configured for "boot to SARAM" operation.
//
//    Connect the SCI-A port to a PC via a transciever and cable.
//    The PC application 'hypterterminal' can be used to view the data
//    from the SCI and to send information to the SCI.  Characters recieved
//    by the SCI port are sent back to the host.
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
//
//    This test recieves and echo-backs data through the SCI-A port.
//
//    1) Configure hyperterminal:
//       Use the included hyperterminal configuration file SCI_96.ht.
//       To load this configuration in hyperterminal: file->open
//       and then select the SCI_96.ht file.
//    2) Check the COM port.
//       The configuration file is currently setup for COM1.
//       If this is not correct, disconnect Call->Disconnect
//       Open the File-Properties dialog and select the correct COM port.
//    3) Connect hyperterminal Call->Call
//       and then start the 2834x SCI echoback program execution.
//    4) The program will print out a greeting and then ask you to
//       enter a character which it will echo back to hyperterminal.
//
//    As is, the program configures SCI-A for 9600 baud with
//    SYSCLKOUT = 300MHz and LSPCLK = 75.0 MHz
//    SYSCLKOUT = 250MHz and LSPCLK = 62.5 Mhz
//    SYSCLKOUT = 200MHz and LSPCLK = 50.0 Mhz

//
//    Watch Variables:
//       LoopCount for the number of characters sent
//       ErrorCount
//
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "..\F28346_Include\DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "..\F28346_Include\Example_2834xSci_Echoback.h"   // Example specific Include file


#define	TMS320C28346CLK		300000000L   


 


#define	LSPCLK		(TMS320C28346CLK/2)
#define	BAUDRATE_A	921600L //1916006

#define	BAUDRATE_B	921600L //115200L

#define	BAUDRATE_C	921600L //921600

#define	BRR_VAL_A		(LSPCLK/(8*BAUDRATE_A)-1)
#define	BRR_VAL_B		(LSPCLK/(8*BAUDRATE_B)-1)
#define BRR_VAL_C		(LSPCLK/(8*BAUDRATE_C)-1)



/*

void main(void)
{

    Uint16 ReceivedChar;
    char *msg;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2834x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2834x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
   // InitGpio(); Skipped for this example

// For this example, only init the pins for the SCI-A port.
// This function is found in the DSP2834x_Sci.c file.
   InitSciaGpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize PIE control registers to their default state.
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

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2834x_InitPeripherals.c
// InitPeripherals(); // Not required for this example

// Step 5. User specific code:

    LoopCount = 0;
    ErrorCount = 0;

    scia_fifo_init();	   // Initialize the SCI FIFO
    scia_echoback_init();  // Initalize SCI for echoback

    msg = "\r\n\n\nHello World!\0";
    scia_msg(msg);

    msg = "\r\nYou will enter a character, and the DSP will echo it back! \n\0";
    scia_msg(msg);

	for(;;)
    {
       msg = "\r\nEnter a character: \0";
       scia_msg(msg);

       // Wait for inc character
       while(SciaRegs.SCIFFRX.bit.RXFFST !=1) { } // wait for XRDY =1 for empty state

       // Get character
       ReceivedChar = SciaRegs.SCIRXBUF.all;

       // Echo character back
       msg = "  You sent: \0";
       scia_msg(msg);
       scia_xmit(ReceivedChar);

       LoopCount++;
    }

}
*/
void scic_init()
{

	ScicRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScicRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all = 0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA =0;   		//ScicRegs.SCICTL2.bit.TXINTENA =1;  20120901
	ScicRegs.SCICTL2.bit.RXBKINTENA =0;
	ScicRegs.SCIHBAUD = BRR_VAL_C >> 8;			// High Value
    ScicRegs.SCILBAUD = BRR_VAL_C & 0xff;		// Low Value
	ScicRegs.SCICCR.bit.LOOPBKENA =0; 			//LOOPBACK DISABLE
	ScicRegs.SCIFFTX.all = 0xc010;
	ScicRegs.SCIFFRX.bit.RXFIFORESET=0;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 0;
	ScicRegs.SCIFFRX.bit.RXFFIL = 16;
	ScicRegs.SCIFFCT.all = 0x00;
	ScicRegs.SCICTL1.all = 0x0023;  				// Relinquish SCI from Reset
	ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;
 	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;

/*
	ScicRegs.SCICCR.all =0x0007;   			// 1 stop bit,  No loopback
						   	   	   	   	   	// No parity,8 char bits,
											// async mode, idle-line protocol
	ScicRegs.SCICTL1.all =0x0003;  			// enable TX, RX, internal SCICLK,
											// Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA =0;   	//ScicRegs.SCICTL2.bit.TXINTENA =1;  20120901
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;

	ScicRegs.SCIHBAUD = BRR_VAL_C >> 8;		// High Value
	ScicRegs.SCILBAUD = BRR_VAL_C & 0xff;	// Low Value

	ScicRegs.SCICCR.bit.LOOPBKENA =0; 		//LOOPBACK DISABLE

	ScicRegs.SCIFFTX.all = 0xC020;

	ScicRegs.SCIFFTX.bit.SCIFFENA =0;		//FIFO Disable;
	ScicRegs.SCIFFTX.bit.TXFFIENA = 0;		//FIFO Interrupt Disable;

	ScicRegs.SCIFFRX.bit.RXFIFORESET=0;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
	ScicRegs.SCIFFRX.bit.RXFFIL = 16;
	ScicRegs.SCIFFCT.all=0x00;

	ScicRegs.SCICTL1.all =0x0023;  			// Relinquish SCI from Reset
	ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;
 	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;
*/

    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

// 	ScicRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
//                                   // No parity,8 char bits,
//                                   // async mode, idle-line protocol
//	ScicRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
//                                   // Disable RX ERR, SLEEP, TXWAKE
//	ScicRegs.SCICTL2.all = 0x0003;
//	ScicRegs.SCICTL2.bit.TXINTENA = 0;
//	ScicRegs.SCICTL2.bit.RXBKINTENA = 0;
//
//
//
//	ScicRegs.SCIHBAUD = BRR_VAL_C>> 8;		// High Value
//	ScicRegs.SCILBAUD = BRR_VAL_C & 0x00ff;	// Low Value
//
//
//	ScicRegs.SCICCR.bit.LOOPBKENA =0; //LOOPBACK DISABLE
//
//
//
//	ScicRegs.SCIFFTX.all = 0xc028;
//	ScicRegs.SCIFFRX.all = 0x0028;
//	ScicRegs.SCIFFCT.all = 0x00;
//
//
//	ScicRegs.SCIFFRX.bit.RXFFIENA = 0;
//	ScicRegs.SCIFFRX.bit.RXFFIL = 16;
//
//	ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;
// 	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;

// 	ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
//                                   // No parity,8 char bits,
//                                   // async mode, idle-line protocol
//	ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
//                                   // Disable RX ERR, SLEEP, TXWAKE
//	ScicRegs.SCICTL2.all =0x0003;
//	ScicRegs.SCICTL2.bit.TXINTENA =1;
//	ScicRegs.SCICTL2.bit.RXBKINTENA =1;
//
//
//
//	ScicRegs.SCIHBAUD = BRR_VAL_C>> 8;		// High Value
//	ScicRegs.SCILBAUD = BRR_VAL_C & 0x00ff;	// Low Value
//
//
//	ScicRegs.SCICCR.bit.LOOPBKENA =0; //LOOPBACK DISABLE
//
//
//
//	ScicRegs.SCIFFTX.all = 0xC02c;
//	ScicRegs.SCIFFRX.all = 0x0028;
//	ScicRegs.SCIFFCT.all=0x00;
//
//
//	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
//	ScicRegs.SCIFFRX.bit.RXFFIL = 16;
//
//	ScicRegs.SCICTL1.all =0x0023;
//
//	ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;
// 	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;

}
// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
//void scib_echoback_init()
void scib_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function


	ScibRegs.SCICCR.all =0x0007;   					// 1 stop bit,  No loopback
													// No parity,8 char bits,
													// async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  					// enable TX, RX, internal SCICLK,
													// Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA =0;   			//ScibRegs.SCICTL2.bit.TXINTENA =1;  20120901
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;

	ScibRegs.SCIHBAUD = BRR_VAL_A >> 8;				// High Value
	ScibRegs.SCILBAUD = BRR_VAL_A & 0xff;			// Low Value
	ScibRegs.SCICCR.bit.LOOPBKENA =0; 				// LOOPBACK DISABLE

	ScibRegs.SCIFFTX.bit.SCIFFENA =1;				// SCI FIFO Enable
	ScibRegs.SCIFFTX.bit.SCIRST = 1;				// SCI Reset clear

	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;			// RX FIFO Interrupt Flag Clear
	ScibRegs.SCIFFRX.bit.RXFFIENA = 1;				// RX FIFO Interrupt Enable
	ScibRegs.SCIFFRX.bit.RXFIFORESET= 1;			// RX FIFO Re Enable
	ScibRegs.SCIFFRX.bit.RXFFIL = 1;

	ScibRegs.SCICTL1.bit.SWRESET =1;				// SCI S/W Rest Disable


//f
}


// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
//void scia_echoback_init()
void scia_init()
{


	SciaRegs.SCICCR.all =0x0007;   					// 1 stop bit,  No loopback
													// No parity,8 char bits,
													// async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  					// enable TX, RX, internal SCICLK,
													// Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA =1;   			//SciaRegs.SCICTL2.bit.TXINTENA =1;  20120901
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
	SciaRegs.SCIHBAUD = BRR_VAL_A >> 8;				// High Value
	SciaRegs.SCILBAUD = BRR_VAL_A & 0xff;			// Low Value
	SciaRegs.SCICCR.bit.LOOPBKENA =0; 				// LOOPBACK DISABLE
	SciaRegs.SCIFFTX.all = 0xC028;					// FIFO LEVEL FIFO = 8
	SciaRegs.SCIFFRX.bit.RXFIFORESET=0;
	SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
	SciaRegs.SCIFFRX.bit.RXFFIL = 16;
	SciaRegs.SCIFFCT.all=0x00;
	SciaRegs.SCICTL1.all =0x0023;  					// Relinquish SCI from Reset
	SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	SciaRegs.SCIFFRX.bit.RXFIFORESET=1;


    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

// 	SciaRegs.SCICCR.all |= 0x0007;   // 1 stop bit,  No loopback
//                                   // No parity,8 char bits,
//                                   // async mode, idle-line protocol
//	SciaRegs.SCICTL1.all |= 0x0003;  // enable TX, RX, internal SCICLK,
//                                   // Disable RX ERR, SLEEP, TXWAKE
//
//	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;	//RX interrupt Enable
//	SciaRegs.SCICTL2.bit.TXINTENA = 1;	//TX interrupt Enable
//
//
//	SciaRegs.SCIHBAUD = BRR_VAL_A >> 8;		// High Value
//	SciaRegs.SCILBAUD = BRR_VAL_A & 0xff;	// Low Value
//
//	SciaRegs.SCICCR.bit.LOOPBKENA = 0; //LOOPBACK DISABLE
//
//	SciaRegs.SCIFFTX.bit.SCIRST = 1;	//FIFO RESET IF 1 THEN FIFO FUNCTION ENABLE ELSE 0 THEN FIFO FUNCTION RESET BUT FIFO CONFIGRATION IS CONTINUE
//	SciaRegs.SCIFFTX.bit.SCIFFENA = 0;  //FIFO TX Enable
//	SciaRegs.SCIFFTX.bit.TXFFIENA = 0;	//TXFIFO INTERRUPT ENABLE
//	SciaRegs.SCIFFTX.bit.TXFFIL = 0;	//TXFIFO INTERRUPT COMPARE VALUE 0~16
//
//	SciaRegs.SCIFFRX.bit.RXFFIENA = 1;	//RXFIFO INTERRUPT ENABLE
//	SciaRegs.SCIFFRX.all |= 0x000f;	//RXFIFO INTERRUPT COMPARE MATCH VALUE 0~16
//
//
//
//	SciaRegs.SCIFFCT.all |= 0x00;
//	SciaRegs.SCICTL1.all |=0x0023;  					// Relinquish SCI from Reset
//	SciaRegs.SCICTL1.bit.SWRESET = 1;	//O THEN SCI MODULE FLAG ALL CLEAR - 1 THEN FLAG READY
//
//	SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;//TXFIFO REENABLE
//	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;//RXFIFO REENABLE
}

// Transmit a character from the SCI
void scic_xmit(int a)
{
    while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScicRegs.SCITXBUF=a;

}
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;

}

void scib_xmit(int a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScibRegs.SCITXBUF=a;

}


void scic_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scic_xmit(msg[i]);
        i++;
    }
}
void scib_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scib_xmit(msg[i]);
        i++;
    }
}

void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

// Initalize the SCI FIFO
// Initalize the SCI FIFO
void scib_fifo_init()
{
    ScibRegs.SCIFFTX.all=0xE040;
   // SciaRegs.SCIFFRX.all=0x204f;
    ScibRegs.SCIFFCT.all=0x0;

}

void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
   // SciaRegs.SCIFFRX.all=0x204f;
    SciaRegs.SCIFFCT.all=0x0;

}


Uint8 scia_getchar(void)
{
	Uint8	ReceivedChar;

	while(SciaRegs.SCIFFRX.bit.RXFFST !=1) {}
		ReceivedChar = SciaRegs.SCIRXBUF.all;

	return ReceivedChar;

}

Uint8 scib_getchar(void)
{

	Uint8	ReceivedChar;

	while(ScibRegs.SCIFFRX.bit.RXFFST !=1) {}
		ReceivedChar = ScibRegs.SCIRXBUF.all;

	return ReceivedChar;

}

Uint8 scic_getchar(void)
{

	Uint8	ReceivedChar;

	while(ScicRegs.SCIFFRX.bit.RXFFST !=1) {}
		ReceivedChar = ScicRegs.SCIRXBUF.all;

	return ReceivedChar;

}
//===========================================================================
// No more.
//===========================================================================

