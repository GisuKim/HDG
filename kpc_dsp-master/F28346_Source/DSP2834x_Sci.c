// TI File $Revision: /main/2 $
// Checkin $Date: August 1, 2008   16:35:40 $
//###########################################################################
//
// FILE:	DSP2834x_Sci.c
//
// TITLE:	DSP2834x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File


#if (CPU_FRQ_300MHZ)
#define	TMS320C28346CLK		300000000L
#endif
#if (CPU_FRQ_250MHZ)
#define	TMS320C28346CLK		250000000L
#endif
#if (CPU_FRQ_200MHZ)
#define	TMS320C28346CLK		200000000L
#endif

#define	LSPCLK		(TMS320C28346CLK/2)

#define	BAUDRATE_A	115200L //1916006
#define	BAUDRATE_B  115200L //115200L
#define	BAUDRATE_C	921600L //921600
#define BAUDRATE_BT 230400L //230400

#define	BRR_VAL_A		(LSPCLK/(8*BAUDRATE_A)-1)
#define	BRR_VAL_B		(LSPCLK/(8*BAUDRATE_B)-1)
#define BRR_VAL_C		(LSPCLK/(8*BAUDRATE_C)-1)
#define BRR_VAL_BT		(LSPCLK/(8*BAUDRATE_BT)-1)

//---------------------------------------------------------------------------
// InitSci:
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
//


void InitSci(void)
{

	scia_Sys_init();
	scib_Sys_init();
//	scic_Sys_init();

}

//---------------------------------------------------------------------------
// Example: InitSciGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SCI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// Only one GPIO pin should be enabled for SCITXDA/B operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA/B operation.
// Comment out other unwanted lines.
// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void EnableSciaRxInterrupt(void)
{
	SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
}
void DisableSciaRxInterrupt(void)
{
	SciaRegs.SCIFFRX.bit.RXFFIENA = 0;
}
void scia_Sys_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function


	SciaRegs.SCICCR.all =0x0007;   					// 1 stop bit,  No loopback
													// No parity,8 char bits,
													// async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  					// enable TX, RX, internal SCICLK,
													// Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA =0;   			//SciaRegs.SCICTL2.bit.TXINTENA =1;  20120901
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;

	SciaRegs.SCICCR.bit.LOOPBKENA =0; 				// LOOPBACK DISABLE
	SciaRegs.SCIFFTX.all = 0xC028;					// FIFO LEVEL FIFO = 8
	SciaRegs.SCIFFRX.bit.RXFIFORESET=0;
	SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
	SciaRegs.SCIFFRX.bit.RXFFIL = 9;
	SciaRegs.SCIFFCT.all=0x00;

	SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	SciaRegs.SCIFFRX.bit.RXFIFORESET=1;



	#if (CPU_FRQ_300MHZ)
	      SciaRegs.SCIHBAUD    = BRR_VAL_A >> 8;				// High Value
	      SciaRegs.SCILBAUD    = BRR_VAL_A & 0xff;				// Low Value
	#endif
	#if (CPU_FRQ_250MHZ)
	      SciaRegs.SCIHBAUD    = BRR_VAL_A >> 8;				// High Value
	      SciaRegs.SCILBAUD    = BRR_VAL_A & 0xff;				// Low Value
	#endif
	#if (CPU_FRQ_200MHZ)
      	  SciaRegs.SCIHBAUD    = BRR_VAL_A >> 8;				// High Value
      	  SciaRegs.SCILBAUD    = BRR_VAL_A & 0xff;				// Low Value
	#endif


	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

}

void scib_Sys_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

	ScibRegs.SCICCR.bit.SCICHAR = 7;		// SCI 송수신 Charcter-length 설정 : 8bit
	ScibRegs.SCICTL1.bit.RXENA = 1;			// SCI 송신기능 Enable

	ScibRegs.SCIFFTX.bit.SCIFFENA = 1;		// SCI FIFO 사용 설정 Enable
	ScibRegs.SCIFFTX.bit.SCIRST = 1;		// SCI 리셋 해제

	// SCI의 수신 FIFO 설정
	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// SCI 수신 FIFO 인터럽트 플래그 클리어
	ScibRegs.SCIFFRX.bit.RXFFIENA = 1;		// SCI 수신 FIFO 인터럽트 Enable
	ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;	// SCI 수신 FIFO RE-enable
	ScibRegs.SCIFFRX.bit.RXFFIL = 5;		// SCI 수신 FIFO 인터럽트 레벨 설정


	ScibRegs.SCICTL1.bit.SWRESET = 1;		// SCI 소프트웨어 리셋 해제
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;		// SCI 수신 인터럽트 허용

	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;	//scirxintb



	#if (CPU_FRQ_300MHZ)
	      ScibRegs.SCIHBAUD    =BRR_VAL_B >> 8;				// High Value
	      ScibRegs.SCILBAUD    =BRR_VAL_B & 0xff;			// Low Value
	#endif
	#if (CPU_FRQ_250MHZ)
	      SciaRegs.SCIHBAUD    =BRR_VAL_B >> 8;				// High Value
	      SciaRegs.SCILBAUD    =BRR_VAL_B & 0xff;			// Low Value
	#endif
	#if (CPU_FRQ_200MHZ)
      	  SciaRegs.SCIHBAUD    =BRR_VAL_B >> 8;				// High Value
      	  SciaRegs.SCILBAUD    =BRR_VAL_B & 0xff;			// Low Value
	#endif


	ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scic_Sys_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

	ScicRegs.SCICCR.bit.SCICHAR = 7;		// SCI 송수신 Charcter-length 설정 : 8bit
	ScicRegs.SCICTL1.bit.RXENA = 1;			// SCI 송신기능 Enable

	ScicRegs.SCIFFTX.bit.SCIFFENA = 1;		// SCI FIFO 사용 설정 Enable
	ScicRegs.SCIFFTX.bit.SCIRST = 1;		// SCI 리셋 해제

	// SCI의 수신 FIFO 설정
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// SCI 수신 FIFO 인터럽트 플래그 클리어
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;		// SCI 수신 FIFO 인터럽트 Enable
	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;	// SCI 수신 FIFO RE-enable
	ScicRegs.SCIFFRX.bit.RXFFIL = 5;		// SCI 수신 FIFO 인터럽트 레벨 설정


	ScicRegs.SCICTL1.bit.SWRESET = 0;		// SCI 소프트웨어 리셋 해제
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;		// SCI 수신 인터럽트 허용
//	ScicRegs.SCIFFTX.bit.SCIRST = 0;		// SCI 리셋 해제
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// PIE 인터럽트(SCIRXINTA) : Enable

	#if (CPU_FRQ_300MHZ)
	      ScicRegs.SCIHBAUD    = BRR_VAL_C >> 8;				// High Value
	      ScicRegs.SCILBAUD    = BRR_VAL_C & 0xff;				// Low Value
	#endif
	#if (CPU_FRQ_250MHZ)
	      SciaRegs.SCIHBAUD    = BRR_VAL_C >> 8;				// High Value
	      SciaRegs.SCILBAUD    = BRR_VAL_C & 0xff;				// Low Value
	#endif
	#if (CPU_FRQ_200MHZ)
      	  SciaRegs.SCIHBAUD    = BRR_VAL_C >> 8;				// High Value
      	  SciaRegs.SCILBAUD    = BRR_VAL_C & 0xff;				// Low Value
	#endif
	ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}
void InitSciGpio()
{
   InitSciaGpio();

#if DSP28_SCIB
   InitScibGpio();
#endif // if DSP28_SCIB
#if DSP28_SCIC
   InitScicGpio();
#endif // if DSP28_SCIC
}

void InitSciaGpio()
{
   EALLOW;

	GpioCtrlRegs.GPBPUD.bit.GPIO35 = 1;    // Disable pull-up for GPIO35 (SCITXDA)
	GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;	   // Enable pull-up for GPIO36 (SCIRXDA)

	GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 3;  // Asynch input GPIO28 (SCIRXDA)

	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 1;   // Configure GPIO28 for SCIRXDA operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 1;   // Configure GPIO29 for SCITXDA operation

    EDIS;
}

#if DSP28_SCIB
void InitScibGpio()
{
	EALLOW;

//	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up for GPIO15 (SCIRXDB)
//	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    // Disable pull-up for GPIO14 (SCITXDB)
//
//	GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
//
//	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;   // Configure GPIO15 for SCIRXDB operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;   // Configure GPIO14 for SCITXDB operation

    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;    // Enable pull-up for GPIO15 (SCIRXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;    // Disable pull-up for GPIO14 (SCITXDB)

    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO15 (SCIRXDB)

    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO15 for SCIRXDB operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO14 for SCITXDB operation

	EDIS;
}
#endif // if DSP28_SCIB

#if DSP28_SCIC
void InitScicGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected I/O pins. Disable
   pull-up for output-only pins to reduce power consumption */
// Pull-ups can be enabled or disabled disabled by the user.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1;	   // Disable pull-up for GPIO63 (SCITXDC)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

    EDIS;
}
#endif // if DSP28_SCIC

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

void scic_Sys_Buad_init(Uint8 Mode)
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
	ScicRegs.SCICCR.bit.SCICHAR = 7;		// SCI 송수신 Charcter-length 설정 : 8bit
	ScicRegs.SCICTL1.bit.RXENA = 1;			// SCI 송신기능 Enable

	ScicRegs.SCIFFTX.bit.SCIFFENA = 1;		// SCI FIFO 사용 설정 Enable
	ScicRegs.SCIFFTX.bit.SCIRST = 1;		// SCI 리셋 해제

	// SCI의 수신 FIFO 설정
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// SCI 수신 FIFO 인터럽트 플래그 클리어
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;		// SCI 수신 FIFO 인터럽트 Enable
	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;	// SCI 수신 FIFO RE-enable
	ScicRegs.SCIFFRX.bit.RXFFIL = 5;		// SCI 수신 FIFO 인터럽트 레벨 설정


	ScicRegs.SCICTL1.bit.SWRESET = 1;		// SCI 소프트웨어 리셋 해제

//	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// PIE 인터럽트(SCIRXINTA) : Enable

	if(Mode==0)
	{
		#if (CPU_FRQ_300MHZ)
			  ScicRegs.SCIHBAUD    = BRR_VAL_C >> 8;				// High Value
			  ScicRegs.SCILBAUD    = BRR_VAL_C & 0xff;				// Low Value
		#endif
		#if (CPU_FRQ_250MHZ)
			  SciaRegs.SCIHBAUD    = BRR_VAL_C >> 8;				// High Value
			  SciaRegs.SCILBAUD    = BRR_VAL_C & 0xff;				// Low Value
		#endif
		#if (CPU_FRQ_200MHZ)
			  SciaRegs.SCIHBAUD    = BRR_VAL_C >> 8;				// High Value
			  SciaRegs.SCILBAUD    = BRR_VAL_C & 0xff;				// Low Value
		#endif
	}
	else if(Mode==1)
	{
		#if (CPU_FRQ_300MHZ)
			  ScicRegs.SCIHBAUD    = BRR_VAL_BT >> 8;				// High Value
			  ScicRegs.SCILBAUD    = BRR_VAL_BT & 0xff;				// Low Value
		#endif
		#if (CPU_FRQ_250MHZ)
			  SciaRegs.SCIHBAUD    = BRR_VAL_BT >> 8;				// High Value
			  SciaRegs.SCILBAUD    = BRR_VAL_BT & 0xff;				// Low Value
		#endif
		#if (CPU_FRQ_200MHZ)
			  SciaRegs.SCIHBAUD    = BRR_VAL_BT >> 8;				// High Value
			  SciaRegs.SCILBAUD    = BRR_VAL_BT & 0xff;				// Low Value
		#endif
	}
	ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

//===========================================================================
// End of file.
//===========================================================================
