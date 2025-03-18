/*======================================================================
	File name	:	DSP2834x_ADC28027.c                    
                    
	Originator	:	Digital Control Systems Group
					SyncWorks

	Target		:	TMS320F28xx DSP

	Version		:	1.00
======================================================================*/

/*======================================================================
	History		:
		2011-10-12,		Version 1.00
======================================================================*/

#define DSP28x_ADC28027_GLOBAL
#include "..\F28346_Include\DSP28x_ADC28027.h"


/*---------------------------------------------------------------------------
	SPI Driver : 16비트 데이터 입출력 함수
---------------------------------------------------------------------------*/
Uint16 SSpi16Driver(Uint16 Data)
{
	Uint16 Read;

	while(!(McbspaRegs.SPCR2.bit.XRDY));
	McbspaRegs.DXR1.all = Data;
	GpioDataRegs.GPBSET.bit.GPIO35 = 1;

	DELAY_US(50L);
	
	while(!(McbspaRegs.SPCR1.bit.RRDY));
	Read = McbspaRegs.DRR1.all;
	GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;
	
	return Read;
}


/*---------------------------------------------------------------------------
	외부 TMS320F28027 ADC 초기화 함수
---------------------------------------------------------------------------*/
void ADC_Cnf(float32 SamFreq, Uint16 MaxConv, Uint16 *ChSel, Uint16 AcqPs)
{

	// 명령 모드 전송 
	SSpi16Driver(ADC_CONFIG);				DELAY_US(50L);			


	// ADC 샘플링 속도 전송 
	UintToFloat.FLOAT32 = SamFreq;
	SSpi16Driver(UintToFloat.word.HIGH);	DELAY_US(50L);
	SSpi16Driver(UintToFloat.word.LOW);		DELAY_US(50L);

	// ADC 변환 채널 수 전송
	SSpi16Driver(MaxConv & 0x001F);			DELAY_US(50L);

	// ADC 변환 채널 순서 전송
	SSpi16Driver(ChSel[0] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[1] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[2] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[3] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[4] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[5] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[6] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[7] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[8] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[9] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[10] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[11] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[12] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[13] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[14] & 0x000F);		DELAY_US(50L);
	SSpi16Driver(ChSel[15] & 0x000F);		DELAY_US(50L);

	// ADC S/H 윈도우 크기 전송
	SSpi16Driver(AcqPs & 0x007F);			DELAY_US(50L);

	// 명령 데이터 종료 
	SSpi16Driver(0xF0F0);

	DELAY_US(50L);
}


/*---------------------------------------------------------------------------
	외부 TMS320F28027 Filter 초기화 함수  
---------------------------------------------------------------------------*/
void Filter_Cnf(Uint16 FilterType, Uint16 FilterCoeffSel)
{
	// 명령 모드 전송 
	SSpi16Driver(FILTER_CONFIG);			DELAY_US(50L);

	// Filter 타입 전송 
	SSpi16Driver(FilterType & 0x000F);		DELAY_US(50L);

	// Filter 계수 전송 
	SSpi16Driver(FilterCoeffSel & 0x000F);	DELAY_US(50L);

	// 명령 데이터 종료 
	SSpi16Driver(0xF0F0);

	DELAY_US(50L);
}


/*---------------------------------------------------------------------------
	외부 TMS320F28027 ADC 운용 시작 함수  
---------------------------------------------------------------------------*/
void ADC_Run(Uint16 FilterEnable)
{
	// 명령 모드 전송 
	SSpi16Driver(ADC_START);	DELAY_US(50L);

	// 명령 데이터 종료
	SSpi16Driver(0xF0F0);		
	DELAY_US(50L);

	// Starts DMA Channel 1
	//StartDMACH1();

	if(FilterEnable==0){
		// 12bit 통신 설정 및 McBSP 수신 인터럽트 Enable
		McbspaRegs.RCR1.bit.RWDLEN1=2;		// 12-bit word
		McbspaRegs.XCR1.bit.XWDLEN1=2;		// 12-bit word
		McbspaRegs.MFFINT.bit.RINT = 1;		// RX Intterrupt Enable 
		DELAY_US(50L);
	}
	else{
		// 16bit 통신 설정 및 McBSP 수신 인터럽트 Enable
		McbspaRegs.RCR1.bit.RWDLEN1=2;		// 16-bit word
		McbspaRegs.XCR1.bit.XWDLEN1=2;		// 16-bit word
		McbspaRegs.MFFINT.bit.RINT = 1;		// RX Intterrupt Enable 
		DELAY_US(50L);
	}
}


/*---------------------------------------------------------------------------
	McBSP-A PIN 설정 (GPIO 20, 21, 22, 23 번을 McBSP 기능으로 설정)
---------------------------------------------------------------------------*/
void Init_Mcbspa_Gpio(void)
{
	EALLOW;

/* Configure McBSP-A pins using GPIO regs */
// This specifies which of the possible GPIO pins will be McBSP functional pins.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2;	// GPIO20 is MDXA pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2;	// GPIO21 is MDRA pin
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2;	// GPIO22 is MCLKXA pin
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 2;	// GPIO23 is MFSXA pin
	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;	// GPIO35 is GPIO35 pin
	GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;		// GPIO35 is Output
	GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;	// Used to McBSP Tx Ready Flag pin

/* Enable internal pull-up for the selected I/O pins. Disable
   pull-ups on output-only pins to reduce power consumption */
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;     // Disable pull-up on GPIO20 (MDXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;     // Enable pull-up on GPIO21 (MDRA)
	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;     // Enable pull-up on GPIO22 (MCLKXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;     // Enable pull-up on GPIO23 (MFSXA)

/* Set qualification for selected input pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3;   // Asynch input GPIO21 (MDRA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 3;   // Asynch input GPIO22 (MCLKXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;   // Asynch input GPIO23 (MFSXA)

	EDIS;

}


/*---------------------------------------------------------------------------
    McBSP를 SPI로 사용할 수 있도록 초기화
---------------------------------------------------------------------------*/
void init_mcbsp_spi_master(void)
{
	// Low Speed Clock Prescaler 설정 
	EALLOW;
	SysCtrlRegs.LOSPCP.all = 0x0001;		// LOSCLK = SYSCLKOUT/(LOSPCP*2)
	EDIS;									// LOSCLK = 300MHz	


	// McBSP-A register settings
    McbspaRegs.SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
	McbspaRegs.SPCR1.all=0x0000;		// Reset Receiver, Right justify word, Digital loopback dis.
    McbspaRegs.PCR.all=0x0F08;			//(CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
    McbspaRegs.SPCR1.bit.CLKSTP = 2;	// Together with CLKXP/CLKRP determines clocking scheme
	McbspaRegs.PCR.bit.CLKXP = 0;		// CPOL = 0, CPHA = 0 falling edge no delay
	McbspaRegs.PCR.bit.CLKRP = 0;
    McbspaRegs.RCR2.bit.RDATDLY=01;		// FSX setup time 1 in master mode. 0 for slave mode (Receive)
    McbspaRegs.XCR2.bit.XDATDLY=01;		// FSX setup time 1 in master mode. 0 for slave mode (Transmit)

	McbspaRegs.RCR1.bit.RWDLEN1=2;		// 16-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=2;		// 16-bit word

    McbspaRegs.SRGR2.all=0x2000;		// CLKSM=1, FPER = 1 CLKG periods
    McbspaRegs.SRGR1.all= 14;			// Frame Width = 1 CLKG period, CLKGDV=38

    McbspaRegs.SPCR2.bit.GRST=1;		// Enable the sample rate generator
	delay_loop();						// Wait at least 2 SRG clock cycles
	McbspaRegs.SPCR2.bit.XRST=1;		// Release TX from Reset
	McbspaRegs.SPCR1.bit.RRST=1;		// Release RX from Reset
    McbspaRegs.SPCR2.bit.FRST=1;		// Frame Sync Generator reset

	McbspaRegs.MFFINT.bit.RINT = 0;		// RX Intterrupt Disable 
	//McbspaRegs.MFFINT.bit.RINT = 1;	// RX Intterrupt Enable 
}

/*---------------------------------------------------------------------------
    McBSP를 SPI로 사용할 수 있도록 초기화
---------------------------------------------------------------------------*/
void init_mcbsp_spi_slave(void)
{
	// Low Speed Clock Prescaler 설정 
	EALLOW;
	SysCtrlRegs.LOSPCP.all = 0x0001;		// LOSCLK = SYSCLKOUT/(LOSPCP*2)
	EDIS;									// LOSCLK = 300MHz	

	// McBSP-A register settings
    McbspaRegs.SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
	McbspaRegs.SPCR1.all=0x0000;		// Reset Receiver, Right justify word, Digital loopback dis.
    McbspaRegs.PCR.all=0x0508;			//(CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
    McbspaRegs.SPCR1.bit.CLKSTP = 2;	// Together with CLKXP/CLKRP determines clocking scheme
	McbspaRegs.PCR.bit.CLKXP = 0;		// CPOL = 0, CPHA = 0 falling edge no delay
	McbspaRegs.PCR.bit.CLKRP = 1;
 
	McbspaRegs.RCR2.bit.RDATDLY=00;		// FSX setup time 1 in master mode. 0 for slave mode (Receive)
	McbspaRegs.XCR2.bit.XDATDLY=00;		// FSX setup time 1 in master mode. 0 for slave mode (Transmit)

	McbspaRegs.RCR1.bit.RWDLEN1=2;		// 16-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=2;		// 16-bit word

    McbspaRegs.SRGR2.all=0x2000;		// CLKSM=1, FPER = 1 CLKG periods
    McbspaRegs.SRGR1.all= 0;			// Frame Width = 1 CLKG period, CLKGDV=38

    McbspaRegs.SPCR2.bit.GRST=1;		// Enable the sample rate generator
	delay_loop();						// Wait at least 2 SRG clock cycles
	McbspaRegs.SPCR2.bit.XRST=1;		// Release TX from Reset
	McbspaRegs.SPCR1.bit.RRST=1;		// Release RX from Reset
    McbspaRegs.SPCR2.bit.FRST=1;		// Frame Sync Generator reset

	McbspaRegs.MFFINT.bit.RINT = 0;		// RX Intterrupt Disable 
	//McbspaRegs.MFFINT.bit.RINT = 1;	// RX Intterrupt Enable 
}


/*---------------------------------------------------------------------------
    delay in McBsp init. must be at least 2 SRG cycles
---------------------------------------------------------------------------*/
void delay_loop(void)
{
    long      i;
    for (i = 0; i < MCBSP_INIT_DELAY; i++) {}
}


/*---------------------------------------------------------------------------
    delay in McBsp init. must be at least 2 SRG cycles
---------------------------------------------------------------------------*/
void clkg_delay_loop(void)
{
    long      i;
    for (i = 0; i < MCBSP_CLKG_DELAY; i++) {}
}
