/*======================================================================
	File name	:	DSP28x_ADC28027.h                    
                    
	Originator	:	Digital Control Systems Group
					SyncWorks

	Target		:	TMS320F28xx DSP

	Version		:	1.00
======================================================================*/

/*======================================================================
	History		:
		2011-11-12,		Version 1.00
======================================================================*/

#ifndef DSP28x_ADC28027_H
#define DSP28x_ADC28027_H

#ifdef DSP28x_ADC28027_GLOBAL
#define DSP28x_ADC28027_EXT

#include "DSP28x_Project.h"


#define ADC_STANDBY			0x2000
#define ADC_CONFIG			0x2001
#define FILTER_CONFIG		0x2002
#define ADC_START			0x2003

#if CPU_FRQ_300MHZ                                          // For 300 MHz SYSCLKOUT
  #define CPU_SPD              300E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif
#if CPU_FRQ_250MHZ                                          // For 250 MHz SYSCLKOUT(default)
  #define CPU_SPD              250E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif
#if CPU_FRQ_200MHZ                                          // For 200 MHz SYSCLKOUT
  #define CPU_SPD              200E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif

#define CLKGDV_VAL           1
#define MCBSP_INIT_DELAY     2*(CPU_SPD/MCBSP_SRG_FREQ)                  // # of CPU cycles in 2 SRG cycles-init delay
#define MCBSP_CLKG_DELAY     2*(CPU_SPD/(MCBSP_SRG_FREQ/(1+CLKGDV_VAL))) // # of CPU cycles in 2 CLKG cycles-init delay

struct	DWORDS {		// description
	Uint16 LOW;			// 하위 Word
	Uint16 HIGH;		// 상위 Word 
};
union	DWORD_SET {
	Uint32 			UINT32;
	float32			FLOAT32;
	struct DWORDS	word;
}UintToFloat;	


#else
#define DSP28x_ADC28027_EXT extern

#endif

DSP28x_ADC28027_EXT Uint16 SSpi16Driver(Uint16 Data);	
DSP28x_ADC28027_EXT void ADC_Cnf(float32 SamFreq, Uint16 MaxConv, Uint16 *ChSel, Uint16 AcqPs);
DSP28x_ADC28027_EXT void Filter_Cnf(Uint16 FilterType, Uint16 FilterCoeffSel);
DSP28x_ADC28027_EXT void ADC_Run(Uint16 FilterEnable);
DSP28x_ADC28027_EXT void Init_Mcbspa_Gpio(void);
DSP28x_ADC28027_EXT void init_mcbsp_spi_master(void);
DSP28x_ADC28027_EXT void init_mcbsp_spi_slave(void);

#endif

