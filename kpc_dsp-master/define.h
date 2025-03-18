/*
 * define.h
 *
 *  Created on: 2014. 2. 19.
 *      Author: Administrator
 */

#ifndef DEFINE_H_
#define DEFINE_H_

//--- Math define ----------------------------------

#define SQRT2						1.414213562
#define SQRT3						1.732050808
#define INV_SQRT3					0.577350269
#define sign(num) ((num>=0)   1:-1)

//--- System define --------------------------------

#define TRUE						1
#define FALSE						0

//--- User define ----------------------------------

#define PI						((float)3.1415926535897932384626433832795)
#define INV_2PI					((float)0.15915494309189533576888376337251)

//--- 2014.2.19

//--- ADRESS SET

//	PAGE1 ADDRESS RANGE	: 0x400000H ~ 0x7FFFFFH
//	PAGE2 ADDRESS RANGE : 0x800000H ~ 0xBFFFFFH
//	XINTF_ZONE0 ADDRESS RANGE : 0x004000 ~ 0x004FFF
//	XINTF_ZONE6 ADDRESS RANGE : 0x100000 ~ 0x1FFFFF
//	XINTF_ZONE7 ADDRESS RANGE : 0x200000 ~ 0x2FFFFF
#define XINTF_ZONE0		0x004000
#define XINTF_ZONE6		0x100000
#define XINTF_ZONE7		0x200000

//Select Encoder#0 Position Data

#define RAM_ZONE6							(volatile int *)(0x100000)
//#define RAM_OE								(volatile int *)(0x100001)

#define RDC_DATA							(volatile int *)(0x200001)


/*
#define REG32(addr) 		(*(volatile unsigned int*)(addr))	//외부 장치 메모리 접근용
#define RDC			REG32(0x210000)
#define ADC1_CONVST		REG32(0x210001)
#define ADC1_EOC		REG32(0x210002)
*/
//--- GPIO SET ------------------

#define HI_LOAD		GpioDataRegs.GPASET.bit.GPIO2 =1
#define LO_LOAD		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1

//#define LED0_OFF GpioDataRegs.GPASET.bit.GPIO10 = 1
//#define LED1_OFF GpioDataRegs.GPASET.bit.GPIO11 = 1
//#define LED2_OFF GpioDataRegs.GPASET.bit.GPIO12 = 1
//#define LED3_OFF GpioDataRegs.GPASET.bit.GPIO13 = 1
//
//#define LED0_ON GpioDataRegs.GPACLEAR.bit.GPIO10 = 1
//#define LED1_ON GpioDataRegs.GPACLEAR.bit.GPIO11 = 1
//#define LED2_ON GpioDataRegs.GPACLEAR.bit.GPIO12 = 1
//#define LED3_ON GpioDataRegs.GPACLEAR.bit.GPIO13 = 1

#define LED0_ON GpioDataRegs.GPASET.bit.GPIO10 = 1
#define LED1_ON GpioDataRegs.GPASET.bit.GPIO11 = 1
#define LED2_ON GpioDataRegs.GPASET.bit.GPIO12 = 1
#define LED3_ON GpioDataRegs.GPASET.bit.GPIO13 = 1

#define LED0_OFF GpioDataRegs.GPACLEAR.bit.GPIO10 = 1
#define LED1_OFF GpioDataRegs.GPACLEAR.bit.GPIO11 = 1
#define LED2_OFF GpioDataRegs.GPACLEAR.bit.GPIO12 = 1
#define LED3_OFF GpioDataRegs.GPACLEAR.bit.GPIO13 = 1

#define MUX_EN_HI GpioDataRegs.GPASET.bit.GPIO2 = 1
#define MUX_S0_HI GpioDataRegs.GPASET.bit.GPIO3 = 1
#define MUX_S1_HI GpioDataRegs.GPASET.bit.GPIO4 = 1
#define MUX_S2_HI GpioDataRegs.GPASET.bit.GPIO5 = 1

#define MUX_EN_LO GpioDataRegs.GPACLEAR.bit.GPIO2 = 1
#define MUX_S0_LO GpioDataRegs.GPACLEAR.bit.GPIO3 = 1
#define MUX_S1_LO GpioDataRegs.GPACLEAR.bit.GPIO4 = 1
#define MUX_S2_LO GpioDataRegs.GPACLEAR.bit.GPIO5 = 1

#define RDC_INHIBIT_HI GpioDataRegs.GPASET.bit.GPIO6 = 1
#define RDC_EN_HI GpioDataRegs.GPASET.bit.GPIO7 = 1

#define RDC_INHIBIT_LO GpioDataRegs.GPACLEAR.bit.GPIO6 = 1
#define RDC_EN_LO GpioDataRegs.GPACLEAR.bit.GPIO7 = 1


typedef union joiner {
    float fp;
    Uint16 w[2];
} COMBINE_DEF;

typedef struct MSG_STRUCT{
                Uint8           m_uMsgType;
                Uint8           m_uDataType;
                COMBINE_DEF      m_ulData;
            }MSG_STRUCT_DEF;

typedef struct SETTING_MSG_STRUCT{
                Uint16      m_uiConnedted;
                Uint16      m_uiPulseWidth;
                Uint16      m_uiPulsePeriode;
                Uint16      m_uiCh1_OnTime;
                Uint16      m_uiCh2_OnTime;
                Uint16      m_uiCh3_OnTime;
                Uint16      m_uiCh4_OnTime;
                Uint16      m_uiCh1_OffTime;
                Uint16      m_uiCh2_OffTime;
                Uint16      m_uiCh3_OffTime;
                Uint16      m_uiCh4_OffTime;
                Uint16      m_uiCh1_PhaseTime;
                Uint16      m_uiCh2_PhaseTime;
                Uint16      m_uiCh3_PhaseTime;
                Uint16      m_uiCh4_PhaseTime;
            }SETTING_MSG_STRUCT_DEF;
typedef struct DATA_MSG_STRUCT{

                Uint8   m_uADCUsing;                //
                float32 m_ADCValue1;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC
                float32 m_ADCValue2;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC
                float32 m_ADCValue3;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC
                float32 m_ADCValue4;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC
                float32 m_ADCValue5;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC
                float32 m_ADCValue6;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC
                float32 m_ADCValue7;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC
                float32 m_ADCValue8;                // 0 ~ 4095(+30[VAR])                       ,       0.0073          Output: ADC

        }DATA_MSG_STRUCT_DEF;


enum dataType{
    Connect = 0,
    Din,
    Dout,
    Ch1_DAC,
    Ch2_DAC,
    Ch3_DAC,
    Ch4_DAC,
    Ch5_DAC,
    Ch6_DAC,
    Ch7_DAC,
    Ch8_DAC,
    PowerState,
    PulseState,
    OnTimeCh1,
    OnTimeCh2,
    OnTimeCh3,
    OnTimeCh4,
    OffTimeCh1,
    OffTimeCh2,
    OffTimeCh3,
    OffTimeCh4,
    PhaseTimeCh1,
    PhaseTimeCh2,
    PhaseTimeCh3,
    PhaseTimeCh4,
    AB_DeadBand,
    SingleOnTime,
    SingleOffTime,
    pulsewidth,
    periode
};

enum dataLength{
    u16 = 0,
    f32
};

enum msgType{
    i_type = 0,
    c_type
};
enum runningMode {
    wait = 0,
    ready,
    run
};


#endif /* DEFINE_H_ */
