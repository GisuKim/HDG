/*
 * main.c
 */

//=======================================================================
//  include Header
//=======================================================================

//#include "..\F28346_Include\DSP28x_Project.h"
#include "F28346_Include\DSP28x_Project.h"
#include "define.h"
#include "stdio.h"
#include "..\UTIL\UTIL_Queue.h"
#include "..\Driver\pwm_setting.h"
#include "..\Driver\DWIN_UI.h"
#include "..\Driver\SerialQueue.h"


//========================================================================
//	Define integer
//========================================================================
/*select debugging Mode*/
#define DEBUGGING			0
/*default Bool Type define*/
#define TRUE				1
#define FALSE				0
#define CLEAR				0
#define SET					1
/*Select Driving Mode Define*/
#define IDLE_MODE			0
#define RUNNING_MODE		1
/*Sensor Value Define*/
#define IMU1750				0
#define ISP1G3A				1
#define ISP1G3A_DATA_NUMBER 38	//ISP Data Number for 1 Frame
#define IMU1750_DATA_NUMBER	36	//IMU Data Number for 1 Frame
/*Data Buffer Size Define*/
#define BUFFER_NUMBER		5		//Buffer Value
#define BUFFER_A			0
#define BUFFER_B			1

#define RS422				0		//RS422 Interface Type define
#define BLUETOOTH			1		//Bluetooth Interface Type define
#define TR_10HZ				100
#define TR_100HZ			10

//#define PWMCH2_CA_ON        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1
//#define PWMCH2_CA_OFF        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1

//========================================================================
//	define function
//========================================================================

/*Defien Interrupt Service Routine*/
interrupt void scia_rx_isr(void);
interrupt void scib_rx_isr(void);
interrupt void scic_rx_isr(void);
interrupt void cpu_timer0_isr(void);
interrupt void xint1_isr(void);
interrupt void xint3_isr(void);
interrupt void xint4_isr(void);
/*Define Functions*/
void InitRDC_Gpio(void);
void PushFrame(void);
void MakeTxFrameB(void);
void MakeTxFrameA(void);
void SwitchInterfaceMux(Uint8 Interface);
void ConfigCommandMode(Uint8 Interface);
void ConfigDataLength(Uint8 CommInterface);
void RunningMode(void);
void BluetoothMode(void);
void BootMode(void);
void IdleMode(void);
Uint8 SensorCommCheck(Uint8 Time);
void TransferCommand(char status,char command,char mode);
Uint8 CommandParsing(char Data);
char TxCommParsing(Uint8 Data);
// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
interrupt void epwm1_isr(void);
interrupt void epwm2_isr(void);
interrupt void epwm3_isr(void);
void sendOnOffTime(char ch, Uint16 on, Uint16 Off, Uint16 phase);
void SendOnTime(char ch, Uint16 on);
void SendOffTime(char ch, Uint16 Off);
void SendPhaseTime(char ch, Uint16 phase);

//========================================================================
//	Debugging Variable
//========================================================================
Uint8	f_SensorReady=0;
Uint8	f_DetectSensorDebugging=0;

Uint8 boot=0;

//========================================================================
//	global Variables
//========================================================================
// Controll Type
Uint8	f_DetectSensorMode=0;			//Scan Sensor Mode Flag
Uint8	g_ubDetectSensorCNT=0;			//Sensor Detected Count
Uint8	g_ubNotDetectSensorCNT=0;		//Sensor Not Detected Count
Uint8	g_ubSensorType=1;				//Sensor Type
Uint8	f_CommStatus=0;					//Senstor Status
Uint8	g_bSamplingCount=0;
Uint8	f_BluetoothFrame=0;
Uint8	g_BufferBend=1;
Uint8	g_BufferAend=1;
//-----------------------------------------------------------------------
//SCIA ISR Header Detect
Uint8	g_HeaderBuff[5];				//Header Detect Buffer
Uint8	g_MakeFrameCNT=1;				//Count to Receive Sensor Data Byte by One Frame maked
Uint8	g_ubSensorDataBuffer[42];		//Sensor 1Frame Packet Buffer
Uint8	f_bDetectSensorHeader=0;		//Header Detector 1 is Header Detected
Uint8	f_bcompleteOneSensorFrame=0;	//End by Sensor 1Frame Collect
Uint8	g_BufferCount=0;				//Data Collect Count
//-----------------------------------------------------------------------
/*RDC Data Variable*/
Uint16	g_rdc_data=0;					//RDC Direct immediate value

//-----------------------------------------------------------------------
/*Ring Buffer Variable*/
#define RINGBUFFER_SIZE	500
Uint8	g_RingBuffer[RINGBUFFER_SIZE][40];
Uint16	g_PushPointer;
Uint16	g_PopPointer;
Uint16	g_BufferLength;
Uint8	f_BufferOVF;


//-----------------------------------------------------------------------
Uint8	g_TotalFrameA[10][40];			//Buffer A Transfer Data Temperature Variable
Uint8	g_TotalFrameB[10][40];			//Buffer B Transfer Data Temperature Variable
Uint8	g_BufferName=0;					//Buffer Switch Variable

Uint8	g_ubBufferSendCounter=0;		//Make Send Data Counter - Use MakeFrameA,B Function
Uint8	g_ubTotalData_Len;				//Total Send Data Length - USE MakeFrameA,B Function
Uint8	g_SendDataNumber=0;				//Using Make Frame - 1Frame Number of byte
Uint8	g_ubBufferNumber=0;				//Using Make Frame -

Uint8	f_DataTxEnd=0;					//Data Transfer Complect Status Flag
Uint8 	g_SubmitData[400];				//Transfer Data Buffer, Include Header And All Buffer
Uint8	f_DataTxFirst=0;				//Create Transfer Data ANd Change Buffer Complete
Uint8	f_BufferEnd=0;					//Complete Get from Temperature Buffer
Uint8	g_SubmitCnt=0;



Uint8	f_bdetectISP1G3Aheader=0;		//ISP1G3A Header Temperature Variable
Uint8	g_HeaderTimeoutCount=0;			//Header Detector Timeout Counter
Uint8	f_HeaderTimeOut=0;				//Header Detector Timeout Flag

//-----------------------------------------------------------------------
/*Control Command Communication*/
Uint8	f_bRecieveCommand=0;		//Recieved Command flag
char	g_ubAckCommandBuffer[5];
Uint8	g_ubCommandCount=0;
Uint8	g_ubMode=RS422;
Uint8	g_ubInterface=RS422;
Uint8	f_CommandReceived=CLEAR;
Uint8	f_DetectCommandHeader=CLEAR;
Uint16 	g_RDCDummyData=0;

Uint32	g_NumofScia=0;
Uint32	g_DetectHeader=0;
Uint32	g_IncrementErrorCnt=0;
Uint32	g_PushCnt=0;
Uint32	g_AcquisitionCnt=0;
Uint16  g_CheckISR=0;

char    g_status = 0;

Uint16  g_pulseWidth = 0;
Uint16  g_periode = 0;

char    g_Ready = 0;
Uint8   g_PulseOn_ch3 = 0;
Uint16 g_EPwm5CMPA = 0;
Uint16 g_EPwm6CMPA = 0;
Uint16 g_VoltageRef = 0;

Uint8 f_DataSendStart = 0;
Uint8 f_DataSendEnd = 0;

enum runningMode mode = wait;

// CPU PARAMETER and SPI BAUD  --------------------------------------------------------------------------------
#if (CPU_FRQ_300MHZ)
#define CLK_TMS320C28346CLK     300000000L
#endif
#if (CPU_FRQ_250MHZ)
#define TMS320C28346CLK     250000000L
#endif
#if (CPU_FRQ_200MHZ)
#define TMS320C28346CLK     200000000L
#endif

#define CLK_LSPCLK      (CLK_TMS320C28346CLK/4)

// *** Define CLK_LSPCLK in InitPeripheralClocks(); ***

#define SPI_MODE_ADC                1
#define SPI_MODE_DAC                0

#define SPI_BAUDRATE                10000L //(10Khz)
#define SPIBRR_VAL                  (CLK_LSPCLK/(SPI_BAUDRATE)-1) //(37500000/(500000)-1) = 74


// ADC and DAC code using SPI_A Master Mode --------------------------------------------------------------------
// ADC DAC 를 위해서 데이터 크기를 8비트로 설정, 모두 8비트 단위로 처리
// ADC 는 8비트 명령을 보내고 busy 신호가 low가 되면 8비트를 두번 읽어서 16비트 ADC 값을 획득함
// DAC 는 칩으로 매번 24비트로된 (채널정보, 제어정보, 16비트 데이터) 데이터가 전송되어야함, 8비트를 3회 전송하는 것으로 처리함.

// ADC 는 데이터(MCU로 입력)가 클럭의 하강에지 에서 변함
// DAC 는 데이터(MCU에서 출력) 가 클럭의 상승에지 에서 변함

// ADC DAC 사용시 클럭 polarity 설정을 가변 시켜주어야함.

void spi_init(char mode)
{
    SpiaRegs.SPICCR.all = 0x0000;

    if(mode == SPI_MODE_ADC)
    {
        SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;            // Data output on falling edge and input rising edge.
        SpiaRegs.SPICTL.all =0x0006;                    // Enable master mode, normal phase,                                                        // enable talk, and SPI int disabled.
        SpiaRegs.SPICTL.bit.CLK_PHASE = 1;              // 이것을 1으로 설정해야  ADC와 호환됨

        SpiaRegs.SPICCR.bit.SPICHAR = 15;               // **** 16-bit char bits ****
    }
    else
    {
        SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;            // Data output on rising edge and input falling edge.
        SpiaRegs.SPICTL.all =0x0006;                    // Enable master mode, normal phase,
                                                        // enable talk, and SPI int disabled.
        SpiaRegs.SPICCR.bit.SPICHAR = 7;               // **** 8-bit char bits ****

    }


    SpiaRegs.SPIBRR =0x7F;                              // for Minimum speed setting
    SpiaRegs.SPICCR.bit.SPISWRESET =1;                  // Relinquish SPI from Reset
    SpiaRegs.SPIPRI.bit.FREE = 1;                       // Set so breakpoints don't disturb xmission

    SpiaRegs.SPIFFTX.bit.TXFIFO      = 1;               // Re-Enable FIFO
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;
}


void spi_xmit_byte(Uint8 a)
{
    // Wait until tx fifo is empty
    while(SpiaRegs.SPIFFTX.bit.TXFFST !=0) { }
    SpiaRegs.SPITXBUF=a;
}

Uint16 spi_xmit_16bit(Uint16 a)
{
    Uint16 rdata;
    // Wait until tx fifo is empty
    while(SpiaRegs.SPIFFTX.bit.TXFFST !=0) { }
    SpiaRegs.SPITXBUF=a;

    // Wait until data is received
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    // Check against sent data
    rdata = SpiaRegs.SPIRXBUF;

    return rdata;
}


void spi_fifo_init()
{
// Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x2041;
    SpiaRegs.SPIFFCT.all=0x0;

}


#define     OUT_C_ON_H()           (GpioDataRegs.GPCSET.bit.GPIO67     = 1)
#define     OUT_C_ON_L()           (GpioDataRegs.GPCCLEAR.bit.GPIO67    = 1)

#define     OUT_C_OFF_H()           (GpioDataRegs.GPCSET.bit.GPIO69     = 1)
#define     OUT_C_OFF_L()           (GpioDataRegs.GPCCLEAR.bit.GPIO69     = 1)

#define     ADC_CS_HIGH()           (GpioDataRegs.GPBSET.bit.GPIO39     = 1)
#define     ADC_CS_LOW()            (GpioDataRegs.GPBCLEAR.bit.GPIO39   = 1)
#define     ADC_BUSY                (GpioDataRegs.GPADAT.bit.GPIO20)

#define     ADC_NO_POWER_DOWN_MODE_EX_CLK  3
#define     ADC_POWER_DOWN_                0
#define     ADC_INTER_CLK_                 1

#define     ADC_SINGLE_MODE         1
#define     ADC_DIFF_MODE           0

#define     ADC_CH0_SINGLE_MODE     1
#define     ADC_CH1_SINGLE_MODE     5
#define     ADC_CH2_SINGLE_MODE     2
#define     ADC_CH3_SINGLE_MODE     6

int16 g_ADC_Result_Ch0 =0;
int16 g_ADC_Result_Ch1 =0;
int16 g_ADC_Result_Ch2 =0;
int16 g_ADC_Result_Ch3 =0;

// ADS8343는 ADC 결과값이 2의 보수 포멧이라고 데이터 시트에 표기 되어있음, 따라서 아래의 함수 반환형은 INT16임
int16 read_ADS8343(Uint8 channel, Uint8 single_diff, Uint8 PD_mode)
{
    Uint8 controlbyte =0;
    Uint16 Shifted_controlbyte =0;
    Uint16 ADC_16bit =0;

    Uint8 StartBit_position = 7;


    int16 result =0;


    // /CS pin low
    ADC_CS_LOW();


    // send Channel and mode using SPI send (8bit)
    controlbyte = (Uint8)((1 << StartBit_position) | (channel<<4) | (single_diff<<2) | (PD_mode<<0));
    Shifted_controlbyte =  ((Uint16)controlbyte)<<1;


    spi_xmit_16bit(Shifted_controlbyte); //DELAY_US(10);


    // wait for busy signal to fall (주의 무한 루프에 빠질수 있음, 만약 busy가 low로 안떨어지면 , 그래서 타임 아웃 처리하는 코드가 추후에 필요함)
    while(ADC_BUSY == 0);   // wait for busy pin  to == 0


    ADC_16bit = spi_xmit_16bit(0x00);


    // /CS pin high (release chip select pin)
    ADC_CS_HIGH();


    // 결과 계산하고  리턴한다.
    result =  (int) ADC_16bit;
    return  result;

}
//--------------------------------------------------------------------------------------------------------------------------


#define     DAC_SYNC_HIGH()         (GpioDataRegs.GPASET.bit.GPIO30     = 1)
#define     DAC_SYNC_LOWH()         (GpioDataRegs.GPACLEAR.bit.GPIO30   = 1)
#define     POWER_LED_ON()          (GpioDataRegs.GPCSET.bit.GPIO80     = 1)
#define     POWER_LED_OFF()         (GpioDataRegs.GPCCLEAR.bit.GPIO80   = 1)
#define     PULSE_LED_ON()          (GpioDataRegs.GPBSET.bit.GPIO46     = 1)
#define     PULSE_LED_OFF()         (GpioDataRegs.GPBCLEAR.bit.GPIO46   = 1)
#define     POWER_SW                (GpioDataRegs.GPBDAT.bit.GPIO46)
#define     EMERGENCY_SW            (GpioDataRegs.GPCDAT.bit.GPIO81)
#define     PULSE_SW                (GpioDataRegs.GPBDAT.bit.GPIO47)
#define     OV_STATE                (GpioDataRegs.GPADAT.bit.GPIO31)
#define     OC_STATE                (GpioDataRegs.GPADAT.bit.GPIO11)
#define     OT_STATE                (GpioDataRegs.GPCDAT.bit.GPIO83)
#define     P_PROTECT_STATE         (GpioDataRegs.GPCDAT.bit.GPIO82)

Uint16 test_DAC_ch =0;
Uint16 test_DAC_val =0;

void write_AD5624(Uint8 cmd, Uint8 addr, Uint16 val)
{
    Uint32 data;
//    Uint8 msg[3];
    Uint16 Shifted_controlbyte =0;
    Uint16 Shifted_controlbyte1 =0;

    data = (0<<22) | ((Uint32)cmd << 19) | ((Uint32)addr << 16 ) | (val);

    Shifted_controlbyte = (data >> 8) & 0x0000ffff;
    Shifted_controlbyte1 = (data << 8) & 0x0000ffff;
//    msg[0] = (data >> 16) & 0x0000ffff;
//    msg[1] = (data >> 8) & 0x000000ff;
//    msg[2] = data & 0x000000ff;

    DAC_SYNC_LOWH();
//    spi_xmit_byte(0);

    spi_xmit_16bit(Shifted_controlbyte);

    spi_xmit_16bit(Shifted_controlbyte1);
//    spi_xmit_byte(msg[0] << 8);
//    spi_xmit_byte(msg[1] << 8);
//    spi_xmit_byte(msg[2] << 8);
    DAC_SYNC_HIGH();

}


typedef struct
{
    int16 ADC_Value_Min[4];
    int16 ADC_Value_Max[4];
    float32 Physical_Value_Min[4];
    float32 Physical_Value_Max[4];
    float32 Scale[4];
    float32 Offset[4];
    int16 ADC_RAW[4];
    float32 convValue[4];
    void (*Conversion)();      // Pointer to the update function
    void (*Calc_Param)();      // Pointer to the update function
}tADC_Object;


void Conv_ADC_to_Physical(tADC_Object *p){

    p->convValue[0] = (p->Scale[0] * (float32)p->ADC_RAW[0] )  + p->Offset[0];
    p->convValue[1] = (p->Scale[1] * (float32)p->ADC_RAW[1] )  + p->Offset[1];
    p->convValue[2] = (p->Scale[2] * (float32)p->ADC_RAW[2] )  + p->Offset[2];
    p->convValue[3] = (p->Scale[3] * (float32)p->ADC_RAW[3] )  + p->Offset[3];

}

void Calc_Scale_Offset(tADC_Object *p){

    p->Scale[0] = ( p->Physical_Value_Max[0] - p->Physical_Value_Min[0] ) / (float32)( (float32)p->ADC_Value_Max[0] - (float32)p->ADC_Value_Min[0] );
    p->Offset[0] = p->Physical_Value_Max[0] - (p->Scale[0] * (float32)p->ADC_Value_Max[0]);

    p->Scale[1] = ( p->Physical_Value_Max[1] - p->Physical_Value_Min[1]) / (float32)( (float32)p->ADC_Value_Max[1] - (float32)p->ADC_Value_Min[1] );
    p->Offset[1] = p->Physical_Value_Max[1] - (p->Scale[1] * (float32)p->ADC_Value_Max[1]);

}


#define ADC_DATA_DEFAULTS_VALUE { \
    {0,0,-32768,-32768},\
    {32767,32767,32767,32767},\
    {0.0, 0.0, 0.0, 0.0}, \
    {10000.0, 10000.0, 10000.0, 10000.0}, \
    {0.0, 0.0, 0.0, 0.0}, \
    {0.0, 0.0, 0.0, 0.0}, \
    {0, 0, 0, 0}, \
    {0.0, 0.0, 0.0, 0.0}, \
    (void (*)(Uint32))Conv_ADC_to_Physical, \
    (void (*)(Uint32))Calc_Scale_Offset}


tADC_Object g_my_ADC = ADC_DATA_DEFAULTS_VALUE;

Uint16 UART_TX_ADC0_value = 0;
Uint16 UART_TX_ADC1_value = 0;

Uint8 g_PacketData[8] = {0,};

MSG_STRUCT_DEF g_stMsgIn;                                       /* Struct of Sensor Data */
MSG_STRUCT_DEF g_stMsgOut;
SETTING_MSG_STRUCT_DEF g_stSettingDataOut;
void SendOnTime(char ch, Uint16 on)
{
    char on_adreess_h = 0;
    char on_adreess_l = 0;
//    Uint8 PacketData[8] = {0,};
    switch(ch)
    {
    case 0x01:
        on_adreess_h = 0x01;
        on_adreess_l = 0x80;
        break;
    case 0x02:
        on_adreess_h = 0x02;
        on_adreess_l = 0x40;
        break;
    case 0x03:
        on_adreess_h = 0x04;
        on_adreess_l = 0x20;
        break;
    case 0x04:
        on_adreess_h = 0x04;
        on_adreess_l = 0x80;
        break;
    }

//    scia_xmit(0x5a);
//    scia_xmit(0xa5);
//    scia_xmit(0x05);
//    scia_xmit(0x82);
//    scia_xmit(on_adreess_h);
//    scia_xmit(on_adreess_l);
//    scia_xmit(on>>8);
//    scia_xmit(0x00FF & on);
//
//    PacketData[0] = 0x5a;
//    PacketData[1] = 0xa5;
//    PacketData[2] = 0x05;
//    PacketData[3] = 0x82;
//    PacketData[4] = on_adreess_h;
//    PacketData[5] = on_adreess_l;
//    PacketData[6] = on>>8;
//    PacketData[7] = 0x00FF & on;
//    g_stSensorMsgIn.m_uTestVal = 0x0001;

//    SetData(&g_stMsgIn);
}

void SendOffTime(char ch, Uint16 Off)
{
    char off_adreess_h = 0;
    char off_adreess_l = 0;
    Uint8 PacketData[8] = {0,};
    switch(ch)
    {
    case 0x01:
        off_adreess_h = 0x02;
        off_adreess_l = 0x00;
        break;
    case 0x02:
        off_adreess_h = 0x02;
        off_adreess_l = 0x60;
        break;
    case 0x03:
        off_adreess_h = 0x04;
        off_adreess_l = 0x40;
        break;
    case 0x04:
        off_adreess_h = 0x05;
        off_adreess_l = 0x00;
        break;

    }

    PacketData[0] = 0x5a;
    PacketData[1] = 0xa5;
    PacketData[2] = 0x05;
    PacketData[3] = 0x82;
    PacketData[4] = off_adreess_h;
    PacketData[5] = off_adreess_l;
    PacketData[6] = Off>>8;
    PacketData[7] = 0x00FF & Off;
//    scia_xmit(0x5a);
//    scia_xmit(0xa5);
//    scia_xmit(0x05);
//    scia_xmit(0x82);
//    scia_xmit(off_adreess_h);
//    scia_xmit(off_adreess_l);
//    scia_xmit(Off>>8);
//    scia_xmit(0x00FF & Off);
//
//    SetData(PacketData);

}

void SendPhaseTime(char ch, Uint16 phase)
{
    char ph_adreess_h = 0;
    char ph_adreess_l = 0;
    Uint8 PacketData[8] = {0,};
    switch(ch)
    {
    case 0x01:
        ph_adreess_h = 0x02;
        ph_adreess_l = 0x20;
        break;
    case 0x02:
        ph_adreess_h = 0x02;
        ph_adreess_l = 0x80;
        break;
    case 0x03:
        ph_adreess_h = 0x04;
        ph_adreess_l = 0x60;
        break;
    case 0x04:
        ph_adreess_h = 0x05;
        ph_adreess_l = 0x20;
        break;

    }
//    scia_xmit(0x5a);
//    scia_xmit(0xa5);
//    scia_xmit(0x05);
//    scia_xmit(0x82);
//    scia_xmit(ph_adreess_h);
//    scia_xmit(ph_adreess_l);
//    scia_xmit(phase>>8);
//    scia_xmit(0x00FF & phase);

    PacketData[0] = 0x5a;
    PacketData[1] = 0xa5;
    PacketData[2] = 0x05;
    PacketData[3] = 0x82;
    PacketData[4] = ph_adreess_h;
    PacketData[5] = ph_adreess_l;
    PacketData[6] = phase>>8;
    PacketData[7] = 0x00FF & phase;

//    SetData(PacketData);
}
void sendOnOffTime(char ch, Uint16 on, Uint16 Off, Uint16 phase)
{
    char on_adreess_h = 0;
    char on_adreess_l = 0;
    char off_adreess_h = 0;
    char off_adreess_l = 0;
    char ph_adreess_h = 0;
    char ph_adreess_l = 0;
    Uint8 PacketData[8] = {0,};

    switch(ch)
    {
    case 0x01:
        on_adreess_h = 0x01;
        on_adreess_l = 0x80;
        off_adreess_h = 0x02;
        off_adreess_l = 0x00;
        ph_adreess_h = 0x02;
        ph_adreess_l = 0x20;
        break;
    case 0x02:
        on_adreess_h = 0x02;
        on_adreess_l = 0x40;
        off_adreess_h = 0x02;
        off_adreess_l = 0x60;
        ph_adreess_h = 0x02;
        ph_adreess_l = 0x80;
        break;
    case 0x03:
        on_adreess_h = 0x04;
        on_adreess_l = 0x20;
        off_adreess_h = 0x04;
        off_adreess_l = 0x40;
        ph_adreess_h = 0x04;
        ph_adreess_l = 0x60;
        break;
    case 0x04:
        on_adreess_h = 0x04;
        on_adreess_l = 0x80;
        off_adreess_h = 0x05;
        off_adreess_l = 0x00;
        ph_adreess_h = 0x05;
        ph_adreess_l = 0x20;
        break;

    }


//    scia_xmit(0x5a);
//    scia_xmit(0xa5);
//    scia_xmit(0x05);
//    scia_xmit(0x82);
//    scia_xmit(on_adreess_h);
//    scia_xmit(on_adreess_l);
//    scia_xmit(on>>8);
//    scia_xmit(0x00FF & on);
//
    PacketData[0] = 0x5a;
    PacketData[1] = 0xa5;
    PacketData[2] = 0x05;
    PacketData[3] = 0x82;
    PacketData[4] = on_adreess_h;
    PacketData[5] = on_adreess_l;
    PacketData[6] = on>>8;
    PacketData[7] = 0x00FF & on;

//    SetData(PacketData);

//    scia_xmit(0x5a);
//    scia_xmit(0xa5);
//    scia_xmit(0x05);
//    scia_xmit(0x82);
//    scia_xmit(off_adreess_h);
//    scia_xmit(off_adreess_l);
//    scia_xmit(Off>>8);
//    scia_xmit(0x00FF & Off);

    PacketData[0] = 0x5a;
    PacketData[1] = 0xa5;
    PacketData[2] = 0x05;
    PacketData[3] = 0x82;
    PacketData[4] = off_adreess_h;
    PacketData[5] = off_adreess_l;
    PacketData[6] = Off>>8;
    PacketData[7] = 0x00FF & Off;

//    SetData(PacketData);

//    scia_xmit(0x5a);
//    scia_xmit(0xa5);
//    scia_xmit(0x05);
//    scia_xmit(0x82);
//    scia_xmit(ph_adreess_h);
//    scia_xmit(ph_adreess_l);
//    scia_xmit(phase>>8);
//    scia_xmit(0x00FF & phase);

    PacketData[0] = 0x5a;
    PacketData[1] = 0xa5;
    PacketData[2] = 0x05;
    PacketData[3] = 0x82;
    PacketData[4] = ph_adreess_h;
    PacketData[5] = ph_adreess_l;
    PacketData[6] = phase>>8;
    PacketData[7] = 0x00FF & phase;

//    SetData(PacketData);
}


//========================================================================
//	Main Routine
//========================================================================
void main(void)
    {

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2834x_SysCtrl.c file.

  	InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2834x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.

    InitGpio();				 //Skipped for this example
    InitSciGpio();				//all sci gpio
    InitOutput_Gpio();
    InitLED_Gpio();				//LED GPIO Initializing
    InitInput_Gpio();
    InitEPwmGpio();

//    SetPWMCH2Enable(0);


//  20181003 for ADC DAC interface ----------------------------------------------------
    InitADS8343_Gpio();     // Init ADC CS, BUSY GPIO
    InitAD5664_Gpio();      // Init DAC /SYNC GPIO
    InitSpiaGpio();         // Setup only the GP I/O only for SPI-A functionality

    spi_fifo_init();   // Initialize the Spi FIFO
//-------------------------------------------------------------------------------------

    //TEST ADC CS PIN
//    while(1){
//        // /CS pin low
//        ADC_CS_LOW();
//
//        DELAY_US(10);
//
//        // /CS pin high (release chip select pin)
//        ADC_CS_HIGH();
//        DELAY_US(10);
//    }



    //----test ADC and DAC -----
       spi_init(SPI_MODE_ADC);        // init SPI

//       spi_init(SPI_MODE_DAC);        // init SPI for DAC
// For this example, only init the pins for the SCI-A port.
// This function is found in the DSP2834x_Sci.c file.

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

   InitSci();	//all sci Initialize

	EALLOW;  // This is needed to write to EALLOW protected registers

	PieVectTable.TINT0 = &cpu_timer0_isr;

    PieVectTable.SCIRXINTA =&scia_rx_isr;
    PieVectTable.SCIRXINTB =&scib_rx_isr;

    PieVectTable.XINT1 = &xint1_isr;
    PieVectTable.XINT3 = &xint3_isr;
    PieVectTable.XINT4 = &xint4_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;

	EDIS;    // This is needed to disable write to EALLOW protected registers

	InitCpuTimers();

//	InitXintf();

   // To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
   // of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2834x_CpuTimers.h), the
   // below settings must also be updated.
#if (CPU_FRQ_300MHZ)
	ConfigCpuTimer(&CpuTimer0, 300, 100000);

#endif
	StartCpuTimer0();
	CpuTimer0Regs.TCR.bit.TIE = 1; // Use write-only instruction to set TSS bit = 0
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block

	//	 Enable CPU INT3 which is connected to EPWM1-6 INT:
	IER |= M_INT8 | M_INT9 | M_INT3 | M_INT1 | M_INT12;

//=======================================================================
//  Interrupt Enable
//=======================================================================
//  Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;	//scirxinta
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;	//scirxintb

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;

    PieCtrlRegs.PIEIER12.bit.INTx1 = 1;          // Enable PIE Group 1 INT4
    PieCtrlRegs.PIEIER12.bit.INTx2 = 1;          // Enable PIE Group 1 INT4
// Enable global Interrupts and higher priority real-time debug events:


   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 5. User specific code:

//****************************************************************************************
// A8M Main roof
////****************************************************************************************

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

   InitEPwm1();
   InitEPwm2();
   InitEPwm4();
   InitEPwm6();
   InitEPwm8();
   InitEPwm3();
   InitEPwm5();
   InitEPwm7();
   InitEPwm9();

   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

   EDIS;

//   EALLOW;
//   EPwm3Regs.TZFRC.bit.OST = 1;
//   EPwm3Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//   EPwm3Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//
//   EPwm5Regs.TZFRC.bit.OST = 1;
//   EPwm5Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//   EPwm5Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//
//   EPwm7Regs.TZFRC.bit.OST = 1;
//   EPwm7Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//   EPwm7Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//
//   EPwm9Regs.TZFRC.bit.OST = 1;
//   EPwm9Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//   EPwm9Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//   EDIS;



    // GPIO27 is XINT1
    EALLOW;
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 4;
    GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL = 46;   // Xint1 is GPIO27
    GpioIntRegs.GPIOXINT4SEL.bit.GPIOSEL = 47;   // Xint1 is GPIO27
    EDIS;

    // Configure XINT1
    XIntruptRegs.XINT1CR.bit.POLARITY = 3;      // Rising edge interrupt
    XIntruptRegs.XINT3CR.bit.POLARITY = 3;      // Rising edge interrupt
    XIntruptRegs.XINT4CR.bit.POLARITY = 3;      // Rising edge interrupt

    // Enable XINT1
    XIntruptRegs.XINT3CR.bit.ENABLE = 0;        // Enable Xint1
    XIntruptRegs.XINT3CR.bit.ENABLE = 1;        // Enable Xint1
    XIntruptRegs.XINT4CR.bit.ENABLE = 1;        // Enable Xint1


   // Only One call
   g_my_ADC.Calc_Param(&g_my_ADC);

   Uint16 TestCnt = 0;
   Uint16 RetunValueOn = 0;
   Uint16 RetunValueOff = 0;
   Uint16 RetunValuePhase = 0;
   Uint8 PacketData[60] = {0,};

   POWER_LED_ON();
   PULSE_LED_OFF();

   DAC_SYNC_HIGH();
   test_DAC_val = 43690;

//   SetPWMCH5AEnable(0);
//   SetPWMCH6AEnable(0);
/*Main While Roof is */

   Uint16 clkdiv_val =1;
   float clkdvider = 1.0;
   if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
   {
       clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
   }
   if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
   {
       clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
   }

   g_periode = (Uint16)(1/((EPwm1Regs.TBPRD * (3.333333 * clkdvider)) * 0.000001))/2;

   g_pulseWidth = (Uint16)((float)EPwm2Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

   g_Ready=1;
   OUT_C_OFF_H();
   DELAY_US(100);
   OUT_C_ON_L();
   DELAY_US(100);
   f_DataSendEnd = 1;


   Uint16 uTest_Write_Buff[QUEUE_ARRAY_SIZE];
   Uint16 uData_Size = 0;
   Uint16 i;
	while(1)
	{


	    if(GetInputMsgCnt() > 0)
	    {
	        GetInputMsg(&g_stMsgOut);

	        ReceiveDataParser(g_stMsgOut);

	    }
	    else
	    {

	        if(GetOutputMsgCnt() > 0)
	        {
	            GetOutputMsg(&g_stMsgOut);

                scia_xmit(0x5a);
                scia_xmit(0xa5);
                scia_xmit(g_stMsgOut.m_uMsgType);
                scia_xmit(g_stMsgOut.m_uDataType);
                scia_xmit(0x00FF & (g_stMsgOut.m_ulData.w[1] >> 8));
                scia_xmit(0x00FF & g_stMsgOut.m_ulData.w[1]);
                scia_xmit(0x00FF & (g_stMsgOut.m_ulData.w[0] >> 8));
                scia_xmit(0x00FF & g_stMsgOut.m_ulData.w[0]);
                scia_xmit(0xfe);
	        }
	        else
	        {
	            if(GetOutputSettingDataCnt() > 0)
	            {
	                GetOutputSettingData(&g_stSettingDataOut);
	                scia_xmit(0x5a);
	                scia_xmit(0xa5);
	                scia_xmit(0xf0);
	                uData_Size = sizeof(g_stSettingDataOut);
	                memcpy(&uTest_Write_Buff, &g_stSettingDataOut, uData_Size);
	                for(i=0 ; i < uData_Size ; i++)
	                {
	                    scia_xmit(uTest_Write_Buff[i] & 0x00ff);
	                    scia_xmit((uTest_Write_Buff[i] >> 8) & 0x00ff);
	                }
	                scia_xmit(0xfe);


	            }

	        }


	    }
//	        if(g_stDataMessageQueue.m_uCount > 0)
//	        {
//	            GetData(&g_stMsgOut);
//
//	            scia_xmit(0x5a);
//	            scia_xmit(0xa5);
//	            scia_xmit(g_stMsgOut.m_uMsgType);
//	            scia_xmit(g_stMsgOut.m_uMsgLen);
//	            scia_xmit(g_stMsgOut.m_uDataType);
//
//	            switch(g_stMsgOut.m_uMsgLen)
//	            {
//	            case 0x00:
//	                scia_xmit(g_stMsgOut.m_uData);
//	                break;
//	            case 0x01:
//	                scia_xmit(0x00FF & (g_stMsgOut.m_uiData >> 8));
//	                scia_xmit(0x00FF & g_stMsgOut.m_uiData);
//	                break;
//	            case 0x02:
//	                memcpy(&uTest_Write_Buff, &g_stMsgOut.m_ufData, 2);
//	                scia_xmit(0x00FF & (uTest_Write_Buff[1] >> 8));
//	                scia_xmit(0x00FF & uTest_Write_Buff[1]);
//	                scia_xmit(0x00FF & (uTest_Write_Buff[0] >> 8));
//	                scia_xmit(0x00FF & uTest_Write_Buff[0]);
//	                break;
//	            }
//	            scia_xmit(0xfe);
//
//
//	        }
//	    }



	    if(f_DataSendStart == 1)
	    {
	        //**********************************************
	        // VOLTAGE
	        //**********************************************
//	        scia_xmit(0x5a);
//	        scia_xmit(0xa5);
//	        scia_xmit(0x05);
//	        scia_xmit(0x82);
//	        scia_xmit(0x00);
//	        scia_xmit(0x60);
//	        scia_xmit(UART_TX_ADC0_value>>8);
//	        scia_xmit(0x00FF & UART_TX_ADC0_value);

//            g_stMsgIn.m_uTeatval1 = 0x5a;
//	        g_stMsgIn.m_uTeatval2 = 0xa5;
//	        g_stMsgIn.m_uTeatval3 = 0x05;
//	        g_stMsgIn.m_uTeatval4 = 0x82;
//	        g_stMsgIn.m_uTeatval5 = 0x00;
//	        g_stMsgIn.m_uTeatval6 = 0x60;
//	        g_stMsgIn.m_uTeatval7 = UART_TX_ADC0_value>>8;
//	        g_stMsgIn.m_uTeatval8 = 0x00FF & UART_TX_ADC0_value;
//	        g_stMsgIn.m_uMsgType = i_type;
//	        g_stMsgIn.m_uMsgLen = u8;
//	        g_stMsgIn.m_uDataType = Din;
//
//	        SetData(&g_stMsgIn);


	        //**********************************************
	        // CURRENT
	        //**********************************************
//	        scia_xmit(0x5a);
//	        scia_xmit(0xa5);
//	        scia_xmit(0x05);
//	        scia_xmit(0x82);
//	        scia_xmit(0x00);
//	        scia_xmit(0x80);
//	        scia_xmit(UART_TX_ADC1_value>>8);
//	        scia_xmit(0x00FF & UART_TX_ADC1_value);

//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x00;
//            PacketData[5] = 0x80;
//            PacketData[6] = UART_TX_ADC1_value>>8;
//            PacketData[7] = 0x00FF & UART_TX_ADC1_value;
//
//            SetData(PacketData);
//
//	        //**********************************************
//	        // Voltage REF
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x00);
////	        scia_xmit(0x00);
////	        scia_xmit(g_VoltageRef>>8);
////	        scia_xmit(0x00FF & g_VoltageRef);
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x00;
//            PacketData[5] = 0x00;
//            PacketData[6] = g_VoltageRef>>8;
//            PacketData[7] = 0x00FF & g_VoltageRef;
//
//            SetData(PacketData);
//
//	        //**********************************************
//	        // Pulse Width
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x00);
////	        scia_xmit(0x20);
////	        scia_xmit(g_pulseWidth>>8);
////	        scia_xmit(0x00FF & g_pulseWidth);
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x00;
//            PacketData[5] = 0x20;
//            PacketData[6] = g_pulseWidth>>8;
//            PacketData[7] = 0x00FF & g_pulseWidth;
//
//            SetData(PacketData);
//
//
//	        //**********************************************
//	        // periode
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x00);
////	        scia_xmit(0x40);
////	        scia_xmit(g_periode>>8);
////	        scia_xmit(0x00FF & g_periode);
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x00;
//            PacketData[5] = 0x40;
//            PacketData[6] = g_periode>>8;
//            PacketData[7] = 0x00FF & g_periode;
//
//            SetData(PacketData);
//
//
//	        //**********************************************
//	        // periode
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x01);
////	        scia_xmit(0x20);
////	        scia_xmit(g_periode>>8);
////	        scia_xmit(0x00FF & g_periode);
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x01;
//            PacketData[5] = 0x20;
//            PacketData[6] = g_periode>>8;
//            PacketData[7] = 0x00FF & g_periode;
//
//            SetData(PacketData);
//
//	        //**********************************************
//	        // OVER VOLTAGE
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x03);
////	        scia_xmit(0x00);
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x03;
//            PacketData[5] = 0x00;
//
//	        if(OV_STATE)
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x00);
//	            PacketData[6] = 0x00;
//	            PacketData[7] = 0x00;
//	        }
//	        else
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x01);
//	            PacketData[6] = 0x00;
//	            PacketData[7] = 0x01;
//	        }
//
//
//            SetData(PacketData);
//	        //**********************************************
//	        // OVER CURRENT
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x03);
////	        scia_xmit(0x20);
////
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x03;
//            PacketData[5] = 0x20;
//
//	        if(OC_STATE)
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x00);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x00;
//	        }
//	        else
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x01);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x01;
//	        }
//
//            SetData(PacketData);
//
//	        //**********************************************
//	        // OVER Time
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x03);
////	        scia_xmit(0x40);
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x03;
//            PacketData[5] = 0x40;
//
//	        if(OT_STATE)
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x00);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x00;
//	        }
//	        else
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x01);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x01;
//	        }
//
//            SetData(PacketData);
//	        //**********************************************
//	        // P-Protection
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x03);
////	        scia_xmit(0x60);
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x03;
//            PacketData[5] = 0x60;
//
//	        if(P_PROTECT_STATE)
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x00);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x00;
//	        }
//	        else
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x01);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x01;
//	        }
//
//            SetData(PacketData);
//	        //**********************************************
//	        // P-Protection
//	        //**********************************************
////	        scia_xmit(0x5a);
////	        scia_xmit(0xa5);
////	        scia_xmit(0x05);
////	        scia_xmit(0x82);
////	        scia_xmit(0x03);
////	        scia_xmit(0x80);
//
//
//            PacketData[0] = 0x5a;
//            PacketData[1] = 0xa5;
//            PacketData[2] = 0x05;
//            PacketData[3] = 0x82;
//            PacketData[4] = 0x03;
//            PacketData[5] = 0x80;
//
//	        if(P_PROTECT_STATE)
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x00);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x00;
//	        }
//	        else
//	        {
////	            scia_xmit(0x00);
////	            scia_xmit(0x01);
//                PacketData[6] = 0x00;
//                PacketData[7] = 0x01;
//	        }
//
//	        EALLOW;
//
//	        RetunValueOn = (Uint16)((float)EPwm3Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
//	        RetunValueOff = (Uint16)((float)(EPwm2Regs.CMPB - EPwm2Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
//	        RetunValuePhase = (Uint16)((float)EPwm2Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
//	        EDIS;
//
//	        sendOnOffTime(0x01, RetunValueOn,RetunValueOff,RetunValuePhase);
//
//	        EALLOW;
//
//	        RetunValueOn = (Uint16)((float)EPwm5Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
//	        RetunValueOff = (Uint16)((float)(EPwm4Regs.CMPB - EPwm4Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
//	        RetunValuePhase = (Uint16)((float)EPwm4Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
//
//	        EDIS;
//
//	        sendOnOffTime(0x02, RetunValueOn,RetunValueOff,RetunValuePhase);
//
//
//	        EALLOW;
//
//	           RetunValueOn = (Uint16)((float)EPwm7Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
//	           RetunValueOff = (Uint16)((float)(EPwm6Regs.CMPB - EPwm6Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
//	           RetunValuePhase = (Uint16)((float)EPwm6Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
//	           EDIS;
//
//	       sendOnOffTime(0x03, RetunValueOn,RetunValueOff,RetunValuePhase);
//
//
//	       EALLOW;
//
//	       RetunValueOn = (Uint16)((float)EPwm9Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
//	       RetunValueOff = (Uint16)((float)(EPwm8Regs.CMPB - EPwm8Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
//	       RetunValuePhase = (Uint16)((float)EPwm8Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
//	       EDIS;
//
//	       sendOnOffTime(0x04, RetunValueOn,RetunValueOff,RetunValuePhase);

	       f_DataSendEnd = 1;
	       f_DataSendStart = 0;
	    }


//	    if(OV_STATE | OC_STATE | OT_STATE | P_PROTECT_STATE)
//	    {
//	        OUT_C_ON_L();
//	        OUT_C_OFF_H();
//	    }

//        if( EMERGENCY_SW == 0)
//        {
//            OUT_C_ON_L();
//            DELAY_US(100);
////                SetPWMCH6AEnable(0);
////                SetPWMCH5AEnable(0);
//                POWER_LED_OFF();
//                PULSE_LED_OFF();
//                OUT_C_OFF_H();
//                DELAY_US(100);
//        }
//        else
//        {
//            if(g_Ready)
//            {
//                switch(mode)
//                 {
//                 case wait:
//                     if(POWER_SW == 0 && PULSE_SW == 1)
//                     {
//                         mode=ready;
//
////                         SetPWMCH6AEnable(1);
//                         POWER_LED_ON();
//                         OUT_C_OFF_L();
//                         DELAY_US(100);
//                         OUT_C_ON_H();
//                         DELAY_US(100);
//                     }
//                     break;
//                 case ready:
//
//                     if(POWER_SW == 0 && PULSE_SW == 0)
//                     {
//                         mode=run;
////                         SetPWMCH5AEnable(1);
//                         PULSE_LED_ON();
//                     }
//
//                     if(POWER_SW == 1)
//                     {
//                         mode=wait;
////                         SetPWMCH6AEnable(0);
//                         POWER_LED_OFF();
//                         OUT_C_OFF_H();
//                         DELAY_US(100);
//                         OUT_C_ON_L();
//                         DELAY_US(100);
//                     }
//
//                     break;
//                 case run:
//                     if(POWER_SW == 1 && PULSE_SW == 0)
//                     {
//                         mode=wait;
////                         SetPWMCH6AEnable(0);
////                         SetPWMCH5AEnable(0);
//                         POWER_LED_OFF();
//                         PULSE_LED_OFF();
//                         OUT_C_OFF_H();
//                         DELAY_US(100);
//                         OUT_C_ON_L();
//                         DELAY_US(100);
//                     }
//
//                     if(POWER_SW == 0 && PULSE_SW == 1)
//                     {
//                         mode=ready;
////                         SetPWMCH5AEnable(0);
//                         PULSE_LED_OFF();
//                     }
//
//
//                     break;
//                 default:
//                     break;
//                 }
//
//            }
//
//
//        }





//	    GpioDataRegs.GPCSET.bit.GPIO66 = 1;   // Load output latch
//	    DELAY_US(500000);
//	    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;   // Load output latch
//
//	    TestCnt++;

//        UART_TX_ADC0_value = 1000;
//           g_ADC_Result_Ch1 = read_ADS8343(ADC_CH1_SINGLE_MODE, ADC_SINGLE_MODE, ADC_POWER_DOWN_);
//           g_ADC_Result_Ch2 = read_ADS8343(ADC_CH2_SINGLE_MODE, ADC_SINGLE_MODE, ADC_POWER_DOWN_);
//           g_ADC_Result_Ch3 = read_ADS8343(ADC_CH3_SINGLE_MODE, ADC_SINGLE_MODE, ADC_POWER_DOWN_);



        // TEST DAC
//        spi_init(SPI_MODE_DAC);        // init SPI for DAC
//        write_AD5624(0x00, test_DAC_ch, test_DAC_val);  // cmd = 0x00 (Write to input register n)
        //test_DAC_val++;


////		scib_xmit(0xa3);
////		scic_xmit(0x3a);
//        scib_xmit(0x5a);
//        scib_xmit(0xa5);
//        scib_xmit(0x05);
//        scib_xmit(0x82);
//
//        scib_xmit(0x00);
//        scib_xmit(0x60);
//
////        scib_xmit(UART_TX_ADC0_value>>8);
////        scib_xmit(0x00FF & UART_TX_ADC0_value);
//
//        scib_xmit(0x00);
//        scib_xmit(0x01);
//
//        DELAY_US(5000);
//
//        scia_xmit(0x5a);
//        scia_xmit(0xa5);
//        scia_xmit(0x05);
//        scia_xmit(0x82);
//        scia_xmit(0x00);
//        scia_xmit(0x60);
////        scia_xmit(TestCnt>>8);
////        scia_xmit(0x00ff & TestCnt);
//        scia_xmit(UART_TX_ADC0_value>>8);
//        scia_xmit(0x00FF & UART_TX_ADC0_value);
//
//        DELAY_US(5000);
//
//        scia_xmit(0x5a);
//        scia_xmit(0xa5);
//        scia_xmit(0x05);
//        scia_xmit(0x82);
//        scia_xmit(0x00);
//        scia_xmit(0x80);
//        scia_xmit(UART_TX_ADC1_value>>8);
//        scia_xmit(0x00FF & UART_TX_ADC1_value);
//
//        DELAY_US(5000);

	}	//Main roof end
}

interrupt void xint1_isr(void)
{
    XIntruptRegs.XINT1CR.bit.ENABLE = 0;        // Enable Xint1
    g_PulseOn_ch3 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void xint3_isr(void)
{


//    if(POWER_SW)
//    {
//        mode = ready;
////        OUT_C_ON_L();
////        OUT_C_OFF_H();
//        POWER_LED_OFF();
//        scia_xmit(0x5a);
//        scia_xmit(0xa5);
//        scia_xmit(0x05);
//        scia_xmit(0x82);
//        scia_xmit(0x03);
//        scia_xmit(0x00);
//        scia_xmit(0x00);
//        scia_xmit(0x01);
//    }
//    else
//    {
//
//
//        OUT_C_OFF_L();
////        DELAY_US(100);
//        OUT_C_ON_H();
//        POWER_LED_ON();
//
//        scia_xmit(0x5a);
//        scia_xmit(0xa5);
//        scia_xmit(0x05);
//        scia_xmit(0x82);
//        scia_xmit(0x03);
//        scia_xmit(0x00);
//        scia_xmit(0x00);
//        scia_xmit(0x00);
//    }
//    // Acknowledge this interrupt to get more from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

interrupt void xint4_isr(void)
{
//    scia_xmit(0x5a);
//    scia_xmit(0xa5);
//    scia_xmit(0x05);
//    scia_xmit(0x82);
//    scia_xmit(0x03);
//    scia_xmit(0x20);
//
//    if(PULSE_SW)
//    {
//        scia_xmit(0x00);
//        scia_xmit(0x00);
//        EALLOW;
//        EPwm3Regs.TZFRC.bit.OST = 1;
//        EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
//        EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW
//
//        EPwm5Regs.TZFRC.bit.OST = 1;
//        EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
//        EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW
//
//        EPwm7Regs.TZFRC.bit.OST = 1;
//        EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
//        EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW
//
//        EPwm9Regs.TZFRC.bit.OST = 1;
//        EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
//        EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW
//        EDIS;
//    }
//    else
//    {
//        EALLOW;
//        EPwm3Regs.TZFRC.bit.OST = 1;
//        EPwm3Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//        EPwm3Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//
//        EPwm5Regs.TZFRC.bit.OST = 1;
//        EPwm5Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//        EPwm5Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//
//        EPwm7Regs.TZFRC.bit.OST = 1;
//        EPwm7Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//        EPwm7Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//
//        EPwm9Regs.TZFRC.bit.OST = 1;
//        EPwm9Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
//        EPwm9Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
//        EDIS;
//        scia_xmit(0x00);
//        scia_xmit(0x01);
//    }
    // Acknowledge this interrupt to get more from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}


interrupt void epwm3_isr(void)
{
// IsrTicker++;
//
// EPwm1Regs.TBPRD = (TBCLK / PwmCarrier) - 1;
// EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD * PwmDuty;


    EALLOW;
    if(g_PulseOn_ch3 == 1)
    {
            EPwm3Regs.TZFRC.bit.OST = 1;
            EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
            EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW
            EPwm3Regs.ETSEL.bit.INTEN = 0;
            g_PulseOn_ch3 = 0;
    }
    EDIS;


 /* Clear INT flag for this timer */
 EPwm3Regs.ETCLR.bit.INT = 1;

 /* Acknowledge this interrupt to receive more interrupts from group 3 */
 PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}



interrupt void cpu_timer0_isr(void)
{

    float clkdvider = 1.0;
    Uint16 clkdiv_val =1;



    EALLOW;

      if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
      {
          clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
      }
      if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
      {
          clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
      }
      EDIS;
//        g_status = 0;
//        g_status = P_PROTECT_STATE;  //Protection
//        g_status = (g_status << 1) + OV_STATE3;  //OT
//        g_status = (g_status << 1) + OV_STATE;  //OV
//        g_status = (g_status << 1) + OV_STATE;  //OC

//      spi_init(SPI_MODE_ADC);
        g_my_ADC.ADC_RAW[0] = read_ADS8343(ADC_CH0_SINGLE_MODE, ADC_SINGLE_MODE, ADC_NO_POWER_DOWN_MODE_EX_CLK); //ADC_POWER_DOWN_); //ADC_NO_POWER_DOWN_MODE_EX_CLK); //ADC_INTER_CLK_);
        g_my_ADC.ADC_RAW[1] = read_ADS8343(ADC_CH1_SINGLE_MODE, ADC_SINGLE_MODE, ADC_NO_POWER_DOWN_MODE_EX_CLK); //ADC_POWER_DOWN_); //ADC_NO_POWER_DOWN_MODE_EX_CLK); //ADC_INTER_CLK_);
        g_my_ADC.Conversion(&g_my_ADC);
//
        UART_TX_ADC0_value = (Uint16)(g_my_ADC.convValue[0]);
        UART_TX_ADC1_value = (Uint16)(g_my_ADC.convValue[1]);



        if(f_DataSendEnd == 1)
        {
            f_DataSendEnd = 0;
            f_DataSendStart = 1;
        }

        // TEST DAC
//        spi_init(SPI_MODE_DAC);        // init SPI for DAC
//        write_AD5624(0x03, 0x07,  g_VoltageRef * (65534 / 100));  // cmd = 0x00 (Write to input register n)
//------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------
//        scia_xmit(0xa5);
//        scia_xmit(UART_TX_ADC0_value>>8);
//        scia_xmit(0x00FF & UART_TX_ADC0_value);
//        scia_xmit(UART_TX_ADC1_value>>8);
//        scia_xmit(0x00FF & UART_TX_ADC1_value);
//        scia_xmit(g_voltageRef>>8);
//        scia_xmit(0x00FF & g_voltageRef);
//        scia_xmit(g_pulseWidth>>8);
//        scia_xmit(0x00FF & g_pulseWidth);
//        scia_xmit(g_periode>>8);
//        scia_xmit(0x00FF & g_periode);
//        scia_xmit(g_status);
//        scia_xmit(0x5a);


//------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------



    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
/*
 * Name of Function :  scia_rx_isr
 * Return : void
 * Description : SCIA Serial Communication Interrupt Service Routine
 * This service routines is purpose of Header detect and Sensor data collect.
 */



Uint32	g_CounterError=0;
Uint32	g_FixCounterError=0;
Uint8*  g_DWINData = 0;
interrupt void scia_rx_isr(void)     // SCI-a
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
	Uint16	ubSensorDataBuffer[16];		//Sensor byte data buffer ;{kor} 버퍼 사이즈가 16인 이유는  FIFO Mode가 16이기 때문
	int i;
	int     NumOfByte =0;

	MSG_STRUCT_DEF InputMsg;

	g_NumofScia++;
	NumOfByte = SciaRegs.SCIFFRX.bit.RXFFST;	/*FIFO Interrupt Received Byte Numbers*/

	for(i=0;i<NumOfByte;i++)
	{
		ubSensorDataBuffer[i] = SciaRegs.SCIRXBUF.all;

	}

    if(ubSensorDataBuffer[0]==0x5a && ubSensorDataBuffer[1]==0xa5 && ubSensorDataBuffer[8]==0xfe)
    {

        InputMsg.m_uMsgType = ubSensorDataBuffer[2];
        InputMsg.m_uDataType = ubSensorDataBuffer[3];
        InputMsg.m_ulData.w[1] = ((ubSensorDataBuffer[4] << 8) & 0xff00) + (ubSensorDataBuffer[5] & 0x00ff);
        InputMsg.m_ulData.w[0] = ((ubSensorDataBuffer[6] << 8) & 0xff00) + (ubSensorDataBuffer[7] & 0x00ff);

        SetInputMsg(&InputMsg);

//        if(msgType == 0x01 && ubSensorDataBuffer[4+(DataLength*2)+1] == 0xfe)
//        {
//            ReceiveDataParser(DataLength, Command, &ubSensorDataBuffer[5]);
//        }
//        else
//        {
//            ReceiveDataParser(DataLength, Command, &ubSensorDataBuffer[4]);
//        }

    }

	SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}
/*
 * Name of Function :  scic_rx_isr
 * Return : void
 * Description : SCIA Serial Communication Interrupt Service Routine
 * This service routines is purpose of Control command Transfer.
 */
interrupt void scib_rx_isr(void)     // SCI-C
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

	Uint16	NumOfByte;
	int i;

	Uint8 ubRxBuffer[10];		/*Received Character Data Buffer*/

	NumOfByte = ScibRegs.SCIFFRX.bit.RXFFST;	/*Size of Received Data*/

	for(i=0;i<NumOfByte;i++)
	{

		ubRxBuffer[i] = ScibRegs.SCIRXBUF.bit.RXDT;	/*Get into Local Buffer from SCIC Receive Buffer*/

		if(f_DetectCommandHeader==CLEAR)
		{
			if(ubRxBuffer[i]=='S')		//Check for Header
			{
				g_ubCommandCount=0;
				g_ubAckCommandBuffer[g_ubCommandCount]=ubRxBuffer[i];
				g_ubCommandCount++;
				f_DetectCommandHeader=SET;
			}
		}//If Wait for Command Header
		else if(f_DetectCommandHeader==SET)	//Header Detected then
		{
			if(ubRxBuffer[i]=='E' && g_ubCommandCount==4)
			{
				g_ubAckCommandBuffer[g_ubCommandCount]=ubRxBuffer[i];
				g_ubCommandCount=0;
				f_DetectCommandHeader=CLEAR;


				f_CommandReceived=SET;

			}
			g_ubAckCommandBuffer[g_ubCommandCount]=ubRxBuffer[i];
			g_ubCommandCount++;

		}//Else if Detected Command Header End
	}//For Using Received Data End



	ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
	ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;


  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
//  for(;;);

}

