/*
 * DWIN_UI.c
 *
 *  Created on: 2018. 9. 30.
 *      Author: Ansukho
 */


#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File
#include "..\define.h"
#include "..\UTIL\UTIL_Queue.h"
#include "DWIN_UI.h"
#include "math.h"

extern char g_Ready;
extern Uint16 g_EPwm5CMPA;
extern Uint16 g_EPwm6CMPA;
extern Uint16 g_VoltageRef;
extern void sendOnOffTime(char ch, Uint16 on, Uint16 Off, Uint16 phase);
extern void SendOnTime(char ch, Uint16 on);
extern void SendOffTime(char ch, Uint16 Off);
extern void SendPhaseTime(char ch, Uint16 phase);
extern void InitEPwm1_AB();
extern void InitEPwm1();
extern void InitEPwm2();
extern void InitEPwm3();
extern void InitEPwm4();
extern void InitEPwm5();
extern void InitEPwm6();
extern void InitEPwm7();
extern void InitEPwm8();
extern void InitEPwm9();
extern void InitEPwm3Event(void);
extern enum runningMode mode;
extern Uint16 g_periode;
extern Uint16 g_pulseWidth;

extern ARRAY_QUEUE_def g_stDataMessageQueue;
extern void SetData(Uint8 *data);

char sciPacketData[8] = {0,0,0,0,0,0,0,0};

float g_clkdvider = 1.0;
Uint16 g_clkdiv_val =1;
SETTING_MSG_STRUCT_DEF g_SaveSettingData;
void ReceiveDataParser(MSG_STRUCT_DEF Data)
{
    float clkdvider = 1.0;
    Uint16 TBRPD_prd = 0;
    Uint16 clkdiv_val =1;
    Uint16 Deadband =0;

    Uint16 RetunValueOn = 0;
    Uint16 RetunValueOff = 0;
    Uint16 RetunValuePhase = 0;

    Uint16 OffTimeValue = 0;

    MSG_STRUCT_DEF outputMsg;
    SETTING_MSG_STRUCT_DEF sendSetMsg;
    switch(Data.m_uMsgType)
    {
    case Connect:
//        Data.m_uDataType
        if(Data.m_uDataType == u16)
        {
            if(Data.m_ulData.w[0] == 0)
            {
                outputMsg.m_uMsgType = Connect;
                outputMsg.m_uDataType =  u16;
                outputMsg.m_ulData.w[0] = 1;
                SetOutputMsg(&outputMsg);
            }else if(Data.m_ulData.w[0] == 1)
            {

                mode = run;
                sendSetMsg = GetSettingData();
                sendSetMsg.m_uiConnedted = 1;

                g_SaveSettingData = sendSetMsg;
                SetOutputSettingData(&sendSetMsg);

            }

        }
        break;
    case pulsewidth:

        if(Data.m_uDataType == u16)
        {
//            Data.m_ulData.w[0]

        }
        EALLOW;

        EPwm2Regs.CMPA.half.CMPA = (Uint16)((float)Data.m_ulData.w[0] / (3.333333 * g_clkdvider));
        EPwm4Regs.CMPA.half.CMPA = (Uint16)((float)Data.m_ulData.w[0] / (3.333333 * g_clkdvider));
        EPwm6Regs.CMPA.half.CMPA = (Uint16)((float)Data.m_ulData.w[0] / (3.333333 * g_clkdvider));
        EPwm8Regs.CMPA.half.CMPA = (Uint16)((float)Data.m_ulData.w[0] / (3.333333 * g_clkdvider));

        g_pulseWidth = (Uint16)((float)EPwm2Regs.CMPA.half.CMPA * (3.333333 * g_clkdvider));


        EDIS;
        break;
    case periode:
        if(Data.m_uDataType == u16)
        {

            SetPeriode(Data.m_ulData.w[0]);
            sendSetMsg = GetSettingData();
            SetOutputSettingData(&sendSetMsg);
        }

        break;

    }


}

SETTING_MSG_STRUCT_DEF GetSettingData()
{
    SETTING_MSG_STRUCT_DEF sendSetMsg;

    Uint16 clkdiv_val =1;
    float clkdvider = 1.0;

    EALLOW;
    if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
    {
        clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
    }
    if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
    {
        clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
    }

    g_pulseWidth = (Uint16)((float)EPwm2Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
    sendSetMsg.m_uiPulseWidth = g_pulseWidth;
    g_periode = (Uint16)((1/((EPwm1Regs.TBPRD * (3.333333 * clkdvider)) * 0.000001))/2);
    sendSetMsg.m_uiPulsePeriode = g_periode;
    sendSetMsg.m_uiCh1_OnTime = (Uint16)((float)EPwm3Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh1_OffTime = (Uint16)((float)(EPwm2Regs.CMPB - EPwm2Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh1_PhaseTime = (Uint16)((float)EPwm2Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));

    sendSetMsg.m_uiCh2_PhaseTime = (Uint16)((float)EPwm4Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh2_OffTime = (Uint16)((float)(EPwm4Regs.CMPB - EPwm4Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh2_OnTime = (Uint16)((float)EPwm5Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

    sendSetMsg.m_uiCh3_PhaseTime = (Uint16)((float)EPwm6Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh3_OffTime = (Uint16)((float)(EPwm6Regs.CMPB - EPwm6Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh3_OnTime = (Uint16)((float)EPwm7Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

    sendSetMsg.m_uiCh4_PhaseTime = (Uint16)((float)EPwm8Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh4_OffTime = (Uint16)((float)(EPwm8Regs.CMPB - EPwm8Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    sendSetMsg.m_uiCh4_OnTime = (Uint16)((float)EPwm9Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

    EDIS;

    return sendSetMsg;
}


void SetPeriode(Uint16 Value)
{
    int checkprd = 1;
    long TBRPD_prd = 0;

    Uint8 divCLK = 0;
    Uint8 divHSPCLK = 0;

    Uint16 clkdvider = 1.0;
    Uint16 clkdiv_val =1;
    Uint16 OffTimeValue = 0;
    EALLOW;


    while(checkprd == 1)
    {
        TBRPD_prd = (long)((((1.0/(float)Value)*1000000.0)/(3.3333333* clkdvider))/2);
        if(TBRPD_prd > 65535)
        {
            clkdvider = clkdvider << 1;
        }
        else
        {
            checkprd = 0;
        }
    }


    switch(clkdvider)
    {
    case 2:
        divCLK = TB_DIV2;
        divHSPCLK = TB_DIV1;
        break;
    case 4:
        divCLK = TB_DIV2;
        divHSPCLK = TB_DIV2;
        break;
    case 8:
        divCLK = TB_DIV4;
        divHSPCLK = TB_DIV2;
        break;
    case 16:
        divCLK = TB_DIV4;
        divHSPCLK = TB_DIV4;
        break;
    default:
        divCLK = TB_DIV1;
        divHSPCLK = TB_DIV1;
        break;
    }

        EPwm1Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm1Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm2Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm2Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm3Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm3Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm4Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm4Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm5Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm5Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm6Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm6Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm7Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm7Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm8Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm8Regs.TBCTL.bit.CLKDIV = divCLK;
        EPwm9Regs.TBCTL.bit.HSPCLKDIV = divHSPCLK;       // Clock ratio to SYSCLKOUT
        EPwm9Regs.TBCTL.bit.CLKDIV = divCLK;



        EPwm1Regs.TBPRD = TBRPD_prd;          // Set Compare B value
        EPwm2Regs.TBPRD = TBRPD_prd;
        EPwm3Regs.TBPRD = TBRPD_prd;
        EPwm4Regs.TBPRD = TBRPD_prd;
        EPwm5Regs.TBPRD = TBRPD_prd;
        EPwm6Regs.TBPRD = TBRPD_prd;
        EPwm7Regs.TBPRD = TBRPD_prd;
        EPwm8Regs.TBPRD = TBRPD_prd;
        EPwm9Regs.TBPRD = TBRPD_prd;

        EPwm3Regs.CMPA.half.CMPA = (Uint16)((float)g_SaveSettingData.m_uiCh1_OnTime / (3.333333 * clkdvider));
        EPwm5Regs.CMPA.half.CMPA = (Uint16)((float)g_SaveSettingData.m_uiCh2_OnTime / (3.333333 * clkdvider));
        EPwm7Regs.CMPA.half.CMPA = (Uint16)((float)g_SaveSettingData.m_uiCh3_OnTime / (3.333333 * clkdvider));
        EPwm9Regs.CMPA.half.CMPA = (Uint16)((float)g_SaveSettingData.m_uiCh4_OnTime / (3.333333 * clkdvider));

        if(g_SaveSettingData.m_uiCh1_OffTime < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)g_SaveSettingData.m_uiCh1_OffTime / (3.333333 * clkdvider));
        }

        EPwm2Regs.CMPB = OffTimeValue + EPwm4Regs.CMPA.half.CMPA;

        if(g_SaveSettingData.m_uiCh2_OffTime < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)g_SaveSettingData.m_uiCh1_OffTime / (3.333333 * clkdvider));
        }

        EPwm4Regs.CMPB = OffTimeValue + EPwm4Regs.CMPA.half.CMPA;


        if(g_SaveSettingData.m_uiCh3_OffTime < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)g_SaveSettingData.m_uiCh3_OffTime / (3.333333 * clkdvider));
        }

        EPwm6Regs.CMPB = OffTimeValue + EPwm6Regs.CMPA.half.CMPA;


        if(g_SaveSettingData.m_uiCh4_OffTime < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)g_SaveSettingData.m_uiCh4_OffTime / (3.333333 * clkdvider));
        }

        EPwm8Regs.CMPB = OffTimeValue + EPwm8Regs.CMPA.half.CMPA;

        EPwm2Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh1_PhaseTime / (3.333333 * clkdvider));
        EPwm3Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh1_PhaseTime / (3.333333 * clkdvider));

        EPwm4Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh2_PhaseTime / (3.333333 * clkdvider));
        EPwm5Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh2_PhaseTime / (3.333333 * clkdvider));

        EPwm6Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh3_PhaseTime / (3.333333 * clkdvider));
        EPwm7Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh3_PhaseTime / (3.333333 * clkdvider));

        EPwm8Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh4_PhaseTime / (3.333333 * clkdvider));
        EPwm9Regs.TBPHS.half.TBPHS = (Uint16)((float)g_SaveSettingData.m_uiCh4_PhaseTime / (3.333333 * clkdvider));


    EDIS;
}
void SetVPData(Uint16 Address, Uint16 Value)
{
    float clkdvider = 1.0;
    Uint16 TBRPD_prd = 0;
    Uint16 clkdiv_val =1;
    Uint16 Deadband =0;

    Uint16 RetunValueOn = 0;
    Uint16 RetunValueOff = 0;
    Uint16 RetunValuePhase = 0;

    Uint16 OffTimeValue = 0;

    EALLOW;
    if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
    {
        clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
    }
    if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
    {
        clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
    }

    g_pulseWidth = (Uint16)((float)EPwm2Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
    g_periode = (Uint16)((1/((EPwm1Regs.TBPRD * (3.333333 * clkdvider)) * 0.000001))/2);
    RetunValueOn = (Uint16)((float)EPwm3Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
    RetunValueOff = (Uint16)((float)(EPwm2Regs.CMPB - EPwm2Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    RetunValuePhase = (Uint16)((float)EPwm2Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));

    RetunValuePhase = (Uint16)((float)EPwm4Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
    RetunValueOff = (Uint16)((float)(EPwm4Regs.CMPB - EPwm4Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    RetunValueOn = (Uint16)((float)EPwm5Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

    RetunValuePhase = (Uint16)((float)EPwm6Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
    RetunValueOff = (Uint16)((float)(EPwm6Regs.CMPB - EPwm6Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    RetunValueOn = (Uint16)((float)EPwm7Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

    RetunValuePhase = (Uint16)((float)EPwm8Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
    RetunValueOff = (Uint16)((float)(EPwm8Regs.CMPB - EPwm8Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
    RetunValueOn = (Uint16)((float)EPwm9Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

    EDIS;

    switch(Address)
    {
    case 0x0000:

        if(Value > 100)
        {
            Value = 100;//Set Voltage Ref
        }
        g_VoltageRef = Value;
        break;
    case 0x0020:    //Set PulseWidth
        EALLOW;

        EPwm2Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));
        EPwm4Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));
        EPwm6Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));
        EPwm8Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));

        g_pulseWidth = (Uint16)((float)EPwm2Regs.CMPA.half.CMPA * (3.333333 * clkdvider));


        EDIS;
        break;
    case 0x0040:    //Set Periode
        EALLOW;
        if(Value < 3)
        {
            EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV2;
            EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV2;


            clkdvider = 1.0;

            clkdiv_val =1;
            if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
            {
                clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
            }

            if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
            {
                clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
            }

            TBRPD_prd = (Uint16)((((1.0/(float)Value)*1000000.0)/(3.3333333* clkdvider))/2);
            EPwm1Regs.TBPRD = TBRPD_prd;          // Set Compare B value
            EPwm2Regs.TBPRD = TBRPD_prd;
            EPwm3Regs.TBPRD = TBRPD_prd;
            EPwm4Regs.TBPRD = TBRPD_prd;
            EPwm5Regs.TBPRD = TBRPD_prd;
            EPwm6Regs.TBPRD = TBRPD_prd;
            EPwm7Regs.TBPRD = TBRPD_prd;
            EPwm8Regs.TBPRD = TBRPD_prd;
            EPwm9Regs.TBPRD = TBRPD_prd;
            g_periode = (Uint16)((1/((EPwm1Regs.TBPRD * (3.333333 * clkdvider)) * 0.000001))/2);
        }
        else
        {
            EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;
            EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV1;

            if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
            {
                clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
            }
            if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
            {
                clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
            }
            TBRPD_prd = (Uint16)((((1.0/(float)Value)*1000000.0)/(3.3333333* clkdvider))/2);
            EPwm1Regs.TBPRD = TBRPD_prd;          // Set Compare B value
            EPwm2Regs.TBPRD = TBRPD_prd;
            EPwm3Regs.TBPRD = TBRPD_prd;
            EPwm4Regs.TBPRD = TBRPD_prd;
            EPwm5Regs.TBPRD = TBRPD_prd;
            EPwm6Regs.TBPRD = TBRPD_prd;
            EPwm7Regs.TBPRD = TBRPD_prd;
            EPwm8Regs.TBPRD = TBRPD_prd;
            EPwm9Regs.TBPRD = TBRPD_prd;
            g_periode = (Uint16)((float)(1.0/((float)(EPwm1Regs.TBPRD * (3.333333 * clkdvider)) * 0.000001))/2);
        }

        EDIS;

        break;
    case 0x0180:   //CH1 OnPulse

        EALLOW;

        EPwm3Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));


        RetunValueOn = (Uint16)((float)EPwm3Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
        EDIS;

        SendOnTime(0x01, RetunValueOn);

        break;

    case 0x0200:   //CH1 OFFPulse
        EALLOW;

        if(Value < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)Value / (3.333333 * clkdvider));
        }

        EPwm2Regs.CMPB = OffTimeValue + EPwm2Regs.CMPA.half.CMPA;

        RetunValueOff = (Uint16)((float)(EPwm2Regs.CMPB - EPwm2Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));

        EDIS;

        SendOffTime(0x01, RetunValueOff);

        break;

    case 0x0220:   //CH1 Phase
        EALLOW;
        EPwm2Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));
        EPwm3Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));

        RetunValuePhase = (Uint16)((float)EPwm2Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
        EDIS;
        SendPhaseTime(0x01, RetunValuePhase);
        break;

    case 0x0240:   //CH2 OnPulse

        EALLOW;

        EPwm5Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));

        RetunValueOn = (Uint16)((float)EPwm5Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
        EDIS;

        SendOnTime(0x02, RetunValueOn);


        break;

    case 0x0260:   //CH2 OFFPulse
        EALLOW;

        if(Value < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)Value / (3.333333 * clkdvider));
        }

        EPwm4Regs.CMPB = OffTimeValue + EPwm4Regs.CMPA.half.CMPA;

        RetunValueOff = (Uint16)((float)(EPwm4Regs.CMPB - EPwm4Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));


        EDIS;

        SendOffTime(0x02, RetunValueOff);

        break;

    case 0x0280:   //CH2 Phase
        EALLOW;
        EPwm4Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));
        EPwm5Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));

        RetunValuePhase = (Uint16)((float)EPwm4Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));


        EDIS;
        SendPhaseTime(0x02, RetunValuePhase);
        break;

    case 0x0420:   //CH3 OnPulse

        EALLOW;

        EPwm7Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));


        RetunValueOn = (Uint16)((float)EPwm7Regs.CMPA.half.CMPA * (3.333333 * clkdvider));

        EDIS;

        SendOnTime(0x03, RetunValueOn);

        break;

    case 0x0440:   //CH3 OFFPulse
        EALLOW;

        if(Value < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)Value / (3.333333 * clkdvider));
        }

        EPwm6Regs.CMPB = OffTimeValue + EPwm6Regs.CMPA.half.CMPA;

        RetunValueOff = (Uint16)((float)(EPwm6Regs.CMPB - EPwm6Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));


        EDIS;

        SendOffTime(0x03, RetunValueOff);

        break;

    case 0x0460:   //CH3 Phase
        EALLOW;
        EPwm6Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));
        EPwm7Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));

        RetunValuePhase = (Uint16)((float)EPwm6Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));

        EDIS;
        SendPhaseTime(0x03, RetunValuePhase);
        break;

    case 0x0480:   //CH4 OnPulse

        EALLOW;

        EPwm9Regs.CMPA.half.CMPA = (Uint16)((float)Value / (3.333333 * clkdvider));


        RetunValueOn = (Uint16)((float)EPwm9Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
        EDIS;

        SendOnTime(0x04, RetunValueOn);

        break;

    case 0x0500:   //CH4 OFFPulse
        EALLOW;

        if(Value < (3.333333 * clkdvider))
        {
            OffTimeValue = (3.333333 * clkdvider) +1;
        }
        else
        {
            OffTimeValue = (Uint16)((float)Value / (3.333333 * clkdvider));
        }

        EPwm8Regs.CMPB = OffTimeValue + EPwm8Regs.CMPA.half.CMPA;

        RetunValueOff = (Uint16)((float)(EPwm8Regs.CMPB - EPwm8Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));


        EDIS;

        SendOffTime(0x04, RetunValueOff);

        break;

    case 0x0520:   //CH4 Phase
        EALLOW;
        EPwm8Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));
        EPwm9Regs.TBPHS.half.TBPHS = (Uint16)((float)Value / (3.333333 * clkdvider));
        RetunValuePhase = (Uint16)((float)EPwm8Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));

        EDIS;
        SendPhaseTime(0x04, RetunValuePhase);
        break;


//    case 0x1100:    //Set Ready
//        g_Ready = 1;
////        SetPWMCH5AEnable(1);
//        break;
//    case 0x1120:    //Set STart
//        g_Ready = 0;
////        SetPWMCH5AEnable(0);
//        break;
//
    case 0x1140:   //Set STart
        if(Value == 1)
        {

            EALLOW;

            RetunValueOn = (Uint16)((float)EPwm3Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
            RetunValueOff = (Uint16)((float)(EPwm2Regs.CMPB - EPwm2Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
            RetunValuePhase = (Uint16)((float)EPwm2Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
            EDIS;

            sendOnOffTime(0x01, RetunValueOn,RetunValueOff,RetunValuePhase);

        }
//        SetPWMCH5AEnable(0);
        break;
    case 0x1240:   //Set STart
        if(Value == 0)
        {

            EALLOW;

            RetunValueOn = (Uint16)((float)EPwm5Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
            RetunValueOff = (Uint16)((float)(EPwm4Regs.CMPB - EPwm4Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
            RetunValuePhase = (Uint16)((float)EPwm4Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));

            EDIS;

            sendOnOffTime(0x02, RetunValueOn,RetunValueOff,RetunValuePhase);

        }
        break;
    case 0x1260:   //Set STart
        if(Value == 0)
        {

            EALLOW;

            RetunValueOn = (Uint16)((float)EPwm7Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
            RetunValueOff = (Uint16)((float)(EPwm6Regs.CMPB - EPwm6Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
            RetunValuePhase = (Uint16)((float)EPwm6Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
            EDIS;

            sendOnOffTime(0x03, RetunValueOn,RetunValueOff,RetunValuePhase);

        }
        break;
    case 0x1280:   //Set STart
        if(Value == 0)
        {

            EALLOW;

            RetunValueOn = (Uint16)((float)EPwm9Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
            RetunValueOff = (Uint16)((float)(EPwm8Regs.CMPB - EPwm8Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
            RetunValuePhase = (Uint16)((float)EPwm8Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
            EDIS;

            sendOnOffTime(0x04, RetunValueOn,RetunValueOff,RetunValuePhase);

        }
        break;
    case 0x1300:   //Set STart
        if(Value == 0)
        {

            EALLOW;

            RetunValueOn = (Uint16)((float)EPwm3Regs.CMPA.half.CMPA * (3.333333 * clkdvider));
            RetunValueOff = (Uint16)((float)(EPwm2Regs.CMPB - EPwm2Regs.CMPA.half.CMPA) * (3.333333 * clkdvider));
            RetunValuePhase = (Uint16)((float)EPwm2Regs.TBPHS.half.TBPHS * (3.333333 * clkdvider));
            EDIS;

            sendOnOffTime(0x01, RetunValueOn,RetunValueOff,RetunValuePhase);

        }
        break;

    case 0x1160:   //AB-Mode
//        g_EPwm5CMPA = (Uint16)((double)Value/3.3);


        EALLOW;

        EPwm1Regs.TZFRC.bit.OST = 1;
        EPwm1Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm1Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm3Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm4Regs.TZFRC.bit.OST = 1;
        EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm5Regs.TZFRC.bit.OST = 1;
        EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm6Regs.TZFRC.bit.OST = 1;
        EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm7Regs.TZFRC.bit.OST = 1;
        EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm8Regs.TZFRC.bit.OST = 1;
        EPwm8Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm8Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm9Regs.TZFRC.bit.OST = 1;
        EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EDIS;
        XIntruptRegs.XINT1CR.bit.ENABLE = 0;        // Enable Xint1
        InitEPwm1_AB();

        break;


    case 0x1200:   //AB-Mode
//        g_EPwm5CMPA = (Uint16)((double)Value/3.3);


        EALLOW;

        EPwm1Regs.TZFRC.bit.OST = 1;
        EPwm1Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm1Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm3Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm4Regs.TZFRC.bit.OST = 1;
        EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm5Regs.TZFRC.bit.OST = 1;
        EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm6Regs.TZFRC.bit.OST = 1;
        EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm7Regs.TZFRC.bit.OST = 1;
        EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm8Regs.TZFRC.bit.OST = 1;
        EPwm8Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm8Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm9Regs.TZFRC.bit.OST = 1;
        EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EDIS;

        InitEPwm1();
        InitEPwm2();
        InitEPwm3Event();


        EALLOW;
        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm2Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
        XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable Xint1
        EDIS;

        break;

    case 0x1340:   //Trig Start
        EALLOW;
        XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable Xint1
        EPwm3Regs.ETSEL.bit.INTEN = 1;
        EPwm3Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm3Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EDIS;
        break;
    case 0x1220:   //Set STart
        EALLOW;

        EPwm1Regs.TZFRC.bit.OST = 1;
        EPwm1Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm1Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EDIS;
        break;
    case 0x1180:   //Set STart
//        g_EPwm5CMPA = (Uint16)((double)Value/3.3);



        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

        InitEPwm1();
        InitEPwm2();
        InitEPwm3();
        InitEPwm4();
        InitEPwm5();
        InitEPwm6();
        InitEPwm7();
        InitEPwm8();
        InitEPwm9();

        XIntruptRegs.XINT1CR.bit.ENABLE = 0;        // Enable Xint1
        SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm2Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW


        EPwm4Regs.TZFRC.bit.OST = 1;
        EPwm4Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm4Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW



        EPwm6Regs.TZFRC.bit.OST = 1;
        EPwm6Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm6Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW



        EPwm8Regs.TZFRC.bit.OST = 1;
        EPwm8Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm8Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
        EDIS;
        break;

    case 0x1100:   //Set STart
        EALLOW;

        EPwm3Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm3Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EPwm5Regs.TZFRC.bit.OST = 1;
        EPwm5Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm5Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EPwm7Regs.TZFRC.bit.OST = 1;
        EPwm7Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm7Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EPwm9Regs.TZFRC.bit.OST = 1;
        EPwm9Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
        EPwm9Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

        EDIS;
        break;

    case 0x1120:   //Set STart
        EALLOW;

        EPwm3Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm5Regs.TZFRC.bit.OST = 1;
        EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm7Regs.TZFRC.bit.OST = 1;
        EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EPwm9Regs.TZFRC.bit.OST = 1;
        EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO; //
        EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // FORCE loW

        EDIS;
        break;
    case 0x0120:   //Set STart
        EALLOW;
        if(Value < 3)
        {
            EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
            EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

            if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
            {
                clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
            }
            if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
            {
                clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
            }
            TBRPD_prd = (Uint16)((((1.0/(float)Value)*1000000.0)/(3.3333333* clkdvider))/2);
            EPwm1Regs.TBPRD = TBRPD_prd;          // Set Compare B value
            g_periode = (Uint16)((1/((EPwm1Regs.TBPRD * (3.333333 * clkdvider)) * 0.000001))/2);
        }
        else
        {
            EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
            EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

            if(EPwm1Regs.TBCTL.bit.CLKDIV != 0)
            {
                clkdvider = clkdvider * (clkdiv_val << EPwm1Regs.TBCTL.bit.CLKDIV);
            }
            if(EPwm1Regs.TBCTL.bit.HSPCLKDIV != 0)
            {
                clkdvider = clkdvider * (EPwm1Regs.TBCTL.bit.HSPCLKDIV * 2);
            }
            TBRPD_prd = (Uint16)((((1.0/(float)Value)*1000000.0)/(3.3333333* clkdvider))/2);
            EPwm1Regs.TBPRD = TBRPD_prd;          // Set Compare B value
            g_periode = (Uint16)((float)(1.0/((float)(EPwm1Regs.TBPRD * (3.333333 * clkdvider)) * 0.000001))/2);
        }

        EDIS;
        break;
    case 0x0140:   //Set STart


        Deadband = (Uint16)((float)Value / (3.333333 * clkdvider));
        EALLOW;
        EPwm1Regs.DBFED = Deadband;
        EPwm1Regs.DBRED = Deadband;
EDIS;
        Deadband = (Uint16)((float)EPwm1Regs.DBRED * (3.333333 * clkdvider));

//        scia_xmit(0x5a);
//        scia_xmit(0xa5);
//        scia_xmit(0x05);
//        scia_xmit(0x82);
//        scia_xmit(0x01);
//        scia_xmit(0x40);
//        scia_xmit(Deadband>>8);
//        scia_xmit(0x00FF & Deadband);

        break;
    case 0x0160:   //Set STart
//        g_EPwm6CMPA = (Uint16)((double)Value/3.3);
        break;

    case 0x1320:   //Set STart
//        g_EPwm6CMPA = (Uint16)((double)Value/3.3);
        break;

    case 0x1500:    //POWER ON
        if(Value == 0)
        {
            mode = ready;

//            OUT_C_ON_L();
//            OUT_C_OFF_H();
//            POWER_LED_OFF();
//            scia_xmit(0x5a);
//            scia_xmit(0xa5);
//            scia_xmit(0x05);
//            scia_xmit(0x82);
//            scia_xmit(0x15);
//            scia_xmit(0x00);
//            scia_xmit(0x00);
//            scia_xmit(0x01);
        }
        else
        {


//            OUT_C_OFF_L();
    //        DELAY_US(100);
//            OUT_C_ON_H();
//            POWER_LED_ON();

//            scia_xmit(0x5a);
//            scia_xmit(0xa5);
//            scia_xmit(0x05);
//            scia_xmit(0x82);
//            scia_xmit(0x15);
//            scia_xmit(0x00);
//            scia_xmit(0x00);
//            scia_xmit(0x00);
        }
        break;
    case 0x1520:    //PULSE ON
//        scia_xmit(0x5a);
//        scia_xmit(0xa5);
//        scia_xmit(0x05);
//        scia_xmit(0x82);
//        scia_xmit(0x15);
//        scia_xmit(0x20);

        if(Value == 0)
        {
//            scia_xmit(0x00);
//            scia_xmit(0x00);
            EALLOW;
            EPwm3Regs.TZFRC.bit.OST = 1;
            EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
            EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW

            EPwm5Regs.TZFRC.bit.OST = 1;
            EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
            EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW

            EPwm7Regs.TZFRC.bit.OST = 1;
            EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
            EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW

            EPwm9Regs.TZFRC.bit.OST = 1;
            EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO   ; //
            EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO   ; // FORCE loW
            EDIS;
        }
        else
        {
            EALLOW;
            EPwm3Regs.TZFRC.bit.OST = 1;
            EPwm3Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
            EPwm3Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

            EPwm5Regs.TZFRC.bit.OST = 1;
            EPwm5Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
            EPwm5Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

            EPwm7Regs.TZFRC.bit.OST = 1;
            EPwm7Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
            EPwm7Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW

            EPwm9Regs.TZFRC.bit.OST = 1;
            EPwm9Regs.TZCTL.bit.TZA = TZ_NO_CHANGE; //
            EPwm9Regs.TZCTL.bit.TZB = TZ_NO_CHANGE; // FORCE loW
            EDIS;
            scia_xmit(0x00);
            scia_xmit(0x01);
        }
        break;
    };
}
