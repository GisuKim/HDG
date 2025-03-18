/*
 * SerialQueue.c
 *
 *  Created on: 2020. 12. 28.
 *      Author: syslabs
 */

#include "SerialQueue.h"


ARRAY_QUEUE_def g_stDataMessageQueue = ARRAY_QUEUE_DEFAULTS;     /* Queue of Sensor Data */

ARRAY_QUEUE_def g_stInputMsgQueue = ARRAY_QUEUE_DEFAULTS;
ARRAY_QUEUE_def g_stOutputMsgQueue = ARRAY_QUEUE_DEFAULTS;
ARRAY_QUEUE_def g_stOutSetDataQueue = ARRAY_QUEUE_DEFAULTS;

//MSG_STRUCT_DEF g_stMsgIn;                                       /* Struct of Sensor Data */
//MSG_STRUCT_DEF g_stMsgOut;                                      /* Struct of Sensor Data */
//DATA_MSG_STRUCT_DEF g_stDataMsgIn;
//DATA_MSG_STRUCT_DEF g_stDataMsgOut;

void SetInputMsg(MSG_STRUCT_DEF *i_pstInputMsg)  // Wake-up, Ts Period
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Write_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Write_Buff[i] =0;
    }
//Set from Struct Data to Queue
    uData_Size = sizeof((*i_pstInputMsg));
    memcpy(&uTest_Write_Buff, &(*i_pstInputMsg), uData_Size);
    g_stInputMsgQueue.Push(&g_stInputMsgQueue, &uTest_Write_Buff);

    uData_Size=0;
}


void GetInputMsg(MSG_STRUCT_DEF *i_pstInputMsg)
// Wake-up, Ts Period
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Read_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Read_Buff[i] =0;
    }
//Get from Queue to Struct Data
    uData_Size = sizeof((*i_pstInputMsg));
    g_stInputMsgQueue.Pop(&g_stInputMsgQueue, &uTest_Read_Buff);
    memcpy(&(*i_pstInputMsg), &uTest_Read_Buff, uData_Size);

    uData_Size=0;
}

Uint16 GetInputMsgCnt()
{
    return g_stInputMsgQueue.m_uCount;
}

Uint16 GetOutputMsgCnt()
{
    return g_stOutputMsgQueue.m_uCount;
}


Uint16 GetOutputSettingDataCnt()
{
    return g_stOutSetDataQueue.m_uCount;
}

void SetOutputMsg(MSG_STRUCT_DEF *i_pstOutputMsg)  // Wake-up, Ts Period
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Write_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Write_Buff[i] =0;
    }
//Set from Struct Data to Queue
    uData_Size = sizeof((*i_pstOutputMsg));
    memcpy(&uTest_Write_Buff, &(*i_pstOutputMsg), uData_Size);
    g_stOutputMsgQueue.Push(&g_stOutputMsgQueue, &uTest_Write_Buff);

    uData_Size=0;
}


void GetOutputMsg(MSG_STRUCT_DEF *i_pstOutputMsg)
// Wake-up, Ts Period
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Read_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Read_Buff[i] =0;
    }
//Get from Queue to Struct Data
    uData_Size = sizeof((*i_pstOutputMsg));
    g_stOutputMsgQueue.Pop(&g_stOutputMsgQueue, &uTest_Read_Buff);
    memcpy(&(*i_pstOutputMsg), &uTest_Read_Buff, uData_Size);

    uData_Size=0;
}


void SetOutputSettingData(SETTING_MSG_STRUCT_DEF *i_pstOutSetData)  // Wake-up, Ts Period
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Write_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Write_Buff[i] =0;
    }
//Set from Struct Data to Queue
    uData_Size = sizeof((*i_pstOutSetData));
    memcpy(&uTest_Write_Buff, &(*i_pstOutSetData), uData_Size);
    g_stOutSetDataQueue.Push(&g_stOutSetDataQueue, &uTest_Write_Buff);

    uData_Size=0;
}


void GetOutputSettingData(SETTING_MSG_STRUCT_DEF *i_pstOutSetData)
// Wake-up, Ts Period
{
    Uint16 i;
    Uint16 uData_Size;
    Uint16 uTest_Read_Buff[QUEUE_ARRAY_SIZE];

    for(i=0; i<QUEUE_ARRAY_SIZE; i++)
    {
        uTest_Read_Buff[i] =0;
    }
//Get from Queue to Struct Data
    uData_Size = sizeof((*i_pstOutSetData));
    g_stOutSetDataQueue.Pop(&g_stOutSetDataQueue, &uTest_Read_Buff);
    memcpy(&(*i_pstOutSetData), &uTest_Read_Buff, uData_Size);

    uData_Size=0;
}
