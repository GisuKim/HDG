
#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File
#include "..\define.h"
#include "..\UTIL\UTIL_Queue.h"

void SetInputMsg(MSG_STRUCT_DEF *i_pstInputMsg);  // Wake-up, Ts Period;
void GetInputMsg(MSG_STRUCT_DEF *i_pstInputMsg);
Uint16 GetInputMsgCnt();

void SetOutputMsg(MSG_STRUCT_DEF *i_pstOutputMsg);
void GetOutputMsg(MSG_STRUCT_DEF *i_pstOutputMsg);
Uint16 GetOutputMsgCnt();

void SetOutputSettingData(SETTING_MSG_STRUCT_DEF *i_pstOutSetData);
void GetOutputSettingData(SETTING_MSG_STRUCT_DEF *i_pstOutSetData);
Uint16 GetOutputSettingDataCnt();
