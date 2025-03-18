/*
 * DWIN_UI.h
 *
 *  Created on: 2018. 9. 30.
 *      Author: Ansukho
 */

#ifndef DRIVER_DWIN_UI_H_
#define DRIVER_DWIN_UI_H_


#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File

void SetVPData(Uint16 Address, Uint16 Value);
void ReceiveDataParser(MSG_STRUCT_DEF DataData);
void SetOutputMsg(MSG_STRUCT_DEF *i_pstOutputMsg);
void SetOutputSettingData(SETTING_MSG_STRUCT_DEF *i_pstOutSetData);
void SetPeriode(Uint16 Value);
SETTING_MSG_STRUCT_DEF GetSettingData();
#endif /* DRIVER_DWIN_UI_H_ */
