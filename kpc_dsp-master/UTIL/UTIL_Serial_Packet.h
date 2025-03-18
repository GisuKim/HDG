/* ==================================================================================
File name:        UTIL_Serial_Packet.H                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments
Description:  





Target: TMS320F280x family
              
=====================================================================================
History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20: 
------------------------------------------------------------------------------------*/

#ifndef __UTILS_SERIAL_PACKET_H__
#define __UTILS_SERIAL_PACKET_H__

extern Uint8 g_SubmitData[400];
//#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File


#define GOOD_PACKET		0
#define WRONG_HEADER		1
#define WRONG_LENGTH		2
#define WRONG_CHECKSUM	3

/*-----------------------------------------------------------------------------
Define the structure of the Packet Driver Object 
-----------------------------------------------------------------------------*/
typedef struct
{
	Uint8 bID;
	Uint8 bInstruction;
	Uint8 bParameterLength;
	Uint8 gbpParameter[400];
	//Uint8 *pui8MsgData;
	Uint8 gbpTxBuffer[420];
    Uint8 (*TxPkt)();      // Pointer to the update function
}tSerialMsgObject;


typedef struct
{
	Uint8 bID;
	Uint8 bInstruction;
	Uint8 bParameterLength;
	Uint8 gbpParameter[128];
	//Uint8 *pui8MsgData;
	Uint8 bRxLength;
	Uint8 gbpRxBuffer[128];
       Uint8 (*RxPkt)();      // Pointer to the update function
}rSerialMsgObject;


/*-----------------------------------------------------------------------------
Define a Serial_packet_handle
-----------------------------------------------------------------------------*/
typedef tSerialMsgObject *TxSerialMsgObj_handle;
typedef rSerialMsgObject *RxSerialMsgObj_handle;

/*------------------------------------------------------------------------------
Default Initializers for the Packet Object 
------------------------------------------------------------------------------*/
#define TX_SERIAL_PACKET  {0x00,   \
                              0x00, \
                              0x00, \
                              {0x00,}, \
                              {0x00,}, \
                             (Uint8 (*)(Uint32))TxPacket \
                             }


#define TX_SERIAL_PKT_DEFAULTS 	TX_SERIAL_PACKET


#define RX_SERIAL_PACKET  {0x00,   \
                              0x00, \
                              0x00, \
                              {0x00,}, \
                               0x00, \
                              {0x00,}, \
                             (Uint8 (*)(Uint32))AnalysisRxPacket \
                             }


#define RX_SERIAL_PKT_DEFAULTS 	RX_SERIAL_PACKET


/*------------------------------------------------------------------------------
 Prototypes for the functions in UTIL_Serial_Packet.c
------------------------------------------------------------------------------*/
Uint8 TxPacket(TxSerialMsgObj_handle);
Uint8 AnalysisRxPacket(RxSerialMsgObj_handle);

#endif  // __UTILS_SERIAL_PACKET_H__
