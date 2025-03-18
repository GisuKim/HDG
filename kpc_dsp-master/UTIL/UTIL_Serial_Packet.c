/* ==================================================================================
File name:       F280XBLDCPWM.C
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description:   This file contains source for the Full Compare BLDC PWM drivers for the F280x
              
Target: TMS320F280x family
              
=====================================================================================
History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20: Using DSP280x v. 1.10 or higher 
------------------------------------------------------------------------------------*/
//#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "..\F28346_Include\DSP2834x_Examples.h"   // DSP2834x Examples Include File

#include "UTIL_Serial_Packet.h"
//#include "SCU_HAL_RS422.h"  // SP6 project
#include "..\F28346_Include\Example_2834xSci_Echoback.h"   // Example specific Include file


Uint8 TxPacket(tSerialMsgObject *p)
{
	Uint8 bCount,bCheckSum,bPacketLength;
	
	p->gbpTxBuffer[0] = 0xfe;
	p->gbpTxBuffer[1] = 0x81;
	p->gbpTxBuffer[2] = 0xff;
	p->gbpTxBuffer[3] = 0x55;
	p->gbpTxBuffer[4] = p->bID;
	p->gbpTxBuffer[5] = p->bParameterLength+2;

	
	//Length(Paramter,Instruction,Checksum)
	p->gbpTxBuffer[6] = p->bInstruction;

	
	for(bCount = 0; bCount < p->bParameterLength; bCount++)
	{
		p->gbpTxBuffer[bCount+7] = p->gbpParameter[bCount];
	}

	
	bCheckSum = 0;
	bPacketLength = p->bParameterLength+4+2+2;		//Header 4 +
	for(bCount = 4; bCount < bPacketLength-1; bCount++) //except  0xff,checksum
	{
		bCheckSum += p->gbpTxBuffer[bCount];
	}

	
	p->gbpTxBuffer[bCount] = (0x00ff) & (~bCheckSum); //Writing Checksum with Bit Inversion



	for(bCount = 0; bCount < bPacketLength; bCount++)
	{
		//scic_xmit(p->gbpTxBuffer[bCount]);
		g_SubmitData[bCount]=p->gbpTxBuffer[bCount];
		//scia_SendByte(p->gbpTxBuffer[bCount]);

		//if(bCount==10)
		//	DELAY_US(1);
	}

	return(bPacketLength);
}



//[h][h][id][pLength][insruction][paraN][paraN-1][para0][chsum]

Uint8 AnalysisRxPacket(rSerialMsgObject *p)
{

	Uint8 bCount, bLength;
	Uint8 bChecksum =0;

	bLength = p->bRxLength;

	if(p->gbpRxBuffer[0] != 0xff || p->gbpRxBuffer[1] != 0xff )
	{

		//CLEAR_BUFFER;
		return WRONG_HEADER;
	}

	if(p->gbpRxBuffer[3] != bLength-4)
	{

		//CLEAR_BUFFER;
		return WRONG_LENGTH;
	}


	bChecksum = 0;
	for(bCount = 2; bCount < bLength-1; bCount++) 
		bChecksum += p->gbpRxBuffer[bCount];
	

	bChecksum =(0x00ff) & (~bChecksum); //Writing Checksum with Bit Inversion


	if(bChecksum != p->gbpRxBuffer[bLength-1])
	{

		//CLEAR_BUFFER;
		return WRONG_CHECKSUM;
	}
	
	p->bID= p->gbpRxBuffer[2];
	p->bParameterLength = p->gbpRxBuffer[3] -2;
	p->bInstruction = p->gbpRxBuffer[4];

	for(bCount = 0; bCount < p->bParameterLength; bCount++) 
		p->gbpParameter[bCount] = p->gbpRxBuffer[5+bCount];

	return GOOD_PACKET;
	
  
}



