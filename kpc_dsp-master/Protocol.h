/*
 * Protocol.h
 *
 *  Created on: 2014. 2. 20.
 *      Author: owner
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_


//---------------------------------------------------------------------------
// Function prototypes and external definitions:
//

Uint16 CalcFieldCRC(Uint16 * pDataArray, Uint16 numberOfWords);
Uint16 Opcode_byte_Gen(Uint8 op_code);
void Frame_TX(Uint8 OPC, Uint16* data);
void Chked_Frame_TX(Uint8* pByte_data, Uint8 byte_data_len);
Uint8 Chked_Receive_Frame(Uint8 chk_op);

//  Maxon EPOS2 Protocol Define
//-----------------------------------------------------------------------
#define  OpCode_ReadObj			0x10  		// Read Object
#define  OpCode_ISReadObj		0x12	 	// InitiateSegmentRead Object
#define  OpCode_SReadObj		0x14	 	// SegmentRead Object
#define  OpCode_WriteObj		0x11		// Write Object
#define  OpCode_ISWriteObj		0x13	 	// InitiateSegmentWrite Object
#define  OpCode_SWriteObj		0x15	 	// SegmentWrite Object



#endif /* PROTOCOL_H_ */
