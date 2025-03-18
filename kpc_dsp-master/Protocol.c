



#include "F28346_Include\DSP28x_Project.h"
#include "F28346_Include\Example_2834xSci_Echoback.h"
#include "Protocol.h"

//-----------------------------------------------------------------------
//  EPOS2 24/2 Protocol Functions 
//-----------------------------------------------------------------------


Uint8 Chked_Receive_Frame(Uint8 chk_op)
{
	Uint8 Rx_buf[100];
	Uint8 i;
	Uint8 CHK_Byte_Len;
	Uint8 f_cpt_rcv_chk=1;
	char *msg1;


	Rx_buf[0]=chk_op;
	scib_xmit(Rx_buf[0]);

	msg1 = "\r\t 4 receive check ok \n\0";
	scic_msg(msg1);

	while(f_cpt_rcv_chk)
		{
			msg1 = "\r\t 5 receive check ok \n\0";
			scib_msg(msg1);
			
			for(i=1;i<CHK_Byte_Len+2;i++)
				{
					Rx_buf[i]=scia_getchar();
					scic_xmit(Rx_buf[i]);
				}
			scib_xmit('O');
			f_cpt_rcv_chk=0;

		}

	return 0;

}


void Chked_Frame_TX(Uint8* pByte_data, Uint8 byte_data_len)
{

	Uint8 f_cpt_tx=1;
	Uint8 ReceivedChar;
	Uint8 i;
	Uint8 op_code_frame_tx;
	Uint8 val;

	
	while(f_cpt_tx)
		{
		val = pByte_data[0];

		char *msg1;

		scia_xmit(val);
		ReceivedChar = scib_getchar();

		msg1 = "\r\t 1 opcode tx  \n\0";
		scic_msg(msg1);

		if (ReceivedChar==0x4f)
			{

			ReceivedChar=0;
				for (i=1;i<byte_data_len;i++)
					{
						val=pByte_data[i];
						scia_xmit(val);
					}

				msg1 = "\r\t 2 all data transfer ok \n\0";
				scic_msg(msg1);


				ReceivedChar = scib_getchar();
				if (ReceivedChar==0x4f)
					{

					ReceivedChar=0;
					msg1 = "\r\t 3 all data check ok \n\0";
					scic_msg(msg1);


					op_code_frame_tx=scib_getchar();

					if(op_code_frame_tx==0){

					scia_xmit('o');
					f_cpt_tx = Chked_Receive_Frame(op_code_frame_tx);
					}

					}

			}

		}




}



void Frame_TX(Uint8 OPC, Uint16* data)
{
	Uint16 	full_data[34];
	Uint16 	op_len;
	Uint16	CRC_Code;
	Uint16 	data_buf;
	Uint8	tx_data[64];
	Uint8 	i,j;
	Uint8	Num_len_byte, Num_len_word;
	char *msg1;

	op_len = Opcode_byte_Gen(OPC);		// = opcode + length-1 
	
	Num_len_word=  (op_len&0x00ff)+1;		// extract actual value of byte length
	Num_len_byte = Num_len_word*2;		// carculating number of byte value 
	full_data[0] = op_len;

	for(i=1;(i<Num_len_word+1);i++)
		{
			full_data[i]=data[i-1];
			full_data[i+1]=0;
		}

	CRC_Code = CalcFieldCRC(full_data,Num_len_word+2);

	full_data[Num_len_word+1] =  CRC_Code;
	CRC_Code=0;

	data_buf = full_data[0];
	tx_data[1]=(data_buf & 0x00ff);
	tx_data[0]=((data_buf>>8) & 0x00ff);


	for(j=1;j<Num_len_byte;j++)
		{
			data_buf = full_data[j];
			tx_data[j*2]=(data_buf & 0x00ff);
			tx_data[(j*2)+1]=((data_buf>>8) & 0x00ff);
		}
	
	msg1 = "\r\t data ready \n\0";
	scic_msg(msg1);
	
	Chked_Frame_TX(tx_data,Num_len_byte+4);

}



// Calculate RS232 Comunication CRC16-CCITT
Uint16 CalcFieldCRC(Uint16 * pDataArray, Uint16 numberOfWords)
{

	Uint16	shifter, c;
	Uint16	carry;
	Uint16	CRC = 0;

	while(numberOfWords--)
	{
		shifter = 0x8000;
		c = *pDataArray++;
	do
		{
			carry = CRC & 0x8000;
			CRC <<= 1;
			if(c & shifter) CRC++;
			if(carry) CRC ^= 0x1021;
			shifter >>= 1;
		}while(shifter);
	}
	return CRC;

}



Uint16 Opcode_byte_Gen(Uint8 op_code)
{

	Uint16 	Length;
	Uint16	WordData;

	switch (op_code){


		case  0x10 : Length = 0x0001;
					break;
		case  0x11 : Length = 0x0003;
					break;
		case  0x12 : Length = 0x0001;
					break;
		case  0x13 : Length = 0x0003;
					break;
		case  0x14 : Length = 0x0000;
					break;
		case  0x15 : Length = 0x001f;
					break;

		}
	WordData = op_code;
	WordData = ((WordData<<8) & 0xff00);
	WordData |= Length;


	return WordData;

}

//===========================================================================
// No more.
//===========================================================================

