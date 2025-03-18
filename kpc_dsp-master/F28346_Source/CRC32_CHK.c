/*
 * CRC32_CHK.c
 *
 *  Created on: 2014. 3. 18.
 *      Author: owner
 */

#include "..\F28346_Include\CRC32_CHK.h"
#include "stdio.h"

#define POLYNOMIAL	0x04c11db7L


static unsigned long crc_table[256];


/* generate the table of CRC remainders for all possible bytes */
void gen_crc_table()
{
	register int i, j;
	register unsigned long crc_accum;

	for(i=0; i<256; i++)
	{
		crc_accum = ((unsigned long) i << 24);
		for(j=0; j<8; j++)
		{
			if(crc_accum & 0x80000000L)
				crc_accum = (crc_accum << 1) ^ POLYNOMIAL;
			else
				crc_accum = (crc_accum << 1);
		}
		crc_table[i] = crc_accum;
#if DEBUG
		printf("%03d=%lx, ", i, crc_accum);

		if(i%7 == 0)
			printf("\n");
#endif
	}
	return;
}


/* update the CRC on the data block one byte at a time */
unsigned long update_crc(unsigned long crc_accum, char *data_blk_ptr,int data_blk_size)
{
	register int i, j;

	for(j=0; j<data_blk_size; j++)
	{
		i = ((int)(crc_accum >> 24) ^ *data_blk_ptr++) & 0xff;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	return crc_accum;
}

//
///**********************************************************
//	CRC-32 test sample
//***********************************************************/
//void main()
//{
//	unsigned long	crc, check;
//	char		in_frame[20];
//	char		out_frame[20];
//
//	gen_crc_table();
//
//	//generate arbitary data
//	in_frame[0] = 0x55;
//	in_frame[1] = 0xA1;
//	in_frame[2] = 0x12;
//	in_frame[3] = 0x34;
//
//	crc = update_crc(0, in_frame, 4);
//	printf("\n\n====== CRC-32 Demo ========\n");
//	printf("Transmitter CRC result = %lX", crc);
//
//	out_frame[0] = in_frame[0];
//	out_frame[1] = in_frame[1];
//	out_frame[2] = in_frame[2];
//	out_frame[3] = in_frame[3];
//	out_frame[4] = (crc & 0xFF000000L) >> 24;
//	out_frame[5] = (crc & 0x00FF0000L) >> 16;
//	out_frame[6] = (crc & 0x0000FF00L) >> 8;
//	out_frame[7] = (crc & 0x000000FFL);
//	check = update_crc(0, out_frame, 8);
//	printf("\nReceiver CRC check = %lX\n", check);
//}
