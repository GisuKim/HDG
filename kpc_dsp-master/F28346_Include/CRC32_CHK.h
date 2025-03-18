/*
 * CRC32_CHK.h
 *
 *  Created on: 2014. 3. 18.
 *      Author: owner
 */
#include "..\F28346_Include\DSP28x_Project.h"
#ifndef CRC32_CHK_H_
#define CRC32_CHK_H_

#define DEBUG	0

void gen_crc_table();
unsigned long update_crc(unsigned long crc_accum, char *data_blk_ptr,int data_blk_size);

#endif /* CRC32_CHK_H_ */
