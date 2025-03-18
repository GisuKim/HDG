/*****************************************************************************
 * @file       SCU_HAL_SPI.c
 * @addtogroup SCU_HAL_SPI
 * @{
 ******************************************************************************/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SP6_SCU_Settings.h"
#include "SCU_HAL_SPI.h" //

#define DUMMY_DATA 					0xFF00


void spi_fifo_init_Interrupt_Mode(Uint16 MCB_MODE)
{
    SpiaRegs.SPICCR.bit.SPISWRESET=0; 			        // Reset SPI
    SpiaRegs.SPICCR.all=0x000F;       			        //16-bit character, No Loopback mode

    if( MCB_MODE==SLAVE_MODE)
    {
        SpiaRegs.SPICTL.all =0x17;    		     	    // Enable Master mode, normal phase,
    }
    if( MCB_MODE==MASTER_MODE)
    {
        SpiaRegs.SPICTL.all =0x13;    		     	    // Enable Slave mode, normal phase,
    }

    SpiaRegs.SPISTS.all=0x0000;
    SpiaRegs.SPIBRR=SPIBRR_VAL;           		        // Baud rate
    SpiaRegs.SPIFFTX.all=0xC028;      			        // Enable FIFO's, set TX FIFO level to 8
    SpiaRegs.SPIFFTX.bit.TXFFIL = SPI_FIFO_LEVEL;
    SpiaRegs.SPIFFRX.all=0x0028;      			        // Set RX FIFO level to 8
    SpiaRegs.SPIFFRX.bit.RXFFIL = SPI_FIFO_LEVEL;
    SpiaRegs.SPIFFCT.all=0x00;
    SpiaRegs.SPIPRI.all=0x0010;

    SpiaRegs.SPICCR.bit.SPISWRESET=1;  			        // Enable SPI
    SpiaRegs.SPIFFTX.bit.TXFIFO=1;                      // Re-Enable FIFO
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}

