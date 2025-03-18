#ifndef EXAMPLE_2834XSCI_ECHOBACK_H_
#define EXAMPLE_2834XSCI_ECHOBACK_H_

// Global counts used in this example

/*-----------------------------------------------------------------------------
Prototypes for the functions in Example_2834xSci_Echoback.c
-----------------------------------------------------------------------------*/
// Initalize the SCIa
void scia_init();
void scib_init();
void scic_init();


// Transmit a character from the SCI
void scia_xmit(int a);
void scib_xmit(int a);
void scic_xmit(int a);

void scia_msg(char * msg);
void scib_msg(char * msg);
void scic_msg(char * msg);

// Initalize the SCIa FIFO
void scia_fifo_init();
void scib_fifo_init();

// Get charicter 
Uint8 scia_getchar();
Uint8 scib_getchar();
Uint8 scic_getchar();




#endif /*EXAMPLE_2834XSCI_ECHOBACK_H_*/
