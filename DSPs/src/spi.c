
#include "global.h" 
interrupt void spiTxFifoIsr(void);
interrupt void spiRxFifoIsr(void);

extern union STATUS_REG status; 
extern union config_reg spi_config; 
extern bool spi_flag; 

#define SPI_TRAME_LENGTH 8
Uint16 sdata[SPI_TRAME_LENGTH]; 
Uint16 rdata[SPI_TRAME_LENGTH]; 

void spi_init()
{
	// Initialize GPIOs
	InitSpiaGpio(); 

	// Initialize SPI FIFO registers
	SpiaRegs.SPICCR.bit.SPISWRESET=0; 		// Reset SPI
	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; 
	SpiaRegs.SPICCR.bit.SPILBK = 0; 			// Enable Loopback mode
	SpiaRegs.SPICCR.bit.SPICHAR = 0xF; 		// 16-bit character

	SpiaRegs.SPICTL.bit.CLK_PHASE = 0; 
	SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0; 	// Disable overrun interupt
	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0;  	// Spi is configured as slave
	SpiaRegs.SPICTL.bit.TALK = 1; 		 	// Enable transmission 
	SpiaRegs.SPICTL.bit.SPIINTENA = 1; 		// Enable SPI Interrupt 

	SpiaRegs.SPISTS.all=0x0000; 			// Clear status registrer

	SpiaRegs.SPIPRI.bit.FREE = 1; 			// Free run
	SpiaRegs.SPIPRI.bit.PRIORITY = 0; 		// Don't care

	SpiaRegs.SPIFFTX.all=0xC028;      // Enable FIFO's, set TX FIFO level to 8
	SpiaRegs.SPIFFTX.bit.TXFIFO = 0; // Reset FIFO
	SpiaRegs.SPIFFTX.bit.TXFFIL = 8; 

	SpiaRegs.SPIFFTX.bit.TXFIFO=1;

	SpiaRegs.SPIFFRX.bit.RXFFOVF = 1; 	// Clear Overflow flag
	SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0; // Reset FIFO
	SpiaRegs.SPIFFRX.bit.RXFFIENA = 1; 		// RX FIFO Interrupt enable

	SpiaRegs.SPIFFRX.bit.RXFFIL = 8; 		// Generate an interrupt when 2 words have been received
	SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1; // Reset FIFO
	SpiaRegs.SPIFFCT.all=0x00;

	SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

	// Init interrupts
	EALLOW;
	PieVectTable.SPIRXINTA = &spiRxFifoIsr;
	PieVectTable.SPITXINTA = &spiTxFifoIsr;
	EDIS;  

	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
	PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1
	PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2

	ENABLE_INT_LVL(M_INT6); 
}

interrupt void spiTxFifoIsr(void)
{
	Uint16 i;
    for(i=0;i<SPI_TRAME_LENGTH;i++)
    {
 	   SpiaRegs.SPITXBUF=sdata[i];      // Send data
    }

	// Acknowledge the transmission
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x20;  		// Issue PIE ACK
}

interrupt void spiRxFifoIsr(void)
{
    Uint16 i;
	for(i=0;i<SPI_TRAME_LENGTH;i++)
	{
	    rdata[i]=SpiaRegs.SPIRXBUF;		// Read data
	}

	// Affect configuration
	spi_config.word.ANGLE1  = rdata[0];	
	spi_config.word.SPEED1  = rdata[1];
	spi_config.word.TORQUE1 = rdata[2];

	spi_config.word.ANGLE2  = rdata[3];	
	spi_config.word.SPEED2  = rdata[4];
	spi_config.word.TORQUE2 = rdata[5];

	spi_config.word.CONFIG  = spi_config.word.CONFIG |  rdata[6];
	spi_config.word.CONFIG  = spi_config.word.CONFIG & ~rdata[7];

	spi_flag = true; 

	// Acknowledge the reception
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow  flag
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}

void spi_answer(union status_reg status)
{
	sdata[0] = 0x0000; 
	sdata[1] = status.all; 
	sdata[2] = status.all;
	sdata[3] = status.all;
	sdata[4] = status.all; 
	sdata[5] = status.all;
	sdata[6] = status.all;
	sdata[7] = status.all; 
}

void spi_clear_buffers()
{
    Uint16 i;

	for(i=0;i<SPI_TRAME_LENGTH;i++)
	{
		// Clear buffer
	    rdata[i]= 0x0000;
		sdata[i]= 0x0000;
	}

}
