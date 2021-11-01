/////////////////////////////////////
//  Generated Initialization File  //
/////////////////////////////////////
#ifndef __C300__ 
#define __C300__ 
	#include "C8051F330.h"
	#include "types.h" 
#endif

// Peripheral specific initialization functions,
// Called from the Init_Device() function
void PCA_Init()
{
    PCA0MD    &= ~0x40;
    PCA0MD    = 0x00;
}

void Timer_Init()
{
    TMOD      = 0x02;
    CKCON     = 0x01;
}

void SPI_Init()
{
    SPI0CFG   = 0x50;
    SPI0CN    = 0x01;
    SPI0CKR   = 0x0B;
}

void ADC_Init()
{
    AMX0P     = 0x04;
    AMX0N     = 0x11;
    ADC0CN    = 0x80;
}

void Voltage_Reference_Init()
{
    REF0CN    = 0x08;
}

void Port_IO_Init()
{
    // P0.0  -  SCK  (SPI0), Push-Pull,  Digital
    // P0.1  -  MISO (SPI0), Open-Drain, Digital
    // P0.2  -  MOSI (SPI0), Push-Pull,  Digital
    // P0.3  -  Skipped,     Push-Pull,  Digital
    // P0.4  -  Skipped,     Open-Drain, Analog
    // P0.5  -  SDA (SMBus), Open-Drain, Digital
    // P0.6  -  SCL (SMBus), Open-Drain, Digital
    // P0.7  -  Unassigned,  Push-Pull,  Digital

    // P1.0  -  Unassigned,  Push-Pull,  Digital
    // P1.1  -  Unassigned,  Push-Pull,  Digital
    // P1.2  -  Unassigned,  Push-Pull,  Digital
    // P1.3  -  Unassigned,  Push-Pull,  Digital
    // P1.4  -  Unassigned,  Push-Pull,  Digital
    // P1.5  -  Unassigned,  Push-Pull,  Digital
    // P1.6  -  Unassigned,  Push-Pull,  Digital
    // P1.7  -  Unassigned,  Push-Pull,  Digital

    P0MDIN    = 0xEF;
    P0MDOUT   = 0x8D;
    P1MDOUT   = 0xFF;
    P0SKIP    = 0x18;
    XBR0      = 0x06;
    XBR1      = 0x40;
}

void Oscillator_Init()
{
    OSCICN    = 0x83;
}

void Interrupts_Init()
{
    IE        = 0xC2;
    EIE1      = 0x09;
}

// Initialization function for device,
// Call Init_Device() from your main program
void Init_Device(void)
{
    PCA_Init();
    Timer_Init();
    SPI_Init();
    ADC_Init();
    Voltage_Reference_Init();
    Port_IO_Init();
    Oscillator_Init();
    Interrupts_Init();
}
