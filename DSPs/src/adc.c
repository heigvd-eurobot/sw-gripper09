/**
 * @title   MotionBoard project v2.0
 * @file    adc.c
 * @brief   Configure and control analog to digital converter
 * @author  Yves Chevallier <yves.chevallier@kalios.ch>
 * @svn     $Id: adc.c 582 2009-01-25 00:36:10Z ychevall $
 */

////////////////////////////////////////////////////////////////////////////////
/// Includes files                                                              
////////////////////////////////////////////////////////////////////////////////
#include "global.h"
#include "adc.h"       
#include "leds.h"

////////////////////////////////////////////////////////////////////////////////
/// Global variables
////////////////////////////////////////////////////////////////////////////////
int32 offset1 = 996; 		//!< Measured
int32 offset2 = 1070; 		//!< Measured

////////////////////////////////////////////////////////////////////////////////
/// Global constants                                                            
////////////////////////////////////////////////////////////////////////////////
#define ADCINA0 0x0
#define ADCINA1 0x1
#define ADCINA2 0x2
#define ADCINA3 0x3
#define ADCINA4 0x4
#define ADCINA5 0x5
#define ADCINA6 0x6
#define ADCINA7 0x7
#define ADCINB0 0x8
#define ADCINB1 0x9
#define ADCINB2 0xA
#define ADCINB3 0xB
#define ADCINB4 0xC
#define ADCINB5 0xD
#define ADCINB6 0xE
#define ADCINB7 0xF

#define ADC_usDELAY   5000L  // 10ms delay after powered ADC

#define ISENSE1		 ADCINA0 // UA Current
#define ISENSE2		 ADCINA1 // UB Current
#define AGND 		 ADCINB6 // AGND

////////////////////////////////////////////////////////////////////////////////
/// Initialize ADC                                                              
////////////////////////////////////////////////////////////////////////////////
void 
Init_adc(void)
{ 
  extern void DSP28x_usDelay(Uint32 Count);  
  volatile unsigned long i, j; 

  AdcRegs.ADCTRL1.bit.RESET     = 1;  // Reset ADC Converter

  AdcRegs.ADCTRL3.all &= ~0x00E0; 

  DELAY_US(ADC_usDELAY); 

  AdcRegs.ADCREFSEL.bit.REF_SEL = 0;  // Internal reference

  AdcRegs.ADCTRL1.bit.ACQ_PS    = 2;  // Acquisition window size 
  AdcRegs.ADCTRL1.bit.SEQ_CASC  = 1;  // Setup cascaded sequencer mode

//  AdcRegs.ADCTRL3.bit.ADCBGRFDN = 3;  // Bandgap and reference circuitry is powered up
//  AdcRegs.ADCTRL3.bit.ADCPWDN   = 1;  // Analog circuitry inside the core is powered up

  AdcRegs.ADCTRL3.all |= 0x00E0; 

  AdcRegs.ADCTRL3.bit.ADCCLKPS  = 1;  // ADCLK = HISPCLK(SYSCLK/2)/ADCCLKPS/2 = 50e6/4 = 12.5MHz

  DELAY_US(ADC_usDELAY);              // Delay before converting ADC channels 
 
  // Maximum Conversion Channels Register
  AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 4 -(1);
  AdcRegs.ADCMAXCONV.bit.MAX_CONV2 = 0; 

  // ADC Input Channel Sequencing Control Registers (Time A)
  AdcRegs.ADCCHSELSEQ1.bit.CONV00 = ISENSE1;
  AdcRegs.ADCCHSELSEQ1.bit.CONV01 = ISENSE2;
  AdcRegs.ADCCHSELSEQ1.bit.CONV02 = AGND;

  // Enable SOCA trigger and interrupt
  AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1; // Enable SOCA trigger
  //AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2 = 0; // Disable SOCB trigger
  AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 1;  // Enable SOCB trigger

  AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;   // Enable Interrupt
  //AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2 = 1;   // Enable Interrupt

  // Reset Sequencer 
  AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
  AdcRegs.ADCTRL2.bit.RST_SEQ2 = 1;
  AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
  AdcRegs.ADCST.bit.INT_SEQ2_CLR = 1;

}

////////////////////////////////////////////////////////////////////////////////
/// Correction DC offset ADC. 
////////////////////////////////////////////////////////////////////////////////
void 
adc_process_offset_correction()
{
	offset1 = AdcRegs.ADCRESULT0;
	offset2 = AdcRegs.ADCRESULT0;
}

////////////////////////////////////////////////////////////////////////////////
/// Save Results in adc data structure     
/// @param current1 	current in A (max A=2.5) IQ16                                  
////////////////////////////////////////////////////////////////////////////////
void 
adc_get_values(_iq16* current1, _iq16* current2)
{	
	_iq28 i1, i2;
	// Values adaptations
	i1 = (((int32)AdcRegs.ADCRESULT0-offset1)*2)*6; // 6 become 3 when a 0.1R SHUNT is soldered
	i2 = (((int32)AdcRegs.ADCRESULT1-offset2)*2)*6;
	if(i1 < 0) i1 = 0;
	if(i2 < 0) i2 = 0; 

	if(GpioDataRegs.GPADAT.bit.GPIO9 == 0)
	{
		i1 = -i1;
	}
	if(GpioDataRegs.GPADAT.bit.GPIO4 == 0)
	{
		i2 = -i2;
	}

	*current1 = i1;
	*current2 = i2;

	// Offset correction
	if(AdcRegs.ADCRESULT2 > 0)
		AdcRegs.ADCOFFTRIM.bit.OFFSET_TRIM -= 1; 
	else
		AdcRegs.ADCOFFTRIM.bit.OFFSET_TRIM += 1;
}

////////////////////////////////////////////////////////////////////////////////
/// End of file.                                                                
////////////////////////////////////////////////////////////////////////////////

