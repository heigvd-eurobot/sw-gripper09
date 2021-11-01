/**
 * @title   MotionBoard project v2.0
 * @file    main.c
 * @brief   This module allow to control tricolour leds
 * @author  Yves Chevallier <yves.chevallier@kalios.ch>
 * @svn     $Id: main.c 585 2009-02-01 12:59:26Z ychevall $
 */

////////////////////////////////////////////////////////////////////////////////
/// Includes files                                                              
////////////////////////////////////////////////////////////////////////////////
#include "global.h"  
#include "leds.h"
#include "eqep.h"
#include "pwm.h"
#include "adc.h"
#include "bangbang.h"
#include "homing.h" 


////////////////////////////////////////////////////////////////////////////////
/// Global variables.
////////////////////////////////////////////////////////////////////////////////
struct motor motor1; 
struct motor motor2;
union status_reg spi_status; 
union config_reg spi_config; 

struct s_bangbang bb1; 

_iq position_angle = 0; 

////////////////////////////////////////////////////////////////////////////////
/// External functions.
////////////////////////////////////////////////////////////////////////////////
extern void spi_init(); 
extern void spi_clear_buffers();
extern void spi_answer(union status_reg status);
extern Uint16 rdata[8]; 
bool spi_flag = false; 

enum e_leds_mode
{
	LEDS_OFF,
	ALTERNATE,
	BLINK_SLOW,
	BLINK_FAST
} leds_mode; 

eqep eqep1 = EQEP1_DEFAULTS; 

#define IMAX (_IQ16(0.5))
#define IMIN (-IMAX)


////////////////////////////////////////////////////////////////////////////////
/// Regulateur PI générique                                                     
/// @param y : Grandeur mesurée                                                 
/// @param w : Valeur de consigne                                               
/// @param kp : Gain proportionnel                                              
/// @param gi : Gain intégral                                                   
/// @param iaction : buffer intégral (ne pas oublier d're-initialiser)          
/// @return Grandeur réglée                                                     
////////////////////////////////////////////////////////////////////////////////
static inline _iq pi_controller (_iq y, _iq w, _iq kp, _iq gi, _iq *iaction)
{
  _iq e, paction, u; 

  // Error
  e = w-y;                               

  // Integral Action
  *iaction += _IQmpy(gi, e); 
  *iaction = _IQsat((*iaction), _IQ(1), _IQ(-1)); 

  // Proportional Action
  paction = _IQmpy(e, kp);  

  // Result
  u = _IQsat((*iaction) + paction, _IQ(1), _IQ(-1));

  return u;
}


////////////////////////////////////////////////////////////////////////////////
/// Regulateurs de position                                                     
/// @param y_position : Grandeur de position mesurée [1.15]                     
/// @param w_position : Consigne de position [1.15]                             
/// @param bldc : Structure de données du moteur réglé                          
/// @return Grandeur réglée [1.15]                                              
////////////////////////////////////////////////////////////////////////////////
static inline _iq pi_controller_s (_iq y_position, _iq w_position, struct controller_parameters* c)
{
  _iq speed;
  speed = pi_controller (y_position, w_position, c->kp, c->gi, &(c->iaction));
  return speed;
}

////////////////////////////////////////////////////////////////////////////////
/// Speed Position Control
////////////////////////////////////////////////////////////////////////////////.
void speed_position()
{
	// Compute current position and current speed
	EQep_process (&eqep1); 
	EQep_velocity (&eqep1); 

	// Synchronous moves
	if(motor1.sync)
	{
		motor1.sync = 0;
		motor1.busy = 1;
	}

    // Velocity and Position Controller
    if(motor1.mode >= POSITION)
	{
	//	#ifdef _DEBUG
		motor1.value = bangbang(motor1.final_position, &bb1);
	//	#endif
		motor1.speed = pi_controller_s(eqep1.position, motor1.position, &motor1.controller_position); 
	}
    if(motor1.mode >= SPEED)
	{
		motor1.torque = pi_controller_s(eqep1.velocity, motor1.speed, &motor1.controller_velocity)*1; 
	}
}

////////////////////////////////////////////////////////////////////////////////
/// PWM ISR
////////////////////////////////////////////////////////////////////////////////
interrupt void pwm_sync_isr(void)
{
	_iq current1, current2; 
	static char counter = 0;

	// Value affectation
	switch(motor1.mode)
	{
		case OFF:
			motor1.torque = 0;
			motor1.speed = 0;
			motor1.value = 0;
			break;
		case TORQUE: 
			motor1.torque = motor1.value; 
			break;
		case SPEED:
			motor1.speed = motor1.value;
			break;
		case POSITION:
			motor1.position = motor1.value; 
			break;
	}

	// ADC get results
	while(AdcRegs.ADCST.bit.SEQ1_BSY == 1);
	adc_get_values(&current1, &current2);  
	
	// Current Control
	if(motor1.mode >= TORQUE)
	{			
		set_pwm1( 
			pi_controller_s (
				current1, 							// Feedback value (current measured) 
				_IQsat (motor1.torque, motor1.maximum_current, -motor1.maximum_current),   // Set-point
				&motor1.controller_current)/3
			);
	}
	else if(motor1.mode == OPEN_LOOP)
	{
		set_pwm1(motor1.value);
	}
	else
	{
		set_pwm1(0); 
	}

	// Speed + Position Control at 1kHz
	if(counter++ == 24)
	{
		counter = 0;

		homing();  				//!< Homing (if motor1.homing_run do the homing phase)
		speed_position();       //!< Do the speed position loop
	}

	// Service the watchdog
	ServiceDog(); 

	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	EPwm1Regs.ETCLR.bit.INT=1; 
}

////////////////////////////////////////////////////////////////////////////////
/// ADC ISR.                                                                     
////////////////////////////////////////////////////////////////////////////////
interrupt 
void adc_isr (void)
{
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
	AdcRegs.ADCTRL2.bit.RST_SEQ2 = 1; 

 	EPwm1Regs.ETCLR.bit.SOCA = 1;
 	EPwm2Regs.ETCLR.bit.SOCB = 1;

	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the a3950
////////////////////////////////////////////////////////////////////////////////
void init_a3950()
{
	// Enable 	GPIO0
	// Mode 	GPIO1
	// Phase 	GPIO9
	// Sleep	GPIO10
	// nFault	GPIO7
	// Isense 	ADCINA0

	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0; // Gpio is a GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;  // Configured as output

	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0; // Gpio is a GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;  // Configured as output 
	EDIS;  
}

////////////////////////////////////////////////////////////////////////////////
/// Initializations.
////////////////////////////////////////////////////////////////////////////////
void init_all()
{
	// Set mode
	motor1.mode = OFF; 
	motor1.sync = false;

	// SPI answer status
	spi_status.all = 0x0000; 

	// Set controllers parameters
	motor1.controller_current.kp  = _IQ(  0.15259); //   100000
	motor1.controller_current.gi  = _IQ(  0.03052); //     2000
	motor1.controller_velocity.kp = _IQ( 61.40000); // 40000000
	motor1.controller_velocity.gi = _IQ(  0.30520); //    20000
	motor1.controller_position.kp = 500000;//_IQ(  3.00000); //   200000
	motor1.controller_position.gi = 2000;//_IQ(  0.00152); //      100
	motor1.maximum_current        = _IQ(  0.50000); // mA

	// Reset intergral values
	motor1.controller_current.iaction = 0;
	motor1.controller_velocity.iaction = 0;
	motor1.controller_position.iaction = 0;

	motor1.busy = 0;
	motor1.sync = 0;

	motor2.busy = 0;

	motor2.sync = 0;

	// Homing 
	motor1.homing_mode = FORWARD;
	motor1.homing_polarity_bw = ACTIVE_LOW; 
	motor1.homing_polarity_fw = ACTIVE_LOW; 
	motor1.homing_speed = 2000;
	motor1.homing_speed_low = 200;
	motor1.homing_acceleration = 10; 
	motor1.homing_phase = HOMING_REACH_HOME; 
	motor1.homing_done = 0;
	motor1.homing_error = 0;

	motor2.homing_mode = FORWARD;
	motor2.homing_polarity_bw = ACTIVE_LOW; 
	motor2.homing_polarity_fw = ACTIVE_LOW; 
	motor2.homing_speed = 2000;
	motor2.homing_speed_low = 200;
	motor2.homing_acceleration = 10; 
	motor2.homing_phase = HOMING_REACH_HOME; 
	motor2.homing_done = 0;
	motor2.homing_error = 0;

	// Bangbang
	bb1.acceleration = 2; 
	bb1.speed = 0;
	bb1.maximum_speed = 10000; 

	// Clear the two leds
	Led1(0); 
	Led2(0); 
}

////////////////////////////////////////////////////////////////////////////////
/// Timer0 interruption.   
////////////////////////////////////////////////////////////////////////////////
interrupt void cpu_timer0_isr()
{
	static char i = 0; 
	switch(leds_mode)
	{
		case LEDS_OFF:
			Led1(0); 
			Led2(0); 
			break;
		case ALTERNATE:
			if (i++ >= 1) 
			{
				i = 0;  
				Led1(0);
				Led2(1);
			}
			else
			{
				Led1(1);
				Led2(0);
			}
			break;
		case BLINK_SLOW:
			if (i++ >= 1) 
			{
				i = 0;  
				Led1(0);
				Led2(0);
			}
			else
			{
				Led1(1);
				Led2(1);
			}
			break;
	}
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

////////////////////////////////////////////////////////////////////////////////
/// Init spi_communication
////////////////////////////////////////////////////////////////////////////////
void init_spi_communication()
{
	// Reset buffers
	spi_status.all = 0x0000;
	spi_clear_buffers(); 
	spi_answer(spi_status);	

	// Leds are blinking
	leds_mode = ALTERNATE; 

	// Before everything, we wait for a defined spi sequence
	while(!(rdata[0] == 0x55AA && 
			rdata[1] == 0xAFFA && 
			rdata[2] == 0xF0F0 &&
			rdata[3] == 0x0000 && 
			rdata[4] == 0x0000 && 
			rdata[5] == 0x0000 &&
			rdata[6] == 0x0000 && 
			rdata[7] == 0x0000)); 

	// Answer this with status values
	spi_status.all = 0x70C5; 
	spi_answer(spi_status);	

	// Waiting for a all zeros answer
	while(!(rdata[0] == 0x0000 && 
			rdata[1] == 0x0000 && 
			rdata[2] == 0x0000 &&
			rdata[3] == 0x0000 &&
			rdata[4] == 0x0000 && 
			rdata[5] == 0x0000 &&
			rdata[6] == 0x0000 && 
			rdata[7] == 0x0000)); 

	spi_status.all = 0x0000;  
	spi_answer(spi_status);	

	// Turn leds off
	leds_mode = LEDS_OFF; 
}

////////////////////////////////////////////////////////////////////////////////
/// Spi receive            
////////////////////////////////////////////////////////////////////////////////
void spi_receive(void)
{
	// Set points for actuator 1
	motor1.final_position = _IQ14int(_IQ14mpy(_IQ14(spi_config.bit.ANGLE1), _IQ14(1.3125)));
	motor1.speed = (_iq)spi_config.bit.SPEED1; 
	motor1.nominal_acceleration = (_iq)spi_config.bit.TORQUE1;

	// Set points for actuator 2
	motor2.final_position = _IQ14int(_IQ14mpy(_IQ14(spi_config.bit.ANGLE2), _IQ14(1.3125)));
	motor2.speed = (_iq)spi_config.bit.SPEED2; 
	motor2.nominal_acceleration = (_iq)spi_config.bit.TORQUE2;

	// Config for actuator 1
	if(spi_config.bit.CLEAR_ERROR1)
		spi_status.bit.ERROR1 = 0;
	if(spi_config.bit.START_MOVE1)
		motor1.sync = 1;
	if(spi_config.bit.HOMING1)
		motor1.homing_run = 1;

	// Config for actuator 2
	if(spi_config.bit.CLEAR_ERROR2)
		spi_status.bit.ERROR2 = 0;
	if(spi_config.bit.START_MOVE2)
		motor2.sync = 1;
	if(spi_config.bit.HOMING2)
		motor2.homing_run = 1;


	// Status
	spi_status.all = 0x0000;
	spi_status.bit.FREE1 = !motor1.busy;
	spi_status.bit.FREE2 = !motor2.busy;
	spi_status.bit.HOMING_DONE1 = motor1.homing_done;
	spi_status.bit.HOMING_DONE2 = motor2.homing_done;
	spi_status.bit.OVERLOAD1 = 0;
	spi_status.bit.OVERLOAD2 = 0;

	spi_answer(spi_status);	
}

////////////////////////////////////////////////////////////////////////////////
/// Main function
////////////////////////////////////////////////////////////////////////////////
void main(void)
{
	// Main initialisations
	InitSysCtrl();
	InitPieVectTable(); 
	DINT;
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;

	// Peripherals initializations
	InitPieVectTable();
	Init_adc();
	Init_leds(); 
	Init_EQep(); 
	Init_epwm(); 
    InitSpiaGpio();
    InitCpuTimers();   
	init_homing(); 
	spi_init(); 

	// Map ISR routines with correct functions
   	REMAP_INTERRUPT(EPWM1_INT, &pwm_sync_isr);
 	REMAP_INTERRUPT(ADCINT, &adc_isr);
	REMAP_INTERRUPT(TINT0, &cpu_timer0_isr);   

	// Enable interrupts
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1; // Set active EPWM1_INT interrupt  (INT3.x)
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1; // Set active SEQ1_INT  interrupt  (INT1.x)       
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Set active TIMER0_INT interrupt (INT1.x)

	// Enable interrupts levels
	ENABLE_INT_LVL(M_INT3);
	ENABLE_INT_LVL(M_INT1); 	
	
	// Configure timer0 (used for leds blinking)
	ConfigCpuTimer(&CpuTimer0, 100, 100000); // Interrupt is raised every 100 [ms]
	StartCpuTimer0();   

	// Enable the main interrupt flag
	EINT;

	// Software initialisations
	EQep1Regs.QPOSCNT = 0; 
	init_all();
	init_a3950(); 

	#ifndef _DEBUG
	init_spi_communication();
	#endif 

	motor1.mode = OFF; 

	// Main loop
	for(;;)
	{
		#ifndef _DEBUG
		if(spi_flag)
		{
			spi_receive();
			spi_flag = 0;
		}
		#endif
	}
}

////////////////////////////////////////////////////////////////////////////////
/// End of file.
////////////////////////////////////////////////////////////////////////////////

