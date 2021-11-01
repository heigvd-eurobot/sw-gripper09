#include "global.h" 
#include "eqep.h"
#include "bangbang.h" 
#include "homing.h" 

extern struct motor motor1;
extern eqep eqep1; 
extern struct s_bangbang bb1;

// Initialize homing GPIO.
void init_homing()
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0; // Gpio is a GPIO
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0; // Gpio is a GPIO
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0; // Gpio is a GPIO
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0; // Gpio is a GPIO
	EDIS;  	
}

// PWM ISR
void homing()
{
	// Homing
	if(motor1.homing_run && !motor1.homing_done)
	{
		motor1.mode = SPEED;
		if(motor1.homing_mode == FORWARD)
		{
			switch(motor1.homing_phase)
			{
				case HOMING_REACH_HOME:
					motor1.value = (motor1.speed > motor1.homing_speed)?motor1.homing_speed:(motor1.speed+motor1.homing_acceleration);
					if(HOMING_FW_1 == 0)
					{
						motor1.homing_phase++;
					}
					break;
				case HOMING_STOP_HOME:
					if(motor1.speed <= 0)
					{
						motor1.value = 0;
						motor1.homing_phase++;
					}
					else
					{
						motor1.value = motor1.speed-motor1.homing_acceleration;
					}
					break;
				case HOMING_FINE_HOME:
					motor1.value = (motor1.speed < -motor1.homing_speed_low)?-motor1.homing_speed_low:(motor1.speed-motor1.homing_acceleration);
					if(HOMING_FW_1 == 1)
					{
						motor1.homing_phase++;
					}					
					break;
				case HOMING_FINISH:
					if(motor1.speed >= 0)
					{
						motor1.speed = 0; 
						motor1.value = 0;
						motor1.homing_phase = 0;
						motor1.homing_run = 0;
						motor1.homing_done = 1;
						eqep1.position = 0; 
						motor1.position = 0; // Reset eqep_position
						motor1.value = 0; 
						motor1.final_position = 0; 
						bb1.position = 0; 
						EQep_reset_position(&eqep1); 
					}
					else
					{
						motor1.value = motor1.speed+motor1.homing_acceleration;
					}
					break;					
			}

		}
	}
}

