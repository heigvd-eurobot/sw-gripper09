
// TI File $Revision: /main/1 $
// Checkin $Date: April 22, 2008   14:35:56 $
//###########################################################################
//
// FILE:   DSP28x_Project.h
//
// TITLE:  DSP28x Project Headerfile and Examples Include File
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x Header Files V1.20 $
// $Release Date: August 1, 2008 $
//###########################################################################

#ifndef DSP28x_PROJECT_H
#define DSP28x_PROJECT_H

#define GLOBAL_Q 16

#include "dsp28xx/DSP280x_Device.h"	  // DSP2833x Headerfile Include File
#include "dsp28xx/DSP280x_Examples.h" // DSP2833x Examples Include File
#include "iq_math/IQmathLib.h"
#include "gpio.h"

#define REMAP_INTERRUPT(A, B) \
	EALLOW;                   \
	PieVectTable.A = B;       \
	EDIS
#define ENABLE_INT_LVL(A) IER |= A
#define DISABLE_INT_LVL(A) IER &= ~A

typedef long long int64;
typedef int bool;

#define true 1
#define false 0

#define PHASE1 GpioDataRegs.GPADAT.bit.GPIO9

struct status_bits
{
	Uint16 ERROR1 : 1;		 // 0 	Error motor 1
	Uint16 OVERLOAD1 : 1;	 // 1 	Depassement courant
	Uint16 END_MOVE1 : 1;	 // 2 	Position atteinte
	Uint16 HOMING_DONE1 : 1; // 3 	Homing done
	Uint16 FREE1 : 1;		 // 4	Free
	Uint16 LED1 : 1;		 // 5 	Led status
	Uint16 NA1 : 2;			 // 6..7

	Uint16 ERROR2 : 1;		 // 8 	Error motor 2
	Uint16 OVERLOAD2 : 1;	 // 9 	Depassement courant
	Uint16 END_MOVE2 : 1;	 // 10 	Position atteinte
	Uint16 HOMING_DONE2 : 1; // 11 	Homing done
	Uint16 FREE2 : 1;		 // 12	Free
	Uint16 LED2 : 1;		 // 13   Led status
	Uint16 NA2 : 2;			 // 14..15
};

union status_reg
{
	Uint16 all;
	struct status_bits bit;
};

struct config_bits
{
	Uint16 ANGLE1 : 16;	 // Angle value (+-32767 = +-180�)
	Uint16 SPEED1 : 16;	 // Speed value ( +32767 = MAX SPEED)
	Uint16 TORQUE1 : 16; // Acc   value ( +32767 = MAX TORQUE)

	Uint16 ANGLE2 : 16;	 // Angle value (+-32767 = +-180�)
	Uint16 SPEED2 : 16;	 // Speed value ( +32767 = MAX SPEED)
	Uint16 TORQUE2 : 16; // Acc   value ( +32767 = MAX TORQUE)

	Uint16 LED_STATE1 : 1;	 // 1 = ON, 0 = OFF
	Uint16 LED_MODE1 : 1;	 // 1 = REMOTE, 0 = AUTO
	Uint16 NA1 : 2;			 // Not assigned
	Uint16 CLEAR_ERROR1 : 1; // Clear status error
	Uint16 STOP_MOVE1 : 1;	 // Stop move
	Uint16 START_MOVE1 : 1;	 // Start move
	Uint16 HOMING1 : 1;		 // Do the homing

	Uint16 LED_STATE2 : 1;	 // 1 = ON, 0 = OFF
	Uint16 LED_MODE2 : 1;	 // 1 = REMOTE, 0 = AUTO
	Uint16 NA2 : 2;			 // Not assigned
	Uint16 CLEAR_ERROR2 : 1; // Clear status error
	Uint16 STOP_MOVE2 : 1;	 // Stop move
	Uint16 START_MOVE2 : 1;	 // Start move
	Uint16 HOMING2 : 1;		 // Do the homing
};

struct config_words
{
	Uint16 ANGLE1 : 16;	 // Angle value (+-32767 = +-180�)
	Uint16 SPEED1 : 16;	 // Speed value ( +32767 = MAX SPEED)
	Uint16 TORQUE1 : 16; // Acc   value ( +32767 = MAX TORQUE)

	Uint16 ANGLE2 : 16;	 // Angle value (+-32767 = +-180�)
	Uint16 SPEED2 : 16;	 // Speed value ( +32767 = MAX SPEED)
	Uint16 TORQUE2 : 16; // Acc   value ( +32767 = MAX TORQUE)

	Uint16 CONFIG : 16; // Drive configuration
};

union config_reg
{
	struct config_words word;
	struct config_bits bit;
};

enum e_mode
{
	OFF,
	OPEN_LOOP,
	TORQUE,
	SPEED,
	POSITION
};

enum e_homing_mode
{
	FORWARD,
	BACKWARD,
	FORWARD_BACKWARD,
	BACKWARD_FORWARD
};

enum e_homing_polarity
{
	ACTIVE_HIGH,
	ACTIVE_LOW
};

enum e_homing_phase
{
	HOMING_REACH_HOME,
	HOMING_STOP_HOME,
	HOMING_FINE_HOME,
	HOMING_FINISH
};

struct controller_parameters
{
	_iq kp;
	_iq gi;
	_iq iaction;
};

struct motor
{
	// Drive state
	enum e_mode mode;
	bool busy;
	bool sync;

	// Bang-bang values
	_iq final_position;

	// Set-points
	_iq value;
	_iq torque;	  // This variable shouldn't be changed directly
	_iq speed;	  // This variable shouldn't be changed directly
	_iq position; // This variable shouldn't be changed directly

	// Controllers
	struct controller_parameters controller_current;
	struct controller_parameters controller_velocity;
	struct controller_parameters controller_position;

	// Constants
	_iq nominal_acceleration;
	_iq nominal_speed;
	_iq maximum_current;
	_iq maximum_speed;
	_iq gear;
	_iq coder;

	// Homing
	char homing_phase; //!< This variable shoudn't be changed manually
	bool homing_run;
	bool homing_done;
	bool homing_error;
	enum e_homing_mode homing_mode;
	enum e_homing_polarity homing_polarity_fw;
	enum e_homing_polarity homing_polarity_bw;
	_iq homing_speed;
	_iq homing_speed_low;
	_iq homing_acceleration;
};

#endif
////////////////////////////////////////////////////////////////////////////////
/// Datatype.
/// For Portability, User is Recommended To Use Following Data Type Size
/// Definitions For 16-bit and 32-Bit Signed/Unsigned Integers
////////////////////////////////////////////////////////////////////////////////
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int boolean;
typedef int bool;
typedef int int1;
typedef int int8;
typedef int int16;
typedef long int32;
typedef long long int64;
typedef unsigned int Uint1;
typedef unsigned int Uint8;
typedef unsigned int Uint16;
typedef unsigned long Uint32;
typedef float float32;
typedef long double float64;

#endif
