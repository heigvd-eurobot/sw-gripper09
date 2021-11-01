#ifndef __EQEP__
#define __EQEP__

#include "global.h"
// Define the structure of the EQEP Object.
typedef struct
{
	int direction;		 // Motor rotation direction (Q0)
	int index_sync_flag; // Index sync status

	long long position; // Position
	long velocity;		// Velocity
	long acceleration;	// Acceleration
	long revolution;	// Number of revolution

	int delta_pos;

	unsigned long oldposition;
	long long oldposition2;
	long long latched_position;

	volatile struct EQEP_REGS *EQepRegs; // Pointer to EQep registers
} eqep;

typedef eqep *eqep_handle;

#define EQEP1_DEFAULTS                           \
	{                                            \
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, &EQep1Regs \
	}
#define EQEP2_DEFAULTS                           \
	{                                            \
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, &EQep2Regs \
	}

void Init_EQep();
void EQep_process(eqep_handle p);
void EQep_velocity(eqep_handle p);
void EQep_reset_position(eqep_handle p);

#endif // __EQEP__
