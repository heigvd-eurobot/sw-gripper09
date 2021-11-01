/**
 * @title   Gripper Board 2009
 * @file    bangbang.c
 * @brief   This module is the path generator 
 * @author  Yves Chevallier <yves.chevallier@kalios.ch>
 * @svn     $Id: main.c 585 2009-02-01 12:59:26Z ychevall $
 */

////////////////////////////////////////////////////////////////////////////////
/// Includes files                                                              
////////////////////////////////////////////////////////////////////////////////
#include "global.h"
#include "bangbang.h"

////////////////////////////////////////////////////////////////////////////////
/// Main bangbang                                                               
////////////////////////////////////////////////////////////////////////////////
_iq bangbang(_iq new_position,  struct s_bangbang * bb)
{
	// Internal variables
	int64 new_position_norm = ((int64)new_position)*256;
	int64 left_position = -(bb->position - new_position_norm); 

	// Phase 0: Need to do something?
	if(new_position_norm == bb->position)
	{
		bb->running = false;
		return 0; // Delta position = 0 
	}

	// Phase 1: Acceleration
	bb->speed += bb->acceleration; 

	// Phase 2: Constant speed
	if(bb->speed > bb->maximum_speed)
		bb->speed -= bb->acceleration; 

	// Phase 3: Deceleration	
	if((((int64)(left_position))*(int64)(2*bb->acceleration)) <= (int64)bb->speed*(int64)(bb->speed+bb->acceleration))
		bb->speed -= 2*bb->acceleration;

	bb->position += bb->speed; 
	return bb->position/256;
}
