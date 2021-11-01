
#include "global.h"

#ifndef __bangbang__
#define __bangbang__

struct s_bangbang
{
	_iq speed; 
	_iq acceleration;
	_iq maximum_speed;
	long long position; 
	bool running;
	bool direction; 
};

_iq bangbang(_iq new_position,  struct s_bangbang * bb);

#endif

////////////////////////////////////////////////////////////////////////////////
/// End of file.                                                                
////////////////////////////////////////////////////////////////////////////////

