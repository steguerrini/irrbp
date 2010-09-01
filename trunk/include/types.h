#ifndef CIrrBP_TYPES_H_
#define CIrrBP_TYPES_H_

//Please note that a number <= 0 is for internal use\deactivation only. So you must use it in your program.
	enum CIB_DFLAG
	{
	NO_DEACTIVATION = 0,
	ON_COLLIDE = 1,
	TIME = 2,
	TR_DEACTIV = 3,
	ON_COLLISION_RELEASE =4,
	};
	enum BODY_OR
	{
		AUTO = -1,
		X = 0,
		Y = 1,
		Z = 2
	};
	
#endif