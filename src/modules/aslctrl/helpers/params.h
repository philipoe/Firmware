#ifndef PARAMS_H
#define PARAMS_H


#include "uORB/topics/aslctrl_parameters.h"

#include <systemlib/param/param.h>

//**********************************************************************
//*** PX4 Parameter defs
//**********************************************************************

// The parameters are defined in params.c. Parameter descriptions can be found there.
// Would like to define the parameters here, but macros for the parameter definition cannot be used in c++ files.


//**********************************************************************
//*** ATTENTION: Actual PARAMETERS are defined within uOrb/topics/...
//**********************************************************************

//**********************************************************************
//*** Main parameter class and functionality
//**********************************************************************
class parameters
{
public:
	parameters(void);
	~parameters(void);

	int init();
	int update();
	int Sanitize(param_t & parameter_handle, float minval, float maxval);

public:
	struct aslctrl_parameters_s p;			//parameters

private:
	bool bInitialized;
	bool bSanityChecksEnabled;
};

#endif /*PARAMS_H*/
