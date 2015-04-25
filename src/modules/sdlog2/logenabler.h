// logenabler.h
// Allows to switch logging for certain message types on/off via parameters (i.e. via param-file or the ground station)
// Author: Philipp Oe. (ASL, ETHZ)

#include <systemlib/param/param.h>

PARAM_DEFINE_INT32(LOG_EKF, 1);		// Enable logging of EKF parameters (1=yes, 0=no).
PARAM_DEFINE_INT32(LOG_RCIn, 1);	// " ... RC Input channels
PARAM_DEFINE_INT32(LOG_MPPT, 1);	// " ... MPPT messages
PARAM_DEFINE_INT32(LOG_POWS, 1);	// " ... Power messages
PARAM_DEFINE_INT32(LOG_ASLC, 1);
PARAM_DEFINE_INT32(LOG_ASLD, 1);
PARAM_DEFINE_INT32(LOG_BATMON, 1);
PARAM_DEFINE_INT32(LOG_ATMO, 1);

struct log_params {
	bool LOG_EKF;	//bool is used to save RAM here. The parameters themselves are int32 !
	bool LOG_RCIn;
	bool LOG_MPPT;
	bool LOG_POWS;
	bool LOG_ASLC;
	bool LOG_ASLD;
	bool LOG_BATMON;
	bool LOG_ATMO;
} pLogEnabler;

void GetLogEnablerParams(void)
{
	param_t handle;
	int temp=0; // Need to convert here from int32 -> bool (bool is used to save RAM)

	handle = param_find("LOG_EKF");
	param_get(handle, &(temp));
	pLogEnabler.LOG_EKF=(temp==1 ? true:false);

	handle = param_find("LOG_RCIn");
	param_get(handle, &(temp));
	pLogEnabler.LOG_RCIn=(temp==1 ? true:false);

	handle = param_find("LOG_MPPT");
	param_get(handle, &(temp));
	pLogEnabler.LOG_MPPT=(temp==1 ? true:false);

	handle = param_find("LOG_POWS");
	param_get(handle, &(temp));
	pLogEnabler.LOG_POWS=(temp==1 ? true:false);

	handle = param_find("LOG_ASLC");
	param_get(handle, &(temp));
	pLogEnabler.LOG_ASLC=(temp==1 ? true:false);

	handle = param_find("LOG_ASLD");
	param_get(handle, &(temp));
	pLogEnabler.LOG_ASLD=(temp==1 ? true:false);

	handle = param_find("LOG_BATMON");
	param_get(handle, &(temp));
	pLogEnabler.LOG_BATMON=(temp==1 ? true:false);

	handle = param_find("LOG_ATMO");
	param_get(handle, &(temp));
	pLogEnabler.LOG_ATMO=(temp==1 ? true:false);
}
