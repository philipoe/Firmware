// logenabler.h
// Allows to switch logging for certain message types on/off via parameters (i.e. via param-file or the ground station)
// Author: Philipp Oe. (ASL, ETHZ)

#include <systemlib/param/param.h>

PARAM_DEFINE_INT32(LOG_EKF, 1);		// Enable logging of EKF parameters (1=yes, 0=no).
PARAM_DEFINE_INT32(LOG_RCIn, 1);	// " ... RC Input channels
PARAM_DEFINE_INT32(LOG_MPPT, 1);	// " ... MPPT messages
PARAM_DEFINE_INT32(LOG_ASLC, 1);
PARAM_DEFINE_INT32(LOG_ASLD, 1);

struct log_params {
	bool LOG_EKF;	//bool is used to save RAM here. The parameters themselves are int32 !
	bool LOG_RCIn;
	bool LOG_MPPT;
	bool LOG_ASLC;
	bool LOG_ASLD;
} pLogEnabler;

struct log_param_handles {
	param_t LOG_EKF;
	param_t LOG_RCIn;
	param_t LOG_MPPT;
	param_t LOG_ASLC;
	param_t LOG_ASLD;
} hLogEnabler;

void GetLogEnablerParams(void)
{
	hLogEnabler.LOG_EKF=param_find("LOG_EKF");
	hLogEnabler.LOG_RCIn=param_find("LOG_RCIn");
	hLogEnabler.LOG_MPPT=param_find("LOG_MPPT");
	hLogEnabler.LOG_ASLC=param_find("LOG_ASLC");
	hLogEnabler.LOG_ASLD=param_find("LOG_ASLD");

	int temp=0; // Need to convert here from int32 -> bool (bool is used to save RAM)
	param_get(hLogEnabler.LOG_EKF, &(temp)); 	pLogEnabler.LOG_EKF=(temp==1 ? true:false);
	param_get(hLogEnabler.LOG_RCIn, &(temp));	pLogEnabler.LOG_RCIn=(temp==1 ? true:false);
	param_get(hLogEnabler.LOG_MPPT, &(temp)); 	pLogEnabler.LOG_MPPT=(temp==1 ? true:false);
	param_get(hLogEnabler.LOG_ASLC, &(temp)); 	pLogEnabler.LOG_ASLC=(temp==1 ? true:false);
	param_get(hLogEnabler.LOG_ASLD, &(temp)); 	pLogEnabler.LOG_ASLD=(temp==1 ? true:false);
}
