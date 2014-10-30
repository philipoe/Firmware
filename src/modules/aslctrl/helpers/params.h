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

struct control_param_handles {

	//--- ASLCTRL GENERAL --------------------
	param_t ASLC_DEBUG;
	param_t ASLC_CtrlType;
	param_t ASLC_GainSch_E;
	param_t ASLC_GainSch_Q;
	param_t ASLC_StallProt;
	param_t ASLC_VelCtrl;
	param_t ASLC_OnRCLoss;
	param_t ASLC_OvSpdProt;
	param_t ASLC_CoordTurn;

	//--- DEMIXER --------------------
	param_t DEMIX_Enabled;
	param_t DEMIX_F_CH2t4;
	param_t DEMIX_F_CH6t1;
	param_t DEMIX_CH2t4;
	param_t DEMIX_CH6t1;

	//--- SAS --------------------
	//Sampling time
	param_t SAS_tSample;
	//Gains
	param_t SAS_RollPGain;
	param_t SAS_PitchPGain;
	param_t SAS_YawPGain;
	param_t SAS_RollPDir;
	param_t SAS_PitchPDir;
	param_t SAS_YawPDir;
	param_t SAS_RollYawDecoupleKari;
	param_t SAS_YawCTkP;
	param_t SAS_YawCTFF;

	//Limiters
	param_t SAS_RCtrlLim;
	param_t SAS_PCtrlLim;
	param_t SAS_YCtrlLim;
	//Filters
	param_t SAS_YawHighPassOmega;
	param_t SAS_PitchLowPassOmega;
	param_t SAS_RollLowPassOmega;
	//Dynamic Pressure Scaling
	param_t SAS_vScaleLimF;
	param_t SAS_vScaleExp;
	//Trim Values
	param_t SAS_TrimAilvNom;
	param_t SAS_TrimAilvMin;
	param_t SAS_TrimAilvMax;
	param_t SAS_TrimElevNom;
	param_t SAS_TrimElevMin;
	param_t SAS_TrimElevMax;
	param_t SAS_TrimRudvNom;
	param_t SAS_TrimRudvMin;
	param_t SAS_TrimRudvMax;

	//---CAS/AP ----------------------
	//Sampling time
	param_t CAS_tSample;
	param_t CAS_fMult;
	//Gains
	param_t CAS_PitchPGain;
	param_t CAS_PitchPGainM;
	param_t CAS_PitchIGain;
	param_t CAS_PitchTCkI;
	param_t CAS_PitchTCILim;
	param_t CAS_RollPGain;
	param_t CAS_RollPGainM;
	param_t CAS_HeadPGain;
	param_t CAS_q2uPGain;
	param_t CAS_p2uPGain;
	//Limiters
	param_t CAS_PitchRateLim;
	param_t CAS_PitchRateILim;
	param_t CAS_RollRateLim;
	param_t CAS_YawRateLim;
	param_t CAS_PitchAngleLim;
	param_t CAS_RollAngleLim;
	//Coordinated Turn feed-forward gains
	param_t CAS_uElevTurnFF;
	//Filters
	param_t CAS_YawLowPassOmega;

	//---WP following ------------------
	param_t HL_fMult;
	param_t HL_WPL1_Damping;
	param_t HL_WPL1_P_vMin;
	param_t HL_WPL1_P_vNom;
	param_t HL_WPL1_P_vMax;
	param_t HL_Vel_vNom;
	param_t HL_Vel_vMin;
	param_t HL_Vel_vMax;
	//Old altitude controller
	param_t HL_vZClimb;
	param_t HL_vZSink;
	param_t HL_AltLowPassOmega;
	param_t HL_AlthMax;
	param_t HL_AlthMin;

	//---TECS ---------------------
	param_t time_const;
	param_t time_const_throt;
	param_t min_sink_rate;
	param_t max_sink_rate;
	param_t max_climb_rate;
	param_t throttle_damp;
	param_t integrator_gain;
	param_t vertical_accel_limit;
	param_t height_comp_filter_omega;
	param_t speed_comp_filter_omega;
	param_t roll_throttle_compensation;
	param_t speed_weight;
	param_t pitch_damping;
	param_t airspeed_min;
	param_t airspeed_trim;
	param_t airspeed_max;
	param_t pitch_limit_min;
	param_t pitch_limit_max;
	param_t throttle_min;
	param_t throttle_max;
	param_t throttle_cruise;
	param_t heightrate_p;
	param_t heightrate_ff;
	param_t speedrate_p;
	param_t throttle_slewrate;
	param_t throttle_ILim;

};

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
	struct control_param_handles h;		//handles
	bool bInitialized;
	bool bSanityChecksEnabled;
};

#endif /*PARAMS_H*/
