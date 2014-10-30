#include"PI.h"
#include"helpfuncs.h"
#include <stdio.h>

PI_Ctrl::PI_Ctrl(void): m_int(0.0f), m_tSample(0.0f)
{
}

PI_Ctrl::PI_Ctrl(float const & PGain, float const & IGain, float const & SatMax, float const & SatMin, float const & IntMax, float const & IntMin, float const & tSample)
	: m_int(0.0f), m_PGain(PGain), m_IGain(IGain), m_SatMax(SatMax), m_SatMin(SatMin), m_tSample(tSample), m_IntMax(IntMax), m_IntMin(IntMin)
{
}
void PI_Ctrl::SetParams(float PGain, float IGain, float SatMax, float SatMin, float IntMax, float IntMin, float tSample)
{
	m_PGain=PGain;
	m_IGain=IGain;
	m_SatMax=SatMax;
	m_SatMin=SatMin;
	m_tSample=tSample;
	m_IntMax=IntMax;
	m_IntMin=IntMin;
}

void PI_Ctrl::SetParams(float PGain, float IGain, float IntMax, float IntMin)
{
	m_PGain=PGain;
	m_IGain=IGain;
	m_IntMax=IntMax;
	m_IntMin=IntMin;
}

void PI_Ctrl::SetPGain(float PGain)
{
	m_PGain=PGain;
}

float PI_Ctrl::step(float const & error)
{
	float output(m_PGain*error+m_IGain*m_int);

	if (output > m_SatMax)
	{
		output = m_SatMax;
		if(error<0.0f) m_int += m_tSample*error;	//Only allow desaturation of m_int
	}
	else if (output < m_SatMin)
	{
		output = m_SatMin;
		if(error>0.0f) m_int += m_tSample*error; //Only allow desaturation of m_int
	}
	else m_int += m_tSample*error;		//Allow both directions

	m_int = limit2(m_int,m_IntMax,m_IntMin);

	return output;
}
