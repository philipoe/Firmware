#include "Filters.h"

//***********************************************************
// HIGH PASS FILTER
//***********************************************************
HighPass::HighPass(float const tSample, float const omega)
	: m_oldValue(0), m_gain(1/(1+tSample*omega))
{
}

HighPass::~HighPass(void)
{
}

float HighPass::update(float const input)
{
	float const output(m_gain*(input + m_oldValue));
	m_oldValue = output-input;
	return output;
}

int HighPass::SetGain(float const tSample, float const omega)
{
	m_gain=1.0f/(1.0f+tSample*omega);
	return 0;
}

//***********************************************************
// LOW PASS FILTER
//***********************************************************
LowPass::LowPass(float const tSample, float const omega)
	: m_oldOutput(0), m_gain(1/(1+tSample*omega)), m_gainInput(tSample*omega)
{
}

float LowPass::update(float const input)
{
	float const output(m_gain*(m_oldOutput+m_gainInput*input));
	m_oldOutput = output;
	return output;
}

int LowPass::SetGains(float const tSample, float const omega)
{
	m_gain=1.0f/(1.0f+tSample*omega);
	m_gainInput=tSample*omega;

	return 0;
}

//***********************************************************
// MOVING AVERAGE FILTER
//***********************************************************
MovingAverage::MovingAverage(int initialvalue, int _order)
	: order(_order), m_oldValue(initialvalue)
{
		divider=1.0f/(1.0f+order);
}

MovingAverage::~MovingAverage(void)
{
}

float MovingAverage::update(float const input)
{
	m_oldValue=(m_oldValue*order+input)*divider;
	return m_oldValue;
}
