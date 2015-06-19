#ifndef FILTERS_H
#define FILTERS_H

//***********************************************************
// HIGH PASS FILTER
//***********************************************************
class HighPass
{
	public:	//TODO Change back to private
		float m_oldValue;
		float m_gain;

	public:
		HighPass(float const tSample, float const omega);
		~HighPass(void);
		float update(float const input);
		int SetGain(float const tSample, float const omega);
		float Get(void) {return m_oldValue;};
};

//***********************************************************
// LOW PASS FILTER
//***********************************************************
class LowPass
{
	private:
		float m_oldOutput;
		float m_gain;
		float m_gainInput;

	public:
		LowPass(float const tSample, float const omega);
		float update(float const input);
		int SetGains(float const tSample, float const omega);
		float Get(void) {return m_oldOutput;};
		void Set(float newFilteredValue) {m_oldOutput=newFilteredValue;};
};

//***********************************************************
// MOVING AVERAGE FILTER
//***********************************************************
class MovingAverage
{
	public:
		MovingAverage(int initialvalue, int _order);
		~MovingAverage(void);
		float update(float const input);
		void Set(float newValue) {m_oldValue=newValue;}
		float Get(void) {return m_oldValue;};

	private:
		int order;
		float m_oldValue;
		float divider;	//Only for filter processing power improvement
};
#endif
		
