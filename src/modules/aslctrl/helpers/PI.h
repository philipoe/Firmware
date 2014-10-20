#ifndef PI_H_
#define PI_H_

class PI_Ctrl
{
public: //TODO TEMP Change back to private; for debugging only.
	float m_int;
	float m_PGain;
	float m_IGain;
	float m_SatMax;
	float m_SatMin;
	float m_tSample;
	float m_IntMax;
	float m_IntMin;
public:
	PI_Ctrl();
	PI_Ctrl(float const & PGain, float const & IGain, float const & SatMax, float const & SatMin, float const & IntMax, float const & IntMin, float const & tSample);

	void SetParams(float PGain, float IGain, float SatMax, float SatMin, float IntMax, float IntMin, float tSample);
	void SetParams(float PGain, float IGain, float IntMax, float IntMin);
	void SetPGain(float PGain);
	float step(float const & error);
};

#endif
