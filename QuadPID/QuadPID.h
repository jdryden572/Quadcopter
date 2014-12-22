/* 
QuadPID.h - PID control library for quadcopter.

*/


#ifndef QuadPID_h
#define QuadPID_h

#include "Arduino.h"

class QuadPID
{
	public: 
		float P, I, D;
		QuadPID(float set_kp, float set_ki, float set_kd, int sampleTime, int maxI);
		float compute(float setPoint, float input);
	private:
		unsigned long _now, _dt, _lastTime;
		float _error, _lastErr, _errSum;
		float _kp, _ki, _kd;
		int _sampleTime, _maxI;
};

#endif