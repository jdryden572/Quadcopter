/* 
QuadPID.cpp - PID control library for quadcopter.

*/

#include "Arduino.h"
#include "QuadPID.h"

QuadPID::QuadPID(float set_kp, float set_ki, float set_kd, int sampleTime, int maxI)
{
	_kp = set_kp;
	_ki = set_ki;
	_kd = set_kd;
	_sampleTime = sampleTime;
	_maxI = maxI;
}

float QuadPID::compute(float setPoint, float input)
{
	_now = micros();
	_dt = _now - _lastTime;
	
	if(_dt >= _sampleTime){
		_error = setPoint - input;
		_errSum += _error;
		
		P = _kp * _error;
		
		/*
		When elapsed time is greater than 10 times the sample rate,
		omit the D term which could cause a large output spike. Also 
		reset errSum to zero the I term. This prevents strange behavior
		when the throttle is lowered below cutoff and then raised again. 
		*/
		if(_dt > 10*_sampleTime){
			D = 0;
			_errSum = 0;
		}
		else{
			D = _kd * (_error - _lastErr);
		}
		
		I = constrain(_ki*_errSum, -_maxI, _maxI);
		
		_lastErr = _error;
		_lastTime = _now;
	}
	
	return (P + I + D);
}