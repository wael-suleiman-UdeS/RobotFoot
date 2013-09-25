#include <cmath>

#include "PID.h"

PID::PID()
{
	_Kp = 0.1;
	_Ki = 0.01;
	_Kd = 0.005;
	_epsilon = 0.01;
	_dt = 0.01;
	_max_value = 1002;
	_min_value = 21;
}

PID::PID(float Kp, float Ki, float Kd,
		 float epsilon, float dt, int max_value,
		 int min_value)
		 : _Kp(Kp), _Ki(Ki), _Kd(Kd), _epsilon(epsilon),
		   _dt(dt), _max_value(max_value), _min_value(min_value)
{ 
	_pre_error = 0;
	_integral = 0;
}

float PID::process_PID(float error)
{
    float derivative;
    float output;

    // In case of error too small then stop integration
    if(std::abs(error) > _epsilon)
    {
        _integral = _integral + (error * _dt);
    }
    derivative = (error - _pre_error) / _dt;
    output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);

    // Saturation Filter
    //if(output > _max_value)
    //{
    //    output = _max_value;
    //}
    //else if(output < _min_value)
    //{
    //    output = _min_value;
    //}

	// Update error
	_pre_error = error;

    return output;
}

void PID::reset()
{
	_pre_error = 0;
	_integral = 0;
}