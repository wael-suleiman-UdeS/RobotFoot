#include "PID.h"

PID::PID(unsigned int Kp, unsigned int Ki, unsigned int Kd,
		 unsigned int epsilon, unsigned int dt, int max_value,
		 int min_value)
		 : _Kp(Kp), _Ki(Ki), _Kd(Kd), _epsilon(epsilon),
		   _dt(dt), _max_value(max_value), _min_value(min_value)
{ 
	_pre_error = 0;
	_integral = 0;
}

float PID::process_PID(float setpoint, float actual_position)
{
    float error;
    float derivative;
    float output;

    // Caculate P,I,D
    error = setpoint - actual_position;

    // In case of error too small then stop integration
    if(abs(error) > _epsilon)
    {
        _integral = _integral + (error * _dt);
    }
    derivative = (error - _pre_error) / _dt;
    output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);

    // Saturation Filter
    if(output > _max_value)
    {
        output = _max_value;
    }
    else if(output < _min_value)
    {
        output = _min_value;
    }

	// Update error
	_pre_error = error;

    return output;
}
