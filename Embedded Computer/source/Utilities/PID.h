#ifndef PID_H_
#define PID_H_
#include <math.h>

// Define parameter
//#define epsilon 0.01
//#define dt 0.01             // 100ms loop time
//#define MAX  4              // For Current Saturation
//#define MIN -4
//#define Kp  0.1
//#define Kd  0.01
//#define Ki  0.005

class PID
{
	public:
		PID(unsigned int Kp, unsigned int Ki, unsigned int Kd,
			unsigned int epsilon, unsigned int dt, int max_value,
			int min_value);
		~PID();

		float process_PID(float setpoint, float actual_position);

	private:
		unsigned int _Kp;
		unsigned int _Ki;
		unsigned int _Kd;
		unsigned int _epsilon;
		unsigned int _dt;
		int _max_value;
		int _min_value;
		float _pre_error;
	    float _integral;
};
#endif /*PID_H_*/