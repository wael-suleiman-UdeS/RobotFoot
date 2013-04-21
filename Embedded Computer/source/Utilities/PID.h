#ifndef PID_H_
#define PID_H_

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
		PID();
		PID(float Kp, float Ki, float Kd,
			float epsilon, float dt, int max_value,
			int min_value);
		~PID(){};

		float process_PID(float error);

	private:
		float _Kp;
		float _Ki;
		float _Kd;
		float _epsilon;
		float _dt;
		int _max_value;
		int _min_value;
		float _pre_error;
	    float _integral;
};
#endif /*PID_H_*/