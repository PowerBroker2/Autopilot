#pragma once
#include "Arduino.h"




const float GRAVITY = 9.80665;




float float_constrian(float input, float min, float max);




class pitch_controller
{
public:
	bool status = false;




	void begin(float setpoint_,     // Degrees
	           int maxRate_up_,     // Degrees per Sec
	           int maxRate_down_,   // Degrees per Sec
	           float kp_,           // Unitless
	           float ki_,           // Unitless
	           float kd_,           // Unitless
	           float roll_comp_,    // Unitless
	           float sampleRate_);  // Hz

	float compute(float pitchAngle, // Degrees
	              float rollAngle,  // Degrees
	              float airspeed);  // m/s

	float get_P_Component();
	float get_I_Component();
	float get_D_Component();




private:
	const float MAX_SETPOINT = 90.0;
	const float MIN_SETPOINT = -90.0;

	const float MAX_T = 1.0;
	const float MIN_T = 0.4;

	const float MAX_P = 3.0;
	const float MIN_P = 0.1;

	const float MAX_I = 0.5;
	const float MIN_I = 0.0;

	const float MAX_D = 0.2;
	const float MIN_D = 0.0;

	const int MAX_RATE_UP = 100;
	const int MIN_RATE_UP = 0;

	const int MAX_RATE_DN = 100;
	const int MIN_RATE_DN = 0;

	const float MAX_ROLL_COMP = 1.5;
	const float MIN_ROLL_COMP = 0.7;

	const int MAX_I_LIMIT = 4500;
	const int MIN_I_LIMIT = 0;




	unsigned long sampleTimer_previous;
	unsigned long sampleTimer_current;
	unsigned long sample_time_actual;

	unsigned int samplePeriod_ms;
	float samplePeriod_s;
	float setpoint;
	float kp;
	float ki;
	float kd;
	float roll_comp;
	int airspeed_scaler = 1;

	float p_val;
	float i_val;
	float d_val;

	int maxRate_up;
	int maxRate_down;
	float i_limit       = 1500; // Centi-Degrees
	float error         = 0;
	float previousError = 0;
	float summedError   = 0;

	float prevPitchAngle = 0;



	
	float find_kp(float kp_,
	              float ki_,
	              float kd_,
	              float samplePeriod_s);
	float find_ki(float ki_, float samplePeriod_s);
	float omega();
	float roll_compensation(float rollAngle, float airspeed);
	float p_component(float input);
	float i_component(float input);
	float d_component(float input);
};