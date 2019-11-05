#include "Autopilot.h"




float float_constrian(float input, float min, float max)
{
	if (input > max)
		return max;
	else if (input < min)
		return min;
	else
		return input;
}




void pitch_controller::begin(float setpoint_,
                             int maxRate_up_,
                             int maxRate_down_,
                             float kp_,
                             float ki_,
                             float kd_,
                             float roll_comp_,
                             float sampleRate_)
{
	update(setpoint_,
	       maxRate_up_,
	       maxRate_down_,
	       kp_,
	       ki_,
	       kd_,
	       roll_comp_,
	       sampleRate_);
}




void pitch_controller::update(float setpoint_,
                              int maxRate_up_,
                              int maxRate_down_,
                              float kp_,
                              float ki_,
                              float kd_,
                              float roll_comp_,
                              float sampleRate_)
{
	setpoint        = float_constrian(setpoint_, MIN_SETPOINT, MAX_SETPOINT);
	maxRate_up      = float_constrian(maxRate_up_, MIN_RATE_UP, MAX_RATE_UP);
	maxRate_down    = float_constrian(maxRate_down_, MIN_RATE_DN, MAX_RATE_DN);
	samplePeriod_s  = float_constrian((1 / sampleRate_), MIN_T, MAX_T);
	samplePeriod_ms = 1000 * samplePeriod_s;
	kp              = float_constrian(find_kp(kp_, ki_, kd_, samplePeriod_s), MIN_P, MAX_P);
	ki              = float_constrian(find_kp(ki_, samplePeriod_s), MIN_I, MAX_I);
	kd              = float_constrian(kd_, MIN_D, MAX_D);
	roll_comp       = float_constrian(roll_comp_, MIN_ROLL_COMP, MAX_ROLL_COMP);

	sampleTimer_current  = millis();
	sampleTimer_previous = sampleTimer_current;
}




float pitch_controller::compute(float pitchAngle, // Degrees
                                float rollAngle,  // Degrees
                                float airspeed)   // m/s
{
	float limited_err;
	float roll_bias;
	float pitch_rate; // Degrees per Sec
	float rate_adj;
	float controller_output;

	sampleTimer_current = millis();
	sample_time_actual = sampleTimer_current - sampleTimer_previous;

	if (sample_time >= samplePeriod_ms)
	{
		sampleTimer_previous += samplePeriod_ms;
		
		previousError = error;
		error         = setpoint - pitchAngle;

		limited_err     = float_constrian(error * omega(), maxRate_down, maxRate_up);
		roll_bias       = roll_compensation(rollAngle, airspeed);
		biasedError     = limited_err + roll_bias;
		pitch_rate      = (pitchAngle - prevPitchAngle) * (sample_time_actual / 1000); // Degrees per Sec
		rate_adj        = biasedError + pitch_rate;
		airspeed_scaler = find_as_scaler(airspeed);
		previous_scaler = scaler_output;
		scaler_output   = airspeed_scaler * rate_adj;

		p_val = p_component();
		i_val = i_component();
		d_val = d_component();
		
		controller_output = (p_val + i_val + d_val) * airspeed_scaler;

		status = true;
		return constrain(controller_output, servoMin, servoMax);
	}

	status = false;
	return 0;
}




float pitch_controller::find_kp(float kp_,
                                float ki_,
                                float kd_,
                                float samplePeriod)
{
	return (kp_ - (ki_ * samplePeriod)) * (samplePeriod - kd_);
}




float pitch_controller::find_ki(float ki_, float samplePeriod)
{
	return ki_ * samplePeriod;
}




float pitch_controller::omega()
{
	return 1000 / samplePeriod;
}




float pitch_controller::roll_compensation(float rollAngle, float airspeed)
{
	float rollAngle_rad = (M_PI / 180) * rollAngle;

	return (roll_comp * GRAVITY) / (airspeed * tan(rollAngle_rad) * sin(rollAngle_rad));
}




float pitch_controller::find_as_scaler(float airspeed)
{
	if (airspeed >= AIRSPEED_THRESHOLD)
		return 1 / airspeed;
	else
		return 2;
}




float pitch_controller::p_component()
{
	return kp * biasedError;
}




float pitch_controller::i_component()
{
	summedError += ((error - previousError) / 2) * (sample_time_actual / 1000);
	summedError  = float_constrian(summedError, -i_limit, i_limit);

	return ki * summedError;
}




float pitch_controller::d_component()
{
	previous_scaler
	scaler_output
	
	return kd;
}