#include "Autopilot.h"




float roll_controller::compute(state_params state)
{
	float controllerOutput;

	if (timer.fire())
	{
		previousError = error;
		error = setpoint - state.roll;

		p_val = constrain(p_component(), -(10 * (long)samplePeriod_ms), 10 * (long)samplePeriod_ms);
		i_val = i_component();
		d_val = d_component();

		controllerOutput = p_val + i_val + d_val;
		controllerOutput = float_constrain(controllerOutput + (outputMin + ((outputMax - outputMin) / 2)), outputMin, outputMax); // add bias (output commands don't have "negative" pulsewidths) and constrain the output

		Serial1.print("Setpoint: "); Serial1.println(setpoint);
		Serial1.print("Current: "); Serial1.println(state.roll);
		Serial1.print("Error: "); Serial1.println(error);
		Serial1.print("P-Value: "); Serial1.println(p_val);
		Serial1.print("Output: "); Serial1.println(controllerOutput);
		Serial1.println();

		status = true;
		return controllerOutput;
	}

	status = false;
	return 0;
}
