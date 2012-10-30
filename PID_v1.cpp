/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <Arduino.h>
#include <PID_v1.h>

/**
 * @brief	The parameters specified here are those for for which we can't set up
 *    		reliable defaults, so we need to have the user set them.
 */
PID::PID(
         float& i_rf_consigne,
         float& i_rf_input,
         float& o_rf_command,
         float i_f_Kp,
         float i_f_Ki,
         float i_f_Kd,
         bool i_b_direction)
		: rf_consigne(i_rf_consigne), rf_input(i_rf_input), rf_command(o_rf_command)
{
	b_auto_mode = false;

	// Default output limit corresponds to the arduino pwm limits
	PID::SetOutputLimits(0, 255);

//	// Default Controller Sample Time is 0.1 seconds
//	ui16_sample_time = 100;

	PID::SetControllerDirection(i_b_direction);
	PID::SetTunings(i_f_Kp, i_f_Ki, i_f_Kd);

//	if((millis() + ui16_sample_time) > ui16_sample_time) lastTime = millis() - ui16_sample_time;
}

/**
 * @brief	This, as they say, is where the magic happens.  this function should be called
 *   		every time "void loop()" executes.  the function will decide for itself whether a new
 *   		pid Output needs to be computed.  returns true when the output is computed,
 *   		false when nothing has been done.
 */
bool PID::Compute()
{
	if(!b_auto_mode) return false;

//	uint32_t now = millis();
//	uint32_t timeChange = (now - lastTime);
//	if(timeChange >= ui16_sample_time)
//	{

	/*Compute all the working error variables*/
	float f_input = rf_input;
	float f_error = rf_consigne - f_input;

	// Integrate error
	f_ITerm += (f_ki * f_error);
	if(f_ITerm > f_out_max) f_ITerm = f_out_max;
	else if(f_ITerm < f_out_min) f_ITerm = f_out_min;

	// Derivate erro
	float dInput = (f_input - lastInput);

	// Compute PID Output
	float output = f_kp * f_error + f_ITerm - f_kd * dInput;

	// Check for min/max output
	if(output > f_out_max) output = f_out_max;
	else if(output < f_out_min) output = f_out_min;
	rf_command = output;

	// Release motor control
	if(rf_consigne == 0 && f_input == 0) rf_command = 0;

	// Remember some variables for next time
	lastInput = f_input;
//	lastTime = now;
	return true;

//	}
//	else return false;
}

/**
 * @brief	This function allows the controller's dynamic performance to be adjusted.
 * 			it's called automatically from the constructor, but tunings can also
 * 			be adjusted on the fly during normal operation
 */
void PID::SetTunings(float i_f_Kp, float i_f_Ki, float i_f_Kd)
{
	if(i_f_Kp < 0 || i_f_Ki < 0 || i_f_Kd < 0) return;

	dispKp = i_f_Kp;
	dispKi = i_f_Ki;
	dispKd = i_f_Kd;

//	float f_sample_timeInSec = ((float) ui16_sample_time) / 1000.0;
	f_kp = i_f_Kp;
	f_ki = i_f_Ki;// * f_sample_timeInSec;
	f_kd = i_f_Kd;// / f_sample_timeInSec;

	if(b_direction == REVERSE)
	{
		f_kp = (0 - f_kp);
		f_ki = (0 - f_ki);
		f_kd = (0 - f_kd);
	}
}

///**
// * @brief	sets the period, in Milliseconds, at which the calculation is performed
// */
//void PID::SetSampleTime(uint16_t i_ui16_sample_time)
//{
////	if(i_ui16_sample_time > 0)
////	{
////		float f_ratio = (float) i_ui16_sample_time / (float) ui16_sample_time;
////		f_ki *= f_ratio;
////		f_kd /= f_ratio;
////		ui16_sample_time = i_ui16_sample_time;
////	}
////	ui16_sample_time = i_ui16_sample_time;
//}

/**
 * @brief	This function will be used far more often than SetInputLimits.  while
 *  		the input to the controller will generally be in the 0-1023 range (which is
 *  		the default already,)  the output will be a little different.  maybe they'll
 *  		be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  		want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  		here.
 */
void PID::SetOutputLimits(float i_f_min, float i_f_max)
{
	if(i_f_min >= i_f_max) return;

	f_out_min = i_f_min;
	f_out_max = i_f_max;

	if(b_auto_mode)
	{
		if(rf_command > f_out_max) rf_command = f_out_max;
		else if(rf_command < f_out_min) rf_command = f_out_min;

		if(f_ITerm > f_out_max) f_ITerm = f_out_max;
		else if(f_ITerm < f_out_min) f_ITerm = f_out_min;
	}
}

/**
 * @brief	Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * 			when the transition from manual to auto occurs, the controller is
 * 			automatically initialized
 */
void PID::SetMode(bool i_b_mode)
{
	bool b_new_auto = (i_b_mode == AUTOMATIC);

	if(b_new_auto == !b_auto_mode)
	{ /*we just went from manual to auto*/
		PID::Initialize();
	}
	b_auto_mode = b_new_auto;
}

/**
 * @brief	does all the things that need to happen to ensure a bumpless transfer
 *  		from manual to automatic mode.
 */
void PID::Initialize()
{
	f_ITerm = rf_command;
	lastInput = rf_input;

	if(f_ITerm > f_out_max) f_ITerm = f_out_max;
	else if(f_ITerm < f_out_min) f_ITerm = f_out_min;
}

/**
 * @brief	The PID will either be connected to a DIRECT acting process (+Output leads
 * 			to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * 			know which one, because otherwise we may increase the output when we should
 * 			be decreasing.  This is called from the constructor.
 */
void PID::SetControllerDirection(bool i_b_direction)
{
	if(b_auto_mode && i_b_direction != b_direction)
	{
		f_kp = (0 - f_kp);
		f_ki = (0 - f_ki);
		f_kd = (0 - f_kd);
	}
	b_direction = i_b_direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::GetKp()
{
	return dispKp;
}
float PID::GetKi()
{
	return dispKi;
}
float PID::GetKd()
{
	return dispKd;
}
int PID::GetMode()
{
	return b_auto_mode ? AUTOMATIC : MANUAL;
}
int PID::GetDirection()
{
	return b_direction;
}

