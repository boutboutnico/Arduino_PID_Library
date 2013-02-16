/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

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
	b_auto_mode = PID_AUTOMATIC;
	b_direction = i_b_direction;

	f_sum_error = 0;
	f_last_error = 0;

	// Default output limit corresponds to the Arduino PWM limits
	PID::SetOutputLimits(0, 255);

	PID::SetTunings(i_f_Kp, i_f_Ki, i_f_Kd);
}

/**
 * @brief	This, as they say, is where the magic happens.  this function should be called
 *   		every time "void loop()" executes.  the function will decide for itself whether a new
 *   		pid Output needs to be computed.  returns true when the output is computed,
 *   		false when nothing has been done.
 */
bool PID::Compute()
{
//	if(!b_auto_mode) return false;
//
//	/*Compute all the working error variables*/
//	float f_input = rf_input;
//	float f_error = rf_consigne - f_input;
//
//	// Integrate error
//	f_ITerm += (f_ki * f_error);
//	if(f_ITerm > f_out_max) f_ITerm = f_out_max;
//	else if(f_ITerm < f_out_min) f_ITerm = f_out_min;
//
//	// Derivate erro
//	float dInput = (f_input - f_last_input);
//
//	// Compute PID Output
//	float output = f_kp * f_error + f_ITerm - f_kd * dInput;
//
//	// Check for min/max output
//	if(output > f_out_max) output = f_out_max;
//	else if(output < f_out_min) output = f_out_min;
//	rf_command = output;
//
//	// Release motor control
//	if(rf_consigne == 0 && f_input == 0) rf_command = 0;
//
//	// Remember some variables for next time
//	f_last_input = f_input;
//	return true;

// Calcul error
	float f_error = (rf_consigne - rf_input);

	// Integrate error
	f_sum_error += f_error;

	// Compute PID Output
	float f_command = f_kp * f_error + f_ki * f_sum_error + f_kd * (f_error - f_last_error);

	// Check for min/max output
	if(f_command > f_out_max) f_command = f_out_max;
	else if(f_command < f_out_min) f_command = f_out_min;
	rf_command = f_command;

	// Release motor control
	if(rf_consigne == 0 && rf_input == 0) rf_command = 0;

	// Remember values
	f_last_error = f_error;

	return true;
}

/**
 * @brief	This function allows the controller's dynamic performance to be adjusted.
 * 			it's called automatically from the constructor, but tunings can also
 * 			be adjusted on the fly during normal operation
 */
void PID::SetTunings(float i_f_Kp, float i_f_Ki, float i_f_Kd)
{
	if(i_f_Kp < 0 || i_f_Ki < 0 || i_f_Kd < 0) return;

	f_kp = i_f_Kp;
	f_ki = i_f_Ki;
	f_kd = i_f_Kd;

	if(b_direction == PID_REVERSE)
	{
		f_kp = (0 - f_kp);
		f_ki = (0 - f_ki);
		f_kd = (0 - f_kd);
	}
}

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
	bool b_new_auto = (i_b_mode == PID_AUTOMATIC);

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
	f_last_input = rf_input;

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

/**
 *  Status Funcions
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 */
float PID::GetKp()
{
	return f_kp;
}
float PID::GetKi()
{
	return f_ki;
}
float PID::GetKd()
{
	return f_kd;
}
int PID::GetMode()
{
	return b_auto_mode ? PID_AUTOMATIC : PID_MANUAL;
}
int PID::GetDirection()
{
	return b_direction;
}

#ifdef __PID_FRONT_END
void PID::SerialReceive()
{

	// read the bytes sent from Processing
	int index = 0;
	byte Auto_Man = -1;
	byte Direct_Reverse = -1;
	while(Serial.available() && index < 26)
	{
		if(index == 0) Auto_Man = Serial.read();
		else if(index == 1) Direct_Reverse = Serial.read();
		else foo.asBytes[index - 2] = Serial.read();
		index++;
	}

	// if the information we got was in the correct format,
	// read it into the system
	if(index == 26 && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
	{
		rf_consigne = double(foo.asFloat[0]);
		//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
		//   value of "Input"  in most cases (as
		//   in this one) this is not needed.
		if(Auto_Man == 0)// * only change the output if we are in
		{                      //   manual mode.  otherwise we'll get an
			rf_command = double(foo.asFloat[2]);//   output blip, then the controller will
		}                                     //   overwrite.

		double p, i, d;// * read in and set the controller tunings
		p = double(foo.asFloat[3]);//
		i = double(foo.asFloat[4]);//
		d = double(foo.asFloat[5]);//
		SetTunings(p, i, d);//

		if(Auto_Man == 0) SetMode(PID_MANUAL);// * set the controller mode
		else SetMode(PID_AUTOMATIC);//

//		if(Direct_Reverse == 0) myPID.SetControllerDirection(DIRECT); // * set the controller Direction
//		else myPID.SetControllerDirection(REVERSE);          //
	}
	Serial.flush();            // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void PID::SerialSend()
{
	Serial.print("PID ");
	Serial.print(rf_consigne);
	Serial.print(" ");
	Serial.print(rf_input);
	Serial.print(" ");
	Serial.print(rf_command);
	Serial.print(" ");

	Serial.print(f_kp);
	Serial.print(" ");
	Serial.print(f_ki);
	Serial.print(" ");
	Serial.print(f_kd);
	Serial.print(" ");

	if(GetMode() == PID_AUTOMATIC) Serial.print("Automatic");
	else Serial.print("Manual");
	Serial.print(" ");

	if(GetDirection() == PID_DIRECT) Serial.println("Direct");
	else Serial.println("Reverse");
}
#endif

