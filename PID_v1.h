#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/
#include <Arduino.h>

/*****************************************************************************
 * DEFINITION
 *****************************************************************************/
#define PID_AUTOMATIC	1
#define PID_MANUAL	0
#define PID_DIRECT  0
#define PID_REVERSE  1

#define __PID_FRONT_END

/*****************************************************************************
 * CLASS
 *****************************************************************************/
/**
 * @class 	xx
 * @brief	xx
 * @author	boutboutnico
 * @date	25 juil. 2012
 */
class PID
{
public:
	/**
	 * @brief	Setpoint.  Initial tuning parameters are also set here
	 * 			constructor.  links the PID to the Input, Output
	 */
	PID(
	    float& i_rf_consigne,
	    float& i_rf_input,
	    float& o_rf_command,
	    float i_f_Kp,
	    float i_f_Ki,
	    float i_f_Kd,
	    bool i_b_direction = PID_DIRECT);

	/**
	 * @brief	sets PID to either Manual (0) or Auto (non-0)
	 */
	void SetMode(bool i_b_mode);

	/**
	 * @brief	performs the PID calculation.  it should be
	 * 			called every time loop() cycles. ON/OFF and
	 * 			calculation frequency can be set using SetMode
	 * 			SetSampleTime respectively
	 */
	bool Compute();

	/**
	 * @brief	clamps the output to a specific range. 0-255 by default, but
	 *			it's likely the user will want to change this depending on
	 *			the application
	 */
	void SetOutputLimits(float i_f_min, float i_f_max);

	/**
	 * @brief	available but not commonly used functions
	 *   		constructor, this function gives the user the option
	 *   		of changing tunings during runtime for Adaptive control
	 * @param i_f_Kp
	 * @param i_f_Ki
	 * @param i_f_Kd
	 */
	void SetTunings(float i_f_Kp, float i_f_Ki, float i_f_Kd);

	/**
	 * @brief	Sets the Direction, or "Action" of the controller. DIRECT
	 *   		means the output will increase when error is positive. REVERSE
	 *   		means the opposite.  it's very unlikely that this will be needed
	 *   		once it is set in the constructor.
	 */
	void SetControllerDirection(bool i_b_direction);

	float GetKp();
	float GetKi();
	float GetKd();
	int GetMode();
	int GetDirection();

#ifdef __PID_FRONT_END
	void SerialReceive();
	void SerialSend();
#endif

private:
	void Initialize();

	/**
	 * @brief	References to the rf_input, rf_command, and rf_consigne variables
	 *			This creates a hard link between the variables and the
	 *			PID, freeing the user from having to constantly tell us
	 *			what these values are.  with pointers we'll just know.
	 */
	float& rf_consigne;
	float& rf_input;
	float& rf_command;

	float f_kp;                  // * (P)roportional Tuning Parameter
	float f_ki;                  // * (I)ntegral Tuning Parameter
	float f_kd;                  // * (D)erivative Tuning Parameter

	float f_sum_error;
	float f_last_error;

	boolean b_auto_mode;

	bool b_direction;

	float f_ITerm;
	float f_last_input;

	float f_out_min, f_out_max;

#ifdef __PID_FRONT_END
	union
	{                // This Data structure lets
		byte asBytes[24];// us take the byte array
		float asFloat[6];// sent from processing and
	}                      // easily convert it to a
	foo;// float array
#endif

};
#endif

