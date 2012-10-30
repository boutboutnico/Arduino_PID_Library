#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0

class PID
{

public:

	//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

	//commonly used functions **************************************************************************

	// Setpoint.  Initial tuning parameters are also set here
	// constructor.  links the PID to the Input, Output, and
	PID(
	    float& i_rf_consigne,
	    float& i_rf_input,
	    float& o_rf_command,
	    float i_f_Kp,
	    float i_f_Ki,
	    float i_f_Kd,
	    bool i_b_direction = DIRECT);

	// * sets PID to either Manual (0) or Auto (non-0)
	void SetMode(bool i_b_mode);

	// * performs the PID calculation.  it should be
	//   called every time loop() cycles. ON/OFF and
	//   calculation frequency can be set using SetMode
	//   SetSampleTime respectively
	bool Compute();

	//clamps the output to a specific range. 0-255 by default, but
	//it's likely the user will want to change this depending on
	//the application
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

	// * Sets the Direction, or "Action" of the controller. DIRECT
	//   means the output will increase when error is positive. REVERSE
	//   means the opposite.  it's very unlikely that this will be needed
	//   once it is set in the constructor.
	void SetControllerDirection(bool i_b_direction);

	// * sets the frequency, in Milliseconds, with which
	//   the PID calculation is performed.  default is 100
	void SetSampleTime(uint16_t i_ui16_sample_time);

	//Display functions ****************************************************************
	float GetKp();						  // These functions query the pid for interal values.
	float GetKi();						  //  they were created mainly for the pid front-end,
	float GetKd();						  // where it's important to know what is actually
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

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

	boolean b_auto_mode;

	uint16_t ui16_sample_time;

	float dispKp;				// * we'll hold on to the tuning parameters in user-entered
	float dispKi;				//   format for display purposes
	float dispKd;				//

	bool b_direction;

	unsigned long lastTime;
	float ITerm, lastInput;

	float f_out_min, f_out_max;

};
#endif

