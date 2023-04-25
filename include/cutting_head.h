#pragma once
#include "arduino.h"

#define PISTON_MIN_POS				102
#define PISTON_MAX_POS				152
#define	PISTON_ANALOG_MIN			0		//experimental value (tested reading the piston position pin at min extension)
#define	PISTON_ANALOG_MAX			1024	//experimental value (tested reading the piston position pin at max extension)
#define PISTON_SPEED_UPDATE_TIME	100		//time in miliseconds between speed measurement of the piston.
#define PISTON_CALIBRATE_TIME 		3000	//amount of ticks that the analog read value must remain constant to consider the piston static
#define	PISTON_P_AREA				5		//size of the area where the piston works in linear mode when on POS_MODE state
#define	PISTON_TOLERANCE			1		//size of the area where the piston works in linear mode when on POS_MODE state
#define	PISTON_MAX_PWM				255		//size of the area where the piston works in linear mode when on POS_MODE state
#define	PISTON_MIN_PWM				150		//size of the area where the piston works in linear mode when on POS_MODE state

#define MOTOR_RPM_UPDATE_TIME 500
#define MOTOR_GEAR_RATIO	99
#define MOTOR_CPR			12

#define CH_BASE1	53.2
#define CH_BASE2	45
#define CH_LONG1	127
#define CH_LONG2	120

class cutting_head;

class piston{
private:
	//piston mechanical values
	float			target_pos;						//Position where the piston is moving to [units].
	float			current_pos;					//Current position of the piston [units].
	float			current_speed;					//Current speed of the piston [units/second].
	//pos controller variables
	unsigned char	max_pwm		= PISTON_MAX_PWM;	//Max pwm value that will be set to the piston.
	unsigned char	min_pwm		= PISTON_MIN_PWM;	//Min pwm value that allows the piston to move.
	float			P_area		= PISTON_P_AREA;	//Distance around the target position where the piston will move proportionally [units].
	float			tolerance	= PISTON_TOLERANCE;	//Allowed error between current_pos and target_pos [units].
	//piston characteristic values
	float			min_pos;						//Length of the piston when completely expanded [units].
	float			max_pos;						//Length of the piston when completely contracted [units].
	unsigned int	analog_min = PISTON_ANALOG_MIN;	//Lower value that the potentiometer pin gives (when piston is fully contracted).
	unsigned int	analog_max = PISTON_ANALOG_MAX;	//Higher value that the potentiometer pin gives (when piston is fully contracted).
	//state variables
	enum states : char { INIT, RUNNING, CALIBRATING_START, CALIBRATING_MAX, CALIBRATING_MIN};
	states	state = INIT;
	unsigned int	calibrate_limit_value;
	unsigned long	calibrate_last_time = millis();
	unsigned long	calibrate_time = PISTON_CALIBRATE_TIME;
	//speed calculation
	unsigned long	speed_last_time = millis();
	unsigned long	speed_update_time = PISTON_SPEED_UPDATE_TIME;

	void	move(float input, unsigned char &out_pwm, unsigned char &out_dir);
	void	controller(unsigned int in_analog_pos, unsigned char &out_pwm, unsigned char &out_dir);
public:
	piston(float min_pos = PISTON_MIN_POS, float max_pos = PISTON_MAX_POS);
	//setters	
	char	set_pos					(float val);
	void	set_max_pwm				(unsigned char val);
	void	set_min_pwm				(unsigned char val);
	void	set_P_area				(float val);
	void	set_tolerance			(float val);
	void	set_analog_limits		(int min, int max);
	void	set_calibration_time	(unsigned int val);
	void	set_speed_update_time	(unsigned long val);
	//getters
	float			get_target_pos			();
	float			get_current_pos			();
	float			get_current_speed		();
	unsigned char	get_max_pwm				();
	unsigned char	get_min_pwm				();
	float			get_P_area				();
	float			get_tolerance			();
	float			get_min_pos				();
	float			get_max_pos				();
	unsigned int	get_analog_min			();
	unsigned int	get_analog_max			();
	int				get_state				();
	unsigned long	get_calibrate_time		();
	unsigned long	get_speed_update_time	();
	//random methods
	void calibrate();	//automatically sets analog_min and max by doing a test moving the piston to its end positions
	void stop();		//sets the stop state(for emergency or stuff).
	void update(unsigned int in_analog_pos,unsigned char& out_pwm, unsigned char& out_dir);	//Put this on the loop
	
	friend class cutting_head;
};

class motor {
private:
	//motor state variables
	int						target_pwm;								//objective rpm of motor
	float					current_rpm;							//current rpm of motor
	//rpm calculation variables
	unsigned long			rpm_last_time = millis();				//saved time used in rpm_update function to count how much time passed since last update
	unsigned long			rpm_update_time = MOTOR_RPM_UPDATE_TIME;
	volatile unsigned int	encoder_ticks	= 0;					//amount of ticks counted by encoder1 since last rpm update
	float					gear_ratio;								//how many revolutions the motor makes for one output shaft revolution 
	float					encoder_cpr;							//amount of notches in a single encoder of the motor

	int		update_rpm();											//updates the rpm if enough time has passed, returns 1 if rpm have been updated and 0 if not
	void	controller(unsigned char &out_pwm);
public:
	motor(float gear_ratio = MOTOR_GEAR_RATIO, float encoder_cpr = MOTOR_CPR);
	//setters
	void	set_target_pwm(int val);					//Set the pwm of the motor. Values between -255 and 255.
	void	set_rpm_update_time(unsigned long val);		//Set the time interval between rpm calculations [ms].
	//getters
	float	get_target_pwm();		//returns the target_rpm of the motor
	float	get_current_rpm();		//returns the current_rpm of the motor
	//random methods
	void	update(unsigned char &out_pwm, unsigned char &out_dir);	//PUT THIS ON VOID LOOP(), returns the state of the motor.
	void	encoder_handler();			//function to call from the handler of the motor in the main program

	friend class cutting_head;
};

class cutting_head {
private:
	motor drill;
	piston piston1;
	piston piston2;
	float long1;
	float long2;
	float base1;
	float base2;
	float min_angle;
	float max_angle;
	float alpha;
	float beta;

	void direct_kinematics(float p1,float p2, float& alpha, float& beta);
	void inverse_kinematics(float alfa, float beta, float& p1, float& p2);
	enum states { RUNNING, CALIBRATING};
	states state = RUNNING;
public:
	cutting_head(
		float _long1 = CH_LONG1, float _long2 = CH_LONG2,
		float _base1 = CH_BASE1, float _base2 = CH_BASE2,
		float _piston_min = PISTON_MIN_POS, float _piston_max = PISTON_MAX_POS,
		float _motor_gear_ratio = MOTOR_GEAR_RATIO, float _motor_CPR = MOTOR_CPR);
	//setters
	void	set_drill_target_pwm			(unsigned char val);				//Values between -255 and 255.
	void	set_drill_rpm_update_time		(unsigned long val);	//set the update_time (time between rpm updates)

	void	set_piston_max_pwm				(unsigned char val);
	void	set_piston_min_pwm				(unsigned char val);
	void	set_piston_P_area				(float val);
	void	set_piston_tolerance			(float val);
	void	set_piston1_analog_limits		(int min, int max);
	void	set_piston2_analog_limits		(int min, int max);
	void	set_piston_calibration_time		(unsigned long val);
	void	set_piston_speed_update_time	(unsigned long val);

	void set_pos_abs(float alfa,float beta);
	void set_pos_relative(float val1, float val2);
	//getters
	float	get_min_angle();
	float	get_max_angle();
	float	get_alpha();
	float	get_beta();

	float get_drill_current_rpm();														//returns the current_rpm of the drill

	float get_piston1_target_pos();
	float get_piston1_current_pos();
	float get_piston1_speed();
	float get_piston2_target_pos();
	float get_piston2_current_pos();
	float get_piston2_speed();

	int	  get_state();
	//random methods
	void calibrate();
	void stop();
	void update(
		unsigned int p1_analog_pos, unsigned char& p1_pwm, unsigned char& p1_dir,
		unsigned int p2_analog_pos, unsigned char& p2_pwm, unsigned char& p2_dir,
		unsigned char& drill_pwm, unsigned char& drill_dir);
	void drill_handler();
};