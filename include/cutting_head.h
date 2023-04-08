#pragma once
#include "arduino.h"

#define	PISTON_ANALOG_MIN			0		//experimental value (tested reading the piston position pin at min extension)
#define	PISTON_ANALOG_MAX			1024	//experimental value (tested reading the piston position pin at max extension)
#define PISTON_SPEED_UPDATE_TIME	100		//time in miliseconds between speed measurement of the piston.
#define PISTON_CALIBRATE_TIME 		3000			//amount of ticks that the analog read value must remain constant to consider the piston static
#define	PISTON_P_BOUNDARY			5		//size of the area where the piston works in linear mode when on POS_MODE state

#define MOTOR_RPM_UPDATE_TIME 500

class cutting_head;

class piston{
private:
	//piston mechanical values
	float	target_pos;							//to what position should the piston move 
	float	current_pos;						//current position of the piston
	float	target_speed = 1;					//values between 0 and 1. (1:pwm 255 and 0:pwm 0)
	float	current_speed;						//piston speed in units/second.
	float	min_pos;							//minimum extension value of the piston (mm)
	float	max_pos;							//maximum extension value of the piston (mm)
	//pos controller variables
	float	P_boundary = PISTON_P_BOUNDARY;
	float	tolerance = 1;	//allowed error between current_pos and target_pos
	//piston electrical values
	unsigned int	analog_min = PISTON_ANALOG_MIN;	//lower value that pin_current_pos reads (0-1024) (when current_pos==min_pos)	
	unsigned int	analog_max = PISTON_ANALOG_MAX;	//higher value that pin_current_pos reads (0-1024) (when current_pos==max_pos)
	//state variables
	enum states : char { INIT, RUNNING, CALIBRATING_START, CALIBRATING_MAX, CALIBRATING_MIN, STOP };
	states	state=INIT;
	unsigned int	calibrate_limit_value;
	unsigned long	calibrate_last_time = millis();
	unsigned long	calibrate_time = PISTON_CALIBRATE_TIME;
	//speed calculation
	unsigned long	speed_last_time = millis();
	unsigned long	speed_update_time = PISTON_SPEED_UPDATE_TIME;

	void	move(float input,unsigned char &out_pwm, unsigned char &out_dir);
	void	controller(unsigned int in_analog_pos, unsigned char &out_pwm, unsigned char &out_dir);
public:
	piston(float min_pos, float max_pos);
	//setters	
	char	set_pos(float val);				//this one controls where the piston will go
	void	set_speed(float val);
	void	set_speed_update_time(unsigned long val);
	void	set_P_boundary(float val);
	void	set_tolerance(float val);
	char	set_analog_limits(int min, int max);			//sets the limit value of the analog values read from the piston
	void	set_calibration_time(unsigned int val);
	//getters
	float	get_target_pos		();
	float	get_current_pos		();
	float	get_current_speed	();
	float	get_max_pos			();
	float	get_min_pos			();
	float	get_tolerance		();
	float	get_analog_min		();
	float	get_analog_max		();
	int		get_state			();
	//random methods
	void calibrate();		//automatically sets analog_min and max by doing a test moving the piston to its end positions
	void stop();									//sets the stop state(for emergency or stuff)
	void begin();
	void update(unsigned int in_analog_pos,unsigned char& out_pwm, unsigned char& out_dir);	//controls the position of the piston, so it can reach the target_pos
	
	friend class cutting_head;
};

class motor {
private:
	//motor state variables
	int		target_speed;											//objective rpm of motor
	float	current_rpm;											//current rpm of motor
	//calibrate method variables
	enum states : char { RUNNING, STOP };
	states	state = RUNNING;							//variable used to let the program know that a calibration is taking place (state=1) or not (state=0)
	//rpm calculation variables
	unsigned long	rpm_last_time = millis();				//saved time used in rpm_update function to count how much time passed since last update
	unsigned long	rpm_update_time = MOTOR_RPM_UPDATE_TIME;
	volatile unsigned int	encoder_ticks	= 0;									//amount of ticks counted by encoder1 since last rpm update
	float			gear_ratio;			//how many revolutions the motor makes for one output shaft revolution 
	float			encoder_cpr;					//amount of notches in a single encoder of the motor

	int		update_rpm();														//updates the rpm if enough time has passed, returns 1 if rpm have been updated and 0 if not
	void	controller(unsigned char &out_pwm);
public:
	motor(float gear_ratio, float encoder_cpr);
	//setters
	void	set_target_speed(int val);		//absolute value of rpm (if its higher than max it will get constrained to max_rpm)
	void	set_rpm_update_time(unsigned long val);		//set the update_time (time between rpm updates)
	//getters
	float	get_target_rpm();			//returns the target_rpm of the motor
	float	get_current_rpm();		//returns the current_rpm of the motor
	int		get_state();				//return current state of motor
	//random methods
	void	stop();					//stops the motion and sets the state to 2 (error)
	void	start();					//sets state to 0 (normal working)
	void	update(unsigned char &out_pwm, unsigned char &out_dir);	//PUT THIS ON VOID LOOP(), returns the state of the motor.
	void	encoder_handler();			//function to call from the handler of the motor in the main program

	friend class cutting_head;
};

class cutting_head {
private:
	piston piston1;
	piston piston2;
	motor drill;
	float long1;
	float long2;
	float base1;
	float base2;
	float min_angle;
	float max_angle;

	void direct_kinematics(float p1,float p2, float& alfa, float& beta);
	void inverse_kinematics(float alfa, float beta, float& p1, float &p2);
	enum states { RUNNING, CALIBRATING, STOP };
	states state = RUNNING;
public:
	cutting_head(float _long1, float _long2, float _base1, float _base2,
		float _piston_min, float _piston_max,
		float _motor_gear_ratio, float _motor_CPR);
	//setters
	void set_drill_target_speed(float);
	void set_drill_update_time(unsigned long val);

	void set_piston_tolerance(float);
	void set_piston_calibrate_time();
	//getters
	float get_min_angle();
	float get_max_angle();

	float get_drill_current_rpm();														//returns the current_rpm of the drill

	float get_piston1_target_pos();
	float get_piston1_current_pos();
	float get_piston1_speed();
	float get_piston2_target_pos();
	float get_piston2_current_pos();
	float get_piston2_speed();

	int	  get_state();
	//random methods
	void set_pos_angle_abs(float alfa,float beta);
	void set_pos_angle_relative(float, float);
	void calibrate();
	void stop();
	void start();
	void update(unsigned int p1_analog_pos, unsigned char& p1_pwm, unsigned char& p1_dir,
				unsigned int p2_analog_pos, unsigned char& p2_pwm, unsigned char& p2_dir,
				unsigned char& drill_pwm, unsigned char& drill_dir);
	void drill_handler();
};