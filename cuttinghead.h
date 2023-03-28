#pragma once
#include "Arduino.h"

#define PISTON_MAX_POS		152		//datasheet value
#define PISTON_MIN_POS		102		//datasheet value
#define PISTON_TOLERANCE	1
#define	PISTON_ANALOG_MIN	0		//experimental value (tested reading the piston position pin at min extension)
#define	PISTON_ANALOG_MAX	1024	//experimental value (tested reading the piston position pin at max extension)
#define PISTON_SPEED_TIME	100		//time in miliseconds between speed measurement of the piston.
#define PISTON_CALIBRATE_TIME 3000			//amount of ticks that the analog read value must remain constant to consider the piston static
#define	PISTON_P_BOUNDARY	5		//size of the area where the piston works in linear mode when on POS_MODE state
#define P_MOTOR_DEFAULT_MAX_RPM 55
#define P_MOTOR_DEFAULT_GEAR_RATIO 99
#define P_MOTOR_DEFAULT_CPR 12
#define P_MOTOR_CALIBRATION_TICKS 5			//how many ticks does the rpm have to stay stable for it to be counted as max (a tick happens every P_MOTOR_RPM_UPDATE_TIME milliseconds)
#define P_MOTOR_RPM_UPDATE_TIME 1000		//how many milliseconds have to pass to calculate the new rpm

#define CH_BASE1 53.2
#define CH_BASE2 45
#define CH_LONG1 127
#define CH_LONG2 120

class piston;
class motor;
class cutting_head;

class piston{
private:
	//piston mechanical values
	float	target_pos;							//to what position should the piston move (cm)
	float	current_pos;						//current position of the piston (cm)
	float	target_speed;						//value between -1 and 1
	float	current_speed;
	float	max_pos = PISTON_MAX_POS;		//maximum extension value of the piston (mm)
	float	min_pos = PISTON_MIN_POS;		//minimum extension value of the piston (mm)
	//pos controller variables
	float	P_boundary = PISTON_P_BOUNDARY;
	float	pos_mode_speed = 1;
	float	tolerance = PISTON_TOLERANCE;	//allowed error between current_pos and target_pos
	//piston electrical values
	int		analog_min = PISTON_ANALOG_MIN;	//lower value that pin_current_pos reads (0-1024) (when current_pos==min_pos)	
	int		analog_max = PISTON_ANALOG_MAX;	//higher value that pin_current_pos reads (0-1024) (when current_pos==max_pos)
	//state variables
	enum states : char { INIT, SPEED_MODE, POS_MODE, CALIBRATING_MAX, CALIBRATING_MIN, STOP };
	states	state=INIT;
	int		calibrate_limit_value;
	unsigned long calibrate_last_time = millis();
	int		calibrate_time = PISTON_CALIBRATE_TIME;
	//speed calculation
	unsigned long	last_time = millis();
	unsigned long	speed_time = PISTON_SPEED_TIME;

	void move(int input,int& out_pwm, int &out_dir);
	void	pos_controller(int in_analog_pos, int &out_pwm, int &out_dir);
	void	speed_controller(int in_analog_pos, int &out_pwm, int &out_dir);
public:
	piston();
	//setters	
	float	set_pos				(float val);				//this one controls where the piston will go
	float	set_pos_mode_speed	(float val);
	float	set_speed			(float val);
	void	set_speed_time		(unsigned long val);	
	void	set_tolerance		(float val);
	void	set_pos_limits		(float min, float max);
	void	set_analog_limits	(int min, int max);			//sets the limit value of the analog values read from the piston
	//getters
	float	get_target_pos		();
	float	get_current_pos		();
	float	get_pos_mode_speed	();
	float	get_target_speed	();
	float	get_current_speed	();
	float	get_max_pos			();
	float	get_min_pos			();
	float	get_tolerance		();
	float	get_analog_min		();
	float	get_analog_max		();
	int		get_state			();
	//random methods
	void calibrate(int time= CHP_CALIBRATE_TIME);		//automatically sets analog_min and max by doing a test moving the piston to its end positions
	void stop();									//sets the stop state(for emergency or stuff)
	void begin();
	void update(int in_analog_pos,int& out_pwm,int& out_dir);	//controls the position of the piston, so it can reach the target_pos
	
	friend class cutting_head;
};

class motor {
private:
	//motor state variables
	float			target_rpm=0;													//objective rpm of motor
	float			current_rpm;													//current rpm of motor
	float			max_rpm= P_MOTOR_DEFAULT_MAX_RPM;								//max rpm that the motor can reach with empty load (use calibrate() or calibrate_lock() to get the true value or set_max_rpm() for aproximations)
	//calibrate method variables
	enum states : char { RUNNING, CALIBRATING, STOP };
	states			state					= RUNNING;								//variable used to let the program know that a calibration is taking place (state=1) or not (state=0)
	float			calibrate_limit_value	= 0;									//max rpm value saved in calibration
	int				calibrate_counter		= 0;									//internal variable used to count for how many rpm_updates the rpm hasnt increased past calibrate_limit_value
	int				calibrate_ticks			= P_MOTOR_CALIBRATION_TICKS;
	//rpm calculation variables
	int				last_time		= millis();										//saved time used in rpm_update function to count how much time passed since last update
	int 			update_time		= P_MOTOR_RPM_UPDATE_TIME;						//time in ms between rpm updates
	volatile int	encoder_ticks	= 0;											//amount of ticks counted by encoder1 since last rpm update
	float			gear_ratio		= P_MOTOR_DEFAULT_GEAR_RATIO;					//how many revolutions the motor makes for one output shaft revolution 
	float			encoder_cpr		= P_MOTOR_DEFAULT_CPR;							//amount of notches in a single encoder of the motor

	int	 update_rpm();																//updates the rpm if enough time has passed, returns 1 if rpm have been updated and 0 if not
	motor();
public:
	//setters
	void set_target_rpm(float);														//absolute value of rpm (if its higher than max it will get constrained to max_rpm)
	void set_target_speed(float);													//values between 0-1, speed is relative to max rpm
	void set_max_rpm(float);														//manually set the max_rpm (used when the calibration is not wanted)
	void set_update_time(int);														//set the update_time (time between rpm updates)
	void set_gear_ratio(float);														//set the gear_ratio of the motor (motor rev/output shaft rev)
	void set_encoder_cpr(float);													//set the encoder_cpr of the motor (count of notches in one encoder disk)
	//getters
	float get_target_rpm();															//returns the target_rpm of the motor
	float get_current_rpm();														//returns the current_rpm of the motor
	float get_max_rpm();															//returns the max_rpm of the motor
	int get_state();																//return current state of motor
	int get_update_time();															//returns update_time
	float get_gear_ratio();															//returns gear ratio
	float get_encoder_cpr();														//returns encoder cpr
	//random methods
	void calibrate( int =P_MOTOR_CALIBRATION_TICKS);								//sets motor state to calibrate (returns -1 if its not posible to calibrate and 1 if success)
	void stop();																	//stops the motion and sets the state to 2 (error)
	void start();																	//sets state to 0 (normal working)
	void update(int& out_pwm);														//PUT THIS ON VOID LOOP(), returns the state of the motor.
	void encoder_handler();															//function to call from the handler of the motor in the main program

	friend class cutting_head;
};

class cutting_head {
private:
	piston piston1;
	piston piston2;
	motor drill;
	float long1=CH_LONG1;
	float long2=CH_LONG2;
	float base1=CH_BASE1;
	float base2=CH_BASE2;
	float min_angle;
	float max_angle;
	void direct_kinematics(float p1,float p2, float& alfa, float& beta);
	void inverse_kinematics(float alfa, float beta, float& p1, float &p2);
	enum states { RUNNING, CALIBRATING, LOCK_CALIBRATING, STOP };
	states state=RUNNING;
public:
	cutting_head();
	//setters
	void set_dimensions(float _long1,float _long2, float _base1, float _base2);

	void set_drill_target_rpm(float);
	void set_drill_target_speed(float);
	void set_drill_max_rpm(float);
	void set_drill_update_time(int);
	void set_drill_gear_ratio(float );
	void set_drill_encoder_cpr(float);

	void set_piston_pos_limit(float min, float max);
	void set_piston_tolerance(float);
	void set_piston_analog_limit(int min, int max);
	//getters
	float get_long1();
	float get_long2();
	float get_base1();
	float get_base2();
	float get_min_angle();
	float get_max_angle();

	float get_drill_target_rpm();														//returns the target_rpm of the drill
	float get_drill_current_rpm();														//returns the current_rpm of the drill
	float get_drill_max_rpm();															//returns the max_rpm of the motor

	float get_piston1_target_pos();
	float get_piston1_current_pos();
	float get_piston2_target_pos();
	float get_piston2_current_pos();

	int	  get_state();
	//random methods
	void set_pos_angle_abs(float alfa,float beta);
	void set_pos_angle_relative(float, float);
	void calibrate_lock(int drill_ticks=P_MOTOR_CALIBRATION_TICKS,int piston_time=CHP_CALIBRATE_TIME);
	void calibrate(int drill_ticks = P_MOTOR_CALIBRATION_TICKS, int piston_time = CHP_CALIBRATE_TIME);
	void stop();
	void start();
	void update(int p1_analog_pos, int& p1_pwm, int& p1_dir,
				int p2_analog_pos, int& p2_pwm, int& p2_dir,
				int& drill_pwm);
	void drill_handler();
};