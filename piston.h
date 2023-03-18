#pragma once
#include "Arduino.h"

#define CHP_DEFAULT_MAX_POS		152		//datasheet value
#define CHP_DEFAULT_MIN_POS		102		//datasheet value
#define CHP_DEFAULT_TOLERANCE	1
#define	CHP_DEFAULT_ANALOG_MIN	14		//experimental value (tested reading the piston position pin at min extension)
#define	CHP_DEFAULT_ANALOG_MAX	1004	//experimental value (tested reading the piston position pin at max extension)

#define CHP_CALIBRATE_TIME 3000			//amount of ticks that the analog read value must remain constant to consider the piston static

class piston;
class motor;
class cutting_head;

class piston{
private:
	//pin assignation
	byte	pin_pwm;			//controls the speed of the piston
	byte	pin_dir;			//direction of the pin (1 expansion, 0 contraction)
	byte	pin_current_pos;	//piston pin that gives its current position
	//piston mechanical values
	float	target_pos;						//to what position should the piston move (cm)
	float	current_pos;					//current position of the piston (cm)
	float	max_pos=CHP_DEFAULT_MAX_POS;		//maximum extension value of the piston (cm)
	float	min_pos=CHP_DEFAULT_MIN_POS;		//minimum extension value of the piston (cm)
	float	tolerance=CHP_DEFAULT_TOLERANCE;	//allowed error between current_pos and target_pos
	//piston electrical values
	int		analog_min=CHP_DEFAULT_ANALOG_MIN;	//lower value that pin_current_pos reads (0-1024) (when current_pos==min_pos)	
	int		analog_max=CHP_DEFAULT_ANALOG_MAX;	//higher value that pin_current_pos reads (0-1024) (when current_pos==max_pos)
	//state variables
	int		state=0;
	int		calibrate_limit_value;
	int		calibrate_last_time = millis();
	int		calibrate_time = CHP_CALIBRATE_TIME;

	void update_current_pos();		//reads the pin value and sets the position of the piston
	float scale(float in, float in_min, float in_max, float out_min, float out_max);
	void move(int);							//values between -1 and 1
	piston(byte _pin_pwm, byte _pin_dir, byte _pin_current_pos); //there is no need to set pinmode
public:
	//setters
	void set_pin_pwm		(byte	);		
	void set_pin_dir		(byte	);		
	void set_pin_current_pos(byte	);		
	void set_target_pos		(float	);		//this one controls where the piston will go
	void set_pos_limit		(float	min,float max);
	void set_tolerance		(float	);
	void set_analog_limit	(int	min,int max);

	//getters
	byte				get_pin_pwm			();
	byte				get_pin_dir			();
	byte				get_pin_current_pos	();
	float				get_target_pos		();
	float				get_current_pos		();
	float				get_max_pos			();
	float				get_min_pos			();
	float				get_tolerance		();
	float				get_analog_min		();
	float				get_analog_max		();
	int					get_state			();
	//random methods
	void calibrate(int= CHP_CALIBRATE_TIME);		//(DOESNT BLOCK THE PROGRAM UNTIL TEST IS DONE)automatically sets analog_min and max by doing a test moving the piston to its end positions
	void stop();									//stops the calibration (for emergency or stuff)
	void start();
	void update();									//controls the position of the piston, so it can reach the target_pos
	
	friend class cutting_head;
};

#define P_MOTOR_DEFAULT_MAX_RPM 55
#define P_MOTOR_DEFAULT_GEAR_RATIO 99
#define P_MOTOR_DEFAULT_CPR 12
#define P_MOTOR_CALIBRATION_TICKS 5			//how many ticks does the rpm have to stay stable for it to be counted as max (a tick happens every P_MOTOR_RPM_UPDATE_TIME milliseconds)
#define P_MOTOR_RPM_UPDATE_TIME 1000		//how many milliseconds have to pass to calculate the new rpm

#define P_MOTOR_PWM_MASK 0b00001000
#define P_MOTOR_DIR_MASK 0b00000100
#define P_MOTOR_ENCODER1_MASK 0b00000010
#define P_MOTOR_ENCODER2_MASK 0b00000001

class motor {
private:
	//pin variables
	byte			pin_pwm;														//pin used to set pwm of motor
	byte			pin_dir;														//pin used to set dir of motor
	byte			pin_encoder1;													//pin used to read the encoder values
	byte			pin_encoder2;													//pin used to read the encoder values
	uint8_t			used_pins=0x00;													//1byte variable for knowing which pins are being used in a motor instance (eg. if no encoder reading is needed the program wont waste time doing it)
	//motor state variables
	bool			dir=0;															//what direction should the motor rotate
	float			target_rpm=0;													//objective rpm of motor
	float			current_rpm;													//current rpm of motor
	float			max_rpm= P_MOTOR_DEFAULT_MAX_RPM;								//max rpm that the motor can reach with empty load (use calibrate() or calibrate_lock() to get the true value or set_max_rpm() for aproximations)
	//calibrate method variables
	int				state					= 0;									//variable used to let the program know that a calibration is taking place (state=1) or not (state=0)
	float			calibrate_limit_value	= 0;									//max rpm value saved in calibration
	int				calibrate_counter		= 0;									//internal variable used to count for how many rpm_updates the rpm hasnt increased past calibrate_limit_value
	int				calibrate_ticks			= P_MOTOR_CALIBRATION_TICKS;
	//rpm calculation variables
	int				last_time		= millis();										//saved time used in rpm_update function to count how much time passed since last update
	int 			update_time		= P_MOTOR_RPM_UPDATE_TIME;						//time in ms between rpm updates
	volatile int	encoder1_ticks	= 0;											//amount of ticks counted by encoder1 since last rpm update
	volatile int	encoder2_ticks	= 0;											//amount of ticks counted by encoder2 since last rpm update
	float			gear_ratio		= P_MOTOR_DEFAULT_GEAR_RATIO;					//how many revolutions the motor makes for one output shaft revolution 
	float			encoder_cpr		= P_MOTOR_DEFAULT_CPR;							//amount of notches in a single encoder of the motor

	int	 update_rpm();																//updates the rpm if enough time has passed, returns 1 if rpm have been updated and 0 if not
	float scale(float in,float in_min,float in_max,float out_min,float out_max);	//scales in value from in_min,in_max value to out_min,out_max (needed bc map() only works for ints)
	motor();
public:
	//setters
	void set_pin_pwm(byte);															//sets pwm pin and marks it as used in used_variables
	void set_pin_dir(byte);															//sets dir pin and marks it as used in used_variables
	void set_pin_encoder1(byte, void (* handler)(void));							//sets encoder1 pin,requires an external function as handler				
	void set_pin_encoder2(byte, void (* handler)(void));							//sets encoder2 pin,requires an external function as handler

	void unset_pin_pwm();															//set pwm pin as not used
	void unset_pin_dir();															//set dir pin as not used
	void unset_pin_encoder1();														//set encoder 1 pin as not used (also dettaches the interruption)
	void unset_pin_encoder2();														//set encoder 2 pin as not used (also dettaches the interruption)

	void set_dir(bool);																//set the direction of the motor
	void set_target_rpm(float);														//absolute value of rpm (if its higher than max it will get constrained to max_rpm)
	void set_target_speed(float);													//values between 0-1, speed is relative to max rpm
	void set_max_rpm(float);														//manually set the max_rpm (used when the calibration is not wanted)

	void set_update_time(int);														//set the update_time (time between rpm updates)
	void set_gear_ratio(float);														//set the gear_ratio of the motor (motor rev/output shaft rev)
	void set_encoder_cpr(float);													//set the encoder_cpr of the motor (count of notches in one encoder disk)
	//getters
	int get_pin_pwm();																//returns the pin_pwm, if the pin is not being used returns -1
	int get_pin_dir();																//returns the pin_dir, if the pin is not being used returns -1
	int get_pin_encoder1();															//returns the pin_encoder1, if the pin is not being used returns -1
	int get_pin_encoder2();															//returns the pin_encoder2, if the pin is not being used returns -1

	bool get_dir();																	//returns dir
	float get_target_rpm();															//returns the target_rpm of the motor
	float get_current_rpm();														//returns the current_rpm of the motor
	float get_max_rpm();															//returns the max_rpm of the motor

	int get_state();																//return current state of motor
	int get_update_time();															//returns update_time
	float get_gear_ratio();															//returns gear ratio
	float get_encoder_cpr();														//returns encoder cpr
	//random methods
	int	 calibrate( int =P_MOTOR_CALIBRATION_TICKS);								//sets motor state to calibrate (returns -1 if its not posible to calibrate and 1 if success)
	void stop();																	//stops the motion and sets the state to 2 (error)
	void start();																	//sets state to 0 (normal working)
	void update();																	//PUT THIS ON VOID LOOP(), returns the state of the motor.
	void encoder1_handler();														//function to call from the handler of the motor in the main program
	void encoder2_handler();														//function to call from the handler of the motor in the main program

	friend class cutting_head;
};

#define CH_BASE1 53.2
#define CH_BASE2 45
#define CH_LONG1 127
#define CH_LONG2 120

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
	int state=0;
public:
	cutting_head(
		byte p1_pwm,byte p1_dir,byte p1_in,
		byte p2_pwm,byte p2_dir,byte p2_in,
		byte m_pwm ,byte m_enc, void (*handler)(void),
		float l1=CH_LONG1, float l2=CH_LONG2, float b1 = CH_BASE1, float b2 = CH_BASE2
	);
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
	void calibrate_lock();
	void calibrate();
	void stop();
	void start();
	void update();
	void drill_handler();
};