#include "cuttinghead.h"
#include <math.h>
#define pi 3.14159265358979323846264

template <typename T, typename U> static U	scale(T in, T in_min, T in_max, U out_min, U out_max)
{
/*
	Scales the "in" value that ranges between "in_min" and "in_max" to a range equivalent
	to "out_min" and "out_max".
	Eg: if "in"= 3, and is in range (0,10), and the output range is (0,100), the function
	will return 30.
*/
	float percentage = static_cast<float>(in - in_min) / static_cast<float>(in_max - in_min);
	return out_min + percentage * (out_max - out_min);
}

template <typename T> static int	constrain(T in, T min, T max)
{
/*
	Contrains the "in" value to be between "min" and "max".
	The type of the variable to be limited needs to have "<" and ">" operators.
	Returns the constrained value.
*/
	if (in > max) return (max);
	if (in < min) return (min);
	return (in);
}

/*								   _      __
							____  (_)____/ /_____  ____ 
						   / __ \/ / ___/ __/ __ \/ __ \
						  / /_/ / (__  ) /_/ /_/ / / / /
						 / .___/_/____/\__/\____/_/ /_/ 
						/_/										*/
//private
void	piston::move(float input, unsigned char& out_pwm, unsigned char& out_dir)
{
/*
	Sets the values of the "pwm_pin" and "dir_pin" that coincide with the input value.
	If the input value is out of the range of the piston, it will be constrained.
*/
	input = constrain (input, -1, 1);
	out_pwm = abs(input) * 255;		//set pwm speed
	out_dir = input > 0;			//if input>0 dir=1 else dir=0
}

void	piston::controller(unsigned int in_analog_pos, unsigned char &out_pwm, unsigned char &out_dir)
{
/*
	The controller is a VSC-SM, it works with a P controller inside the P area, and at max speed
	outside of it. It also has an area set by the tolerance value that stops the piston because it
	is considered to be in the correct position.
*/
	float	dif = target_pos - current_pos;
	float	abs_dif = abs(dif);

	if (abs_dif < tolerance)
		out_pwm = 0;
	else
	{
		if(P_boundary && abs_dif < P_boundary)
			out_pwm = target_speed * 255 * constrain(abs_dif / P_boundary, 0 , 1);
		else
			out_pwm = target_speed * 255;
		out_dir = dif > 0;
	}
}
//public
piston::piston(float _min_pos, float _max_pos): min_pos(_min_pos), max_pos(_max_pos)
{}

char	piston::set_pos(float input)
{
/*
	Sets the position to where the piston will go.
	Sets the state to POS_MODE if not in STOP state.
	Only works if the piston is not in STOP state.
	If the position is out of range, it will be constrained.
	Returns 1 on success or 0 if couldn't be set.
*/
	if (STATE != STOP)
	{
		state = RUNNING;
		input = constrain (input, min_pos, max_pos);
		target_pos = input;
		return (1);
	}
	return (0);
}

void	piston::set_speed(float input)
{
/*
	Sets the speed at which the piston will move when going to a position.
	"input" will be constrained between 0 and 1, equivalent to a percentage of max speed.
	Returns the value set.
*/
	input = constrain(input, 0, 1);
	target_speed = input;
}

void	piston::set_speed_update_time(unsigned long input)
{
/*
	Sets the time in milliseconds between speed measurements.
	Lower times mean more updates but less precision.
*/
	speed_update_time = input;
}

void	piston::set_P_boundary(float val)
{
/*
	Sets the distance from the "target_pos" where the piston will work in proportional mode.
	Outside of this area it will move at it's max set speed.
	If the input value is negative it will be converted to positive.
	Eg. if the target position is 100, and the P_boundary is 5, the area in (95,105) will be
	the proportional area.
*/
	if (val < 0) P_boundary = -val;
	else	P_boundary = val;
}

float	piston::set_tolerance(float input)
{
/*
	Sets the tolerance of the piston (the range where the position is considered ok).
	it will set the absolute value of the input.
	Returns the tolerance.
*/
	if (input >= 0) tolerance = input;
	else			tolerance = -input;
}

char	piston::set_pos_limits(float min, float max)
{
/*
	Sets the limit positions of the piston.
	If "min" is greater than "max", the limit values will still be set so that 
	the minimum position is smaller than the maximum. 
	If both values are the same, no assignment will be made.
	Returns "1" if the assignment has been made and "0" otherwise.
*/
	if (min < max) 		{ min_pos = min; max_pos = max; return (1); }
	else if (min > max)	{ min_pos = max; max_pos = min; return (1); }
	else				return (0);
}

int	piston::set_analog_limits(int min, int max)
{
/*
	Sets the limit values read of the potentiometer of the piston.
	If "min" is greater than "max", the limit values will still be set so that 
	the minimum is smaller than the maximum. 
	If both values are the same, no assignment will be made.
	Returns "1" if the assignment has been made and "0" otherwise.
*/
	if (min < max)		{ analog_min = min; analog_max = max; return (1); }
	else if (min > max)	{ analog_min = max; analog_max = min; return (1); }
	else				return (0);
}

void	set_calibration_time(unsigned int val)
{
/*
	Used when calibrating.
	Sets the time in milliseconds that the read max or min analog value of the piston mustn't update
	in order to consider it an extreme value.
*/
	calibrate_time = val;
}

// these functions return the requested values from the piston.
float	piston::get_target_pos()		{return target_pos		;}
float	piston::get_current_pos()		{return current_pos		;}
float	piston::get_current_speed()		{return current_speed	;}
float	piston::get_max_pos()			{return max_pos			;}
float	piston::get_min_pos()			{return min_pos			;}
float	piston::get_tolerance()			{return tolerance		;}
float	piston::get_analog_min()		{return analog_min		;}
float	piston::get_analog_max()		{return analog_max		;}
int		piston::get_state()				{return state			;}

void	piston::calibrate ()
{
/*
	This functions sets the state of the piston to CALIBRATE.
	Will go to CALIBRATE state only if it is not CALIBRATING or STOP.
	Multiple executions of the function wont reset the calibrate until it's done.
*/
	if (state != CALIBRATING_MAX && state != CALIBRATING_MIN && state != STOP)
		state = CALIBRATING_START;
}

void	piston::stop()
{
/*
	Sets the state of the piston to stop, so it will stop moving and won't do anything until resetted.
*/
	state = STOP;
}

void	piston::begin()
{
/*
	sets the piston's state to INIT to reset it.
*/
	state = INIT;
}

void	piston::update(unsigned int in_analog_pos, unsigned char& out_pwm, unsigned char& out_dir)
{
/*
	This function handles the piston behaviour, and updates some parameters like the speed or position.
	It does different things depending on the state that the piston is currently on.
	The input variables are:
		-in_analog_pos: the value read from analogRead of the potentiometer of the piston.
		-out_pwm: the value that will be sent to the PWM pin of the piston.
		-out_dir: the value that will be setn to the dir pin of the piston.
	The states do:
		-INIT: sets the piston speed to 0, and updates the current pos.
		-POS_MODE: controls the piston by setting the position.
		-SPEED_MODE: controls the piston by setting the speed.
		-CALIBRATING_MAX: expands the piston to see whats the max analog value read.
		-CALIBRATING_MIN: expands the piston to see whats the min analog value read.
		-STOP: stops the piston and forbids using the "set_pos" and "set_speed" methods.
*/
// the init state is before everything else so that no calculation is made.
	if (state == INIT){
		move(0, out_pwm, out_dir);
		target_pos = current_pos = scale(in_analog_pos, analog_min, analog_max, min_pos, max_pos);
		target_speed = 0;
		return;
	}
// This part updates the "current_pos" and "current_speed" variables of the piston
	float			last_pos = current_pos;
	unsigned long	current_time = millis();

	current_pos = scale(in_analog_pos, analog_min, analog_max, min_pos, max_pos);

	if (current_time - speed_last_time >= speed_update_time)
		current_speed = (current_pos - last_pos) / (current_time - speed_last_time);
	speed_last_time = millis();
// This part is the state machine of the piston
	if (state == RUNNING)
		controller(in_analog_pos, out_pwm, out_dir);
	if (state == CALIBRATING_START){
		state = CALIBRATING_MAX;
		calibrate_limit_value = 0;
		calibrate_last_time = millis();
	}
	if (state == CALIBRATING_MAX) {							//move the piston only if its not being calibrated
		move(target_speed,out_pwm,out_dir);
		if (in_analog_pos > calibrate_limit_value) {			//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = in_analog_pos;
			calibrate_last_time = current_time;
		}
		if (current_time - calibrate_last_time >= calibrate_time) {		//if sample is big enough and there was no new highest value for a while save it
			analog_max = in_analog_pos;
			state = CALIBRATING_MIN;
			calibrate_last_time = current_time;
		}
	}
	if (state == CALIBRATING_MIN) {											//move the piston only if its not being calibrated
		move(-target_speed,out_pwm,out_dir);
		if (in_analog_pos < calibrate_limit_value) {			//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = in_analog_pos;
			calibrate_last_time = current_time;
		}
		if (current_time - calibrate_last_time >= calibrate_time) {		//if sample is big enough and there was no new highest value for a while save it
			analog_min = in_analog_pos;
			state = INIT;
		}
	}
	if (state == STOP)
		move(0, out_pwm, out_dir);
}
/*											__
						   ____ ___  ____  / /_____  _____
						  / __ `__ \/ __ \/ __/ __ \/ ___/
						 / / / / / / /_/ / /_/ /_/ / /
						/_/ /_/ /_/\____/\__/\____/_/						*/
//PRIVATE METHODS
int motor::update_rpm()
{
	int time = millis();
	if (time - last_time >= update_time) {
		current_rpm = encoder_ticks * 60.0 / ((time - last_time) * gear_ratio * encoder_cpr) * 1000;
		encoder_ticks = 0;
		last_time = millis();
		return 1;
	}
	return 0;
}
//PUBLIC METHODS
motor::motor() {}

void motor::set_target_rpm(float in) {target_rpm = in;}
void motor::set_target_speed(float in) { set_target_rpm(scale(constrain(in,0,1),0,1,0,max_rpm)); }
void motor::set_max_rpm(float in) { max_rpm = in; }
void motor::set_update_time(int in) { update_time = in; }
void motor::set_gear_ratio(float in){ gear_ratio=in;}
void motor::set_encoder_cpr(float in){ encoder_cpr=in;}

float motor::get_target_rpm()	{ return target_rpm		;}
float motor::get_current_rpm()	{ return current_rpm	;}
float motor::get_max_rpm()		{ return max_rpm		;}
int   motor::get_state()		{ return state			;}
int	  motor::get_update_time()	{ return update_time	;}
float motor::get_gear_ratio()	{ return gear_ratio		;}
float motor::get_encoder_cpr()	{ return encoder_cpr	;}

void motor::calibrate(int in) {
	if (state == RUNNING) {
		state = CALIBRATING;
		calibrate_limit_value = 0;
		calibrate_counter = 0;
	}
}
void motor::stop() { state = STOP; }
void motor::start() { state = RUNNING; }
void motor::update(int& out_pwm) {
	if (state == RUNNING) {
		update_rpm();
		out_pwm = scale(constrain(target_rpm, 0, max_rpm), 0, max_rpm, 0, 255);
	}
	if (state == CALIBRATING) {
		if (update_rpm())calibrate_counter++;
		out_pwm = 255;
		if (current_rpm > calibrate_limit_value) {		//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = current_rpm;
			calibrate_counter = 0;
		}
		if (calibrate_counter >= calibrate_ticks) {	//if sample is big enough and there was no new highest value for a while save it
			set_target_rpm(0);
			out_pwm = 0;
			max_rpm = current_rpm;
			state = RUNNING;
		}
	}
	if (state == STOP) {
		out_pwm = 0;
	}
}
void motor::encoder_handler() { encoder_ticks++; }

/*						              __  __  _                __                   __
						  _______  __/ /_/ /_(_)___  ____ _   / /_  ___  ____ _____/ /
						 / ___/ / / / __/ __/ / __ \/ __ `/  / __ \/ _ \/ __ `/ __  / 
						/ /__/ /_/ / /_/ /_/ / / / / /_/ /  / / / /  __/ /_/ / /_/ /  
						\___/\__,_/\__/\__/_/_/ /_/\__, /  /_/ /_/\___/\__,_/\__,_/   
												  /____/											*/

//private
void cutting_head::inverse_kinematics(float alfa, float beta, float& p1, float& p2) {
	float L = sqrt(base2 * base2 + long1 * long1);
	float w = atan(base2 / long1);
	float a = alfa * pi / 180;
	float b = beta * pi / 180;
	float ang1 = pi / 2 - a - w;
	p1 = sqrt(L * L + base1 * base1 - 2 * base1 * L * cos(ang1));
	float ang2 = pi / 2 - b - w;
	p2 = sqrt(L * L + base1 * base1 - 2 * base1 * L * cos(ang2));
}
void cutting_head::direct_kinematics(float p1, float p2, float& alfa, float& beta) {
	float L = sqrt(base2 * base2 + long1 * long1);
	float w = atan(base2 / long1);
	float ang1 = acos((p1 * p1 - L * L - base1 * base1) / (-2 * L * base1));
	float ang2 = acos((p2 * p2 - L * L - base1 * base1) / (-2 * L * base1));
	alfa = pi / 2 - w - ang1;
	beta = pi / 2 - w - ang2;
	alfa *= 180 / pi;
	beta *= 180 / pi;
}
//public
cutting_head::cutting_head(){
	direct_kinematics(piston1.get_max_pos(), piston2.get_min_pos(), min_angle, max_angle);
}

void cutting_head::set_dimensions(float _long1, float _long2, float _base1, float _base2) {
	long1 = _long1;
	long2 = _long2;
	base1 = _base1;
	base2 = _base2;
	direct_kinematics(piston1.get_max_pos(), piston2.get_min_pos(), min_angle, max_angle);
}

void cutting_head::set_drill_target_rpm		(float in) { drill.set_target_rpm(in)	;}
void cutting_head::set_drill_target_speed	(float in) { drill.set_target_speed(in)	;}
void cutting_head::set_drill_max_rpm		(float in) { drill.set_max_rpm(in)		;}
void cutting_head::set_drill_update_time	(int   in) { drill.set_update_time(in)	;}
void cutting_head::set_drill_gear_ratio		(float in) { drill.set_gear_ratio(in)	;}
void cutting_head::set_drill_encoder_cpr	(float in) { drill.set_encoder_cpr(in)	;}

void cutting_head::set_piston_pos_limit(float min,float max){ piston1.set_pos_limit(min,max); piston2.set_pos_limit(min,max); }
void cutting_head::set_piston_tolerance(float in)			{ piston1.set_tolerance(in); piston2.set_tolerance(in); }
void cutting_head::set_piston_analog_limit(int min, int max){ piston1.set_analog_limit(min, max); piston2.set_analog_limit(min, max);}

float cutting_head::get_long1()					{ return long1						;}
float cutting_head::get_long2()					{ return long2						;}
float cutting_head::get_base1()					{ return base1						;}
float cutting_head::get_base2()					{ return base2						;}
float cutting_head::get_min_angle()				{ return min_angle*180/pi			;}
float cutting_head::get_max_angle()				{ return max_angle*180/pi			;}
float cutting_head::get_drill_target_rpm()		{ return drill.get_target_rpm()		;}													//returns the target_rpm of the drill
float cutting_head::get_drill_current_rpm()		{ return drill.get_current_rpm()	;}														//returns the current_rpm of the drill
float cutting_head::get_drill_max_rpm()			{ return drill.get_max_rpm()		;}															//returns the max_rpm of the cutting_head
float cutting_head::get_piston1_target_pos()	{ return piston1.get_target_pos()	;}
float cutting_head::get_piston1_current_pos()	{ return piston1.get_current_pos()	;}
float cutting_head::get_piston2_target_pos()	{ return piston2.get_target_pos()	;}
float cutting_head::get_piston2_current_pos()	{ return piston2.get_current_pos()	;}
int	  cutting_head::get_state()					{ return state						;}

void cutting_head::set_pos_angle_abs(float alfa, float beta) {
	float p1, p2;
	if (alfa > max_angle)alfa = max_angle;
	else if (alfa < min_angle)alfa = min_angle;
	if (beta> max_angle)beta = max_angle;
	else if (beta< min_angle)beta = min_angle;
	inverse_kinematics(alfa,beta,p1,p2);
	piston1.set_target_pos(p1);
	piston2.set_target_pos(p2);
}
void cutting_head::set_pos_angle_relative(float alfa, float beta) {
	float percentage1 = (alfa +1) / 2;
	alfa= min_angle + (max_angle-min_angle) * percentage1;
	float percentage2 = (beta +1) / 2;
	beta = min_angle + (max_angle - min_angle) *percentage2;
	set_pos_angle_abs(alfa,beta);
}
void cutting_head::calibrate_lock(int drill_ticks, int piston_time) {
	if (state == RUNNING||state==CALIBRATING) { 
		state = LOCK_CALIBRATING;
		drill.calibrate(drill_ticks);
		piston1.calibrate(piston_time);
		piston2.calibrate(piston_time);
	}
}
void cutting_head::calibrate(int drill_ticks, int piston_time) {
	if (state == RUNNING||state==LOCK_CALIBRATING) {
		state = CALIBRATING;
		drill.calibrate(drill_ticks);
		piston1.calibrate(piston_time);
		piston2.calibrate(piston_time);
	} 
}
void cutting_head::stop() { state = STOP; }
void cutting_head::start() { state = RUNNING; }
void cutting_head::update(	int p1_analog_pos, int& p1_pwm, int& p1_dir,
							int p2_analog_pos, int& p2_pwm, int& p2_dir,
							int& drill_pwm){
	if (state == RUNNING) {
		drill.update(drill_pwm);
		piston1.update(p1_analog_pos,p1_pwm,p1_dir);
		piston2.update(p2_analog_pos,p2_pwm,p2_dir);
	}
	if (state == CALIBRATING) {
		int drill_state = drill.get_state();
		int piston1_state=piston1.get_state();
		int piston2_state=piston2.get_state();

		if (drill_state != drill.RUNNING)drill.update(drill_pwm);
		else drill.set_target_speed(0);

		if (piston1_state != piston1.RUNNING)piston1.update(p1_analog_pos, p1_pwm, p1_dir);
		else piston1.set_target_pos(piston1.get_current_pos());

		if(piston2_state!=piston2.RUNNING)piston2.update(p2_analog_pos, p2_pwm, p2_dir);
		else piston2.set_target_pos(piston2.get_current_pos());

		if (drill_state == drill.RUNNING && piston1_state == piston1.RUNNING && piston2_state == piston2.RUNNING)state = RUNNING;
	}
	if (state == LOCK_CALIBRATING) {
		int drill_state = drill.get_state();
		int piston1_state = piston1.get_state();
		int piston2_state = piston2.get_state();

		if (drill_state != drill.RUNNING)drill.update(drill_pwm);
		else { 
			drill.set_target_speed(0);
			if (piston1_state != piston1.RUNNING)piston1.update(p1_analog_pos, p1_pwm, p1_dir);
			else {
				piston1.set_target_pos(piston1.get_current_pos());
				if (piston2_state != piston2.RUNNING)piston2.update(p2_analog_pos, p2_pwm, p2_dir);
				else { 
					piston2.set_target_pos(piston2.get_current_pos());
					state = RUNNING;
				}
			}
		}
	}
	if (state == STOP) {
		drill.stop();
		piston1.stop();
		piston2.stop();
	}
}
void cutting_head::drill_handler() { drill.encoder_handler(); }