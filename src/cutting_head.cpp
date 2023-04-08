#include "cutting_head.h"
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

static float	ft_constrain(float in, float min, float max)
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
	input = ft_constrain (input, -1, 1);
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
			out_pwm = target_speed * 255 * ft_constrain(abs_dif / P_boundary, 0 , 1);
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
	Sets the state to RUNNING.
	Only works if the piston is not in STOP state.
	If the position is out of range, it will be constrained.
	Returns 1 on success or 0 if couldn't be set.
*/
	if (state != STOP)
	{
		state = RUNNING;
		input = ft_constrain (input, min_pos, max_pos);
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
	input = ft_constrain(input, 0, 1);
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

void	piston::set_tolerance(float input)
{
/*
	Sets the tolerance of the piston (the range where the position is considered ok).
	it will set the absolute value of the input.
	Returns the tolerance.
*/
	if (input >= 0) tolerance = input;
	else			tolerance = -input;
}

char	piston::set_analog_limits(int min, int max)
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

void	piston::set_calibration_time(unsigned int val)
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
		speed_last_time = millis();
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
//private
int motor::update_rpm()
{
/*
	updates the current rpm value if enough time has passed.
	returns 1 if the rpm value has been updated or 0 otherwise.
*/
	unsigned long time = millis();
	if (time - rpm_last_time >= rpm_update_time) {
		current_rpm = encoder_ticks * 60.0 / ((time - rpm_last_time) * gear_ratio * encoder_cpr) * 1000;
		encoder_ticks = 0;
		rpm_last_time = millis();
		return (1);
	}
	return (0);
}

void	motor::controller(unsigned char &out_pwm)
{
/*
	The controller is a PI, it calculates the current error of rpm, and sets an "out_pwm" according
	to it and the accumulated error of previous ticks.
	To avoid the sum of error to grow too much, if the pwm value is already max it won't add the 
	new error.
*/
}
//PUBLIC METHODS
motor::motor(float _gear_ratio, float _encoder_cpr): gear_ratio(_gear_ratio), encoder_cpr(_encoder_cpr)
{}

void motor::set_target_speed(int in)
{
/*
	Sets the speed in rpm at which the motor has to spin.
	Negative values mean spin in the other direction.
*/
	if (state != STOP)
	{
	state = RUNNING;
	target_speed = ft_constrain(in, -255, 255);
	}
}

void motor::set_rpm_update_time(unsigned long in)
{
/*
	Time between rpm measurements.
*/
	rpm_update_time = in;
}

float motor::get_target_rpm()	{ return target_speed		;}
float motor::get_current_rpm()	{ return current_rpm	;}
int   motor::get_state()		{ return state			;}

void motor::stop()
{
	state = STOP;
}

void motor::start()
{
	state = RUNNING;
}

void motor::update(unsigned char &out_pwm, unsigned char &out_dir)
{
	update_rpm();
	if (state == RUNNING){
		out_pwm = target_speed;
		out_dir = target_speed > 0;
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
cutting_head::cutting_head(float _long1, float _long2, float _base1, float _base2,
	float _piston_min, float _piston_max,
	float _motor_gear_ratio, float _motor_CPR):
piston1(_piston_min, _piston_max), piston2(_piston_min, _piston_max), drill(_motor_gear_ratio, _motor_CPR)
{
	long1 = _long1;
	long2 = _long2;
	base1 = _base1;
	base2 = _base2;
	direct_kinematics(_piston_min, _piston_max, min_angle, max_angle);
}

void cutting_head::set_drill_target_speed	(float in) {drill.set_target_speed(in);}
void cutting_head::set_drill_update_time	(unsigned long in) {drill.set_rpm_update_time(in);}

void cutting_head::set_piston_tolerance(float in)
{
	piston1.set_tolerance(in);
	piston2.set_tolerance(in);
}

float cutting_head::get_min_angle()				{ return min_angle*180/pi			;}
float cutting_head::get_max_angle()				{ return max_angle*180/pi			;}
float cutting_head::get_drill_current_rpm()		{ return drill.get_current_rpm()	;}														//returns the current_rpm of the drill
float cutting_head::get_piston1_target_pos()	{ return piston1.get_target_pos()	;}
float cutting_head::get_piston1_current_pos()	{ return piston1.get_current_pos()	;}
float cutting_head::get_piston1_speed()			{ return piston1.get_current_speed();}
float cutting_head::get_piston2_target_pos()	{ return piston2.get_target_pos()	;}
float cutting_head::get_piston2_current_pos()	{ return piston2.get_current_pos()	;}
float cutting_head::get_piston2_speed()			{ return piston2.get_current_speed();}
int	  cutting_head::get_state()					{ return state						;}

void cutting_head::set_pos_angle_abs(float alfa, float beta) {
	float p1, p2;
	if (alfa > max_angle)alfa = max_angle;
	else if (alfa < min_angle)alfa = min_angle;
	if (beta> max_angle)beta = max_angle;
	else if (beta< min_angle)beta = min_angle;
	inverse_kinematics(alfa,beta,p1,p2);
	piston1.set_pos(p1);
	piston2.set_pos(p2);
}

void cutting_head::set_pos_angle_relative(float alfa, float beta) {
	float percentage1 = (alfa +1) / 2;
	alfa= min_angle + (max_angle-min_angle) * percentage1;
	float percentage2 = (beta +1) / 2;
	beta = min_angle + (max_angle - min_angle) *percentage2;
	set_pos_angle_abs(alfa,beta);
}

void cutting_head::calibrate() {
	if (state == RUNNING) {
		state = CALIBRATING;
		piston1.calibrate();
		piston2.calibrate();
	} 
}
void cutting_head::stop()
{
	state = STOP;
	drill.stop();
	piston1.stop();
	piston2.stop();
}
void cutting_head::start()
{ 
	state = RUNNING;
	drill.start();
	piston1.begin();
	piston2.begin();
}
void cutting_head::update(
	unsigned int p1_analog_pos, unsigned char& p1_pwm, unsigned char& p1_dir,
	unsigned int  p2_analog_pos, unsigned char& p2_pwm, unsigned char& p2_dir,
	unsigned char& drill_pwm, unsigned char& drill_dir)
{
	drill.update(drill_pwm, drill_dir);
	piston1.update(p1_analog_pos,p1_pwm,p1_dir);
	piston2.update(p2_analog_pos,p2_pwm,p2_dir);
	if (state == CALIBRATING) {
		int piston1_state=piston1.get_state();
		int piston2_state=piston2.get_state();

		if (piston1_state == piston1.INIT && piston2_state == piston2.INIT)
			state = RUNNING;
	}
}
void cutting_head::drill_handler() { drill.encoder_handler(); }