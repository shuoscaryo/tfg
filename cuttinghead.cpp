#include "cuttinghead.h"
#include <math.h>
#define pi 3.14159265358979323846264

static float scale(float in, float in_min, float in_max, float out_min, float out_max) {
	float percentage = (in - in_min) / (in_max - in_min);
	return out_min + percentage * (out_max - out_min);
}

/*								   _      __
							____  (_)____/ /_____  ____ 
						   / __ \/ / ___/ __/ __ \/ __ \
						  / /_/ / (__  ) /_/ /_/ / / / /
						 / .___/_/____/\__/\____/_/ /_/ 
						/_/										*/
//private
void piston::move(int input, int& out_pwm, int& out_dir) {
	if (input > 1)		 input = 1;	//conbstrain the input between -1 and 1 
	else if (input < -1) input = -1;
	out_pwm = abs(input) * 255;		//set pwm speed
	out_dir = input > 0;			//if input>0 dir=1 else dir=0
}
//public
piston::piston() {}

void piston::set_target_pos		(float input)	{
	if (input < min_pos)		target_pos = min_pos;
	else if (input > max_pos)	target_pos = max_pos;
	else						target_pos = input;
}
void piston::set_pos_limit(float min, float max) {
	if (min < max) { min_pos = min; max_pos = max; }
	else		   { min_pos = max; max_pos = min; }
}
void piston::set_tolerance		(float input)	{
	if (input >= 0) tolerance = input;
	else			tolerance = -input;
}
void piston::set_analog_limit(int min, int max) {
	if (min < max)	{ analog_min = min; analog_max = max; }
	else			{ analog_min = max; analog_max = min; }
}

float	piston::get_target_pos()		{return target_pos		;}
float	piston::get_current_pos()		{return current_pos		;}
float	piston::get_max_pos()			{return max_pos			;}
float	piston::get_min_pos()			{return min_pos			;}
float	piston::get_tolerance()			{return tolerance		;}
float	piston::get_analog_min()		{return analog_min		;}
float	piston::get_analog_max()		{return analog_max		;}
int		piston::get_state()				{return state			;}

void piston::calibrate(int in) {
	if (state == RUNNING||state==INIT) {
		calibrate_time = in;
		state = CALIBRATING_MAX;
		calibrate_limit_value = 0;
		calibrate_last_time=millis();
	}
}
void piston::stop() {state = STOP;}
void piston::start() {state = INIT;}

void piston::update(int in_analog_pos,int& out_pwm,int& out_dir) {

	current_pos = scale(in_analog_pos, analog_min, analog_max, min_pos, max_pos);

	if (state == INIT) {
		target_pos = current_pos;
		state = RUNNING;
	}
	if (state == RUNNING) {
		float dif = target_pos - current_pos;				//calculate the difference between the current pos and the target pos
		float abs_dif = dif * ((dif > 0) - (dif < 0));		//get the absolute value of the difference
		if (tolerance >= abs_dif) { dif = 0; }				//if the piston is inside the tolerance stop moving
		if (dif > 0)move(1, out_pwm, out_dir);				//current control method 
		else if (dif == 0)move(0, out_pwm, out_dir);
		else move(-1, out_pwm,out_dir);
	}
	if (state==CALIBRATING_MAX) {							//move the piston only if its not being calibrated
		int current_time = millis();
		move(1,out_pwm,out_dir);
		if (in_analog_pos > calibrate_limit_value) {			//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = in_analog_pos;
			calibrate_last_time = current_time;
		}
		if (current_time-calibrate_last_time >= calibrate_time) {		//if sample is big enough and there was no new highest value for a while save it
			analog_max = in_analog_pos;
			state = CALIBRATING_MIN;
			calibrate_last_time = current_time;
		}
	}
	if (state == CALIBRATING_MIN) {											//move the piston only if its not being calibrated
		int current_time = millis();
		move(-1,out_pwm,out_dir);
		if (in_analog_pos < calibrate_limit_value) {			//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = in_analog_pos;
			calibrate_last_time = current_time;
		}
		if (current_time - calibrate_last_time >= calibrate_time) {		//if sample is big enough and there was no new highest value for a while save it
			analog_min = in_analog_pos;
			target_pos = CHP_DEFAULT_MIN_POS;
			state = RUNNING;
		}
	}
	if (state == STOP) {
		move(0,out_pwm,out_dir);
	}
}
/*											__
						   ____ ___  ____  / /_____  _____
						  / __ `__ \/ __ \/ __/ __ \/ ___/
						 / / / / / / /_/ / /_/ /_/ / /
						/_/ /_/ /_/\____/\__/\____/_/						*/
//PRIVATE METHODS
int motor::update_rpm() {
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