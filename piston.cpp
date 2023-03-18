#include "piston.h"
#include <math.h>
#define pi 3.14159265358979323846264
/*								   _      __
							____  (_)____/ /_____  ____ 
						   / __ \/ / ___/ __/ __ \/ __ \
						  / /_/ / (__  ) /_/ /_/ / / / /
						 / .___/_/____/\__/\____/_/ /_/ 
						/_/										*/
//private
void piston::update_current_pos() { current_pos = scale(analogRead(pin_current_pos), analog_min, analog_max, min_pos, max_pos); }
float piston::scale(float input,float in_min, float in_max, float out_min, float out_max) {
	float percentage = (input - in_min) / (in_max- in_min);
	return out_min+ (out_max- out_min) * percentage;
}
void piston::move(int input) {
	if (input > 0)		input = 1;
	else if (input < 0)	input = -1;
	else input = 0;
	if (input == 1) {
		digitalWrite(pin_pwm, true);
		digitalWrite(pin_dir, true);
	}
	else if (input == -1) {
		digitalWrite(pin_pwm, true);
		digitalWrite(pin_dir, false);
	}
	else
		digitalWrite(pin_pwm, false);
}
//public
piston::piston(	byte _pin_pwm, byte _pin_dir, byte _pin_current_pos) {
	set_pin_pwm(_pin_pwm);
	set_pin_dir(_pin_dir);
	set_pin_current_pos(_pin_current_pos);
	update_current_pos();
	target_pos = current_pos;
}

void piston::set_pin_pwm		(byte input )	{pin_pwm = input;pinMode(pin_pwm, OUTPUT);}
void piston::set_pin_dir		(byte input )	{pin_dir = input;pinMode(pin_dir, OUTPUT);}
void piston::set_pin_current_pos(byte input)	{ pin_current_pos = input; pinMode(pin_current_pos, INPUT); }
void piston::set_target_pos		(float input)	{target_pos = input;}
void piston::set_pos_limit(float min, float max) { min_pos = min; max_pos = max; }
void piston::set_tolerance		(float input)	{tolerance = input;}
void piston::set_analog_limit(int min, int max) { analog_min = min; analog_max = max; }

byte	piston::get_pin_pwm()			{return pin_pwm			;}
byte	piston::get_pin_dir()			{return pin_dir			;}
byte	piston::get_pin_current_pos()	{return pin_current_pos	;}
float	piston::get_target_pos()		{return target_pos		;}
float	piston::get_current_pos()		{return current_pos		;}
float	piston::get_max_pos()			{return max_pos			;}
float	piston::get_min_pos()			{return min_pos			;}
float	piston::get_tolerance()			{return tolerance		;}
float	piston::get_analog_min()		{return analog_min		;}
float	piston::get_analog_max()		{return analog_max		;}
int		piston::get_state()				{return state			;}

void piston::calibrate(int in) {
	if (state == 0) {
		calibrate_time = in;
		state = 1;
		calibrate_limit_value = analogRead(pin_current_pos);
		calibrate_last_time=millis();
	}
}
void piston::stop() {
	state = 3;
}
void piston::start() {
	state = 0;
}
void piston::update() {
	if (state == 0) {
		update_current_pos();
		float dif = target_pos - current_pos;				//calculate the difference between the current pos and the target pos
		float abs_dif = dif * ((dif > 0) - (dif < 0));		//get the absolute value of the difference
		if (tolerance >= abs_dif) { dif = 0; }				//if the piston is inside the tolerance stop moving
		if (dif > 0)move(1);
		else if (dif == 0)move(0);
		else move(-1);
	}
	if (state==1) {											//move the piston only if its not being calibrated
		int read_value = analogRead(pin_current_pos);		//save current analog read in variable
		int current_time = millis();
		move(1);
		if (read_value > calibrate_limit_value) {			//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = read_value;
			calibrate_last_time = current_time;
		}
		if (current_time-calibrate_last_time >= calibrate_time) {		//if sample is big enough and there was no new highest value for a while save it
			analog_max = read_value;
			state = 2;
			calibrate_last_time = current_time;
		}
	}
	if (state == 2) {											//move the piston only if its not being calibrated
		int read_value = analogRead(pin_current_pos);		//save current analog read in variable
		int current_time = millis();
		move(-1);
		if (read_value < calibrate_limit_value) {			//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = read_value;
			calibrate_last_time = current_time;
		}
		if (current_time - calibrate_last_time >= calibrate_time) {		//if sample is big enough and there was no new highest value for a while save it
			analog_min = read_value;
			state = 0;
		}
	}
	if (state == 3) {
		move(0);
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
		if (used_pins & P_MOTOR_ENCODER1_MASK) {
			current_rpm = encoder1_ticks * 60.0 / ((time - last_time) * gear_ratio * encoder_cpr) * 1000;
			encoder1_ticks = 0;
		}
		if (used_pins & P_MOTOR_ENCODER2_MASK) {
			current_rpm = encoder2_ticks * 60.0 / ((time - last_time) * gear_ratio * encoder_cpr) * 1000;
			encoder2_ticks = 0;
		}
		last_time = millis();
		return 1;
	}
	return 0;
}
float motor::scale(float in, float in_min, float in_max, float out_min, float out_max) {
	float percentage = (in - in_min) / (in_max - in_min);
	return out_min + percentage * (out_max - out_min);
}
//PUBLIC METHODS
motor::motor() {}

void motor::set_pin_pwm(byte in) {
	pin_pwm = in;
	used_pins = used_pins | P_MOTOR_PWM_MASK;
}
void motor::set_pin_dir(byte in) { 
	pin_dir = in;
	used_pins = used_pins | P_MOTOR_DIR_MASK;
}
void motor::set_pin_encoder1(byte in, void (*handler)(void)) {
	if (used_pins & P_MOTOR_ENCODER1_MASK) {					//if the pin is being used
		detachInterrupt(digitalPinToInterrupt(pin_encoder1));	//detach current pin interruption
	}
	pin_encoder1=in;														//set new pin
	pinMode(pin_encoder1, INPUT);
	attachInterrupt(digitalPinToInterrupt(pin_encoder1),handler,RISING);	//attach new interruption
	used_pins = used_pins | P_MOTOR_ENCODER1_MASK;							//set encoder pin as used
}
void motor::set_pin_encoder2(byte in, void (*handler)(void)) {
	if (used_pins & P_MOTOR_ENCODER2_MASK) {					//if the pin is being used
		detachInterrupt(digitalPinToInterrupt(pin_encoder2));	//detach current pin interruption
	}
	pin_encoder2 = in;											//set new pin
	pinMode(pin_encoder2, INPUT);
	attachInterrupt(digitalPinToInterrupt(pin_encoder2),handler,RISING);		//attach new interruption
	used_pins = used_pins | P_MOTOR_ENCODER2_MASK;				//set encoder pin as used
}

void motor::unset_pin_pwm() { used_pins = used_pins & (~P_MOTOR_PWM_MASK); }
void motor::unset_pin_dir() { used_pins = used_pins & (~P_MOTOR_DIR_MASK); }
void motor::unset_pin_encoder1() { if (used_pins & P_MOTOR_ENCODER1_MASK) { detachInterrupt(digitalPinToInterrupt(pin_encoder1)); }used_pins = used_pins & (~P_MOTOR_ENCODER1_MASK); }
void motor::unset_pin_encoder2() { if (used_pins & P_MOTOR_ENCODER2_MASK) { detachInterrupt(digitalPinToInterrupt(pin_encoder2)); }used_pins = used_pins & (~P_MOTOR_ENCODER2_MASK); }

void motor::set_dir(bool in) { dir = in; }
void motor::set_target_rpm(float in) {target_rpm = in;}
void motor::set_target_speed(float in) { set_target_rpm(scale(constrain(in,0,1),0,1,0,max_rpm)); }
void motor::set_max_rpm(float in) { max_rpm = in; }

void motor::set_update_time(int in) { update_time = in; }
void motor::set_gear_ratio(float in){ gear_ratio=in;}
void motor::set_encoder_cpr(float in){ encoder_cpr=in;}

int motor::get_pin_pwm()		{ if (used_pins & P_MOTOR_PWM_MASK)return pin_pwm; else return -1; }
int motor::get_pin_dir()		{ if (used_pins & P_MOTOR_DIR_MASK)return pin_dir; else return -1; }
int motor::get_pin_encoder1()	{ if (used_pins & P_MOTOR_ENCODER1_MASK)return pin_encoder1; else return -1; }
int motor::get_pin_encoder2()	{ if (used_pins & P_MOTOR_ENCODER2_MASK)return pin_encoder2; else return -1; }

bool motor::get_dir() { return dir; }
float motor::get_target_rpm() { return target_rpm; }
float motor::get_current_rpm() { return current_rpm; }
float motor::get_max_rpm() { return max_rpm; }

int motor::get_state() { return state; }
int	motor::get_update_time() { return update_time; }
float motor::get_gear_ratio() { return gear_ratio; }
float motor::get_encoder_cpr() { return encoder_cpr; }

int motor::calibrate(int in) {
	if (
		(!(used_pins & P_MOTOR_PWM_MASK)) ||								//if pwm not selected
		(!(used_pins & (P_MOTOR_ENCODER1_MASK | P_MOTOR_ENCODER2_MASK)))	//or no encoder selected
		) {
		return -1;
	}
	state = 1;
	calibrate_limit_value = 0;
	calibrate_counter = 0;
	return 1;
}
void motor::stop() { state = 2; }
void motor::start() { state = 0; }
void motor::update() {
	if (state == 0) {
		if (update_rpm()) {
			if (used_pins & P_MOTOR_PWM_MASK) { analogWrite(pin_pwm, scale(constrain(target_rpm, 0, max_rpm), 0, max_rpm, 0, 255)); }
			if (used_pins & P_MOTOR_DIR_MASK) { digitalWrite(pin_dir, dir); }
		}
	}
	if (state == 1) {
		if (update_rpm())calibrate_counter++;
		if (used_pins & P_MOTOR_PWM_MASK)analogWrite(pin_pwm, 255);
		if (current_rpm > calibrate_limit_value) {		//if read value is greater than saved, save the new value and reset counter
			calibrate_limit_value = current_rpm;
			calibrate_counter = 0;
		}
		if (calibrate_counter >= calibrate_ticks) {	//if sample is big enough and there was no new highest value for a while save it
			set_target_rpm(0);
			analogWrite(pin_pwm, target_rpm);
			max_rpm = current_rpm;
			state = 0;
		}
	}
	if (state == 2) {
		if (used_pins & P_MOTOR_PWM_MASK)analogWrite(pin_pwm, 0);
	}
}
void motor::encoder1_handler() { encoder1_ticks++; }
void motor::encoder2_handler() { encoder2_ticks++; }

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
cutting_head::cutting_head(
	byte p1_pwm, byte p1_dir, byte p1_in,
	byte p2_pwm, byte p2_dir, byte p2_in,
	byte m_pwm, byte m_enc, void (*handler)(void),
	float l1,float l2, float b1,float b2
	) :
	piston1(p1_pwm, p1_dir, p1_in), piston2(p2_pwm, p2_dir, p2_in),
	long1(l1),long2(l2),base1(b1),base2(b2)
{
	direct_kinematics(piston1.get_max_pos(), piston2.get_min_pos(), min_angle, max_angle);
	drill.set_pin_pwm(m_pwm);
	drill.set_pin_encoder1(m_enc, handler);
}

void cutting_head::set_dimensions(float _long1, float _long2, float _base1, float _base2) {
	long1 = _long1;
	long2 = _long2;
	base1 = _base1;
	base2 = _base2;
	direct_kinematics(piston1.get_max_pos(), piston2.get_min_pos(), min_angle, max_angle);
}

void cutting_head::set_drill_target_rpm(float in) { drill.set_target_rpm(in); }
void cutting_head::set_drill_target_speed(float in) { drill.set_target_speed(in); }
void cutting_head::set_drill_max_rpm(float in) { drill.set_max_rpm(in); }
void cutting_head::set_drill_update_time(int in) { drill.set_update_time(in); }
void cutting_head::set_drill_gear_ratio(float in) { drill.set_gear_ratio(in); }
void cutting_head::set_drill_encoder_cpr(float in) { drill.set_encoder_cpr(in); }

void cutting_head::set_piston_pos_limit(float min,float max) { piston1.set_pos_limit(min,max); piston2.set_pos_limit(min,max); }
void cutting_head::set_piston_tolerance(float in) { piston1.set_tolerance(in); piston2.set_tolerance(in); }
void cutting_head::set_piston_analog_limit(int min, int max) { piston1.set_analog_limit(min, max); piston2.set_analog_limit(min, max);}

float cutting_head::get_long1() { return long1; }
float cutting_head::get_long2(){return long2;}
float cutting_head::get_base1(){return base1;}
float cutting_head::get_base2(){return base2;}
float cutting_head::get_min_angle(){return min_angle*180/pi;}
float cutting_head::get_max_angle(){return max_angle*180/pi;}
float cutting_head::get_drill_target_rpm(){return drill.get_target_rpm(); }													//returns the target_rpm of the drill
float cutting_head::get_drill_current_rpm(){return drill.get_current_rpm(); }														//returns the current_rpm of the drill
float cutting_head::get_drill_max_rpm(){return drill.get_max_rpm(); }															//returns the max_rpm of the cutting_head
float cutting_head::get_piston1_target_pos(){return piston1.get_target_pos(); }
float cutting_head::get_piston1_current_pos(){return piston1.get_current_pos(); }
float cutting_head::get_piston2_target_pos(){return piston2.get_target_pos(); }
float cutting_head::get_piston2_current_pos(){return piston2.get_current_pos(); }
int	  cutting_head::get_state() { return state; }

void cutting_head::set_pos_angle_abs(float alfa, float beta) {
	float p1, p2;
	if (alfa > max_angle)alfa = max_angle;
	else if (alfa < min_angle)alfa = min_angle;
	if (beta> max_angle)beta = max_angle;
	else if (beta< min_angle)beta = min_angle;
	Serial.print(state);
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
void cutting_head::calibrate_lock() { 
	if (state == 0) { 
		state = 2;
		drill.calibrate();
		piston1.calibrate();
		piston2.calibrate();
	}
}
void cutting_head::calibrate() { 
	if (state == 0) {
		state = 1;
		drill.calibrate();
		piston1.calibrate();
		piston2.calibrate();
	} 
}
void cutting_head::stop() { state = 1; }
void cutting_head::start() { state = 0; }
void cutting_head::update() {
	if (state == 0) {
		drill.update();
		piston1.update();
		piston2.update();
	}
	if (state == 1) {
		int drill_state = drill.get_state();
		int piston1_state=piston1.get_state();
		int piston2_state=piston2.get_state();

		if (drill_state != 0)drill.update();
		else drill.set_target_speed(0);
		if (piston1_state != 0)piston1.update();
		else piston1.set_target_pos(piston1.get_current_pos());
		if(piston2_state!=0)piston2.update();
		else piston2.set_target_pos(piston2.get_current_pos());
		if (drill_state == 0 && piston1_state == 0 && piston2_state == 0)state = 0;
	}
	if (state == 2) {
		int drill_state = drill.get_state();
		int piston1_state = piston1.get_state();
		int piston2_state = piston2.get_state();

		if (drill_state != 0)drill.update();
		else { 
			drill.set_target_speed(0);
			if (piston1_state != 0)piston1.update();
			else {
				piston1.set_target_pos(piston1.get_current_pos());
				if (piston2_state != 0)piston2.update();
				else { 
					piston2.set_target_pos(piston2.get_current_pos());
					state = 0;
				}
			}
		}
	}
	if (state == 3) {
		drill.stop();
		piston1.stop();
		piston2.stop();
	}
}
void cutting_head::drill_handler() { drill.encoder1_handler(); }