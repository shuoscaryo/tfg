#include "arduino.h"
#include "cutting_head.h"
#include "gpio_exp.h"
#include "Adafruit_INA219.h"
//#include "neopixel.h"
#include <Wire.h>



#define PISTON_MAX_POS	152
#define PISTON_MIN_POS	102



#define GPIO_DIR 0x20
#define PISTON1_INA_DIR 0x40
#define PISTON2_INA_DIR 0x41
#define MOTOR_INA_DIR 0x44

#define P1_POS_PIN 9
#define P1_DIR_PIN 1
#define P1_PWM_PIN 2

#define P2_POS_PIN 10
#define P2_DIR_PIN 2
#define P2_PWM_PIN 3

#define M_ENC_PIN 8
#define M_DIR_PIN 0
#define M_PWM_PIN 1

gpio_exp gpio(GPIO_DIR);
/*cutting_head ch(LONG1, LONG2, BASE1, BASE2,
		PISTON_MIN_POS, PISTON_MAX_POS,
		MOTOR_GEAR_RATIO, MOTOR_CPR);*/
Adafruit_INA219 p1_ina(PISTON1_INA_DIR);
Adafruit_INA219 p2_ina(PISTON2_INA_DIR);
Adafruit_INA219 m_ina(MOTOR_INA_DIR);
//Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
cutting_head ch;
unsigned int encoder_ticks = 0;

void encoderISR()
{
	ch.drill_handler();
}

void setup()
{
	Serial.begin(115200);
	p1_ina.begin();
	p2_ina.begin();
	m_ina.begin();
	Wire.begin();
//	pixels.begin();
	pinMode(1,OUTPUT);
	pinMode(0, OUTPUT);
	digitalWrite(0,true);
	attachInterrupt(digitalPinToInterrupt(M_ENC_PIN), encoderISR, RISING);
}
#include <math.h>

void loop()
{
	unsigned int p1_analog_in = analogRead(P1_POS_PIN);
	unsigned int p2_analog_in = analogRead(P2_POS_PIN);
	unsigned char p1_pwm, p1_dir, p2_pwm, p2_dir, m_pwm, m_dir;
	static unsigned long last_time = millis();
	unsigned long time = millis();
	static unsigned char state;
	static float angle = 0;


	float p1_mA = p1_ina.getCurrent_mA();
	float p2_mA = p2_ina.getCurrent_mA();
	float m_mA = m_ina.getCurrent_mA();
	if (Serial.available())
	{
		state = Serial.read() - '0';
		Serial.print("state:");
		Serial.println(state);
	}
	switch (state)
	{
		case 0:
			ch.set_pos_relative(0,0);
			break;
		case 1:
			if (time - last_time > 150)
			{
				angle += 5;
				if (angle > 360)
					angle -= 360;
				last_time = millis();
			}
			ch.set_pos_relative(cos(angle * 3.141592 / 180.0), sin(angle * 3.141592 / 180.0));
			break;
		case 2:
			ch.calibrate();
			state= 3;
		case 3:
			if (ch.get_state() == 0)
				state = 1;
			break;
		default:
			break;
	}

//	update_neopixel(pixels);
	ch.update(p1_analog_in, p1_pwm, p1_dir, p2_analog_in, p2_pwm, p2_dir, m_pwm, m_dir);
	analogWrite(P1_PWM_PIN,p1_pwm);
	gpio.write(P1_DIR_PIN, p1_dir);
	analogWrite(P2_PWM_PIN,p2_pwm);
	gpio.write(P2_DIR_PIN, p2_dir);
	analogWrite(M_PWM_PIN,m_pwm);
	gpio.write(M_DIR_PIN, m_dir);
	gpio.update();
	Serial.print("m_rpm:");
	Serial.print(ch.get_drill_current_rpm());
	Serial.print(" p1_mA:");
	Serial.print(p1_mA);
	Serial.print(" p2_mA:");
	Serial.print(p2_mA);
	Serial.print(" m_mA:");
	Serial.print(m_mA);
	Serial.print(" alfa:");
	Serial.print(ch.get_alpha());
	Serial.print(" beta:");
	Serial.println(ch.get_beta());
}