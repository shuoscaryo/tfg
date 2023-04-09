#include "arduino.h"
#include "cutting_head.h"
#include "gpio_exp.h"
#include "Adafruit_INA219.h"
#include "neopixel.h"
#include <Wire.h>

#define BASE1	53.2
#define BASE2	45
#define LONG1	127
#define LONG2	120

#define PISTON_MAX_POS	152
#define PISTON_MIN_POS	102

#define MOTOR_GEAR_RATIO	99
#define MOTOR_CPR			12

#define GPIO_DIR 0x20
#define PISTON1_INA_DIR 0x40
#define PISTON2_INA_DIR 0x41
#define MOTOR_INA_DIR 0x42

#define P1_POS_PIN 9
#define P1_DIR_PIN 5
#define P1_PWM_PIN 2

#define P2_POS_PIN 10
#define P2_DIR_PIN 6
#define P2_PWM_PIN 3

#define M_DIR_PIN 4
#define M_PWM_PIN 1

gpio_exp gpio(GPIO_DIR);
cutting_head ch(LONG1, LONG2, BASE1, BASE2,
		PISTON_MIN_POS, PISTON_MAX_POS,
		MOTOR_GEAR_RATIO, MOTOR_CPR);
Adafruit_INA219 p1_ina(PISTON1_INA_DIR);
Adafruit_INA219 p2_ina(PISTON2_INA_DIR);
Adafruit_INA219 m_ina(MOTOR_INA_DIR);
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_RGB + NEO_KHZ800);

void setup()
{
	Serial.begin(115200);
	Wire.begin();
	p1_ina.begin();
	p2_ina.begin();
	m_ina.begin();
	ch.calibrate();
	//set the connection pin with the robot to 1.
	pinMode(0, OUTPUT);
	digitalWrite(0,true);
	//
	pinMode(P1_DIR_PIN,OUTPUT);
	pinMode(P2_DIR_PIN,OUTPUT);
}

void loop()
{
	unsigned int p1_analog_in = analogRead(P1_POS_PIN);
	unsigned int p2_analog_in = analogRead(P2_POS_PIN);
	unsigned char p1_pwm, p1_dir, p2_pwm, p2_dir, m_pwm, m_dir;

	update_neopixel(pixels);

	ch.update(
		p1_analog_in, p1_pwm, p1_dir,
		p2_analog_in, p2_pwm, p2_dir,
		m_pwm, m_dir);
	analogWrite(P1_PWM_PIN,p1_pwm);
	gpio.write(P1_DIR_PIN, p1_dir);
	analogWrite(P2_PWM_PIN,p2_pwm);
	gpio.write(P2_DIR_PIN, p2_dir);
	analogWrite(M_PWM_PIN, m_dir);
	gpio.write(M_DIR_PIN, m_dir);

	float p1_mA = p1_ina.getCurrent_mA();
	float p2_mA = p2_ina.getCurrent_mA();
	float m_mA = m_ina.getCurrent_mA();

	Serial.print(ch.get_alpha());
	Serial.print(",");
	Serial.print(ch.get_beta());
	Serial.print(",");
	Serial.print(ch.get_drill_current_rpm());
	Serial.print(",");
	Serial.print(p1_mA);
	Serial.print(",");
	Serial.print(p2_mA);
	Serial.print(",");
	Serial.println(m_mA);
}
