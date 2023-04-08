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
}

void loop()
{
	update_neopixel(pixels);
}
