#include "cutting_head.h"
#include "gpio_exp.h"
#include "arduino.h"
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

gpio_exp GPIO(GPIO_DIR);
cutting_head CH(LONG1, LONG2, BASE1, BASE2,
		PISTON_MIN_POS, PISTON_MAX_POS,
		MOTOR_GEAR_RATIO, MOTOR_CPR);

void setup()
{
	Serial.begin(115200);
	Wire.begin();
}

void loop()
{

}