#include "gpio_exp.h"
#include <Wire.h>

gpio_exp::gpio_exp(byte i2c_address)
{
	this->i2c_address = i2c_address;
}

void gpio_exp::update()
{
	Wire.beginTransmission(i2c_address);
	Wire.write(values);
	Wire.endTransmission();
}

void	gpio_exp::write(byte pos, byte val)
{
	if(pos >= 0 && pos < 8)
		bitWrite(values, pos, val > 0);
}

byte	gpio_exp::read(byte pos)
{
	byte gpio_val = 0;
	Wire.requestFrom(i2c_address, 1);
	while (Wire.available())
		gpio_val = Wire.read();
	return bitRead(gpio_val, pos);
}