#include "gpio_exp.h"
#include <Wire.h>

gpio_exp::gpio_exp(uchar i2c_address)
{
	this->i2c_address = i2c_address;
	Wire.begin();
}

void	gpio_exp::write(uchar pos, uchar val)
{
	uchar gpio_val = Wire.read();
	bitWrite(gpio_val, pos, val);
	Wire.beginTransmission(i2c_address);
	Wire.write(gpio_val);
	Wire.endTransmission();
}

uchar	read::read(uchar pos)
{
	Wire.requestFrom(i2c_address, (uint8_t)1);
	uchar gpio_val = Wire.read();
	return bitRead(gpio_val, pos);
}