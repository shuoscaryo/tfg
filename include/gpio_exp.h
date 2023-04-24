#pragma once
#include "Arduino.h"

class gpio_exp{
private:
	byte	values;
	byte	i2c_address;
public:
	gpio_exp(byte i2c_address);

	void	update();
	void	write(byte pos, byte val);
	byte	read(byte pos);
};