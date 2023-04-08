#pragma once
#include "Arduino.h"

typedef unsigned char uchar;

class gpio_exp{
private:
	uchar	i2c_address;
public:
	gpio_exp(uchar i2c_address);

	void	write(uchar pos, uchar val);
	uchar	read(uchar pos);
};