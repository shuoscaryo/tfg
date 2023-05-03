#include "neopixel.h"
#define RISE_TIME 5
#define HOLD_TIME 1000
#define FALL_TIME 2

static unsigned int	color_val(unsigned char color)
{
	switch (color)
	{
		case 0:
			return 0xff0000;
		case 1:
			return 0x00ff00;
		case 2:
			return 0x0000ff;
		default:
			return 0x000000;
	}
}

void	update_neopixel(Adafruit_NeoPixel &pixels)
{
	unsigned long time = millis();
	static unsigned long last_time = millis();
	static unsigned char state;
	static unsigned char brightness;
	static unsigned char color;
	const unsigned char MAX_BRIGHTNESS = 50;

	pixels.begin();
	switch (state)
	{
		case 0:
			if (time - last_time >= RISE_TIME)
			{
				brightness ++;
				last_time = millis();
			}
			if (brightness > MAX_BRIGHTNESS)
				brightness = MAX_BRIGHTNESS;
			if (brightness == MAX_BRIGHTNESS)
				state++;
      		break;
    	case 1:
			if (time - last_time >= HOLD_TIME)
			{
				state++;
				last_time = millis();		
			}
      		break;
    	case 2:
			if (time - last_time >= FALL_TIME)
			{
				brightness -= (brightness > 0);
				last_time = millis();
			}
			if (brightness == 0)
			{
				color = (color + 1) % 3;
				state = 0;
			}
			break;
		   	default:
			state = 0;
			color = 0;
			break;
	}
	pixels.fill(color_val(color));
	pixels.setBrightness(brightness);
    pixels.show();
}