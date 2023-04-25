#include "builtin_led.h"
#define RISE_TIME 5
#define HOLD_TIME 1000
#define FALL_TIME 2

void	update_builtin_led()
{
	unsigned long time = millis();
	static unsigned long last_time = millis();
	static unsigned char state;
	static unsigned char brightness;
	const unsigned char MAX_BRIGHTNESS = 255;

	switch (state)
	{
		case 0:
			if (time - last_time >= RISE_TIME)
			{
				brightness++;
				last_time = millis();
			}
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
				brightness--;
				last_time = millis();
			}
			if (brightness == 0)
				state = 0;
			break;
		   	default:
			state = 0;
			brightness = 0;
			break;
	}
	analogWrite(11,255 - brightness);
}