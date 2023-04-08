#include "neopixel.h"



void	update_neopixel(Adafruit_NeoPixel pixels)
{
	unsigned long time = millis();
	static unsigned long last_time = millis();
	static unsigned char state ;
	static unsigned char brightness;

	pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
	switch (state) {
		case 0:
			if (time - last_time >= 5)
				brightness ++;
			if (brightness > 255)
				brightness = 255;
			pixels.fill(0xFF0000);
			pixels.setBrightness(brightness);
			if (brightness == 255)
				state++;
      		break;
    	case 1:
			pixels.fill(0xFF0000);
			pixels.setBrightness(255);
			if (time - last_time >= 1000)
				state++;
      		break;
    	case 2:
			if (time - last_time >= 5)
				brightness -= (brightness > 0);
			pixels.fill(0xFF0000);
			pixels.setBrightness(brightness);
			if (brightness == 0)
				state++;
      		break;
		case 3:
			if (time - last_time >= 5)
				brightness ++;
			if (brightness > 255)
				brightness = 255;
			pixels.fill(0x00FF00);
			pixels.setBrightness(brightness);
			if (brightness == 255)
				state++;
      		break;
    	case 4:
			pixels.fill(0x00FF00);
			pixels.setBrightness(255);
			if (time - last_time >= 1000)
				state++;
      		break;
    	case 5:
			if (time - last_time >= 5)
				brightness -= (brightness > 0);
			pixels.fill(0x00FF00);
			pixels.setBrightness(brightness);
			if (brightness == 0)
				state++;
      		break;
		case 6:
			if (time - last_time >= 5)
				brightness ++;
			if (brightness > 255)
				brightness = 255;
			pixels.fill(0x0000FF);
			pixels.setBrightness(brightness);
			if (brightness == 255)
				state++;
      		break;
    	case 7:
			pixels.fill(0x0000FF);
			pixels.setBrightness(255);
			if (time - last_time >= 1000)
				state++;
      		break;
    	case 8:
			if (time - last_time >= 5)
				brightness -= (brightness > 0);
			pixels.fill(0x0000FF);
			pixels.setBrightness(brightness);
			if (brightness == 0)
				state = 0;
      		break;
    	default:
			pixels.fill(0x000000);
      		break;
	}
	last_time = millis();
    pixels.show();
}