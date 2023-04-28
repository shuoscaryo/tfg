#include "arduino.h"
#include "cutting_head.h"
#include "gpio_exp.h"
#include "Adafruit_INA219.h"
#include "builtin_led.h"
#include <Wire.h>
#include <math.h>
#include "crc.h"
#include "defines.h"

gpio_exp gpio(GPIO_DIR);
Adafruit_INA219 p1_ina(PISTON1_INA_DIR);
Adafruit_INA219 p2_ina(PISTON2_INA_DIR);
Adafruit_INA219 m_ina(MOTOR_INA_DIR);
cutting_head ch;

unsigned char in_message[IN_MESSAGE_SIZE] = {0};
unsigned char out_message[OUT_MESSAGE_SIZE] = {0};

void encoderISR(){ch.drill_handler();}

void setup()
{
	Serial.begin(115200);
	p1_ina.begin();
	p2_ina.begin();
	m_ina.begin();
	Wire.begin();
	pinMode(1,OUTPUT);
	pinMode(0, OUTPUT);
	digitalWrite(0,true);
	attachInterrupt(digitalPinToInterrupt(M_ENC_PIN), encoderISR, RISING);
}

void loop()
{
	unsigned int p1_analog_in = analogRead(P1_POS_PIN);
	unsigned int p2_analog_in = analogRead(P2_POS_PIN);
	unsigned char p1_pwm, p1_dir, p2_pwm, p2_dir, m_pwm, m_dir;
	float p1_mA = p1_ina.getCurrent_mA();
	float p2_mA = p2_ina.getCurrent_mA();
	float m_mA = m_ina.getCurrent_mA();

	static unsigned long last_time = millis();
	unsigned long time = millis();
	static unsigned char state;
	static float angle = 0;
	static float circle_radius = 1;

	if (Serial.available())
	{
		static unsigned char serial_state;
		static unsigned int count;
		union float_to_byte
		{
			float f;
			unsigned char str[sizeof(float)];
		};
		
		switch (serial_state)
		{
			case 0: //look for message begin byte value.
				if(IN_MESSAGE_SIZE >= 4)
				{
					in_message[0] = Serial.read();
					if (in_message[0] == BEGIN_BYTE)
					{
						serial_state = 1;
						count = 1;
					}
				}
				break;
			case 1:	//get message
				in_message[count++] = Serial.read();
				if (count == IN_MESSAGE_SIZE)
				{
					if (check_crc(in_message,IN_MESSAGE_SIZE))
					{
						switch (in_message[1])
						{
						case COMMAND_STOP:
							ch.stop();
							state = 0;
							break;
						case COMMAND_MOVE_TO_CENTER:
							ch.set_pos_relative(0,0);
							state = 0;
							break;
						case COMMAND_MOVE_TO_PLACE:
							if(IN_MESSAGE_SIZE >= 12)
							{
								float_to_byte pos1, pos2;
								for (int i = 0; i < 4; i++){
									pos1.str[i] = in_message[i + 2];
									pos2.str[i] = in_message[i + 6];
								}
								ch.set_pos_abs(pos1.f, pos2.f);
								state = 0;
							}
							break;
						case COMMAND_MOVE_CIRCLES:
							if(IN_MESSAGE_SIZE >= 8)
							{
								float_to_byte pos;
								for (int i = 0; i < 4; i++)
									pos.str[i] = in_message[i + 2];
								if (pos.f > 1.0) pos.f = 1.0;
								if (pos.f < 0.0) pos.f = 0.0;
								circle_radius = pos.f;
								state = 1;
							}
							break;
						case COMMAND_SET_MOTOR_PWM:
							if (IN_MESSAGE_SIZE >= 8)
							{
								ch.set_drill_target_pwm(in_message[2]);
								state = 1;
							}
							break;
						case COMMAND_REQUEST_ALFA:
							if (OUT_MESSAGE_SIZE >= 8)
							{
								out_message[0] = BEGIN_BYTE;
								out_message[1] = in_message[1];
								float_to_byte pos;
								pos.f = ch.get_alpha();
								for (int i = 0; i < 4; i ++)
									out_message[i + 2] = pos.str[i];
								add_crc(out_message, OUT_MESSAGE_SIZE);
								for (auto i: out_message)Serial.write(i);
							}
							break;
						case COMMAND_REQUEST_BETA:
							if (OUT_MESSAGE_SIZE >= 8)
							{
								out_message[0] = BEGIN_BYTE;
								out_message[1] = in_message[1];
								float_to_byte pos;
								pos.f = ch.get_beta();
								for (int i = 0; i < 4; i ++)
									out_message[i + 2] = pos.str[i];
								add_crc(out_message, OUT_MESSAGE_SIZE);
								for (auto i: out_message)Serial.write(i);
							}
							break;
						case COMMAND_REQUEST_RPM:
							if (OUT_MESSAGE_SIZE >= 8)
							{
								out_message[0] = BEGIN_BYTE;
								out_message[1] = in_message[1];
								float_to_byte pos;
								pos.f = ch.get_drill_current_rpm();
								for (int i = 0; i < 4; i ++)
									out_message[i + 2] = pos.str[i];
								add_crc(out_message, OUT_MESSAGE_SIZE);
								for (auto i: out_message)Serial.write(i);
							}
							break;
						case COMMAND_REQUEST_INA_PISTON1:
							if (OUT_MESSAGE_SIZE >= 8)
							{
								out_message[0] = BEGIN_BYTE;
								out_message[1] = in_message[1];
								float_to_byte pos;
								pos.f = p1_mA;
								for (int i = 0; i < 4; i ++)
									out_message[i + 2] = pos.str[i];
								add_crc(out_message, OUT_MESSAGE_SIZE);
								for (auto i: out_message)Serial.write(i);
							}
							break;
						case COMMAND_REQUEST_INA_PISTON2:
							if (OUT_MESSAGE_SIZE >= 8)
							{
								out_message[0] = BEGIN_BYTE;
								out_message[1] = in_message[1];
								float_to_byte pos;
								pos.f = p2_mA;
								for (int i = 0; i < 4; i ++)
									out_message[i + 2] = pos.str[i];
								add_crc(out_message, OUT_MESSAGE_SIZE);
								for (auto i: out_message)Serial.write(i);
							}
							break;
						case COMMAND_REQUEST_INA_DRILL:
							if (OUT_MESSAGE_SIZE >= 8)
							{
								out_message[0] = BEGIN_BYTE;
								out_message[1] = in_message[1];
								float_to_byte pos;
								pos.f = m_mA;
								for (int i = 0; i < 4; i ++)
									out_message[i + 2] = pos.str[i];
								add_crc(out_message, OUT_MESSAGE_SIZE);
								for (auto i: out_message)Serial.write(i);
							}
							break;
						case COMMAND_REQUEST_INA_ALL:
							if (OUT_MESSAGE_SIZE >= 8)
							{
								out_message[0] = BEGIN_BYTE;
								out_message[1] = in_message[1];
								float_to_byte pos;
								pos.f = m_mA + p1_mA + p2_mA;
								for (int i = 0; i < 4; i ++)
									out_message[i + 2] = pos.str[i];
								add_crc(out_message, OUT_MESSAGE_SIZE);
								for (auto i: out_message)Serial.write(i);
							}
							break;
						default:
							break;
						}
					}
					serial_state = 0;
				}
				break;
			default:
				serial_state = 0;
		}
	}
	switch (state)
	{
		case 0:
			break;
		case 1:
			if (time - last_time > 150)
			{
				angle += 5;
				if (angle > 360)
					angle -= 360;
				last_time = millis();
			}
			ch.set_pos_relative(circle_radius * cos(angle * 3.141592 / 180.0),
								circle_radius * sin(angle * 3.141592 / 180.0));
			break;
		default:
			state = 0;
		break;
	}

	update_builtin_led();
	ch.update(p1_analog_in, p1_pwm, p1_dir, p2_analog_in, p2_pwm, p2_dir, m_pwm, m_dir);
	analogWrite(P1_PWM_PIN,p1_pwm);
	gpio.write(P1_DIR_PIN, p1_dir);
	analogWrite(P2_PWM_PIN,p2_pwm);
	gpio.write(P2_DIR_PIN, p2_dir);
	analogWrite(M_PWM_PIN,m_pwm);
	gpio.write(M_DIR_PIN, m_dir);
	gpio.update();
}