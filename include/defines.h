#pragma once
////////////////////////////I2C ADDRESSES
#define GPIO_DIR 0x20
#define PISTON1_INA_DIR 0x40
#define PISTON2_INA_DIR 0x41
#define MOTOR_INA_DIR 0x44
////////////////////////////PIN CONECTIONS
#define P1_POS_PIN 9
#define P1_DIR_PIN 1
#define P1_PWM_PIN 2

#define P2_POS_PIN 10
#define P2_DIR_PIN 2
#define P2_PWM_PIN 3

#define M_ENC_PIN 8
#define M_DIR_PIN 0
#define M_PWM_PIN 1
////////////////////////////UART PROTOCOL
#define SERIAL_BAUDS 115200
#define BEGIN_BYTE 0xfa
#define IN_MESSAGE_SIZE 12				//begin byte[1] + command[1] + data[8] + CRC[2]
#define OUT_MESSAGE_SIZE 8				//begin byte[1] + command[1] + data[4] + CRC[2]

#define COMMAND_STOP 0					//Stops every movement
#define COMMAND_MOVE_TO_CENTER 1		//Moves the cutting head to the central position.
#define COMMAND_MOVE_TO_PLACE 2			//Moves the cutting head to the position specified by parameters 0 and 1. (from 102 to 152)
#define COMMAND_MOVE_CIRCLES 3			//Moves in circles of a given radius. 1 parameter int from 0 to 100.
#define COMMAND_SET_MOTOR_PWM 4			//Sets the pwm sent to the motor.
#define COMMAND_REQUEST_ALFA 5			
#define COMMAND_REQUEST_BETA 6
#define COMMAND_REQUEST_RPM 7			//Sends a float value with the rpm
#define COMMAND_REQUEST_INA_PISTON1 8	//Sends a float value with the Piston1 amp consumption
#define COMMAND_REQUEST_INA_PISTON2 9	//Sends a float value with the Piston2 amp consumption
#define COMMAND_REQUEST_INA_DRILL 10		//Sends a float value with the DRILL amp consumption
#define COMMAND_REQUEST_INA_ALL 11		//Sends a float value with the sum of all INA
