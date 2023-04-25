#pragma once
#include <stddef.h>
#include "Arduino.h"
uint16_t crc_16(const unsigned char* input_str, size_t num_bytes);	//calculate crc of byte array (only data to be sent) and return it as 2byte int
void add_crc(unsigned char* message, size_t num_bytes);				//calculate and add the crc to the message. The message needs to have 2 free bytes
																	//at the end to add the crc. If the data to be sent is 6 bytes, message has to be
																	//8 bytes
bool check_crc(const unsigned char *message, size_t num_bytes);		//check if incoming message (data + crc(2 bytes)) was distorted by calculating
																	//crc of incoming data and comparing it with incoming crc