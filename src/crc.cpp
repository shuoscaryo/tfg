#include <stddef.h>
#include "crc.h"

#define CRC_POLY_16 0xA001;     //variables used to calculate crc
#define CRC_START_16 0x0000;

static bool crc_tab16_init = false; //used to check if crc_tab has already been initialized
static uint16_t crc_tab16[256];     //used to calculate crc

//private functions that will be used in the file
static void init_crc16_tab(void);											//it executes once to initialize crc_tab

void add_crc(unsigned char* message,size_t num_bytes) {
    if (message != NULL && num_bytes > 2) {             //check if mensaje is long enough and pointer is not null
        uint16_t crc = crc_16(message, num_bytes - 2);  //calculate crc of data (last 2 values are not considered data)
        message[num_bytes - 2] = crc >> 8;              //save crc in last 2 bytes
        message[num_bytes - 1] = crc;
    }
}

bool check_crc(const unsigned char *buffer, size_t size){
    //     calculate new crc           compare it with incoming message crc
    return crc_16(buffer, size - 2) == (buffer[size - 2] << 8 | buffer[size - 1]);
}

uint16_t crc_16(const unsigned char *input_str, size_t num_bytes){
    uint16_t crc,tmp,short_c;
    const unsigned char *ptr=input_str;
    if (!crc_tab16_init)init_crc16_tab();
    crc = CRC_START_16;
    if (ptr != NULL)
        for (unsigned int i = 0; i < num_bytes; i++){
            short_c = 0x00ff & (uint16_t)*ptr;
            tmp = crc ^ short_c;
            crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];
            ptr++;
        }
    return crc;
}

void init_crc16_tab(void){
    uint16_t crc,c;
    for (int i = 0; i < 256; i++){
        crc = 0;
        c = i;
        for (int j = 0; j < 8; j++){
            if ((crc ^ c) & 0x0001) { crc = (crc >> 1) ^ CRC_POLY_16; }
            else { crc = crc >> 1; }
            c = c >> 1;
        }
        crc_tab16[i] = crc;
    }
    crc_tab16_init = true;
}
