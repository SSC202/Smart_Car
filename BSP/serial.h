#ifndef __SERIAL_H
#define __SERIAL_H

#include "usart.h"

typedef union Pack_Float {
    float float_data;
    uint8_t float_byte[4];
} Pack_Float;

extern Pack_Float ball_x;
extern Pack_Float ball_y;
extern Pack_Float ball_r;
extern uint8_t receive_buffer[1];

void Serial_Decode();

#endif