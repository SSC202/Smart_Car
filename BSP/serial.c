#include "serial.h"

uint8_t receive_buffer[1];
uint8_t receive_data[12] = {0};
uint8_t ball_data[12]    = {0};

Pack_Float ball_x;
Pack_Float ball_y;
Pack_Float ball_r;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static int count = 0;
    if (huart->Instance == USART3) {
        switch (count) {
            case 0:
                if (receive_buffer[0] == 0xFF) {
                    count++;
                } else {
                    count = 0;
                }
                break;
            case 1:
                receive_data[0] = receive_buffer[0];
                count++;
                break;
            case 2:
                receive_data[1] = receive_buffer[0];
                count++;
                break;
            case 3:
                receive_data[2] = receive_buffer[0];
                count++;
                break;
            case 4:
                receive_data[3] = receive_buffer[0];
                count++;
                break;
            case 5:
                receive_data[4] = receive_buffer[0];
                count++;
                break;
            case 6:
                receive_data[5] = receive_buffer[0];
                count++;
                break;
            case 7:
                receive_data[6] = receive_buffer[0];
                count++;
                break;
            case 8:
                receive_data[7] = receive_buffer[0];
                count++;
                break;
            case 9:
                receive_data[8] = receive_buffer[0];
                count++;
                break;
            case 10:
                receive_data[9] = receive_buffer[0];
                count++;
                break;
            case 11:
                receive_data[10] = receive_buffer[0];
                count++;
                break;
            case 12:
                receive_data[11] = receive_buffer[0];
                count++;
                break;
            case 13:
                if (receive_buffer[0] == 0xEE) {
                    for (int i = 0; i < 12; i++) {
                        ball_data[i] = receive_data[i];
                    }
                    for (int i = 0; i < 12; i++) {
                        receive_data[i] = 0;
                    }
                    count = 0;
                } else {
                    for (int i = 0; i < 12; i++) {
                        receive_data[i] = 0;
                    }
                    count = 0;
                }
                break;
            default:
                count = 0;
                break;
        }
        HAL_UART_Receive_IT(&huart3, (uint8_t *)receive_buffer, 1);
    }
}

void Serial_Decode()
{
    for (int i = 0; i < 4; i++) {
        ball_x.float_byte[i] = ball_data[i];
    }
    for (int i = 4; i < 8; i++) {
        ball_y.float_byte[i - 4] = ball_data[i];
    }
    for (int i = 8; i < 12; i++) {
        ball_r.float_byte[i - 8] = ball_data[i];
    }
}