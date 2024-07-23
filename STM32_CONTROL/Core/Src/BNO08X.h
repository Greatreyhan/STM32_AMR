/*
 * BNO08X.h
 *
 *  Created on: Feb 12, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_BNO08X_H_
#define SRC_BNO08X_H_

#include "main.h"

typedef struct {
    uint8_t header;
    uint8_t index;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t x_acceleration;
    int16_t y_acceleration;
    int16_t z_acceleration;
    uint8_t mi_mr_reserved[3];
    uint8_t checksum;

    int16_t setpoint_yaw;
} BNO08X_Typedef;

void BNO08X_GetData(BNO08X_Typedef *sensorData);
void BNO08X_Init(UART_HandleTypeDef *huart);
void BNO08X_Set_Init_Yaw(BNO08X_Typedef *sensorData);
int16_t BNO08X_relative_yaw(int16_t current_position, int16_t reading);


#endif /* SRC_BNO08X_H_ */
