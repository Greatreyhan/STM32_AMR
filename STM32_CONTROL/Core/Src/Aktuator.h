/*
 * Aktuator.h
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_AKTUATOR_H_
#define SRC_AKTUATOR_H_

#include "main.h"

typedef struct{
	GPIO_TypeDef*				PORT_IN1;
	GPIO_TypeDef*				PORT_IN2;
	GPIO_TypeDef*				PORT_IN3;
	GPIO_TypeDef*				PORT_IN4;
	uint16_t						PIN_IN1;
	uint16_t						PIN_IN2;
	uint16_t						PIN_IN3;
	uint16_t						PIN_IN4;
}aktuator_t;

void aktuator_up(aktuator_t drv);
void aktuator_down(aktuator_t drv);
void aktuator_reset(aktuator_t drv);

#endif /* SRC_AKTUATOR_H_ */
