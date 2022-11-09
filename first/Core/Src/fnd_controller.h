/*
 * fnd_controller.h
 *
 *  Created on: Oct 23, 2022
 *      Author: JHKIm
 */

#ifndef SRC_FND_CONTROLLER_H_
#define SRC_FND_CONTROLLER_H_

#include "main.h"


#define HIGH 1
#define LOW 0

#define false 0
#define true 1



void send(uint8_t x);

//void init_fnd(SPI_HandleTypeDef * hspi);
void init_fnd();

void send_port(uint8_t x, uint8_t port);

void digit4_show(int n, int replay, uint8_t showZero);

void digit4_replay(int n, int replay);

void digit4(int n);

void digit4showZero_replay(int n, int replay);

void digit4showZero(int n);

void digit2(int n, int port, int replay);

void digit2_port(int n, int port);

void digit4_temper(int n, int replay);


#endif /* SRC_FND_CONTROLLER_H_ */
