/*
 * sht4.h
 *
 *  Created on: 6. 3. 2024
 *      Author: Antonín
 */

#ifndef INC_SHT4_H_
#define INC_SHT4_H_

#include "main.h"



#define SHT4x_ADDR (0x46<<1)
#define HUMIDITY_COMMAND_HIGH_PRESS 0xFD

typedef struct {
	float temp;
	float humidity;
}SHT4x_result;



SHT4x_result SHT4x_Humidity_Meas(I2C_HandleTypeDef i2c1);
void SHT4x_init();

#endif /* INC_SHT4_H_ */
