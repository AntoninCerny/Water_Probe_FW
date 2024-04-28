/*
 * SHT4.c
 *
 *  Created on: 6. 3. 2024
 *      Author: Antonín
 */
#include "sht4.h"

void SHT4x_init(){
	//SHT4x_result results;
}


SHT4x_result SHT4x_Humidity_Meas(I2C_HandleTypeDef i2c1){
	uint8_t buf[12];
	uint8_t command = HUMIDITY_COMMAND_HIGH_PRESS;
	SHT4x_result results;

	uint8_t checksum_t = 0;
	uint32_t t_ticks = 0;
	float t_degC = 0;
	float rh_pRH = 0;
	uint32_t rh_ticks = 0;
	uint8_t checksum_rh = 0;

	HAL_I2C_Master_Transmit(&i2c1, SHT4x_ADDR, &command, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Receive(&i2c1, SHT4x_ADDR, buf, 6, HAL_MAX_DELAY);


	t_ticks = buf[0]*256 + buf[1];
	checksum_t = buf[2];
	rh_ticks = buf[3] * 256 + buf[4];
	checksum_rh = buf[5];
	t_degC = -45 + 175 * (float)t_ticks/65535;
	rh_pRH = -6 + 125 * (float)rh_ticks/65535;
	results.temp = t_degC;
	results.humidity = rh_pRH;
	return results;

}



