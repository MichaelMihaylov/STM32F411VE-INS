/*
 * IMU_Sensors.h
 *
 *  Created on: 13.02.2019 ã.
 *      Author: mihay002
 */

#ifndef IMU_SENSORS_H_
#define IMU_SENSORS_H_

#include "Typedef.h"

void IMU_Sensors_MainFunction(void);

void IMU_Sensors_Init(I2C_HandleTypeDef * pI2CHandleP, SPI_HandleTypeDef * pSPIHandleP);

boolean Sensors_Calibrate(void);

void IMU_Sensors_GetGyroData(float * flGyroXP, float * flGyroYP, float * flGyroZP);
void IMU_Sensors_GetAccelData(float * flAccelXP, float * flAccelYP, float * flAccelZP);
void IMU_Sensors_GetMagnData(float * flMagnXP, float * flMagnYP, float * flMagnZP);


#endif /* IMU_SENSORS_H_ */
