/*
 * IMU_Sensors.h
 *
 *  Created on: 13.02.2019 ã.
 *      Author: mihay002
 */

#ifndef IMU_SENSORS_H_
#define IMU_SENSORS_H_

void IMU_Sensors_MainFunction(void);

void IMU_Sensors_Init(I2C_HandleTypeDef * pI2CHandleP, SPI_HandleTypeDef * pSPIHandleP);

boolean Sensors_Calibrate(void);

void IMU_Sensors_GetGyroData(float * flGyroP);
void IMU_Sensors_GetAccelData(float * flAccelP);
void IMU_Sensors_GetMagnData(float * flMagnP);


#endif /* IMU_SENSORS_H_ */
