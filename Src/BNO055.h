/*
 * BNO055.h
 *
 *  Created on: Jun 24, 2019
 *      Author: Michael
 */

#ifndef BNO055_H_
#define BNO055_H_

extern volatile int16_t qw, qx, qy, qz;

extern void BNO055_MainFunction(void);
extern void BNO055_Init(I2C_HandleTypeDef * pI2CHandleP);

#endif /* BNO055_H_ */
