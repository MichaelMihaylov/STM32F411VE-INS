/*
 * Typedef.h
 *
 *  Created on: 14.02.2019 ã.
 *      Author: mihay002
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

typedef enum
{
  cFalse,
  cTrue
} boolean;

typedef enum tSensorReadState
{
  eSenRead_Idle,
  eSenRead_Gyro,
  eSenRead_Accel,
  eSenRead_Magn
} tSensorReadState;


#endif /* TYPEDEF_H_ */
