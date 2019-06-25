#include "stm32f4xx_hal.h"
#include "main.h"
#include "BNO055_def.h"

typedef enum tSensorReadState
{
  eSenRead_Idle,
  eSenRead_Gyro,
  eSenRead_Accel,
  eSenRead_Magn,
  eSenRed_Quat
} tSensorReadState;

volatile int16_t qw = 0, qx = 0, qy = 0, qz = 0;

static tSensorReadState eI2CSensorReadState = eSenRead_Idle;
static I2C_HandleTypeDef * pI2CHandle;
static uint8_t i2cRxBuf[8];

static void QuatConvData(void);
static void BNO_Init(void);
static void BNO_Write(uint8_t buffer, uint8_t WriteAddr);
static void ReadI2CSens(void);
static HAL_StatusTypeDef QuatRead(void);

void BNO055_Init(I2C_HandleTypeDef * pI2CHandleP)
{
	pI2CHandle = pI2CHandleP;

	BNO_Init();
}

void BNO055_MainFunction(void)
{
	ReadI2CSens();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	ReadI2CSens();
}

static void BNO_Init(void)
{
	BNO_Write( BNO055_POWER_MODE_NORMAL,BNO055_PWR_MODE_ADDR);
	BNO_Write( BNO055_OPERATION_MODE_NDOF,BNO055_OPR_MODE_ADDR);
}

static void BNO_Write(uint8_t buffer, uint8_t WriteAddr)
{
	uint8_t TxBuff[2];
	TxBuff[0] = WriteAddr;
	TxBuff[1] = buffer;
	HAL_I2C_Master_Transmit(pI2CHandle, BNO055_I2C_ADDR1, TxBuff, 2, 50);
}

static void ReadI2CSens(void)
{
	HAL_StatusTypeDef HalResultL = HAL_OK;

	switch(eI2CSensorReadState)
	{
	case eSenRead_Idle:
		HalResultL = QuatRead();
		eI2CSensorReadState = (HAL_OK == HalResultL) ? eSenRed_Quat : eSenRead_Idle;
		break;
	case eSenRed_Quat:
		QuatConvData();
		eI2CSensorReadState = eSenRead_Idle;
		break;
	default:
		eI2CSensorReadState = eSenRead_Idle;
		break;
	}
}

static HAL_StatusTypeDef QuatRead(void)
{
	return HAL_I2C_Mem_Read_IT(pI2CHandle, BNO055_I2C_ADDR1, BNO055_QUATERNION_DATA_W_LSB_ADDR, 1, i2cRxBuf, 8);
}

static void QuatConvData(void)
{
	qw = (int16_t)(i2cRxBuf[0] | ((uint16_t)i2cRxBuf[1] << 8));
	qx = (int16_t)(i2cRxBuf[2] | ((uint16_t)i2cRxBuf[3] << 8));
	qy = (int16_t)(i2cRxBuf[3] | ((uint16_t)i2cRxBuf[5] << 8));
	qz = (int16_t)(i2cRxBuf[4] | ((uint16_t)i2cRxBuf[7] << 8));
}
