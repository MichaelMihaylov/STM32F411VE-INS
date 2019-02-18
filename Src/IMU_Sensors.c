/*
 * IMU_Sensors.c
 *
 *  Created on: 12.02.2019 ã.
 *      Author: mihay002
 */
#include "stm32f4xx_hal.h"
#include <math.h>
#include "lsm303dlhc.h"
#include "L3GD20.h"
#include "main.h"

#include "Typedef.h"


#define CALIB_SAMPLES 100
/* Private function definitions */
 
static void ReadI2CSens(void);

static HAL_StatusTypeDef AccelRead(void);
static HAL_StatusTypeDef MagnRead(void);
static void AccelConvData(void);
static void MagnConvData(void);
static void Accel_Write(uint8_t buffer, uint8_t WriteAddr);
static void Magn_Write(uint8_t buffer, uint8_t WriteAddr);
static void LSM303DLHC_Accel_Init(void);
static void LSM303DLHC_Magn_Init(void);
static void Gyro_L3GD20_Write(uint8_t buffer, uint8_t WriteAddr);
static void Gyro_L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct);
static void Accel_Init(void);
static void Magn_Init(void);
static void Gyro_Init(void);
static void ReadSPISens(void);
static void L3GD20_ReadXYZ(void);
static void ReadSPISens(void);
static void GyroConvData(void);
static void SetMinMax(float * flMinP, float * flMaxP, float * flAxisP);

/* Private Data */

static uint8_t spiRxBuf[6];
static uint8_t i2cRxBuf[6];

//static int16_t gyroBuf[3];
static int16_t accelBuf[3];
static int16_t magBuf[3];


static AccelStruct LSM303Accel;
static MagStruct LSM303Magn;
static GyroStruct L3GD20Gyro;

static tSensorReadState eI2CSensorReadState = eSenRead_Idle;

static I2C_HandleTypeDef * pI2CHandle;
static SPI_HandleTypeDef * pSPIHandle;

/* Public functions --------------------------------------------------------------------------------------------------------------------*/
void EXTI1_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

	//L3GD20_ReadXYZ();
}

void IMU_Sensors_MainFunction(void)
{
	ReadI2CSens();
	ReadSPISens();
}

void IMU_Sensors_Init(I2C_HandleTypeDef * pI2CHandleP, SPI_HandleTypeDef * pSPIHandleP)
{
	pI2CHandle = pI2CHandleP;
	pSPIHandle = pSPIHandleP;

	Accel_Init();
	Gyro_Init();
	Magn_Init();
}

void IMU_Sensors_MagnCalibrate(void)
{
	SetMinMax(&LSM303Magn.flMagnXMin, &LSM303Magn.flMagnXMax, &LSM303Magn.flMagnX);
	SetMinMax(&LSM303Magn.flMagnYMin, &LSM303Magn.flMagnYMax, &LSM303Magn.flMagnY);
	SetMinMax(&LSM303Magn.flMagnZMin, &LSM303Magn.flMagnZMax, &LSM303Magn.flMagnZ);

}

boolean Sensors_Calibrate(void)
{
	static uint8_t u8CalibrationSamples = 0;
	boolean bResultL = cFalse;
	
	if(CALIB_SAMPLES > u8CalibrationSamples)
	{		
		ReadI2CSens();
		ReadSPISens();

		L3GD20Gyro.flGyroXOff += L3GD20Gyro.flGyroX;
		L3GD20Gyro.flGyroYOff += L3GD20Gyro.flGyroY;
		L3GD20Gyro.flGyroZOff += L3GD20Gyro.flGyroZ;
		
		LSM303Accel.flAccelXOff += LSM303Accel.flAccelX;
		LSM303Accel.flAccelYOff += LSM303Accel.flAccelY;
		LSM303Accel.flAccelZOff += LSM303Accel.flAccelZ;
		
		u8CalibrationSamples++;
		bResultL = cFalse;
	}
	else
	{
		L3GD20Gyro.flGyroXOff /= u8CalibrationSamples;
		L3GD20Gyro.flGyroYOff /= u8CalibrationSamples;
		L3GD20Gyro.flGyroZOff /= u8CalibrationSamples;
		
		LSM303Accel.flAccelXOff /= u8CalibrationSamples;
		LSM303Accel.flAccelYOff /= u8CalibrationSamples;
		LSM303Accel.flAccelZOff /= u8CalibrationSamples;

		/* To account for gravity on Z axis */
		LSM303Accel.flAccelZOff --;
		
		bResultL = cTrue;
	}
	return bResultL;
}

void IMU_Sensors_GetGyroData(float * flGyroXP, float * flGyroYP, float * flGyroZP)
{
	*flGyroXP = L3GD20Gyro.flGyroX - L3GD20Gyro.flGyroXOff;
	*flGyroYP = L3GD20Gyro.flGyroY - L3GD20Gyro.flGyroYOff;
	*flGyroZP = L3GD20Gyro.flGyroZ - L3GD20Gyro.flGyroZOff;
}

void IMU_Sensors_GetAccelData(float * flAccelXP, float * flAccelYP, float * flAccelZP)
{
	*flAccelXP = LSM303Accel.flAccelX - LSM303Accel.flAccelXOff;
	*flAccelYP = LSM303Accel.flAccelY - LSM303Accel.flAccelYOff;
	*flAccelZP = LSM303Accel.flAccelZ - LSM303Accel.flAccelZOff;
}

void IMU_Sensors_GetMagnData(float * flMagnXP, float * flMagnYP, float * flMagnZP)
{
	float flDeltaXL, flDeltaYL, flDeltaZL;
	float flOffXL, flOffYL, flOffZL;

	SetMinMax(&LSM303Magn.flMagnXMin, &LSM303Magn.flMagnXMax, &LSM303Magn.flMagnX);
	SetMinMax(&LSM303Magn.flMagnYMin, &LSM303Magn.flMagnYMax, &LSM303Magn.flMagnY);
	SetMinMax(&LSM303Magn.flMagnZMin, &LSM303Magn.flMagnZMax, &LSM303Magn.flMagnZ);

	flDeltaXL = LSM303Magn.flMagnXMax - LSM303Magn.flMagnXMin;
	flDeltaYL = LSM303Magn.flMagnYMax - LSM303Magn.flMagnYMin;
	flDeltaZL = LSM303Magn.flMagnZMax - LSM303Magn.flMagnZMin;

	flOffXL = (LSM303Magn.flMagnXMax + LSM303Magn.flMagnXMin)/ 2.0f;
	flOffYL = (LSM303Magn.flMagnYMax + LSM303Magn.flMagnYMin)/ 2.0f;
	flOffZL = (LSM303Magn.flMagnZMax + LSM303Magn.flMagnZMin)/ 2.0f;

	*flMagnXP = (LSM303Magn.flMagnX - flOffXL) * (flDeltaXL / LSM303Magn.flMaxDelta);
	*flMagnYP = (LSM303Magn.flMagnY - flOffYL) * (flDeltaYL / LSM303Magn.flMaxDelta);
	*flMagnZP = (LSM303Magn.flMagnZ - flOffZL) * (flDeltaZL / LSM303Magn.flMaxDelta);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	ReadI2CSens();
}

/*void EXTI1_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	
	L3GD20_ReadXYZ();
}*/



/* Private functions -------------------------------------------------------------------------------------------------------------------*/
static void SetMinMax(float * flMinP, float * flMaxP, float * flAxisP)
{
	if(*flAxisP > *flMaxP)
	{
		*flMaxP = *flAxisP;
	}

	if(*flAxisP < *flMinP)
	{
		*flMinP = *flAxisP;
	}

	if( LSM303Magn.flMaxDelta < (*flMaxP - *flMinP))
	{
		LSM303Magn.flMaxDelta = *flMaxP - *flMinP;
	}
}

static void Accel_Init(void)
{
	LSM303DLHC_Accel_Init();
}

static void Magn_Init(void)
{
	LSM303DLHC_Magn_Init();
}

static void Gyro_Init(void)
{
	L3GD20_InitTypeDef sGyroL;

	sGyroL.Power_Mode = L3GD20_MODE_ACTIVE;
	sGyroL.Output_DataRate = L3GD20_OUTPUT_DATARATE_4;
	sGyroL.Axes_Enable = L3GD20_AXES_ENABLE;
	sGyroL.Band_Width = L3GD20_BANDWIDTH_4;
	sGyroL.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	sGyroL.Endianness = L3GD20_BLE_MSB;
	sGyroL.Full_Scale = L3GD20_FULLSCALE_2000;
	L3GD20Gyro.flGyroX = 0;
	L3GD20Gyro.flGyroY = 0;
	L3GD20Gyro.flGyroZ = 0;
	L3GD20Gyro.flGyroXOff = 0;
	L3GD20Gyro.flGyroYOff = 0;
	L3GD20Gyro.flGyroZOff = 0;
	L3GD20Gyro.s32GyroXRaw = 0;
	L3GD20Gyro.s32GyroYRaw = 0;
	L3GD20Gyro.s32GyroZRaw = 0;
	L3GD20Gyro.u8CalCnt = 0;

	Gyro_L3GD20_Init(&sGyroL);
}

static void Gyro_L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct)
{
	uint8_t ctrl1 = 0x00, ctrl3 = 0x00, ctrl4 = 0x00;

	/* Configure MEMS: data rate, power mode, full scale and axes */
	ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
						L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);
	
	ctrl3 |= L3GD20_INT2INTERRUPT_ENABLE;
	
	ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
						L3GD20_InitStruct->Full_Scale);
	/* Write value to MEMS CTRL_REG1 register */
	Gyro_L3GD20_Write(ctrl1, L3GD20_CTRL_REG1_ADDR);
	
	/* Write value to MEMS CTRL_REG3 register */
	Gyro_L3GD20_Write(ctrl3, L3GD20_CTRL_REG3_ADDR);
	
	/* Write value to MEMS CTRL_REG4 register */
	Gyro_L3GD20_Write(ctrl4, L3GD20_CTRL_REG4_ADDR);
}

static void Gyro_L3GD20_Write(uint8_t buffer, uint8_t WriteAddr)
{
	uint8_t txBuf[2];
	
	txBuf[0] = WriteAddr;
	txBuf[1] = buffer;
	
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pSPIHandle, txBuf, 2, 50);
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

static void LSM303DLHC_Accel_Init(void)
{
	Accel_Write( 0x67, 0x20);
	Accel_Write( 0x10, 0x22);
	Accel_Write( 0x68, 0x23);
	
	LSM303Accel.flAccelX = 0;
	LSM303Accel.flAccelY = 0;
	LSM303Accel.flAccelZ = 0;
	LSM303Accel.flAccelXOff = 0;
	LSM303Accel.flAccelYOff = 0;
	LSM303Accel.flAccelZOff = 0;
	LSM303Accel.u8CalCnt = 0;
}

static void LSM303DLHC_Magn_Init(void)
{
	Magn_Write(0x9C, 0x00);
	Magn_Write(0xE0, 0x01);
	Magn_Write(0x00, 0x02);
	
	LSM303Magn.flMagnX = 0;
	LSM303Magn.flMagnY = 0;
	LSM303Magn.flMagnZ = 0;
	LSM303Magn.u8CalCnt = 0;
}

static void Accel_Write(uint8_t buffer, uint8_t WriteAddr)
{
	uint8_t TxBuff[2];
	TxBuff[0] = WriteAddr;
	TxBuff[1] = buffer;
	HAL_I2C_Master_Transmit(pI2CHandle, ACC_I2C_ADDRESS, TxBuff, 2, 50);
}

static void Magn_Write(uint8_t buffer, uint8_t WriteAddr)
{
	uint8_t TxBuff[2];
	TxBuff[0] = WriteAddr;
	TxBuff[1] = buffer;
	HAL_I2C_Master_Transmit(pI2CHandle, MAG_I2C_ADDRESS, TxBuff, 2, 50);
}

static void ReadI2CSens(void)
{
	HAL_StatusTypeDef HalResultL = HAL_OK;

	switch(eI2CSensorReadState)
	{
	case eSenRead_Idle:
		HalResultL = AccelRead();
		eI2CSensorReadState = (HAL_OK == HalResultL) ? eSenRead_Accel : eSenRead_Idle;
		break;
	case eSenRead_Accel:
		AccelConvData();
		HalResultL = MagnRead();
		eI2CSensorReadState = (HAL_OK == HalResultL) ? eSenRead_Magn : eSenRead_Idle;
		break;
	case eSenRead_Magn:
		MagnConvData();
		eI2CSensorReadState = eSenRead_Idle;
		break;
	default:
		eI2CSensorReadState = eSenRead_Idle;
		break;
	}
}

static void ReadSPISens(void)
{
	L3GD20_ReadXYZ();
	GyroConvData();
}

static void L3GD20_ReadXYZ(void)
{
	uint8_t u8RegAddrL = 0xE8;

	if(HAL_SPI_GetState(pSPIHandle) == HAL_SPI_STATE_READY)
	{
		HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(pSPIHandle, &u8RegAddrL, 1, 50);
		HAL_SPI_Receive(pSPIHandle, spiRxBuf, 6, 50);
		HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
	}

//	L3GD20Gyro.s32GyroXRaw = (spiRxBuf[0]<<8 | spiRxBuf[1]);
//	L3GD20Gyro.s32GyroYRaw = (spiRxBuf[2]<<8 | spiRxBuf[3]);
//	L3GD20Gyro.s32GyroZRaw = (spiRxBuf[4]<<8 | spiRxBuf[5]);
	//L3GD20Gyro.u8CalCnt++;
}

static HAL_StatusTypeDef AccelRead(void)
{
	return HAL_I2C_Mem_Read_IT(pI2CHandle, ACC_I2C_ADDRESS, 0xA8, 1, i2cRxBuf, 6);
}

static HAL_StatusTypeDef MagnRead(void)
{
	return HAL_I2C_Mem_Read_IT(pI2CHandle, MAG_I2C_ADDRESS, 0x03, 1, i2cRxBuf, 6);
}

static void AccelConvData(void)
{
		accelBuf[0] = ((i2cRxBuf[0]<<8 | i2cRxBuf[1])>>4);
		accelBuf[0] = ( accelBuf[0] & 0x800 ? accelBuf[0] | 0xf000 : accelBuf[0] );

		accelBuf[1] = ((i2cRxBuf[2]<<8 | i2cRxBuf[3])>>4);
		accelBuf[1] = ( accelBuf[1] & 0x800 ? accelBuf[1] | 0xf000 : accelBuf[1] );

		accelBuf[2] = ((i2cRxBuf[4]<<8 | i2cRxBuf[5])>>4);
		accelBuf[2] = ( accelBuf[2] & 0x800 ? accelBuf[2] | 0xf000 : accelBuf[2] );


		LSM303Accel.flAccelX = accelBuf[0] * (0.004);
		LSM303Accel.flAccelY = accelBuf[1] * (0.004);
		LSM303Accel.flAccelZ = accelBuf[2] * (0.004);
		
//		LSM303Accel.flAccelX -= LSM303Accel.flAccelXOff;
//		LSM303Accel.flAccelY -= LSM303Accel.flAccelYOff;
//		LSM303Accel.flAccelZ -= LSM303Accel.flAccelZOff;
}

static void MagnConvData(void)
{
	magBuf[0] = ((i2cRxBuf[1]<<8 | i2cRxBuf[0])>>4);
	magBuf[0] = ( magBuf[0] & 0x800 ? magBuf[0] | 0xf000 : magBuf[0] );
	magBuf[1] = ((i2cRxBuf[3]<<8 | i2cRxBuf[2])>>4);
	magBuf[1] = ( magBuf[1] & 0x800 ? magBuf[1] | 0xf000 : magBuf[1] );
	magBuf[2] = ((i2cRxBuf[5]<<8 | i2cRxBuf[4])>>4);
	magBuf[2] = ( magBuf[2] & 0x800 ? magBuf[2] | 0xf000 : magBuf[2] );
	
	LSM303Magn.flMagnX = (float)magBuf[0] / 230;
	LSM303Magn.flMagnY = (float)magBuf[1] / 230;
	LSM303Magn.flMagnZ = (float)magBuf[2] / 205;
}

static void GyroConvData(void)
{
	/* Calculate the average Gyro value from the values aquired asynchronously */
	int16_t s16GyroXL, s16GyroYL, s16GyroZL;
	
	s16GyroXL = (spiRxBuf[0]<<8 | spiRxBuf[1]);
	s16GyroYL = (spiRxBuf[2]<<8 | spiRxBuf[3]);
	s16GyroZL = (spiRxBuf[4]<<8 | spiRxBuf[5]);
	
	L3GD20Gyro.flGyroX = (s16GyroXL * 0.070);// / L3GD20Gyro.u8CalCnt;
	L3GD20Gyro.flGyroY = (s16GyroYL * 0.070);// / L3GD20Gyro.u8CalCnt;
	L3GD20Gyro.flGyroZ = (s16GyroZL * 0.070);// / L3GD20Gyro.u8CalCnt;

	/* Reset the sums of the asynchronously aquired gyro data and the number of aquisitions counter */
//	L3GD20Gyro.s32GyroXRaw = 0;
//	L3GD20Gyro.s32GyroYRaw = 0;
//	L3GD20Gyro.s32GyroZRaw = 0;
//	L3GD20Gyro.u8CalCnt = 0;
	/* Apply calibration corrections */
//	L3GD20Gyro.flGyroX -= L3GD20Gyro.flGyroXOff;
//	L3GD20Gyro.flGyroY -= L3GD20Gyro.flGyroYOff;
//	L3GD20Gyro.flGyroZ -= L3GD20Gyro.flGyroZOff;
}
