/* USER CODE BEGIN Header */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "L3GD20.h"
#include "lsm303dlhc.h"
#include "MadgwickAHRS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum tSensorReadState
{
  eSenRead_Idle,
  eSenRead_Accel,
  eSenRead_Magn
} tI2CSensorReadState;

typedef enum
{
  cFalse,
  cTrue
} boolean;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/*	KOSTAL SofiaSoft Bulgaria EOOD
 * 	47A Tsarigradsko Shose Blvd.
 *	Sofia 1124
 *	Bulgaria					  */
/* Private variables ---------------------------------------------------------*/
//osThreadId sensorTaskHandle;

uint8_t spiTxBuf[2];
uint8_t spiRxBuf[6];
uint8_t i2cTxBuf[2];
uint8_t i2cRxBuf[6];
uint8_t magRxBuf[6];
int16_t gyroBuf[3];
int16_t accelBuf[3];
int16_t magBuf[3];
uint32_t u32LastCalc;
volatile float flAccelX;
volatile float flAccelY;
volatile float flAccelZ;
volatile float flMagX;
volatile float flMagY;
volatile float flMagZ;
float compoundErr = 0;
volatile float roll, pitch, yaw;
volatile float flAngleX;
volatile float flAngleY;
volatile float flAngleZ;
volatile uint16_t count;
const float PI = 3.1415927;
const uint8_t cCalibrationSamples = 40;
GyroStruct L3GD20Gyro;
AccelStruct LSM303Accel;
static tI2CSensorReadState eI2CSensorReadState = eSenRead_Idle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void sensorRead_5ms (void const * argument);

void L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct);
static void readGyroXYZ(GyroStruct *GyroData);
static void readMagXYZ(void);
static void readAccXYZ(AccelStruct *AccelData);
static void LSM303DLHC_Init(void);
void mag_Write(uint8_t buffer, uint8_t WriteAddr);
uint8_t mag_Read(uint8_t ReadAddr);

static void toEulerAngle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Interrupt handler for DataReady Pin of L3GD20 */
static void readGyroXYZ_IT(void);
void EXTI1_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

//	uint32_t u32TickL = 0;
//	float flTempL;

	readGyroXYZ_IT();
	//readGyroXYZ(&L3GD20Gyro);

//	if(L3GD20Gyro.u8calCnt < 200)
//	{
//		L3GD20Gyro.flgyroXOff += L3GD20Gyro.flgyroX;
//		L3GD20Gyro.flgyroYOff += L3GD20Gyro.flgyroY;
//		L3GD20Gyro.flgyroZOff += L3GD20Gyro.flgyroZ;
//		L3GD20Gyro.u8calCnt++;
//	}
//	else
//		if( L3GD20Gyro.u8calCnt == 200 )
//		{
//			L3GD20Gyro.flgyroXOff /= 200;
//			L3GD20Gyro.flgyroYOff /= 200;
//			L3GD20Gyro.flgyroZOff /= 200;
//			L3GD20Gyro.u8calCnt++ ;
//		}
//		else
//			{
//				L3GD20Gyro.flgyroX -= L3GD20Gyro.flgyroXOff;
//				L3GD20Gyro.flgyroY -= L3GD20Gyro.flgyroYOff;
//				L3GD20Gyro.flgyroZ -= L3GD20Gyro.flgyroZOff;
//				if(0 == u32LastCalc)
//				{
//					u32LastCalc = HAL_GetTick();
//				}
//				else
//				{
//					u32TickL = HAL_GetTick();
//					u32TickL -= u32LastCalc;
//					flTempL = (float)u32TickL/1000;
//					flAngleX += L3GD20Gyro.flgyroX * flTempL;
//					flAngleY += L3GD20Gyro.flgyroY * flTempL;
//					flAngleZ += L3GD20Gyro.flgyroZ * flTempL;
//					u32LastCalc = HAL_GetTick();
//				}
//			}
}

void EXTI2_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
//	readAccXYZ(&LSM303Accel);
//	if(LSM303Accel.u8calCnt < 200)
//	{
//		LSM303Accel.flaccelXOff += LSM303Accel.flaccelX;
//		LSM303Accel.flaccelYOff += LSM303Accel.flaccelY;
//		LSM303Accel.flaccelZOff += LSM303Accel.flaccelZ;
//		LSM303Accel.u8calCnt++;
//	}
//	else
//		if( LSM303Accel.u8calCnt == 200 )
//		{
//			LSM303Accel.flaccelXOff /= 200;
//			LSM303Accel.flaccelYOff /= 200;
//			LSM303Accel.flaccelZOff /= 200;
//			LSM303Accel.flaccelZOff -= 1; // According to gravity on Z axis
//			LSM303Accel.u8calCnt++ ;
//		}
//		else
//			{
//				LSM303Accel.flaccelX -= LSM303Accel.flaccelXOff;
//				LSM303Accel.flaccelY -= LSM303Accel.flaccelYOff;
//				LSM303Accel.flaccelZ -= LSM303Accel.flaccelZOff;
//			}

	//readMagXYZ();
}

void EXTI4_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
}

void accelerometer_Init(void)
{
	LSM303DLHC_Init();
}

void gyroscope_Init(void)
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
	L3GD20Gyro.u8CalCnt = 0;

	L3GD20_Init(&sGyroL);
	/* In order for the DRDY interrupt to trigger the data has to be read first*/
	readGyroXYZ(&L3GD20Gyro);
}

void magnetometer_Init(void)
{

}

void IMU_Init(void)
{
	accelerometer_Init();
	gyroscope_Init();
	magnetometer_Init();
}

void L3GD20_Write(uint8_t buffer, uint8_t WriteAddr)
{
  /* Chip select (CS) to begin */
	spiTxBuf[0] = WriteAddr;
	spiTxBuf[1] = buffer;
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spiTxBuf, 2, 50);
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

void L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct)
{
  uint8_t ctrl1 = 0x00, ctrl3 = 0x00, ctrl4 = 0x00;

  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
                    L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);

  ctrl3 |= L3GD20_INT2INTERRUPT_ENABLE;

  ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
                    L3GD20_InitStruct->Full_Scale);
  /* Write value to MEMS CTRL_REG1 register */
  L3GD20_Write(ctrl1, L3GD20_CTRL_REG1_ADDR);

  /* Write value to MEMS CTRL_REG3 register */
  L3GD20_Write(ctrl3, L3GD20_CTRL_REG3_ADDR);

  /* Write value to MEMS CTRL_REG4 register */
  L3GD20_Write(ctrl4, L3GD20_CTRL_REG4_ADDR);
}

static void readGyroXYZ(GyroStruct *GyroData)
{
	uint8_t u8RegAddrL = 0xE8;

	if(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
	{
		HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &u8RegAddrL, 1, 50);
		HAL_SPI_Receive(&hspi1, spiRxBuf, 6, 50);
		HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
	}
}

static void readGyroXYZ_IT(void)
{
	HAL_SPI_StateTypeDef state;
	uint8_t u8RegAddrL = 0xE8;

	uint8_t u8BuffL[7];
	u8BuffL[0] = 0xE8;
	u8BuffL[1] = 0;
	u8BuffL[2] = 0;
	u8BuffL[3] = 0;
	u8BuffL[4] = 0;
	u8BuffL[5] = 0;
	u8BuffL[6] = 0;
	uint8_t u8RxBuffL[7];
	u8RxBuffL[0] = 0;
	u8RxBuffL[1] = 0;
	u8RxBuffL[2] = 0;
	u8RxBuffL[3] = 0;
	u8RxBuffL[4] = 0;
	u8RxBuffL[5] = 0;
	u8RxBuffL[6] = 0;
	if(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
	{
		HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
		//HAL_SPI_Transmit(&hspi1, &u8RegAddrL, 1, 50);
		//HAL_SPI_Receive_IT(&hspi1, spiRxBuf, 6);
		HAL_SPI_TransmitReceive_IT(&hspi1, u8BuffL, u8RxBuffL,7);
	}
	else
	{
		state = HAL_SPI_GetState(&hspi1);
	}
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
//	uint32_t u32TickL = 0;
//	float flTempL;
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
	gyroBuf[0] = (spiRxBuf[0]<<8 | spiRxBuf[1]);
	gyroBuf[1] = (spiRxBuf[2]<<8 | spiRxBuf[3]);
	gyroBuf[2] = (spiRxBuf[4]<<8 | spiRxBuf[5]);

	L3GD20Gyro.flGyroX = gyroBuf[0] * 0.070;
	L3GD20Gyro.flGyroY = gyroBuf[1] * 0.070;
	L3GD20Gyro.flGyroZ = gyroBuf[2] * 0.070;

	L3GD20Gyro.flGyroX += L3GD20Gyro.flGyroX - L3GD20Gyro.flGyroXOff;
	L3GD20Gyro.flGyroY += L3GD20Gyro.flGyroY - L3GD20Gyro.flGyroYOff;
	L3GD20Gyro.flGyroZ += L3GD20Gyro.flGyroZ - L3GD20Gyro.flGyroZOff;

//	if(0 == u32LastCalc)
//	{
//		u32LastCalc = HAL_GetTick();
//	}
//		else
//		{
//			u32TickL = HAL_GetTick();
//			u32TickL -= u32LastCalc;
//			flTempL = (float)u32TickL/1000;
//			flAngleX += L3GD20Gyro.flgyroX * flTempL;
//			flAngleY += L3GD20Gyro.flgyroY * flTempL;
//			flAngleZ += L3GD20Gyro.flgyroZ * flTempL;
//			u32LastCalc = HAL_GetTick();
//		}
}

void accel_Write(uint8_t buffer, uint8_t WriteAddr)
{
	i2cTxBuf[0] = WriteAddr;
	i2cTxBuf[1] = buffer;
	HAL_I2C_Master_Transmit(&hi2c1, ACC_I2C_ADDRESS, i2cTxBuf, 2, 50);
}

uint8_t accel_Read(uint8_t ReadAddr)
{
  /* Chip select (CS) to begin */
	uint8_t value;
	HAL_I2C_Master_Transmit(&hi2c1, ACC_I2C_ADDRESS, &ReadAddr, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, ACC_I2C_ADDRESS, &value, 1, 50);
	return value;
}

void mag_Write(uint8_t buffer, uint8_t WriteAddr)
{
  /* Chip select (CS) to begin */
	i2cTxBuf[0] = WriteAddr;
	i2cTxBuf[1] = buffer;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_I2C_ADDRESS, i2cTxBuf, 2, 50);

}

uint8_t mag_Read(uint8_t ReadAddr)
{
  /* Chip select (CS) to begin */
	uint8_t value;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_I2C_ADDRESS, &ReadAddr, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, MAG_I2C_ADDRESS, &value, 1, 50);
	return value;
}

static void readAccXYZ(AccelStruct *AccelData)
{

	HAL_GPIO_WritePin(GPIOD, LD4_Pin , GPIO_PIN_SET);
	HAL_I2C_Mem_Read_IT(&hi2c1, ACC_I2C_ADDRESS, 0xA8, 1, i2cRxBuf, 6);
	HAL_GPIO_WritePin(GPIOD, LD4_Pin , GPIO_PIN_RESET);

}
static HAL_StatusTypeDef AccelRead(void)
{
	return HAL_I2C_Mem_Read_IT(&hi2c1, ACC_I2C_ADDRESS, 0xA8, 1, i2cRxBuf, 6);
}

static HAL_StatusTypeDef MagnRead(void)
{
	return HAL_I2C_Mem_Read_IT(&hi2c1, MAG_I2C_ADDRESS, 0x03, 1, i2cRxBuf, 6);
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
}

static void MagnConvData(void)
{
	magBuf[0] = ((magRxBuf[1]<<8 | magRxBuf[0])>>4);
	magBuf[0] = ( magBuf[0] & 0x800 ? magBuf[0] | 0xf000 : magBuf[0] );
	magBuf[1] = ((magRxBuf[3]<<8 | magRxBuf[2])>>4);
	magBuf[1] = ( magBuf[1] & 0x800 ? magBuf[1] | 0xf000 : magBuf[1] );
	magBuf[2] = ((magRxBuf[5]<<8 | magRxBuf[4])>>4);
	magBuf[2] = ( magBuf[2] & 0x800 ? magBuf[2] | 0xf000 : magBuf[2] );
	flMagX = (float)magBuf[0] / 450;
	flMagY = (float)magBuf[1] / 450;
	flMagZ = (float)magBuf[2] / 450;
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

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	ReadI2CSens();
}

//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//}

static void readMagXYZ(void)
{
	uint8_t i = 0x00;
	uint8_t u8adrValL = 0x00;

	for(i = 0, u8adrValL = 0x03; u8adrValL <= 0x08; i++, u8adrValL++)
	{
		magRxBuf[i] = mag_Read(u8adrValL);
	}
	magBuf[0] = ((magRxBuf[1]<<8 | magRxBuf[0])>>4);
	magBuf[0] = ( magBuf[0] & 0x800 ? magBuf[0] | 0xf000 : magBuf[0] );
	magBuf[1] = ((magRxBuf[3]<<8 | magRxBuf[2])>>4);
	magBuf[1] = ( magBuf[1] & 0x800 ? magBuf[1] | 0xf000 : magBuf[1] );
	magBuf[2] = ((magRxBuf[5]<<8 | magRxBuf[4])>>4);
	magBuf[2] = ( magBuf[2] & 0x800 ? magBuf[2] | 0xf000 : magBuf[2] );
	flMagX = (float)magBuf[0] / 450;
	flMagY = (float)magBuf[1] / 450;
	flMagZ = (float)magBuf[2] / 450;
}
static void LSM303DLHC_Init(void)
{
	accel_Write( 0x67, 0x20);
	accel_Write( 0x10, 0x22);
	accel_Write( 0x68, 0x23);

	mag_Write(0x9C, 0x00);
	mag_Write(0x80, 0x01);
	mag_Write(0x00, 0x02);


	LSM303Accel.flAccelX = 0;
	LSM303Accel.flAccelY = 0;
	LSM303Accel.flAccelZ = 0;
	LSM303Accel.flAccelXOff = 0;
	LSM303Accel.flAccelYOff = 0;
	LSM303Accel.flAccelZOff = 0;
	LSM303Accel.u8CalCnt = 0;

	/**/
}
static void toEulerAngle(void)
{
//	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
//	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
//	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);

	yaw = atan2f(2.0f*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
	pitch = -asinf(2.0f * (q1*q3 - q0*q2));
	roll = atan2f(2.0f*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);

    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   -= 5.0f; // Declination at Sofia Bulgaria
    roll *= 180.0f / PI;

}
void Madgwick_Task(void *pvParameters)
{
	TickType_t xDelay = 5 / portTICK_PERIOD_MS;
	uint8_t blink = 0;
	//vTaskDelay(500*xDelay);
//	mahony_init();
	for(;;)
	{
//		flMagX = 0;
//		flMagY = 0;
//		flMagZ = 0;

//		MadgwickAHRSupdate(-flAngleX, -flAngleY, flAngleZ,
//				LSM303Accel.flaccelX, LSM303Accel.flaccelY, LSM303Accel.flaccelZ,
//							flMagX, flMagY, flMagZ);
//		mahony_update(flAngleX, flAngleY, flAngleZ,
//						flAccelX, flAccelY, flAccelZ,
//						flMagX, flMagY, flMagZ);
//		flAngleX = 0;
//		flAngleY = 0;
//		flAngleZ = 0;
//		toEulerAngle();
		if( 0 == blink )
		{
			HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin , GPIO_PIN_SET);
			blink = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin , GPIO_PIN_RESET);
			blink = 0;
		}

		vTaskDelay(xDelay);
	}
}

void AccelMag_Task(void *pvParameters)
{
	TickType_t xDelay = 10 / portTICK_PERIOD_MS;
	vTaskDelay(5*xDelay);

	accel_Write( 0x97, 0x20);
	accel_Write( 0x10, 0x22);
	accel_Write( 0x68, 0x23);

	mag_Write(0x9C, 0x00);
	mag_Write(0x80, 0x01);
	mag_Write(0x00, 0x02);
	readAccXYZ(&LSM303Accel);

	readMagXYZ();

	for(;;)
	{

		vTaskDelay(xDelay);
	}
}

void Gyro_Task(void *pvParameters)
{
	TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
	/* In order for the DRDY interrupt to trigger the data has to be read first*/
	readGyroXYZ(&L3GD20Gyro);
	for(;;)
	{
		if( count > 1000 )
		{
			//HAL_GPIO_WritePin(GPIOD, LD3_Pin , GPIO_PIN_SET);
		}
//		if( fabs(flGyroY) > 1000 )
//		{
//			HAL_GPIO_WritePin(GPIOD, LD4_Pin , GPIO_PIN_SET);
//		}
//		if( fabs(flGyroZ) > 1000 )
//		{
//			HAL_GPIO_WritePin(GPIOD, LD5_Pin , GPIO_PIN_SET);
//		}
		count = 0;
		vTaskDelay(xDelay);
	}
}

//boolean Sensors_Calibrate(void)
//{
//  boolean bIsCalibrationDoneL = cFalse;
//  static uint8_t u8CalibrationCounterL = 0;
//  static boolean bIsSensCalibrationDoneL = cFalse;
//
//  if( cFalse == bIsSensCalibrationDoneL )
//  {
//    LSM303Accel.flaccelXOff += LSM303Accel.flaccelX;
//    LSM303Accel.flaccelYOff += LSM303Accel.flaccelY;
//    LSM303Accel.flaccelZOff += LSM303Accel.flaccelZ;
//
//    AccelRead();
//
//    u8CalibrationCounterL++;
//
//    if( cCalibrationSamples == u8CalibrationCounterL )
//    {
//      bIsCalibrationDoneL = cTrue;
//      // move out of sensor calibration section if more sensors are calibrated
//      bIsSensCalibrationDoneL = cTrue;
//
//      LSM303Accel.flaccelXOff /= cCalibrationSamples;
//      LSM303Accel.flaccelYOff /= cCalibrationSamples;
//      LSM303Accel.flaccelZOff /= cCalibrationSamples;
//    }
//  }
//
//  return bIsCalibrationDoneL;
//}

//static void ReadGyro(void)
//{
//
//}

//void sensorRead_5ms (void const * argument)
//{
//	static boolean bSensorsAreCalibratedL = cFalse;
//   /* Infinite loop */
//	for(;;)
//	{
////	    if(cFalse == bSensorsAreCalibratedL)
////	    {
////	        bSensorsAreCalibratedL = Sensors_Calibrate();
////	    }
////	    else
////	    {
//	    	ReadI2CSens();
//	    	//ReadGyro();
////	    }
//
//		vTaskDelay(5);
//	}
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  xTaskCreate(Madgwick_Task,
  			  (const char* const)"Madgwick_Task",
  			  configMINIMAL_STACK_SIZE,
  			  0,
  			  2,
  			  0);
//  xTaskCreate(Gyro_Task,
//  			  (const char* const)"gyro_task",
//  			  configMINIMAL_STACK_SIZE,
//  			  0,
//  			  2,
//  			  0);
//  xTaskCreate(AccelMag_Task,
//  			  (const char* const)"AccelMag_Task",
//  			  configMINIMAL_STACK_SIZE,
//  			  0,
//  			  2,
//  			  0);
  HAL_Delay(500);
  IMU_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT2_Pin */
  GPIO_InitStruct.Pin = INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10000);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
