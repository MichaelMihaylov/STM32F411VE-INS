
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include "L3GD20.h"
#include "lsm303dlhc.h"
#include "MadgwickAHRS.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/*	KOSTAL SofiaSoft Bulgaria EOOD
 * 	47A Tsarigradsko Shose Blvd.
 *	Sofia 1124
 *	Bulgaria					  */
/* Private variables ---------------------------------------------------------*/
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
GyroStruct L3GD20Gyro;
AccelStruct LSM303Accel;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct);
static void readGyroXYZ(GyroStruct *GyroData);
static void readMagXYZ(void);
static void readAccXYZ(AccelStruct *AccelData);
static void LSM303DLHC_Init(void);
void mag_Write(uint8_t buffer, uint8_t WriteAddr);
uint8_t mag_Read(uint8_t ReadAddr);

static void toEulerAngle(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Interrupt handler for DataReady Pin of L3GD20 */
void EXTI1_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	uint32_t u32TickL = 0;
	float flTempL;
	readGyroXYZ(&L3GD20Gyro);
	if(L3GD20Gyro.u8calCnt < 200)
	{
		L3GD20Gyro.flgyroXOff += L3GD20Gyro.flgyroX;
		L3GD20Gyro.flgyroYOff += L3GD20Gyro.flgyroY;
		L3GD20Gyro.flgyroZOff += L3GD20Gyro.flgyroZ;
		L3GD20Gyro.u8calCnt++;
	}
	else
		if( L3GD20Gyro.u8calCnt == 200 )
		{
			L3GD20Gyro.flgyroXOff /= 200;
			L3GD20Gyro.flgyroYOff /= 200;
			L3GD20Gyro.flgyroZOff /= 200;
			L3GD20Gyro.u8calCnt++ ;
		}
		else
			{
				L3GD20Gyro.flgyroX -= L3GD20Gyro.flgyroXOff;
				L3GD20Gyro.flgyroY -= L3GD20Gyro.flgyroYOff;
				L3GD20Gyro.flgyroZ -= L3GD20Gyro.flgyroZOff;
				if(0 == u32LastCalc)
				{
					u32LastCalc = HAL_GetTick();
				}
				else
				{
					u32TickL = HAL_GetTick();
					u32TickL -= u32LastCalc;
					flTempL = (float)u32TickL/1000;
					flAngleX += L3GD20Gyro.flgyroX * flTempL;
					flAngleY += L3GD20Gyro.flgyroY * flTempL;
					flAngleZ += L3GD20Gyro.flgyroZ * flTempL;
					u32LastCalc = HAL_GetTick();
				}
			}
}

void EXTI2_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
	readAccXYZ(&LSM303Accel);
	if(LSM303Accel.u8calCnt < 200)
	{
		LSM303Accel.flaccelXOff += LSM303Accel.flaccelX;
		LSM303Accel.flaccelYOff += LSM303Accel.flaccelY;
		LSM303Accel.flaccelZOff += LSM303Accel.flaccelZ;
		LSM303Accel.u8calCnt++;
	}
	else
		if( LSM303Accel.u8calCnt == 200 )
		{
			LSM303Accel.flaccelXOff /= 200;
			LSM303Accel.flaccelYOff /= 200;
			LSM303Accel.flaccelZOff /= 200;
			LSM303Accel.flaccelZOff -= 1; // According to gravity on Z axis
			LSM303Accel.u8calCnt++ ;
		}
		else
			{
				LSM303Accel.flaccelX -= LSM303Accel.flaccelXOff;
				LSM303Accel.flaccelY -= LSM303Accel.flaccelYOff;
				LSM303Accel.flaccelZ -= LSM303Accel.flaccelZOff;
			}

	readMagXYZ();
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
	L3GD20Gyro.flgyroX = 0;
	L3GD20Gyro.flgyroY = 0;
	L3GD20Gyro.flgyroZ = 0;
	L3GD20Gyro.flgyroXOff = 0;
	L3GD20Gyro.flgyroYOff = 0;
	L3GD20Gyro.flgyroZOff = 0;
	L3GD20Gyro.u8calCnt = 0;

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
	uint16_t i = 0;
	uint8_t u8testValL = 0;
	HAL_StatusTypeDef status = HAL_OK;

	for(i = 0, u8testValL = 0xA8 ; u8testValL < 0xAE ; i++, u8testValL++)
	{
		spiTxBuf[0] = u8testValL;
		HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

		status = HAL_SPI_Transmit(&hspi1, &spiTxBuf[0], 1, 50);
		if(HAL_OK == status)
		{
			status = HAL_SPI_Receive(&hspi1, &spiRxBuf[i], 1, 50);
			if(HAL_OK == status)
			{
				//HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin , GPIO_PIN_SET);
			}
		}
		HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
	}

	gyroBuf[0] = (spiRxBuf[0]<<8 | spiRxBuf[1]);
	gyroBuf[1] = (spiRxBuf[2]<<8 | spiRxBuf[3]);
	gyroBuf[2] = (spiRxBuf[4]<<8 | spiRxBuf[5]);

	GyroData->flgyroX = gyroBuf[0] * 0.070;
	GyroData->flgyroY = gyroBuf[1] * 0.070;
	GyroData->flgyroZ = gyroBuf[2] * 0.070;
}

void accel_Write(uint8_t buffer, uint8_t WriteAddr)
{
  /* Chip select (CS) to begin */
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
	uint8_t i = 0x00;
	uint8_t u8adrValL = 0x00;

	for(i = 0, u8adrValL = 0x28; u8adrValL <= 0x2D; i++, u8adrValL++)
	{
		i2cRxBuf[i] = accel_Read(u8adrValL);
	}

	accelBuf[0] = ((i2cRxBuf[0]<<8 | i2cRxBuf[1])>>4);
	accelBuf[0] = ( accelBuf[0] & 0x800 ? accelBuf[0] | 0xf000 : accelBuf[0] );
	accelBuf[1] = ((i2cRxBuf[2]<<8 | i2cRxBuf[3])>>4);
	accelBuf[1] = ( accelBuf[1] & 0x800 ? accelBuf[1] | 0xf000 : accelBuf[1] );
	accelBuf[2] = ((i2cRxBuf[4]<<8 | i2cRxBuf[5])>>4);
	accelBuf[2] = ( accelBuf[2] & 0x800 ? accelBuf[2] | 0xf000 : accelBuf[2] );


	AccelData->flaccelX = accelBuf[0] * (0.004);
	AccelData->flaccelY = accelBuf[1] * (0.004);
	AccelData->flaccelZ = accelBuf[2] * (0.004);
}
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


	LSM303Accel.flaccelX = 0;
	LSM303Accel.flaccelY = 0;
	LSM303Accel.flaccelZ = 0;
	LSM303Accel.flaccelXOff = 0;
	LSM303Accel.flaccelYOff = 0;
	LSM303Accel.flaccelZOff = 0;
	LSM303Accel.u8calCnt = 0;

	/**/
	readAccXYZ(&LSM303Accel);
	readMagXYZ();
}
static void toEulerAngle(void)
{
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
}
void Madgwick_Task(void *pvParameters)
{
	TickType_t xDelay = 5 / portTICK_PERIOD_MS;
	vTaskDelay(500*xDelay);
//	mahony_init();
	for(;;)
	{
//		flMagX = 0;
//		flMagY = 0;
//		flMagZ = 0;
		MadgwickAHRSupdate(flAngleX, flAngleY, flAngleZ,
							flAccelX, flAccelY, flAccelZ,
							flMagX, flMagY, flMagZ);
//		mahony_update(flAngleX, flAngleY, flAngleZ,
//						flAccelX, flAccelY, flAccelZ,
//						flMagX, flMagY, flMagZ);
		flAngleX = 0;
		flAngleY = 0;
		flAngleZ = 0;
		toEulerAngle();

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
			HAL_GPIO_WritePin(GPIOD, LD3_Pin , GPIO_PIN_SET);
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  xTaskCreate(Madgwick_Task,
  			  (const char* const)"madgwick_task",
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB_OTG_FS init function */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
