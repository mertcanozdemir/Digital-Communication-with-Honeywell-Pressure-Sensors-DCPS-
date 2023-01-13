/*******************************************************************************
 * Title                 :   DCPS Digital Communication with Honeywell Pressure Sensors(DCPS)
 * Filename              :   DCPS.c
 * Author                :   Mertcan
 * Origin Date           :   Dec 27, 2022
 * Version               :		
 * Compiler              :	 STMCube IDE
 * Target                :
 * Notes                 :
 *******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
 *
 *    Date    Software Version    Initials   Description
 *  Dec 27, 2022                  Module Created.
 *
 *******************************************************************************/
/** \file DCPS.c
 * \brief This module contains the Digital Communication of Pressure Sensors(DCPS)
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
/**
 * #include "main.h"
 * #include "i2c.h"
 * #include "FreeRTOS.h"
 * #include "task.h"
 * #include "cmsis_os.h"
 */
/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/
#define NUMBER_OF_CALCULATION 	10 							/*<! To get average pressure value from the communication. */
#define FLOAT_TO_INT(x) 		((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define FREERTOS				1 					/*<! if FreeRTOS included, you shall change the value to 1. In other ways, you shall change the value to 0. */

#define MAX_ADC_VALUE			14745.0f					/*<! change maximum ADC value from the datasheet (%90 of its value) */
#define MAX_PRESSURE_VALUE		60.0f  						/*<! change maximum pressure value from the datasheet */
#define	MIN_ADC_VALUE			1570.0f						/*<! change minimum ADC value from the datasheet (%10 of its value) */
#define MIN_PRESSURE_VALUE		0.0f						/*<! change minimum pressure value from the datasheet */

#define DATA_LIMIT_COUNT		3 						/*<! ADC readings cycle can be limited here */
/** @defgroup I2C_Memory_Address_Size I2C Memory Address Size
 * @{
 */
#define I2C_MEMADD_SIZE_8BIT            0x00000001U
#define I2C_MEMADD_SIZE_16BIT           0x00000010U
/******************************************************************************
 * Module Typedefs
 *******************************************************************************/
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;
/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/
static const u8 DEV_ADDR = 0x28 << 1; /*<! Add device address that is placed in data sheet and shift it left one bit. (For example; 0x28) */
u8 buffer1[2], buffer2[2], i;
s16 adcval1;
float pressure1, Pval;
I2C_HandleTypeDef *I2cHandle_3 = &hi2c3; /*<! You shall add which I2C_Handle will be used. Here, hi2c3 is connected */
float PressureBuffer[NUMBER_OF_CALCULATION] = { 0 };
/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function Definitions
 *******************************************************************************/
/**
 * @brief  I2Cx error treatment function
 * @param  Addr: I2C Address
 * @param  Reg: Reg Address
 * @param  pBuffer: pointer to read data buffer
 * @param  Length: length of the data
 * @retval 0 if no problems to read multiple data
 */
static void I2Cx_Error(I2C_HandleTypeDef *I2cHandle)
{
	/* De-initialize the I2C communication BUS */
	HAL_I2C_DeInit(I2cHandle);
	/* Re-Initialize the I2C communication BUS */
	MX_I2Cx_Init(); /* You shall change 'x' value to utilized I2C communication BUS */
}
/**
 * @brief  Reads multiple data on the BUS.
 * @param  *I2cHandle: HAL_I2C_REGISTER_CALLBACK e.g. &hi2c3
 * @param  Addr: I2C Dev Address
 * @param  Reg: Register Address
 * @param  *pBuffer: pointer to read data buffer
 * @param  Length: length of the data
 * @retval 1 if HAL_OK, 0 if problem occurred while reading
 */
u8 ReadBuffer(I2C_HandleTypeDef *I2CHandle, u8 Addr, u8 Reg, u8 *pBuffer, u16 Length)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(I2CHandle, Addr, (u16) Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, 100);
	/* Check the communication status */
	if (status == HAL_OK)
	{
		return 1;
	}
	else
	{
		/* Re-Initialize the BUS */
		I2Cx_Error(I2cHandle);
		return 0;
	}
}

#if FREERTOS
/**
 * @fn void GetI2CPressureValue()
 * @brief The function is related with I2C communication of the pressure sensor.
 *
 * @pre
 * @post
 */
void GetI2CPressureValue(void)
{
	u16 dataCounter = 0;
	float dataSum = 0;
	for (;;)
	{
		if (ReadBuffer(I2cHandle_3, DEV_ADDR, 0x00, buffer1, 2) == TRUE)
		{
			adcval1 = ((s16) buffer1[0] << 8) | (buffer1[1]);
			pressure1 = ((((float) adcval1 - MIN_ADC_VALUE)) * (MAX_PRESSURE_VALUE - MIN_PRESSURE_VALUE)) / (MAX_ADC_VALUE - MIN_ADC_VALUE);
		}
		else
		{
			I2Cx_Error(I2cHandle_3);
		}
		PressureBuffer[dataCounter] = pressure1;
		dataCounter++;
		if (dataCounter >= DATA_LIMIT_COUNT)
		{
			dataCounter = 0;
		}
		dataSum = 0;
		for (int i = 0; i < DATA_LIMIT_COUNT; i++)
		{
			dataSum += PressureBuffer[i];
		}
		Pval = (float) (dataSum / DATA_LIMIT_COUNT);
		osDelay(100);
	}
}

#else
/**
 * @fn void GetI2CPressureValue()
 * @brief The function is related with I2C communication of the pressure sensor.
 *
 * @pre
 * @post
 */
void GetI2CPressureValue(void)
{
	while (1)
	{
		if (ReadBuffer(I2cHandle_3, DEV_ADDR, 0x00, buffer1, 2) == TRUE) // Pressure sensor 1
		{
			adcval1 = ((s16) buffer1[0] << 8) | (buffer1[1]);
			pressure1 = ((((float) adcval1 - MIN_ADC_VALUE)) * (MAX_PRESSURE_VALUE - MIN_PRESSURE_VALUE)) / (MAX_ADC_VALUE - MIN_ADC_VALUE);
		}
		else
		{
			I2Cx_Error(I2cHandle_3);
		}
	}
}

#endif
/******************************************************************************
 *************** END OF FUNCTIONS *********************************************
 ******************************************************************************/
