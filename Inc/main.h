/* USER CODE BEGIN Header */
/*
The MIT License (MIT)

Copyright (c) 2019 Techno Road Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_SW_Pin GPIO_PIN_13
#define USER_SW_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_2
#define LD3_GPIO_Port GPIOA
#define CONF_SW1_Pin GPIO_PIN_3
#define CONF_SW1_GPIO_Port GPIOA
#define CONF_SW2_Pin GPIO_PIN_4
#define CONF_SW2_GPIO_Port GPIOA
#define CONF_SW3_Pin GPIO_PIN_5
#define CONF_SW3_GPIO_Port GPIOA
#define CONF_SW4_Pin GPIO_PIN_6
#define CONF_SW4_GPIO_Port GPIOA
#define CONF_SW5_Pin GPIO_PIN_7
#define CONF_SW5_GPIO_Port GPIOA
#define DEBUG_PIN_Pin GPIO_PIN_2
#define DEBUG_PIN_GPIO_Port GPIOB
#define ADIS_nRST_Pin GPIO_PIN_15
#define ADIS_nRST_GPIO_Port GPIOA
#define SPI1_MI_Pin GPIO_PIN_4
#define SPI1_MI_GPIO_Port GPIOB
#define SPI1_MO_Pin GPIO_PIN_5
#define SPI1_MO_GPIO_Port GPIOB
#define SPI1_nCS_Pin GPIO_PIN_6
#define SPI1_nCS_GPIO_Port GPIOB
#define ADIS_DR_Pin GPIO_PIN_7
#define ADIS_DR_GPIO_Port GPIOB
#define ADIS_SYNC_Pin GPIO_PIN_8
#define ADIS_SYNC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LD1_ON HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,GPIO_PIN_RESET)
#define LD1_OFF HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,GPIO_PIN_SET)
#define LD2_ON HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET)
#define LD2_OFF HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET)
#define LD3_ON HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_RESET)
#define LD3_OFF HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET)
#define CSW1_STAT !HAL_GPIO_ReadPin(CONF_SW1_GPIO_Port,CONF_SW1_Pin)
#define CSW2_STAT !HAL_GPIO_ReadPin(CONF_SW2_GPIO_Port,CONF_SW2_Pin)
#define CSW3_STAT !HAL_GPIO_ReadPin(CONF_SW3_GPIO_Port,CONF_SW3_Pin)
#define CSW4_STAT !HAL_GPIO_ReadPin(CONF_SW4_GPIO_Port,CONF_SW4_Pin)
#define CSW5_STAT !HAL_GPIO_ReadPin(CONF_SW5_GPIO_Port,CONF_SW5_Pin)
#define READ_USER_SW !HAL_GPIO_ReadPin(USER_SW_GPIO_Port,USER_SW_Pin)
#define DEBUG_PIN_HIGH HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port,DEBUG_PIN_Pin,GPIO_PIN_SET)
#define DEBUG_PIN_LOW HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port,DEBUG_PIN_Pin,GPIO_PIN_RESET)

#define CSW1_MASK 0x01
#define CSW2_MASK 0x02
#define CSW3_MASK 0x04
#define CSW4_MASK 0x08
#define CSW5_MASK 0x10
#define CSW_MODE_MASK 0x07

#define USB_SPI_MODE 0b0000
#define YawPitchRoll 0b0001
#define GYRO_Degree 0b0010
#define GYRO_ACC_HexData 0b0011
#define GYRO_ACC_BinaryData 0b0100
#define YawPitchRoll_ACC 0b0101
#define GYRO_ACC_TEMP 0b0110

//#define BoardName "TR-IMU-Platform"
#define BoardName "TR-IMU1647X"

// Please Change this value when changing the TIM3 cycle.
#define CONTROL_CYCLE 0.01 	//[s]

// Address to start writing data to flash memory
#define DATA_ADDR 0x0801F800

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
