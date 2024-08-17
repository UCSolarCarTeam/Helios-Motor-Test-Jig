/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l1xx_hal.h"

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
#define Supply_Current_Pin GPIO_PIN_0
#define Supply_Current_GPIO_Port GPIOC
#define Precharger_Current_Pin GPIO_PIN_1
#define Precharger_Current_GPIO_Port GPIOC
#define Motor_2_Voltage_Pin GPIO_PIN_1
#define Motor_2_Voltage_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Supply_Voltage_Pin GPIO_PIN_4
#define Supply_Voltage_GPIO_Port GPIOA
#define Precharger_Voltage_Pin GPIO_PIN_6
#define Precharger_Voltage_GPIO_Port GPIOA
#define Motor_1_Current_Pin GPIO_PIN_7
#define Motor_1_Current_GPIO_Port GPIOA
#define Motor_2_Current_Pin GPIO_PIN_1
#define Motor_2_Current_GPIO_Port GPIOB
#define Faults_Pin GPIO_PIN_11
#define Faults_GPIO_Port GPIOB
#define Radio_GPIO_Pin GPIO_PIN_12
#define Radio_GPIO_GPIO_Port GPIOB
#define CAN_INT_Pin GPIO_PIN_6
#define CAN_INT_GPIO_Port GPIOC
#define RX0BF_Pin GPIO_PIN_7
#define RX0BF_GPIO_Port GPIOC
#define RX1BF_Pin GPIO_PIN_8
#define RX1BF_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_10
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_11
#define LED_3_GPIO_Port GPIOC
#define Button_3_Pin GPIO_PIN_12
#define Button_3_GPIO_Port GPIOC
#define Button_4_Pin GPIO_PIN_2
#define Button_4_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Gate_Driver_Placeholder_1_Pin GPIO_PIN_4
#define Gate_Driver_Placeholder_1_GPIO_Port GPIOB
#define Gate_Driver_Placeholder_2_Pin GPIO_PIN_5
#define Gate_Driver_Placeholder_2_GPIO_Port GPIOB
#define Gate_Driver_Placeholder_3_Pin GPIO_PIN_6
#define Gate_Driver_Placeholder_3_GPIO_Port GPIOB
#define Gate_Driver_Placeholder_4_Pin GPIO_PIN_7
#define Gate_Driver_Placeholder_4_GPIO_Port GPIOB
#define Button_1_Pin GPIO_PIN_8
#define Button_1_GPIO_Port GPIOB
#define Button_2_Pin GPIO_PIN_9
#define Button_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADC_BUF_LEN 20
#define UART_BUF_LEN 50
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
