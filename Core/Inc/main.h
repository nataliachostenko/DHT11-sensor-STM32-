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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DHT11_Pin GPIO_PIN_1
#define DHT11_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_2
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_12
#define D5_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_13
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_14
#define D1_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_3
#define RS_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_4
#define E_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_5
#define RW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DHT11_PORT DHT11_GPIO_Port
#define DHT11_PIN DHT11_Pin

// Definicja typu stanu dla DHT11
typedef enum {
    DHT11_IDLE,
    DHT11_START,
    DHT11_WAIT_FOR_RESPONSE,
    DHT11_RESPONSE_LOW,
    DHT11_RESPONSE_HIGH,
    DHT11_RECEIVE_DATA,
    DHT11_DATA_READY,
    DHT11_ERROR
} DHT11_State;

extern volatile DHT11_State dht11_state; // Zmienna przechowująca aktualny stan DHT11
extern volatile uint32_t msTicks; // Licznik ticków dla opóźnień
extern uint8_t data[5]; // Bufor na otrzymane dane z DHT11
extern uint8_t bits[40]; // Bufor na otrzymane bity danych z DHT11

void DHT11_Start(void); // Funkcja inicjalizująca komunikację z DHT11
void Process_Data(uint8_t *data); // Funkcja przetwarzająca otrzymane dane
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin); // Ustawia pin jako wejście
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin); // Ustawia pin jako wyjście

extern volatile uint8_t tempInterval;
extern volatile uint8_t humInterval;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
