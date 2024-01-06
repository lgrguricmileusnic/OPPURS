/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_NEWLINE '\r'
#define TX_NEWLINE "\r\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
xTaskHandle vTaskHandleHeartBeat, vTaskHandleUARTControls, vTaskHandlePlayback;
uint8_t buttonPressed;
uint32_t frequency;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
void vTaskHeartBeat(void *pvParameters);

void vTaskUARTControls(void *pvParameters);

void vTaskPlayback(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int check_password(char line[60]);

/* main .c */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == USER_BUTTON_PIN) {
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
        buttonPressed = 1;
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    } else {
        __NOP();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */
    buttonPressed = 0;
    frequency = 440;

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
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    HAL_UART_Receive_DMA(&huart2, rx_data, 1);
    xTaskCreate(vTaskHeartBeat,
                "TASK HEARTBEAT",
                configMINIMAL_STACK_SIZE,
                NULL,
                5,
                &vTaskHandleHeartBeat);
    xTaskCreate(vTaskUARTControls,
                "TASK CONTROLS",
                configMINIMAL_STACK_SIZE,
                NULL,
                5,
                &vTaskHandleUARTControls);
    /* USER CODE END 2 */

    /* Call init function for freertos objects (in freertos.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void vTaskHeartBeat(void *pvParameters) {
    const portTickType t1 = 100 / portTICK_PERIOD_MS;
    const portTickType t2 = 200 / portTICK_PERIOD_MS;
    const portTickType t3 = 100 / portTICK_PERIOD_MS;
    const portTickType t4 = 600 / portTICK_PERIOD_MS;

    gpio_led_state(LED4_GREEN_ID, 0);
    while (1) {
        gpio_led_state(LED4_GREEN_ID, 1);
        vTaskDelay(t1);
        gpio_led_state(LED4_GREEN_ID, 0);
        vTaskDelay(t2);
        gpio_led_state(LED4_GREEN_ID, 1);
        vTaskDelay(t3);
        gpio_led_state(LED4_GREEN_ID, 0);
        vTaskDelay(t4);
    }
}

void vTaskUARTControls(void *pvParameters) {
    char c;
    int line_tail;
    enum State state;
    char line[60] = {0};
    const char *welcome_msg = "\rLab 4: Programming industrial embedded systems. Please enter password:\r\n";

    line_tail = 0;
    state = AUTH;
    USART2_SendString(welcome_msg);
    while (1) {
        if (USART2_ReadChar(&c) != 0) {
            if (c == RX_NEWLINE) {
                USART2_SendString(TX_NEWLINE);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                switch (state) {
                    case AUTH:
                        if (check_password(line)) {
                            state = CONTROL;
                            USART2_SendString("OK!\r\n");
                            xTaskCreate(vTaskPlayback,
                                        "TASK PLAYBACK",
                                        configMINIMAL_STACK_SIZE,
                                        NULL,
                                        1,
                                        &vTaskHandlePlayback);
                        } else {
                            USART2_SendString(welcome_msg);
                        }
                        break;
                    case CONTROL:
                        break;
                }
                line_tail = 0;
                memset(line, '\0', 60);
            } else {
                line[line_tail] = c;
                line_tail++;
                USART2_SendChar(c);
            }
        }
    };
}

void vTaskPlayback(void *pvParameters) {

};

int check_password(char input[60]) {
    const char *password = "36523763";
    return strcmp(input, password) == 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
