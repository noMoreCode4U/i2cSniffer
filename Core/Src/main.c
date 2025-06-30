/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_SCL_PORT        GPIOB
#define I2C_SCL_PIN         GPIO_PIN_6
#define I2C_SCL_PIN_POS     6U

#define I2C_SDA_PORT        GPIOB
#define I2C_SDA_PIN         GPIO_PIN_7
#define I2C_SDA_PIN_POS     7U

#define SCL_READ()          ((I2C_SCL_PORT->IDR & I2C_SCL_PIN) ? 1 : 0)
#define SDA_READ()          ((I2C_SDA_PORT->IDR & I2C_SDA_PIN) ? 1 : 0)

typedef enum {
    I2C_SNIFFER_STATE_IDLE,
    I2C_SNIFFER_STATE_START,
    I2C_SNIFFER_STATE_ADDR_BYTE,
    I2C_SNIFFER_STATE_ACK_NACK_AFTER_ADDR,
    I2C_SNIFFER_STATE_DATA_BYTE,
    I2C_SNIFFER_STATE_ACK_NACK_AFTER_DATA
} I2C_Sniffer_State_TypeDef;

volatile I2C_Sniffer_State_TypeDef i2c_sniffer_state = I2C_SNIFFER_STATE_IDLE;
volatile uint8_t current_byte_decoded = 0;
volatile uint8_t bit_counter = 0;
volatile uint8_t i2c_transfer_direction = 0;
volatile uint8_t detected_slave_address = 0;
volatile uint8_t detected_register_address = 0xFF;
volatile uint8_t received_data_byte = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_Print(char *str);
static void I2C_Sniffer_GPIO_EXTI_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Print(char *str) {
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, strlen(str));
}


static void I2C_Sniffer_GPIO_EXTI_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    I2C_SCL_PORT->MODER &= ~(GPIO_MODER_MODER6);
    I2C_SDA_PORT->MODER &= ~(GPIO_MODER_MODER7);

    I2C_SCL_PORT->PUPDR |= (1U << (I2C_SCL_PIN_POS * 2));
    I2C_SDA_PORT->PUPDR |= (1U << (I2C_SDA_PIN_POS * 2));

    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB;

    EXTI->IMR |= (1U << I2C_SCL_PIN_POS);
    EXTI->RTSR |= (1U << I2C_SCL_PIN_POS);
    EXTI->FTSR |= (1U << I2C_SCL_PIN_POS);

    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 0);
}
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  I2C_Sniffer_GPIO_EXTI_Init();
  UART_Print("I2C Sniffer Started (Register-Level).\r\n");
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI9_5_IRQHandler(void) {

	if ((EXTI->PR & (1U << I2C_SCL_PIN_POS)) != 0) {
        EXTI->PR = (1U << I2C_SCL_PIN_POS);

        uint8_t scl_val = SCL_READ();
        uint8_t sda_val = SDA_READ();

        char msg[100];

        if (scl_val == 1 && sda_val == 1 && i2c_sniffer_state != I2C_SNIFFER_STATE_IDLE) {
            sprintf(msg, "I2C STOP Condition detected.\r\n");
            UART_Print(msg);
            i2c_sniffer_state = I2C_SNIFFER_STATE_IDLE;
            detected_register_address = 0xFF;
            bit_counter = 0;
            current_byte_decoded = 0;
            return;
        }

        if (scl_val == 1 && sda_val == 0 && i2c_sniffer_state == I2C_SNIFFER_STATE_IDLE) {
            sprintf(msg, "\r\nI2C START Condition detected.\r\n");
            UART_Print(msg);
            i2c_sniffer_state = I2C_SNIFFER_STATE_START;
            current_byte_decoded = 0;
            bit_counter = 0;
            detected_register_address = 0xFF;
            return;
        }


        switch (i2c_sniffer_state) {
            case I2C_SNIFFER_STATE_IDLE:
                break;

            case I2C_SNIFFER_STATE_START:
                if (scl_val == 0) {
                    i2c_sniffer_state = I2C_SNIFFER_STATE_ADDR_BYTE;
                    current_byte_decoded = 0;
                    bit_counter = 0;
                }
                break;

            case I2C_SNIFFER_STATE_ADDR_BYTE:
                if (scl_val == 1) {
                    current_byte_decoded = (current_byte_decoded << 1) | sda_val;
                    bit_counter++;
                    if (bit_counter == 8) { // 7 bits address + 1 bit R/W
                        detected_slave_address = (current_byte_decoded >> 1);
                        i2c_transfer_direction = (current_byte_decoded & 0x01); // 0 for write, 1 for read
                        sprintf(msg, "  Slave Addr: 0x%02X, Direction: %s\r\n",
                                detected_slave_address,
                                (i2c_transfer_direction == 0) ? "WRITE" : "READ");
                        UART_Print(msg);
                        i2c_sniffer_state = I2C_SNIFFER_STATE_ACK_NACK_AFTER_ADDR;
                    }
                }
                break;

            case I2C_SNIFFER_STATE_ACK_NACK_AFTER_ADDR:
                if (scl_val == 0) {
                    sprintf(msg, "  ACK/NACK after Addr: %s\r\n", sda_val == 0 ? "ACK" : "NACK");
                    UART_Print(msg);

                    i2c_sniffer_state = I2C_SNIFFER_STATE_DATA_BYTE;
                    current_byte_decoded = 0;
                    bit_counter = 0;
                }
                break;

            case I2C_SNIFFER_STATE_DATA_BYTE:
                if (scl_val == 1) {
                    current_byte_decoded = (current_byte_decoded << 1) | sda_val;
                    bit_counter++;
                    if (bit_counter == 8) {
                        received_data_byte = current_byte_decoded;

                        if (i2c_transfer_direction == 0) {
                            if (detected_register_address == 0xFF) {
                                detected_register_address = received_data_byte;
                                sprintf(msg, "    Register Address: 0x%02X\r\n", detected_register_address);
                                UART_Print(msg);
                            } else {
                                sprintf(msg, "    Data Written to 0x%02X: 0x%02X\r\n", detected_register_address, received_data_byte);
                                UART_Print(msg);
                            }
                        } else {
                            sprintf(msg, "    Data Read from Slave: 0x%02X\r\n", received_data_byte);
                            UART_Print(msg);
                        }
                        i2c_sniffer_state = I2C_SNIFFER_STATE_ACK_NACK_AFTER_DATA;
                        bit_counter = 0;
                        current_byte_decoded = 0;
                    }
                }
                break;

            case I2C_SNIFFER_STATE_ACK_NACK_AFTER_DATA:

                if (scl_val == 0) {
                    sprintf(msg, "  ACK/NACK after Data: %s\r\n", sda_val == 0 ? "ACK" : "NACK");
                    UART_Print(msg);
                    i2c_sniffer_state = I2C_SNIFFER_STATE_DATA_BYTE;
                    bit_counter = 0;
                    current_byte_decoded = 0;
                }
                break;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
