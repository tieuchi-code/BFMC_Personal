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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mydriver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} IMU_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO055_CHIP_ID         0x00
#define BNO055_CHIP_ID_VAL     0xA0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for UART_TX */
osThreadId_t UART_TXHandle;
const osThreadAttr_t UART_TX_attributes = {
  .name = "UART_TX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for QUICKRUN */
osThreadId_t QUICKRUNHandle;
const osThreadAttr_t QUICKRUN_attributes = {
  .name = "QUICKRUN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Servo */
osThreadId_t ServoHandle;
const osThreadAttr_t Servo_attributes = {
  .name = "Servo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for UART_RX */
osThreadId_t UART_RXHandle;
const osThreadAttr_t UART_RX_attributes = {
  .name = "UART_RX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Fushion */
osThreadId_t FushionHandle;
const osThreadAttr_t Fushion_attributes = {
  .name = "Fushion",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imu */
osMessageQueueId_t imuHandle;
const osMessageQueueAttr_t imu_attributes = {
  .name = "imu"
};
/* Definitions for as6500 */
osMessageQueueId_t as6500Handle;
const osMessageQueueAttr_t as6500_attributes = {
  .name = "as6500"
};
/* Definitions for command */
osMessageQueueId_t commandHandle;
const osMessageQueueAttr_t command_attributes = {
  .name = "command"
};
/* Definitions for response */
osMessageQueueId_t responseHandle;
const osMessageQueueAttr_t response_attributes = {
  .name = "response"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void imu_func(void *argument);
void uart_tx_func(void *argument);
void quickrun_func(void *argument);
void servo_func(void *argument);
void uart_rx_func(void *argument);
void fushion_func(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
  CHI_UART_init();
  I2C1_Master_Init();

  CHI_UART_print_log("BNO055 scan...\r\n");

  uint8_t id = BNO055_ReadReg(BNO055_CHIP_ID);
  if (id != BNO055_CHIP_ID_VAL)
  {
      CHI_UART_print_log("BNO055 NOT FOUND\r\n");
      while (1);
  }

  CHI_UART_print_log("BNO055 OK\r\n");
  BNO055_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of imu */
  imuHandle = osMessageQueueNew (16, sizeof(uint8_t), &imu_attributes);

  /* creation of as6500 */
  as6500Handle = osMessageQueueNew (16, sizeof(uint8_t), &as6500_attributes);

  /* creation of command */
  commandHandle = osMessageQueueNew (16, sizeof(uint8_t), &command_attributes);

  /* creation of response */
  responseHandle = osMessageQueueNew (16, sizeof(uint8_t8), &response_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of IMU */
  IMUHandle = osThreadNew(imu_func, NULL, &IMU_attributes);

  /* creation of UART_TX */
  UART_TXHandle = osThreadNew(uart_tx_func, NULL, &UART_TX_attributes);

  /* creation of QUICKRUN */
  QUICKRUNHandle = osThreadNew(quickrun_func, NULL, &QUICKRUN_attributes);

  /* creation of Servo */
  ServoHandle = osThreadNew(servo_func, NULL, &Servo_attributes);

  /* creation of UART_RX */
  UART_RXHandle = osThreadNew(uart_rx_func, NULL, &UART_RX_attributes);

  /* creation of Fushion */
  FushionHandle = osThreadNew(fushion_func, NULL, &Fushion_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_imu_func */
/**
  * @brief  Function implementing the IMU thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_imu_func */
void imu_func(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_uart_tx_func */
/**
* @brief Function implementing the UART_TX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_tx_func */
void uart_tx_func(void *argument)
{
  /* USER CODE BEGIN uart_tx_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uart_tx_func */
}

/* USER CODE BEGIN Header_quickrun_func */
/**
* @brief Function implementing the QUICKRUN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_quickrun_func */
void quickrun_func(void *argument)
{
  /* USER CODE BEGIN quickrun_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END quickrun_func */
}

/* USER CODE BEGIN Header_servo_func */
/**
* @brief Function implementing the Servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_func */
void servo_func(void *argument)
{
  /* USER CODE BEGIN servo_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END servo_func */
}

/* USER CODE BEGIN Header_uart_rx_func */
/**
* @brief Function implementing the UART_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_rx_func */
void uart_rx_func(void *argument)
{
  /* USER CODE BEGIN uart_rx_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uart_rx_func */
}

/* USER CODE BEGIN Header_fushion_func */
/**
* @brief Function implementing the Fushion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fushion_func */
void fushion_func(void *argument)
{
  /* USER CODE BEGIN fushion_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fushion_func */
}

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
#ifdef USE_FULL_ASSERT
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
