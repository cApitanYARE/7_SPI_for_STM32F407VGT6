/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Варіант 2 - Помаранчевий світлодіод (PD13) та LIS3DSH
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "MY_LIS3DSH.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_WINDOW_SIZE 5    // Розмір вікна фільтра ковзного середнього
#define MOTION_THRESHOLD   50  // Поріг чутливості до руху
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Змінні для акселерометра LIS3DSH */
volatile uint8_t data_ready_flag = 0; // Прапорець переривання
LIS3DSH_DataRaw raw_data;              // Змінна для сирих даних з акселерометра

/* Буфери для фільтрації шуму SPI */
int16_t x_buf[FILTER_WINDOW_SIZE] = {0};
int16_t y_buf[FILTER_WINDOW_SIZE] = {0};
int16_t z_buf[FILTER_WINDOW_SIZE] = {0};
uint8_t buf_index = 0;

/* Попередні значення для порівняння переміщення */
int16_t prev_x = 0, prev_y = 0, prev_z = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int16_t Apply_Moving_Average(int16_t new_value, int16_t *buffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Фільтр ковзного середнього для усунення шуму SPI
  */
int16_t Apply_Moving_Average(int16_t new_value, int16_t *buffer) {
    buffer[buf_index] = new_value;
    int32_t sum = 0;
    for (uint8_t i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum += buffer[i];
    }
    return (int16_t)(sum / FILTER_WINDOW_SIZE);
}

/**
  * @brief Обробник переривання EXTI від акселерометра
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        data_ready_flag = 1;
    }
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* Конфігурація акселерометра LIS3DSH */
  LIS3DSH_InitTypeDef lis3dsh_init;

  lis3dsh_init.dataRate = LIS3DSH_DATARATE_100;        // 100 Hz
  lis3dsh_init.fullScale = LIS3DSH_FULLSCALE_2;        // Діапазон +-2g
  lis3dsh_init.antiAliasingBW = LIS3DSH_FILTER_BW_800; // Смуга фільтрації 800Hz
  lis3dsh_init.enableAxes = LIS3DSH_XYZ_ENABLE;        // Увімкнути X, Y, Z
  lis3dsh_init.interruptEnable = 0;                    // Вимкнено переривання в ініціалізації

  // Ініціалізація акселерометра
  LIS3DSH_Init(&hspi1, &lis3dsh_init);

  int16_t filtered_x, filtered_y, filtered_z;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
    /* USER CODE BEGIN WHILE */
while (1) {
	/* Зчитування сирих даних з акселерометра */
	raw_data = LIS3DSH_GetDataRaw();

	/* Фільтрація шумових коливань по кожній осі */
	filtered_x = Apply_Moving_Average(raw_data.x, x_buf);
	filtered_y = Apply_Moving_Average(raw_data.y, y_buf);
	filtered_z = Apply_Moving_Average(raw_data.z, z_buf);

	buf_index = (buf_index + 1) % FILTER_WINDOW_SIZE;

	/* Обчислення різниці (дельти) для виявлення переміщення */
	int32_t delta_x = abs(filtered_x - prev_x);
	int32_t delta_y = abs(filtered_y - prev_y);
	int32_t delta_z = abs(filtered_z - prev_z);

	/* Перевірка перевищення порогу руху */
	if ((delta_x > MOTION_THRESHOLD) ||
		(delta_y > MOTION_THRESHOLD) ||
		(delta_z > MOTION_THRESHOLD)) {

		/* Варіант 2: Засвічуємо помаранчевий світлодіод (GPIOD, GPIO_PIN_13) */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	} else {
		/* Вимикаємо світлодіод за відсутності руху */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	}

	/* Збереження поточних значень */
	prev_x = filtered_x;
	prev_y = filtered_y;
	prev_z = filtered_z;

	HAL_Delay(50); // Невелика затримка для стабільності
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
  RCC_OscInitStruct.PLL.PLLN = 144;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
