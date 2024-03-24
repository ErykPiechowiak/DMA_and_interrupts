/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "bme280.h"
#include "bme280_defs.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEFAULT_FONT FONT_6X8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
uint8_t get_weather_data(struct bme280_data *data, struct bme280_dev *dev, struct bme280_settings *settings);
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bme280_delay_us(uint32_t period, void *intf_ptr);
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
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
	DWT->CYCCNT = 0;                                // Clear counter
	DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;             // Enable counter
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
  MX_SPI1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();
  struct bme280_dev bme280;
  bme280.chip_id = BME280_I2C_ADDR_PRIM;
  bme280.delay_us = bme280_delay_us;
  bme280.write = bme280_i2c_write;
  bme280.read = bme280_i2c_read;
  bme280.intf = BME280_I2C_INTF;

  struct bme280_settings settings;
  settings.filter = BME280_FILTER_COEFF_16;
  settings.osr_h = BME280_OVERSAMPLING_2X;
  settings.osr_p = BME280_OVERSAMPLING_16X;
  settings.osr_t = BME280_OVERSAMPLING_2X;

  uint8_t result;
  result = bme280_init(&bme280);
  if(result == BME280_OK){
	  uint8_t desired_settings  = BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP | BME280_SEL_OSR_HUM | BME280_SEL_FILTER;
	  result = bme280_set_sensor_settings(desired_settings, &settings, &bme280);
	 // result = bme280_get_sensor_settings(&settings, &bme280);
  }

  struct bme280_data sensor_data;
  char disp_temp[30];
  char disp_press[30];
  char disp_hum[30];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//LCD_Test();
	  //API from Bosch test
	  result = get_weather_data(&sensor_data, &bme280, &settings);
	  sprintf(disp_temp,"Temperature: %ld.%02ld",sensor_data.temperature/100,sensor_data.temperature%100);
	  sprintf(disp_hum,"Humidity: %lu.%02lu",sensor_data.humidity/1000,sensor_data.humidity%1000);
	  sprintf(disp_press,"Pressure: %lu",sensor_data.pressure/100 + 8);
	  LCD_PutStr(50, 56, disp_temp, DEFAULT_FONT, C_GREEN, C_BLACK);
	  //HAL_Delay(500);
	  LCD_PutStr(50, 76, disp_press, DEFAULT_FONT, C_GREEN, C_BLACK);
	  //HAL_Delay(500);
	  LCD_PutStr(50, 96, disp_hum, DEFAULT_FONT, C_GREEN, C_BLACK);
	  HAL_Delay(500);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint8_t get_weather_data(struct bme280_data *data, struct bme280_dev *dev, struct bme280_settings *settings){
	int8_t rslt = BME280_OK;
	uint32_t req_delay;
	uint8_t sensor_mode = BME280_POWERMODE_FORCED;
	rslt = bme280_cal_meas_delay(&req_delay, settings);
	req_delay/=1000; //convert to ms
	if(rslt != BME280_OK){
		return -1;
	}
	rslt = bme280_set_sensor_mode(sensor_mode, dev);
	if(rslt != BME280_OK){
		return -1;
	}
	HAL_Delay(req_delay);
	rslt = bme280_get_sensor_data(BME280_PRESS | BME280_TEMP | BME280_HUM, data, dev);
	if(rslt != BME280_OK){
		return -1;
	}
	return BME280_OK;
}


BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(&hi2c2, BME280_I2C_ADDR_PRIM<<1, &reg_addr, 1, 1000);
	if(ret != HAL_OK){
		return BME280_E_COMM_FAIL;
	}
	ret = HAL_I2C_Master_Receive(&hi2c2, BME280_I2C_ADDR_PRIM<<1, reg_data, (uint16_t)len,1000);
	if(ret != HAL_OK){
		return BME280_E_COMM_FAIL;
	}
	return BME280_INTF_RET_SUCCESS;
}


BME280_INTF_RET_TYPE bme280_i2c_read_DMA(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
	HAL_I2C_Master_Receive_DMA(&hi2c2, BME280_I2C_ADDR_PRIM<<1, reg_data, (uint16_t)len);
}

BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
	HAL_StatusTypeDef ret;
	uint8_t data_to_send[len+1];
	data_to_send[0]=reg_addr;
	for(int i=0;i<len;i++){
		data_to_send[i+1] = reg_data[i];
	}
	ret = HAL_I2C_Master_Transmit(&hi2c2, BME280_I2C_ADDR_PRIM<<1, data_to_send, len+1, 1000);
	if(ret != HAL_OK){
		return BME280_E_COMM_FAIL;
	}
	/*
	ret = HAL_I2C_Master_Transmit(&hi2c2, BME280_I2C_ADDR_PRIM<<1, reg_data, (uint16_t)len, 1000);
	if(ret != HAL_OK){
		return BME280_E_COMM_FAIL;
		}
		*/
	return BME280_INTF_RET_SUCCESS;

}
void bme280_delay_us(uint32_t period, void *intf_ptr){
	HAL_Delay(period/1000);
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
