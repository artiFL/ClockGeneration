/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cli.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SHL(x,y) ((uint32_t)1<<y)*x

#define VERRSION2

#define				FREQ						2412000000

#ifdef VERRSION2
#define REF_CLK 50000000
#else
#define REF_CLK 27000000
#endif


unsigned long long frequency = 200000000;

unsigned long long startFrequence;
unsigned long long step;
unsigned long long end;

// Register 0:
uint16_t INT = 0;
uint16_t FRAC = 0;

// Register 1:
uint8_t phase_adj;
uint8_t prescaler = 0;
uint16_t phase = 1;
uint16_t MOD = 4095;

// Register 2:
uint8_t low_noise_spur;
uint8_t muxout;

#if REF_CLK < 30000000
uint8_t ref_doubler = 1;
#else
uint8_t ref_doubler = 0;
#endif

uint8_t rdiv2 = 1;
uint16_t r_counter = 10;
uint8_t dbl_buf;
uint8_t charge_pump_current = 0b111;
uint8_t ldf = 1;
uint8_t ldp;
uint8_t pd_polarity = 1;
uint8_t powerdown;
uint8_t cp_three_state;
uint8_t counter_reset;

// Register 3:
uint8_t band_mode_clksel;
uint8_t abp;
uint8_t chg_cancel;
uint8_t csr;
uint8_t clkdiv_mode;
uint16_t clock_divider = 150;

// Register 4:
uint8_t feedback_sel = 1;
uint8_t rf_div_sel = 2;   // 0 = /1, 1=/2, 2=/4 ...
uint8_t band_select_clkdiv = 4;
uint8_t vco_pwrdown = 0;
uint8_t mtld = 1;
uint8_t aux_outsel = 0;
uint8_t aux_outena;
uint8_t aux_pwr;
uint8_t rf_ena = 1; // 0 - output disabled
uint8_t out_pwr = 3; // 0 - min, 3 - max

// Register 5:
uint8_t ld_pinmode = 1;

uint32_t reg[6] =
{ 0, 0, 0, 0, 0, 0 };

uint32_t pfd_freq;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print(const char* fmt, ...);
void print(const char* fmt, ...)
{
	va_list argp;
	va_start(argp, fmt);

	char* string[120];
	bzero(string, sizeof(string));

	if (0 < vsprintf(string, fmt, argp))
	{
		CDC_Transmit_FS((uint8_t *)string, strlen(string));
	}
	va_end(argp);
}



static void prepare_registers()
{

	if (frequency >= 2200000000)
		rf_div_sel = 0;
	if (frequency < 2200000000)
		rf_div_sel = 1;
	if (frequency < 1100000000)
		rf_div_sel = 2;
	if (frequency < 550000000)
		rf_div_sel = 3;
	if (frequency < 275000000)
		rf_div_sel = 4;
	if (frequency < 137500000)
		rf_div_sel = 5;
	if (frequency < 68750000)
		rf_div_sel = 6;

	INT = (frequency * (1 << rf_div_sel)) / pfd_freq;
	FRAC = (((frequency * (1 << rf_div_sel)) % pfd_freq) * 4095) / pfd_freq;

	reg[0] = SHL(INT, 15) | SHL(FRAC, 3);
	reg[1] = SHL(phase_adj, 28) | SHL(prescaler, 27) | SHL(phase, 15)
			| SHL(MOD, 3) | 0b001;
	reg[2] = SHL(low_noise_spur, 29) | SHL(muxout, 26) | SHL(ref_doubler, 25)
			| SHL(rdiv2, 24) | SHL(r_counter, 14) | SHL(dbl_buf, 13)
			| SHL(charge_pump_current, 9) | SHL(ldf, 8) | SHL(ldp, 7)
			| SHL(pd_polarity, 6) | SHL(powerdown, 5) | SHL(cp_three_state, 4)
			| SHL(counter_reset, 3) | 0b010;
	reg[3] = SHL(band_mode_clksel, 23) | SHL(abp, 22) | SHL(chg_cancel, 21)
			| SHL(csr, 18) | SHL(clkdiv_mode, 15) | SHL(clock_divider, 3)
			| 0b011;
	reg[4] = SHL(feedback_sel, 23) | SHL(rf_div_sel, 20)
			| SHL(band_select_clkdiv, 12) | SHL(vco_pwrdown, 9) | SHL(mtld, 10)
			| SHL(aux_outsel, 9) | SHL(aux_outena, 8) | SHL(aux_pwr, 6)
			| SHL(rf_ena, 5) | SHL(out_pwr, 3) | 0b100;
	reg[5] = SHL(ld_pinmode, 22) | SHL(0b11, 19) | 0b101;

}

static void sendRegisterToAdf(uint16_t reg_id)
{

	GPIOA->BSRR |= GPIO_BSRR_BR4;

	HAL_Delay(10);
	uint8_t data = (uint8_t) (reg[reg_id] >> 24);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	data = (uint8_t) (reg[reg_id] >> 16);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	data = (uint8_t) (reg[reg_id] >> 8);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	data = (uint8_t) (reg[reg_id]);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);

	GPIOA->BSRR |= GPIO_BSRR_BS4;
	HAL_Delay(10);

	GPIOA->BSRR |= GPIO_BSRR_BR4;
	//HAL_Delay(250);
//#error
}

static void updateAllRegisters()
{
	for (int i = 5; i >= 0; i--)
	{
		sendRegisterToAdf(i);
	}
}

uint8_t flag_reciver;
uint8_t flag_start_spam;

static void get_message_Callback(uint8_t argc, const char *argv[])
{
	flag_reciver = 1;

	if (scan_cmd(argv[1], "Hz"))
	{
		frequency = atof(argv[2]);
	}

	else if (scan_cmd(argv[1], "MHz"))
	{
		frequency = atof(argv[2]) * 1000000;
	}

	else if (scan_cmd(argv[1], "GHz"))
	{
		frequency = atof(argv[2]) * 1000000000;
	}
	print("clock set %d", frequency);
}

static void range_spam_Callback(uint8_t argc, const char *argv[])
{
	if (scan_cmd(argv[1], "1"))
	{
		flag_start_spam = 1;

		startFrequence = atof(argv[2]);
		frequency = startFrequence;

		step = atof(argv[3]);

		end = atof(argv[4]);

	}
	else if (scan_cmd(argv[1], "0"))
	{
		flag_start_spam = 0;
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
	pfd_freq = (REF_CLK * (1.0 + ref_doubler)) / (r_counter * ((1.0 + rdiv2)));
	flag_reciver = 0;
	flag_start_spam = 0;
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	prepare_registers();
	updateAllRegisters();

	cli_init();
	registration("clock", "", (void*) get_message_Callback);
	registration("start_spam", "", (void*) range_spam_Callback);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (flag_reciver)
		{
			flag_reciver = 0;
			prepare_registers();
			updateAllRegisters();
		}

		if (flag_start_spam)
		{
			frequency += step;

			if (frequency >= end)
			{
				frequency = startFrequence;
			}
			prepare_registers();
			updateAllRegisters();
			HAL_Delay(200);
		}
		else
		{
			GPIOC->BSRR |= GPIO_BSRR_BS13;
			HAL_Delay(100);
			GPIOC->BSRR |= GPIO_BSRR_BR13;
			HAL_Delay(500);
		}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

