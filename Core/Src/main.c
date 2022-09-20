#include "main.h"
#include "usb_device.h"

#include "cli.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "adf435x.h"

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

adf435xSettings adf4351;
adf435xSettings adf4350;

SPI_HandleTypeDef hspi1;

uint8_t flag_reciver;
uint8_t flag_start_spam;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

static void print(const char *fmt, ...)
{
	va_list argp;
	va_start(argp, fmt);

	char string[120];
	bzero(string, sizeof(string));

	if (0 < vsprintf(string, fmt, argp))
	{
		CDC_Transmit_FS((uint8_t*) string, strlen(string));
	}
	va_end(argp);
}

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

static inline void set_low_cs()
{
	GPIOA->BSRR |= GPIO_BSRR_BR4;
}

static inline void set_high_cs()
{
	GPIOA->BSRR |= GPIO_BSRR_BS4;
}

static inline void send_message(uint8_t *data, uint8_t size)
{
	HAL_SPI_Transmit(&hspi1, data, size, 1000);
//#warning uncomment "&"
}

static inline void set_low_cs_4350()
{
	GPIOA->BSRR |= GPIO_BSRR_BR3;
}

static inline void set_high_cs_4350()
{
	GPIOA->BSRR |= GPIO_BSRR_BS3;
}

int main(void)
{
	flag_reciver = 0;
	flag_start_spam = 0;

	uint8_t status = 1;

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USB_DEVICE_Init();

	cli_init();
	registration("clock", "", (void*) get_message_Callback);
	registration("start_spam", "", (void*) range_spam_Callback);

//****************************************************************************
	status = adf435x_init(&adf4351, ADF4351_TYPE, 50000000,
						(void*) set_low_cs, (void*) set_high_cs,
						(void*) send_message, (void*) HAL_Delay);
	if (status == 0)
	{
		print("adf4351 Init Success %d \r\n", 50000000);
		status = set_clock(&adf4351, 100000000);
		if (status == 0)
			print("adf4351 set clock - %d \r\n", 100000000);
		else
			print("adf4351 Errorcode %d \r\n", status);
	}
	else
		print("adf4351 Errorcode %d \r\n", status);

//****************************************************************************
	status = adf435x_init(&adf4350, ADF4350_TYPE, 27000000,
						(void*) set_low_cs_4350, (void*) set_high_cs_4350,
						(void*) send_message, (void*) HAL_Delay);
	if (status == 0)
	{
		print("adf4350 Init Success %d \r\n", 27000000);
		status = set_clock(&adf4350, 170000000);
		if (status == 0)
			print("adf4350 set clock - %d \r\n", 170000000);
		else
			print("adf4351 Errorcode %d \r\n", status);
	}
	else
		print("adf4351 Errorcode %d \r\n", status);
//****************************************************************************



	while (1)
	{
		if (flag_reciver)
		{
			flag_reciver = 0;
			set_clock(&adf4351, frequency);
		}

		if (flag_start_spam)
		{
			frequency += step;

			if (frequency >= end)
			{
				frequency = startFrequence;
			}
			set_clock(&adf4351, frequency);
			HAL_Delay(200);
		}
		else
		{
			GPIOC->BSRR |= GPIO_BSRR_BS13;
			HAL_Delay(100);
			GPIOC->BSRR |= GPIO_BSRR_BR13;
			HAL_Delay(500);
		}
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

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

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

static void MX_SPI1_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

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


	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}

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

