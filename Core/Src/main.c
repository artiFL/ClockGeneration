#include "main.h"
#include "usb_device.h"
#include "cli.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

#include "adf4351lib.h"


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


SPI_HandleTypeDef hspi1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

void print(const char *fmt, ...);
void print(const char *fmt, ...)
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

adf435xSettings* adf4351_1;
//adf435xSettings adf4351_2;

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

static inline void set_low_cs()
{
	GPIOA->BSRR |= GPIO_BSRR_BR4;
}
static inline void set_high_cs()
{
	GPIOA->BSRR |= GPIO_BSRR_BS4;
}

static inline void set_low_cs_2()
{
	GPIOA->BSRR |= GPIO_BSRR_BR2;
}
static inline void set_high_cs_2()
{
	GPIOA->BSRR |= GPIO_BSRR_BS2;
}

static inline void send_message(uint8_t* data, uint8_t size)
{
	HAL_SPI_Transmit(&hspi1, &data, size, 1000);
}

int main(void)
{
	flag_reciver = 0;
	flag_start_spam = 0;
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USB_DEVICE_Init();


	adf4351_1 = malloc(sizeof(adf4351_1));
	memset(adf4351_1, 0, sizeof(adf4351_1));

	adf435x_init(adf4351_1, REF_CLK, (void*)set_low_cs, (void*)set_high_cs, (void*)send_message, (void*)HAL_Delay);

	//adf4351_2 = adf435x_init(adf4351_2, REF_CLK, (void*)set_low_cs_2, (void*)set_high_cs_2, (void*)send_message, (void*)HAL_Delay);


	prepare_registers(adf4351_1, FREQ);
	update_all_register(adf4351_1);

	//prepare_registers(adf4351_2, frequency);
	//update_all_register(adf4351_2);

	cli_init();
	registration("clock", "", (void*) get_message_Callback);
	registration("start_spam", "", (void*) range_spam_Callback);

	while (1)
	{
		if (flag_reciver)
		{
			flag_reciver = 0;
			prepare_registers(adf4351_1, frequency);
			update_all_register(adf4351_1);
		}

		if (flag_start_spam)
		{
			frequency += step;

			if (frequency >= end)
			{
				frequency = startFrequence;
			}
			prepare_registers(adf4351_1, frequency);
			update_all_register(adf4351_1);
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

/*
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
*/

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
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

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
/*

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	*/
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
