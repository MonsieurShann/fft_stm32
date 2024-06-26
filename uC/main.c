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
#include "arm_math.h"
#include <stdio.h>
#include "core_cm4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES 2048  // Nombre de points pour la FFT
#define SAMPLING_FREQUENCY 1024.0f
float32_t signal[SAMPLES] = {0.0};
float32_t output[SAMPLES] = {0.0};
q15_t signal_q15[SAMPLES] = {0.0};
q15_t output_q15[SAMPLES] = {0.0};
q31_t signal_q31[SAMPLES] = {0.0};
q31_t output_q31[SAMPLES] = {0.0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void generate_sine_wave(float32_t* signal, float32_t frequency, uint32_t nb_s, uint32_t s_freq) 
{
    for (uint32_t i = 0; i < nb_s; i++) {
        float32_t angle = 2 * PI * frequency * ((float32_t)i / s_freq);
        signal[i] += arm_sin_f32(angle);
    }
}

void generate_sine_wave_q15(q15_t *input_q15, uint32_t freq, uint32_t nb_s, uint32_t s_freq) 
{
    for (uint32_t i = 0; i < nb_s; i++) {
        float32_t angle = 2 * PI * s_freq * ((float32_t)i / s_freq);
        float32_t sin_val = arm_sin_f32(angle);
        input_q15[i] = (q15_t)(sin_val * 32767); // Conversion en q15
    }
}

void generate_sine_wave_q31(q31_t *input_q31, uint32_t freq, uint32_t nb_s, uint32_t s_freq)
{
    for (uint32_t i = 0; i < nb_s; i++) {
        float32_t angle = 2 * PI * s_freq * ((float32_t)i / s_freq);
        float32_t sin_val = arm_sin_f32(angle);
        input_q31[i] = (q31_t)(sin_val * 32767); // Conversion en q15
    }
}

void combine_signals_evolve(float32_t *signal, uint32_t *frequencies, uint32_t frequenciesSize, uint32_t nb_s, uint32_t s_freq)
{
  for (int i=0; i<frequenciesSize; i++)
    generate_sine_wave(signal, frequencies[i], nb_s, s_freq);
}

void combine_signals_evolve_q15(q15_t *signal, uint32_t *frequencies, uint32_t frequenciesSize, uint32_t nb_s, uint32_t s_freq)
{
  for (int i=0; i<frequenciesSize; i++)
    generate_sine_wave_q15(signal, frequencies[i], nb_s, s_freq);
}

void combine_signals_evolve_q31(q31_t *signal, uint32_t *frequencies, uint32_t frequenciesSize, uint32_t nb_s, uint32_t s_freq)
{
  for (int i=0; i<frequenciesSize; i++)
    generate_sine_wave_q31(signal, frequencies[i], nb_s, s_freq);
}

void perform_fft(uint32_t nb_s) {
    arm_cfft_instance_f32 fft_instance;

    // Initialisation de l'instance de FFT pour une longueur de 1024
    arm_cfft_init_f32(&fft_instance, nb_s);

    // Calcul de la FFT
    arm_cfft_f32(&fft_instance, signal, 0, 1);

    // Conversion des résultats de la FFT en magnitudes
    arm_cmplx_mag_f32(signal, output, nb_s / 2);
}

void perform_fft_q15(uint32_t nb_s) {
    arm_cfft_instance_q15 fft_instance;

    // Initialisation de l'instance de FFT pour une longueur de 1024
    arm_cfft_init_q15(&fft_instance, nb_s);

    // Calcul de la FFT
    arm_cfft_q15(&fft_instance, signal_q15, 0, 1);

    // Conversion des résultats de la FFT en magnitudes
    arm_cmplx_mag_q15(signal_q15, output_q15, nb_s / 2);
}

void perform_fft_q31(uint32_t nb_s) {
    arm_cfft_instance_q31 fft_instance;

    // Initialisation de l'instance de FFT pour une longueur de 1024
    arm_cfft_init_q31(&fft_instance, nb_s);

    // Calcul de la FFT
    arm_cfft_q31(&fft_instance, signal_q31, 0, 1);

    // Conversion des résultats de la FFT en magnitudes
    arm_cmplx_mag_q31(signal_q31, output_q31, nb_s / 2);
}

void send_result(float32_t *tx_data, uint8_t len)
{
  uint32_t data;
  uint32_t sof = 5678;
  printf("%ld\n", sof);
  HAL_Delay(100);
  for (int i=0; i<len; i++){
    data = (uint32_t)(tx_data[i]*1000000);
    printf("%ld\n", data);
  }
}

void send_benchmark(uint32_t **tx)
{
  uint32_t sof = 1234;
  printf("%ld\n", sof);
  HAL_Delay(100);
  for (int j=0; j<3; j++)
  {
    for (int i=0; i<8; i++)
      printf("%ld\n", tx[j][i]);
  }
}

void start_measurement(void)
{
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t stop_measurement(void)
{
  return DWT->CYCCNT;
}

float32_t time_of_execution(uint32_t nb_of_cycles)
{
  return 1000000*(float32_t)nb_of_cycles / HAL_RCC_GetHCLKFreq();
}

void benchmark_f32(uint32_t *freqs, uint32_t s_freq, uint32_t *time_of_exec_us)
{
  uint32_t nb_of_cycles = 0;
  uint32_t nb_s = 8;

  for (int i=0; i<8; i++)
  {
    nb_s *= 2;
    combine_signals_evolve(signal, freqs, 3, nb_s, s_freq);
    start_measurement();
    for (int j=0; j<100; j++)
      perform_fft(nb_s);
    nb_of_cycles = stop_measurement();
    time_of_exec_us[i] = time_of_execution(nb_of_cycles);
  }
}

void benchmark_q15(uint32_t *freqs, uint32_t s_freq, uint32_t *time_of_exec_us)
{
  uint32_t nb_of_cycles = 0;
  uint32_t nb_s = 8;

  for (int i=0; i<8; i++)
  {
    nb_s *= 2;
    combine_signals_evolve_q15(signal_q15, freqs, 3, nb_s, s_freq);
    start_measurement();
    for (int j=0; j<100; j++)
      perform_fft_q15(nb_s);
    nb_of_cycles = stop_measurement();
    time_of_exec_us[i] = time_of_execution(nb_of_cycles);
  }
}

void benchmark_q31(uint32_t *freqs, uint32_t s_freq, uint32_t *time_of_exec_us)
{
  uint32_t nb_of_cycles = 0;
  uint32_t nb_s = 8;

  for (int i=0; i<8; i++)
  {
    nb_s *= 2;
    combine_signals_evolve_q31(signal_q31, freqs, 3, nb_s, s_freq);
    start_measurement();
    for (int j=0; j<100; j++)
      perform_fft_q31(nb_s);
    nb_of_cycles = stop_measurement();
    time_of_exec_us[i] = time_of_execution(nb_of_cycles);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  uint32_t nb_of_cycles = 0;
  uint32_t nb_s = 8;
  uint32_t s_freq = 1024;
  uint32_t frequencies[3] = {10, 20, 65};
  uint32_t time_of_exec_us[8] = {0};
  uint32_t time_of_exec_us_q15[8] = {0};
  uint32_t time_of_exec_us_q31[8] = {0};
  uint32_t *time_of_execs[3] = {time_of_exec_us, time_of_exec_us_q15, time_of_exec_us_q31};

  benchmark_f32(frequencies, s_freq, time_of_exec_us);
  benchmark_q15(frequencies, s_freq, time_of_exec_us_q15);
  benchmark_q31(frequencies, s_freq, time_of_exec_us_q31);
  send_benchmark(time_of_execs);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	send_benchmark(time_of_execs);
	//send_result(output, 150);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
	/**/
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART2 and Loop until the end of transmission */
 HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
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

