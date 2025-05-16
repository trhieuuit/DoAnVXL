/**
  ******************************************************************************
  * @file    main.c
  * @author  Yohanes Erwin Setiawan (modified for WS2812 8x8 with custom FFT)
  * @date    May 2025
  ******************************************************************************
  * @brief   Spectrum analyzer using 16 point FFT (custom, no CMSIS-DSP)
  *          Audio input from MAX9812 (ADC1, PA1)
  *          1. ADC:
  *             - ADC1 channel 1 (PA1) (10-bit, MAX9812 input)
  *          2. TIMER:
  *             - TIM3 for interrupt and ADC trigger (72 kHz)
  *             - TIM1 for WS2812 PWM (800 kHz)
  *          3. PWM:
  *             - TIM1 for WS2812 (PA8)
  *          4. DISPLAY:
  *             - WS2812 LED matrix 8x8 (64 LEDs)
  *          5. FFT:
  *             - Length = 16 point (custom, fixed-point int16_t)
  *             - Sampling frequency = 36 kHz
  *             - Nyquist frequency = 36 kHz / 2 = 18 kHz
  *             - Bandwidth = 18 kHz / 8 = 2.25 kHz
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
#define N 16
#define NUM_LEDS 64 // 8x8 matrix
#define WS2812_BUFFER_SIZE (NUM_LEDS * 24) // 24 bits per LED
#define WS2812_RESET_PULSES 50 // >50us reset
#define WS2812_TOTAL_SIZE (WS2812_BUFFER_SIZE + WS2812_RESET_PULSES)
#define MATRIX_SIZE 8
#define PI 3.14159265358979323846

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;

volatile uint16_t adc_value = 0;
volatile uint8_t n_count = 0;
volatile uint8_t n_done = 0;
int16_t fft_input[2 * N]; // Real + Imaginary (interleaved)
int16_t fft_output[N]; // Magnitude output
uint16_t MAG[N];
uint8_t led_data[NUM_LEDS][3]; // GRB format for 64 LEDs
uint8_t ws2812_buffer[WS2812_TOTAL_SIZE]; // PWM buffer for DMA

// Lookup table for sin/cos (fixed-point, scaled to 16384 = 1.0)
const int16_t sin_table[N/4] = {0, 3196, 6270, 9102}; // sin(0, π/8, π/4, 3π/8) * 16384
const int16_t cos_table[N/4] = {16384, 15137, 11585, 6270}; // cos(0, π/8, π/4, 3π/8) * 16384

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void fft(void);
void mag_to_leds(void);
void ws2812_send(void);

/* Private user code ---------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        static uint8_t s = 0;

        // Read ADC value (triggered by TIM3)
        if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK)
        {
            adc_value = HAL_ADC_GetValue(&hadc1);
        }

        // Sampling N point FFT at 36 kHz (every 2 interrupts)
        s++;
        if (s >= 2)
        {
            if (n_done == 0)
            {
                // Convert ADC to int16_t, remove DC offset (~512)
                fft_input[2 * n_count] = (int16_t)adc_value - 512; // Real part
                fft_input[2 * n_count + 1] = 0; // Imaginary part
                n_count++;

                if (n_count >= N)
                {
                    n_done = 1;
                    n_count = 0;
                }
            }
            s = 0;
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();

    // Start TIM3 and ADC
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_ADC_Start(&hadc1);

    while (1)
    {
        if (n_done)
        {
            fft();
            mag_to_leds();
            ws2812_send();
            n_done = 0;
        }
    }
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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @retval None
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADCEx_Calibration_Start(&hadc1);
}

/**
  * @brief TIM1 Initialization Function
  * @retval None
  */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 89;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM3 Initialization Function
  * @retval None
  */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999; // 72 MHz / 1000 = 72 kHz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief Custom 16-point FFT (fixed-point)
  */
void fft(void)
{
    uint8_t i, j, k, l;
    uint8_t m = 4; // log2(16)
    uint8_t nm1 = N - 1;
    uint8_t nd2 = N / 2;
    int16_t tr, ti, ur, ui, sr, si;
    uint8_t jm1, ip;

    // Bit-reversal
    j = nd2;
    for (i = 1; i <= N - 2; i++)
    {
        if (i < j)
        {
            tr = fft_input[2 * j];
            ti = fft_input[2 * j + 1];
            fft_input[2 * j] = fft_input[2 * i];
            fft_input[2 * j + 1] = fft_input[2 * i + 1];
            fft_input[2 * i] = tr;
            fft_input[2 * i + 1] = ti;
        }
        k = nd2;
        while (k <= j)
        {
            j -= k;
            k >>= 1;
        }
        j += k;
    }

    // Butterfly computation
    for (l = 1; l <= m; l++)
    {
        uint8_t le = 1 << l; // 2^l
        uint8_t le2 = le >> 1; // 2^(l-1)
        ur = 16384; // 1.0 in Q14
        ui = 0;
        uint8_t angle_step = N / le; // Angle increment per stage
        for (j = 0; j < le2; j++)
        {
            // Get sin/cos from lookup table
            uint8_t idx = (j * angle_step) % (N/4);
            sr = cos_table[idx];
            si = -sin_table[idx];
            for (i = j; i <= nm1; i += le)
            {
                ip = i + le2;
                // Butterfly
                tr = ((int32_t)fft_input[2 * ip] * ur - (int32_t)fft_input[2 * ip + 1] * ui) >> 14;
                ti = ((int32_t)fft_input[2 * ip] * ui + (int32_t)fft_input[2 * ip + 1] * ur) >> 14;
                fft_input[2 * ip] = fft_input[2 * i] - tr;
                fft_input[2 * ip + 1] = fft_input[2 * i + 1] - ti;
                fft_input[2 * i] += tr;
                fft_input[2 * i + 1] += ti;
            }
            // Update twiddle factors
            int32_t temp = ((int32_t)ur * sr - (int32_t)ui * si) >> 14;
            ui = ((int32_t)ur * si + (int32_t)ui * sr) >> 14;
            ur = temp;
        }
    }

    // Compute magnitude (skip DC)
    for (i = 0; i <= N/2; i++)
    {
        int32_t real = fft_input[2 * i];
        int32_t imag = fft_input[2 * i + 1];
        fft_output[i] = (int16_t)((real * real + imag * imag) >> 10); // Approximate sqrt
    }
}

/**
  * @brief Map FFT magnitudes to LED matrix
  */
void mag_to_leds(void)
{
    uint8_t i, j;

    // Scale and limit magnitude
    for (i = 0; i <= N/2; i++)
    {
        MAG[i] = fft_output[i] >> 4; // Scale to 0-8
        if (MAG[i] > 8)
        {
            MAG[i] = 8;
        }
    }

    // Clear LED data
    for (i = 0; i < NUM_LEDS; i++)
    {
        led_data[i][0] = 0; // G
        led_data[i][1] = 0; // R
        led_data[i][2] = 0; // B
    }

    // Map FFT magnitudes to 8x8 matrix (zigzag layout)
    for (i = 0; i < MATRIX_SIZE; i++) // Columns (frequency bins)
    {
        uint8_t bin = i + 1; // Skip DC (bin 0)
        uint8_t mag = MAG[bin];
        for (j = 0; j < mag; j++) // Rows (amplitude)
        {
            // Calculate LED index in zigzag pattern
            uint8_t row = j;
            uint8_t col = i;
            uint8_t index;
            if (col % 2 == 0)
            {
                // Even columns: top to bottom
                index = col * MATRIX_SIZE + row;
            }
            else
            {
                // Odd columns: bottom to top
                index = col * MATRIX_SIZE + (MATRIX_SIZE - 1 - row);
            }

            // Set color based on amplitude
            if (mag < 3)
            {
                // Low amplitude: Blue
                led_data[index][0] = 0;  // G
                led_data[index][1] = 0;  // R
                led_data[index][2] = 16; // B
            }
            else if (mag < 6)
            {
                // Medium amplitude: Green
                led_data[index][0] = 16; // G
                led_data[index][1] = 0;  // R
                led_data[index][2] = 0;  // B
            }
            else
            {
                // High amplitude: Red
                led_data[index][0] = 0;  // G
                led_data[index][1] = 16; // R
                led_data[index][2] = 0;  // B
            }
        }
    }
}

/**
  * @brief Send data to WS2812 LEDs via DMA
  */
void ws2812_send(void)
{
    uint8_t i, j;
    uint32_t buffer_index = 0;

    // Fill PWM buffer
    for (i = 0; i < NUM_LEDS; i++)
    {
        // GRB order
        for (j = 7; j != 255; j--)
        {
            ws2812_buffer[buffer_index++] = (led_data[i][0] & (1 << j)) ? 60 : 30; // Green
        }
        for (j = 7; j != 255; j--)
        {
            ws2812_buffer[buffer_index++] = (led_data[i][1] & (1 << j)) ? 60 : 30; // Red
        }
        for (j = 7; j != 255; j--)
        {
            ws2812_buffer[buffer_index++] = (led_data[i][2] & (1 << j)) ? 60 : 30; // Blue
        }
    }

    // Add reset pulses
    for (i = 0; i < WS2812_RESET_PULSES; i++)
    {
        ws2812_buffer[buffer_index++] = 0;
    }

    // Start DMA transfer
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)ws2812_buffer, WS2812_TOTAL_SIZE);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
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
}
#endif /* USE_FULL_ASSERT */
