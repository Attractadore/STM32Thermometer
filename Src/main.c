/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>
#include <iso646.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void DisplaySelectDR();
void DisplaySelectCR();
void DisplaySelectWrite();
void DisplayEnable();
void DisplayWait4();
void DisplayBits4(uint8_t bits);

void DisplayInit4();
void DisplaySetCursor4(uint8_t y, uint8_t x);

typedef enum {
    DISPLAY_DATA_LENGTH_4 = 0,
    DISPLAY_DATA_LENGTH_8 = 1 << 4,
} DisplayDataLength;

typedef enum {
    DISPLAY_1_LINE  = 0,
    DISPLAY_2_LINES = 1 << 3,
} DisplayNumberOfLines;

typedef enum {
    DISPLAY_FONT_5X8  = 0,
    DISPLAY_FONT_5X10 = 1 << 2,
} DisplayFont;

void DisplayFunctionSet4(
    DisplayDataLength data_length,
    DisplayNumberOfLines nlines,
    DisplayFont font
);

typedef enum {
    DISPLAY_DISPLAY_ON  = 1 << 2,
    DISPLAY_DISPLAY_OFF = 0,
} DisplayDisplayState;

typedef enum {
    DISPLAY_CURSOR_ON  = 1 << 1,
    DISPLAY_CURSOR_OFF = 0,
} DisplayCursorState;

typedef enum {
    DISPLAY_CURSOR_BLINK_ON  = 1 << 0,
    DISPLAY_CURSOR_BLINK_OFF = 0,
} DisplayCursorBlinkState;

void DisplayControl4(
    DisplayDisplayState dpy,
    DisplayCursorState cursor,
    DisplayCursorBlinkState blink
);

typedef enum {
    DISPLAY_CURSOR_DIRECTION_RIGHT = 1 << 1,
    DISPLAY_CURSOR_DIRECTION_LEFT  = 0,
} DisplayCursorDirection;

typedef enum {
    DISPLAY_DONT_SHIFT        = 0,
    DISPLAY_SHIFT_WITH_CURSOR = 1 << 0,
} DisplayDisplayShift;

void DisplayEntryMode4(
    DisplayCursorDirection dir,
    DisplayDisplayShift shift
);

void DisplayClear4();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DisplaySelectDR() {
    HAL_GPIO_WritePin(DPY_RS_GPIO_Port, DPY_RS_Pin, 1);
}

void DisplaySelectCR() {
    HAL_GPIO_WritePin(DPY_RS_GPIO_Port, DPY_RS_Pin, 0);
}

void DisplaySelectWrite() {
    //HAL_GPIO_WritePin(DPY_RW_GPIO_Port, DPY_RW_Pin, 0);
}

void DisplayEnable() {
    HAL_GPIO_WritePin(DPY_EN_GPIO_Port, DPY_EN_Pin, 1);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DPY_EN_GPIO_Port, DPY_EN_Pin, 0);
    HAL_Delay(1);
}

void DisplayWait4() {
    HAL_Delay(2);
}

void DisplayBits4(uint8_t bits) {
    HAL_GPIO_WritePin(DPY_4_GPIO_Port, DPY_4_Pin, bits & 1u);
    bits >>= 1;
    HAL_GPIO_WritePin(DPY_5_GPIO_Port, DPY_5_Pin, bits & 1u);
    bits >>= 1;
    HAL_GPIO_WritePin(DPY_6_GPIO_Port, DPY_6_Pin, bits & 1u);
    bits >>= 1;
    HAL_GPIO_WritePin(DPY_7_GPIO_Port, DPY_7_Pin, bits & 1u);
    bits >>= 1;
}

void DisplayCommand4(uint8_t bits) {
    DisplaySelectCR();
    DisplaySelectWrite();
    DisplayBits4(bits >> 4);
    DisplayEnable();
    DisplayBits4(bits & 0xF);
    DisplayEnable();
    DisplayWait4();
}

void DisplayCharacter4(char c) {
    uint8_t bits = c;
    DisplaySelectDR();
    DisplaySelectWrite();
    DisplayBits4(bits >> 4);
    DisplayEnable();
    DisplayBits4(bits & 0xF);
    DisplayEnable();
    DisplayWait4();
}

void DisplayString4(const char* str) {
    while(*str) {
        DisplayCharacter4(*str++);
    }
}

void DisplayInit4() {
    DisplaySelectCR();
    DisplaySelectWrite();

    HAL_Delay(40);
    DisplayBits4(0x3);
    DisplayEnable();
    HAL_Delay(5);
    DisplayBits4(0x3);
    DisplayEnable();
    HAL_Delay(1);
    DisplayBits4(0x3);
    DisplayEnable();
    DisplayBits4(0x2);
    DisplayEnable();

    DisplayFunctionSet4(
        DISPLAY_DATA_LENGTH_4,
        DISPLAY_1_LINE,
        DISPLAY_FONT_5X8
    );
    DisplayControl4(
        DISPLAY_DISPLAY_ON,
        DISPLAY_CURSOR_OFF,
        DISPLAY_CURSOR_BLINK_OFF
    );
    DisplayEntryMode4(
        DISPLAY_CURSOR_DIRECTION_RIGHT,
        DISPLAY_DONT_SHIFT
    );
    DisplayClear4();
    DisplaySetCursor4(0, 0);
}

void DisplayClear4() {
    DisplayCommand4(0x01);
}

void DisplaySetCursor4(uint8_t y, uint8_t x) {
    assert(y >= 0 and y <= 2);
    assert(x >= 0 and x <= 16);
    uint8_t bits = (y ? 0xC0 : 0x80) + x;
    DisplayCommand4(bits);
}

void DisplayEntryMode4(
    DisplayCursorDirection dir,
    DisplayDisplayShift shift
) {
    uint8_t bits = 0x04 | dir | shift;
    DisplayCommand4(bits);
}

void DisplayControl4(
    DisplayDisplayState dpy,
    DisplayCursorState cursor,
    DisplayCursorBlinkState blink
) {
    uint8_t bits = 0x08 | dpy | cursor | blink;
    DisplayCommand4(bits);
}

void DisplayFunctionSet4(
    DisplayDataLength data_length,
    DisplayNumberOfLines nlines,
    DisplayFont font
) {
    uint8_t bits = 0x20 | data_length | nlines | font;
    DisplayCommand4(bits);
}

float GetResistance(float v, float v0, float topr) {
    float a = v / v0;
    return a / (1.0f - a) * topr;
}

static const float very_hot = INFINITY;
static const float very_cold = -INFINITY;
float GetTemperature(float r) {
    static const float resistances[] = {
        260.06,
        267.55,
        275.28,
        283.27,
        291.53,
        300.07,
        308.91,
        318.06,
        327.52,
        337.3,
        347.42,
        357.9,
        368.74,
        379.95,
        391.56,
        403.58,
        415.96,
        428.77,
        442.04,
        455.79,
        470.03,
        484.79,
        500.08,
        515.94,
        532.37,
        549.41,
        567.19,
        585.64,
        604.78,
        624.63,
        645.24,
        666.62,
        688.83,
        711.88,
        735.82,
        760.68,
        786.31,
        812.94,
        840.61,
        869.38,
        899.28,
        930.37,
        962.7,
        996.34,
        1031.3,
        1067.7,
        1106,
        1145.8,
        1187.3,
        1230.5,
        1275.5,
        1322.4,
        1371.2,
        1422.1,
        1475.1,
        1530.5,
        1588.3,
        1648.7,
        1711.6,
        1777.3,
        1845.9,
        1917.5,
        1992.2,
        2070.3,
        2151.8,
        2237,
        2325.9,
        2418.9,
        2516.1,
        2617.9,
        2724.3,
        2835.7,
        2952.3,
        3074.4,
        3202.3,
        3336.3,
        3475.6,
        3621.5,
        3774.4,
        3934.7,
        4102.6,
        4278.8,
        4463.5,
        4657.2,
        4860.6,
        5074,
        5300.2,
        5537.8,
        5787.7,
        6050.4,
        6326.8,
        6617.5,
        6923.5,
        7245.6,
        7584.7,
        7942,
        8312.2,
        8701.9,
        9112.3,
        9544.6,
        10000,
        10481,
        10988,
        11523,
        12088,
        12683,
        13313,
        13977,
        14679,
        15420,
        16204,
        17033,
        17910,
        18838,
        19820,
        20860,
        21969,
        23145,
        24392,
        25715,
        27119,
        28609,
        30192,
        31874,
        33662,
        35563,
        37587,
        39740,
        42033,
        44475,
        47077,
        49850,
        52808,
        55963,
        59331,
        62927,
        66697,
        70721,
        75017,
        79606,
        84510,
        89752,
        95358,
        101360,
        107780,
        114660,
        121900,
        129660,
        137960,
        146860,
        156410,
        166640,
        177620,
        189410,
        202070,
        215670,
        230150,
        245690,
        262360,
        280260,
        299470,
        320120,
        342300,
        366150,
        391800,
        419380,
        449070,
        481020,
        515430,
        552490,
        592430,
        635490,
        681910,
        732000,
        786040,
        844390,
        907400,
        975470,
        1049000,
        1128600,
        1214600,
    };

    static const float base_temp    = 125.0f;
    static const float temp_step    = -1.0f;

    enum { resistance_cnt           = sizeof(resistances) / sizeof(float) };
    static const float min_r        = resistances[0];
    static const float max_r        = resistances[resistance_cnt - 1];

    if (r < min_r) {
        return very_hot;
    } else if (r >= max_r) {
        return very_cold;
    }

    size_t i = 0;
    for (; resistances[i + 1] <= r; i++) {
        assert(i + 1 < resistance_cnt);
    }

    float low_r     = resistances[i];
    float high_r    = resistances[i + 1];
    float low_temp  = base_temp + i * temp_step;

    float a = (r - low_r) / (high_r - low_r);

    return low_temp + a * temp_step;
}

float GetTemperatureFromADC() {
    static const float base_voltage = 5.0f;
    static const float top_r = 10000.0f;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
    float voltage = 3.3f * adc_val / (1 << 12);

    float r = GetResistance(voltage, base_voltage, top_r);
    return GetTemperature(r);
}

void PrintTemperature(float t) {
    char buf[17] = {0};
    char print_buf[17] = {0};

    if (t == very_hot) {
        snprintf(buf, sizeof(buf), "VERY HOT");
    } else if (t == very_cold) {
        snprintf(buf, sizeof(buf), "VERY COLD");
    } else {
        snprintf(buf, sizeof(buf), "%.1f C", t);
    }

    static const size_t w = sizeof(buf) - 1;
    size_t i = 0;
    for (; buf[i]; i++) {}
    size_t l = i;

    size_t x = (w - l) - (w - l) / 2;
    for (i = 0; i < x; i++) {
        print_buf[i] = ' ';
    }
    for (size_t j = 0; j < l; j++, i++) {
        print_buf[i] = buf[j];
    }
    for (; i < sizeof(buf) - 1; i++) {
        print_buf[i] = ' ';
    }

    DisplaySetCursor4(0, 0);
    DisplayString4(print_buf);
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  DisplayInit4();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    static const float period_ms = 1000;
    enum { temp_history_length = 60 };
    float temp_history[temp_history_length] = {0};
    size_t temp_history_idx = 0;
    {
    float t = GetTemperatureFromADC();
    PrintTemperature(t);
    for (size_t i = 0; i < temp_history_length; i++) {
        temp_history[i] = t;
    }
    }

    while (true) {
  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        float new_t = GetTemperatureFromADC();
        temp_history[temp_history_idx++] = new_t;
        if (temp_history_idx == temp_history_length) {
            temp_history_idx = 0;
        }
        float t = 0.0f;
        for (size_t i = 0; i < temp_history_length; i++) {
            t += temp_history[i];
        }
        t /= temp_history_length;
        PrintTemperature(t);
        HAL_Delay(period_ms);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 130;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DebugLED_GPIO_Port, DebugLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DPY_RS_Pin|DPY_EN_Pin|DPY_4_Pin|DPY_5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DPY_6_Pin|DPY_7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DebugLED_Pin */
  GPIO_InitStruct.Pin = DebugLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DebugLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DPY_RS_Pin DPY_EN_Pin DPY_4_Pin DPY_5_Pin */
  GPIO_InitStruct.Pin = DPY_RS_Pin|DPY_EN_Pin|DPY_4_Pin|DPY_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DPY_6_Pin DPY_7_Pin */
  GPIO_InitStruct.Pin = DPY_6_Pin|DPY_7_Pin;
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
