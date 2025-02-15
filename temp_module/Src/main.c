#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"

/* Constants for thermistor calculations */
#define RT0 10000    // Thermistor resistance at 25°C
#define B 7500       // Beta constant
#define VCC 3.3      // Supply voltage
#define R 10000      // Voltage divider resistor
#define T0 298.15    // Reference temperature in Kelvin (25°C)
#define NUM_SENSORS 4 // Number of thermistor sensors

/* Handle structures */
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
USART_HandleTypeDef huart2;
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

/* Global variables */
float temperature[NUM_SENSORS];
volatile uint8_t adcConvComplete = 0;
volatile uint32_t errorCount = 0;

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_UART2_Init(void);
void readSensors(void);
void calculateTemperatures(float *Tmax, float *Tmin, float *avg);
void Error_Handler(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CAN_Init();
    MX_UART2_Init();

    /* Start CAN peripheral with error checking */
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }

    /* Configure CAN filter to accept all messages */
    CAN_FilterTypeDef canFilterConfig;
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0;
    canFilterConfig.FilterIdLow = 0;
    canFilterConfig.FilterMaskIdHigh = 0;
    canFilterConfig.FilterMaskIdLow = 0;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Main loop */
    while (1) {
        /* Toggle LED to indicate system is running */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        /* Read sensor data */
        readSensors();

        float Tmax, Tmin, avg;
        calculateTemperatures(&Tmax, &Tmin, &avg);

        /* Prepare CAN message */
        TxHeader.ExtId = 0x1839F380;
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = 8;

        /* Pack data - using multiplication to preserve decimal places */
        int16_t maxTemp = (int16_t)(Tmax * 10);
        int16_t minTemp = (int16_t)(Tmin * 10);
        int16_t avgTemp = (int16_t)(avg * 10);

        TxData[0] = (uint8_t)(maxTemp >> 8);
        TxData[1] = (uint8_t)(maxTemp & 0xFF);
        TxData[2] = (uint8_t)(minTemp >> 8);
        TxData[3] = (uint8_t)(minTemp & 0xFF);
        TxData[4] = (uint8_t)(avgTemp >> 8);
        TxData[5] = (uint8_t)(avgTemp & 0xFF);
        TxData[6] = NUM_SENSORS;
        TxData[7] = errorCount > 255 ? 255 : (uint8_t)errorCount;

        /* Transmit CAN message with retry mechanism */
        uint8_t retryCount = 0;
        while (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
            retryCount++;
            if (retryCount >= 3) {
                Error_Handler();
                break;
            }
            HAL_Delay(10);
        }

        /* Print message to ITM data console */
        char buffer[100];
        int len_uart = sprintf(buffer, "CAN message sent\r\n");
        for (int i = 0; i < len_uart; i++) {
            ITM_SendChar(buffer[i]);
        }

        /* Send debug information via ITM data console */
        len_uart = sprintf(buffer, "Tmax: %.1f, Tmin: %.1f, Avg: %.1f, Errors: %lu\r\n",
                         Tmax, Tmin, avg, errorCount);
        for (int i = 0; i < len_uart; i++) {
            ITM_SendChar(buffer[i]);
        }

        HAL_Delay(1000);  // 1 second delay between measurements
    }
}

void readSensors(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    for (int i = 0; i < NUM_SENSORS; i++) {
        /* Configure ADC channel */
        sConfig.Channel = ADC_CHANNEL_0 + i;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  // Increased sampling time for stability

        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            errorCount++;
            continue;
        }

        /* Perform ADC conversion with timeout */
        if (HAL_ADC_Start(&hadc1) != HAL_OK) {
            errorCount++;
            continue;
        }

        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

            /* Calculate temperature using Steinhart-Hart equation */
            float voltage = (float)adcValue * VCC / 4095.0f;
            float resistance = R * voltage / (VCC - voltage);

            if (resistance > 0) {  // Prevent log of negative number
                float steinhart = log(resistance / RT0) / B;
                steinhart += 1.0f / T0;
                temperature[i] = (1.0f / steinhart) - 273.15f;

                /* Sanity check on temperature value */
                if (temperature[i] < -50.0f || temperature[i] > 150.0f) {
                    temperature[i] = 25.0f;  // Default to room temperature if reading is invalid
                    errorCount++;
                }
            } else {
                temperature[i] = 25.0f;
                errorCount++;
            }
        } else {
            temperature[i] = 25.0f;
            errorCount++;
        }

        HAL_ADC_Stop(&hadc1);
    }
}

void calculateTemperatures(float *Tmax, float *Tmin, float *avg) {
    *Tmax = temperature[0];
    *Tmin = temperature[0];
    *avg = temperature[0];

    for (int i = 1; i < NUM_SENSORS; i++) {
        if (temperature[i] > *Tmax) *Tmax = temperature[i];
        if (temperature[i] < *Tmin) *Tmin = temperature[i];
        *avg += temperature[i];
    }

    *avg /= NUM_SENSORS;

    // Print temperature data to ITM data console
    char buffer[100];
    int len_uart = sprintf(buffer, "Tmax: %.1f, Tmin: %.1f, Avg: %.1f, Errors: %lu\r\n",
                         *Tmax, *Tmin, *avg, errorCount);
    for (int i = 0; i < len_uart; i++) {
        ITM_SendChar(buffer[i]);
    }
}

static void MX_ADC1_Init(void) {
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* Calibrate ADC before use */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_CAN_Init(void) {
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 9;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_UART2_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = USART_WORDLENGTH_8B;
    huart2.Init.StopBits = USART_STOPBITS_1;
    huart2.Init.Parity = USART_PARITY_NONE;
    huart2.Init.Mode = USART_MODE_TX_RX;
    //huart2.Init.HwFlowCtl = USART_HWCONTROL_NONE;
    //huart2.Init.OverSampling = USART_OVERSAMPLING_16;

    if (HAL_USART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /* Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Initialize the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;                  // Enable HSE
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;              // Enable PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;      // Set HSE as PLL source
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;              // Multiply HSE by 9 (for 8 MHz, gives 72 MHz)

    /* Initialize the RCC Oscillator */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();  // Handle error
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1, and PCLK2 clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as system clock
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;       // Set AHB clock divider
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;        // Set APB1 clock divider
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;        // Set APB2 clock divider

    /* Initialize the CPU, AHB and APB clocks */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();  // Handle error
    }

    /* Enable the ITM peripheral */
    ITM->LAR = 0xC5ACCE55;
    ITM->TER = 0xFFFFFFFF;
}

void Error_Handler(void) {
    errorCount++;
    /* Visual feedback for errors */
    for(int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}
