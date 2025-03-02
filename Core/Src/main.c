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

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <string.h>

#include "luts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** Clamp para evitar wind-up do integrador. */
#define CLAMP(val, min, max) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))

/** Constantes do controlador PI, calculadas por Ziegler-Nichols. */
#define L 9.02f
#define T 344.21f
#define Ti (2 * L)
#define Td (L / 2.0f)
#define Kp (1.2f * (T / L))
#define Ki (Kp / Ti)
#define Kd (Kp * Td)

/** Número de medições para média de leituras **/
#define MEAN_EXP 5
#define MEAN_SPAN (1 << MEAN_EXP)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static struct control_area {
    /* O bloco de dados para controle e o id devem estar nesta ordem, para acesso direto de memória pelo computador. */
    volatile struct {
        char id[4];
        float sensor_measures[SENSOR_AMOUNT];
        float duty_cycle;
        float desired;
    } data;

    uint16_t adc_measures[SENSOR_AMOUNT][MEAN_SPAN];

    struct {
        uint32_t pos;
        uint32_t neg;
    } pwm_pulse;

    float I;
    float prev_err;
    uint32_t prev_ms;

    bool ready;
} control = {
        .data =
                {
                        .id =
                                {
                                        '!',
                                        'C',
                                        'T',
                                        'R',
                                },
                        .desired = 25.0f,
                },
        .ready = false,
};

/** ADC DMA buffer. */
static uint32_t adc_dma_buffer[SENSOR_AMOUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** Ajuste do PID, com medidas anti-windup. */
static float P_I_D(const float dt, const float desired, const float measured) {
    const float err = desired - measured;
    const float P = Kp * err;
    const float I_inc = Ki * err * dt;
    float D = Kd * (err - control.prev_err) / (dt + 0.000001f);

    control.prev_err = err;

    const float windup_check = P + control.I + I_inc;

    if (windup_check > 100) {
        return 100;
    }

    if (windup_check < -100) {
        return -100;
    }

    control.I += I_inc;

    return P + control.I;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    uint32_t span_sum = 0;
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
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_ADC_Start_DMA(&hadc1, adc_dma_buffer, SENSOR_AMOUNT);

    while (!control.ready) {
        HAL_Delay(1);
    }

    for (enum sensor_id sensor_id = 0; sensor_id < SENSOR_AMOUNT; sensor_id++) {
        memset(control.adc_measures[sensor_id], (int) adc_dma_buffer[sensor_id], sizeof(control.adc_measures[sensor_id]));
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        for (enum sensor_id sensor_id = 0; sensor_id < 2; sensor_id++) {
            for (size_t measure_i = 0; measure_i < MEAN_SPAN; measure_i++) {
                span_sum += control.adc_measures[sensor_id][measure_i];
            }
            span_sum <<= MEAN_EXP;
            control.data.sensor_measures[sensor_id] = adc_luts[sensor_id][span_sum];
        }

        float mean_current_measure = 0;
        for (enum sensor_id sensor_id = 0; sensor_id < SENSOR_AMOUNT; sensor_id++) {
            mean_current_measure += control.data.sensor_measures[sensor_id];
        }
        mean_current_measure /= SENSOR_AMOUNT;
        const float dt = (float) (HAL_GetTick() - control.prev_ms) / 1000.0f;
        control.data.duty_cycle = P_I_D(dt, control.data.desired, mean_current_measure);
        control.prev_ms = HAL_GetTick();

        control.data.duty_cycle = CLAMP(control.data.duty_cycle, -100, 100);
        if (control.data.duty_cycle > 0) {
            control.pwm_pulse.pos = (uint32_t) (((control.data.duty_cycle / 100.0f) * TIM3->ARR) + 0.5f);
            control.pwm_pulse.neg = 0;
        } else {
            control.pwm_pulse.pos = 0;
            control.pwm_pulse.neg = (uint32_t) (((control.data.duty_cycle / -100.0f) * TIM3->ARR) + 0.5f);
        }

        TIM3->CCR3 = control.pwm_pulse.pos;
        TIM3->CCR4 = control.pwm_pulse.neg;

        HAL_Delay(1);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    static size_t measure_index = 0;

    for (enum sensor_id sensor_id = 0; sensor_id < SENSOR_AMOUNT; sensor_id++) {
        control.adc_measures[sensor_id][measure_index] = adc_dma_buffer[sensor_id];
    }

    measure_index = (measure_index + 1) % MEAN_SPAN;
    control.ready = true;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
        HAL_Delay(500);
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
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
