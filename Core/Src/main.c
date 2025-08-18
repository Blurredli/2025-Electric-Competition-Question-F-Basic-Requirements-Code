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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "AD9959.h"
#include "ADF4351.h"
#include "tjc_usart_hmi.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"
#include <math.h>      // �ṩ fabsf, roundf, fmaxf ����ѧ����
#include <stdlib.h>    // �ṩ abs (����)
#include "RDA5807M.h"
#include <si5351.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DETECT_BUF_LEN 25

#define F_START    87800000UL   // ��ʼ RF Ƶ�� 88 MHz
#define F_END      108100000UL  // ���� RF Ƶ�� 108 MHz
#define F_STEP     100000UL     // �������� 100 kHz

// DDS ͨ������
#define DDS_CH_FM       CH0            // AD9959 ͨ�� 0 ���� FM LO
#define DDS_CH_AM       CH1            // AD9959 ͨ�� 1 ���� AM LO
#define DDS_AMP         334         // ������ֵ��10-bit��
#define DDS_PHASE       0            // ��λУ��ֵ (0)
#define DETECT_THRESHOLD 1.7f        // �����ֵ��<1.7V ��ʾ�ɹ������δ���Ĭ�� 3.3V


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t detect_buf1[DETECT_BUF_LEN];
volatile uint16_t detect_buf2[DETECT_BUF_LEN];
/* 
DetectConvEnd1�������ADC�Ƿ�ɼ����
0��û�вɼ����
1���ɼ���ϣ���stm32f1xx_it���DMA����жϽ����޸�
 */
volatile uint8_t  DetectConvEnd1 = 0;
volatile uint8_t  DetectConvEnd2 = 0;
volatile uint8_t  DetectConvEnd3 = 0;
uint8_t found = 0;        // ��ʾ�Ƿ��Ѿ���Ƶ

#define FM_MODE         1            // FM ģʽ
#define AM_MODE         2            // AM ģʽ

uint8_t MODE = FM_MODE; // Ĭ��ģʽΪ FM
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == KEY2_Pin)
    {
        // �򵥵���������
        static uint32_t last_time = 0;
        uint32_t current_time = HAL_GetTick();
        
        if((current_time - last_time) > 20) // 50ms����
        {
            // ȷ�ϰ���״̬
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
            {
                // �������´���
                found = 0;
            }
        }
        last_time = current_time;
    }
}

/**
 * @brief ͨ��ָ�� DDS ͨ��������Ҳ����ȴ��ȶ�
 */
static void DDS_Output_Channel(uint8_t ch, uint32_t freq)
{
    AD9959_Set_Fre(ch,   freq);
    AD9959_Set_Amp(ch,   DDS_AMP);
    AD9959_Set_Phase(ch, DDS_PHASE);
    IO_Update();
    delay_us(1000);
}

/**
 * @brief ͬʱ������ͨ�� DDS ���
 */
static void DDS_Output_Two(uint32_t lo_fm, uint32_t lo_am)
{
    DDS_Output_Channel(DDS_CH_FM, lo_fm);
    DDS_Output_Channel(DDS_CH_AM, lo_am);
}

/**
 * @brief ͬʱʹ�� TIM3 ���� ADC1/ADC2 + DMA �ɼ���·����ѹ
 * @param[out] v_fm ��� FM ͨ��ƽ����ѹ (V)
 * @param[out] v_am ��� AM ͨ��ƽ����ѹ (V)
 */
static void Compute_Dual_Voltage(float *v_fm, float *v_am)
{
    DetectConvEnd1 = DetectConvEnd2 = 0;
    // ���� DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)detect_buf1, DETECT_BUF_LEN);
    // HAL_ADC_Start_DMA(&hadc2, (uint32_t*)detect_buf2, DETECT_BUF_LEN);
    // ���� TIM3 ������· ADC
    HAL_TIM_Base_Start(&htim3);
    // �ȴ���·���
    while (!DetectConvEnd1);
    // while (!(DetectConvEnd1 && DetectConvEnd2));
    // ֹͣ������ DMA
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);
    // HAL_ADC_Stop_DMA(&hadc2);

    // ���� FM ƽ����ѹ
    uint32_t sum1 = 0;
    for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum1 += detect_buf1[i];
    *v_fm = ((float)sum1 / DETECT_BUF_LEN) * (3.3f / 4095.0f);     // PA5
    *v_am = *v_fm;     // PA5
    // ���� AM ƽ����ѹ
    // uint32_t sum2 = 0;
    // for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum2 += detect_buf2[i];
    // *v_am = ((float)sum2 / DETECT_BUF_LEN) * (3.3f / 4095.0f);     // PA0

}

/**
 * @brief �ɼ���ͨ��ADC��ѹ��DMA��ʽ��������ƽ����ѹֵ����λ��V��
 * @return ƽ����ѹֵ (float)
 */
static float Compute_Single_Voltage(void)
{
    DetectConvEnd1 = 0;
    // ����DMA�ɼ�
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)detect_buf1, DETECT_BUF_LEN);
    // ������ʱ������ADC
    HAL_TIM_Base_Start(&htim3);
    // �ȴ��ɼ���ɣ�����DMA�ж�������conv_end=1��
    while (!DetectConvEnd1);
    // ֹͣ��ʱ����DMA
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);

    // ����ƽ��ֵ
    uint32_t sum = 0;
    for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum += detect_buf1[i];
    float voltage = ((float)sum / DETECT_BUF_LEN) * (3.3f / 4095.0f); // ����ο���ѹ3.3V��12λADC
    return voltage;
}


///**
// * @brief �ɼ���ͨ��ADC��ѹ��DMA��ʽ��������ƽ����ѹֵ����λ��V��
// * @return ƽ����ѹֵ (float)
// */
//static float MODE_Voltage(void)
//{
//    DetectConvEnd3 = 0;
//    // ����DMA�ɼ�
//    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)detect_buf1, DETECT_BUF_LEN);
//    // ������ʱ������ADC
//    HAL_TIM_Base_Start(&htim3);
//    // �ȴ��ɼ���ɣ�����DMA�ж�������DetectConvEnd3=1��
//    while (!DetectConvEnd3);
//    // ֹͣ��ʱ����DMA
//    HAL_TIM_Base_Stop(&htim3);
//    HAL_ADC_Stop_DMA(&hadc3);

//    // ����ƽ��ֵ
//    uint32_t sum = 0;
//    for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum += detect_buf1[i];
//    float voltage = ((float)sum / DETECT_BUF_LEN) * (3.3f / 4095.0f); // ����ο���ѹ3.3V��12λADC
//    return voltage;
}
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  delay_init(168);                            /* ��ʱ��ʼ�� */
  HAL_Delay(50); 
  AD9959_Init();
  // ADF4351Init();
  // RDA5807M_init();    // ��ʼ�� RDA5807M
  // initRingBuffer();		//��ʼ�����λ�����
  // HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);	//�򿪴��ڽ����ж�

  // char str[100];
  // uint32_t nowtime = HAL_GetTick();
  // uint32_t last_m = 0xFFFFFFFF;  // ��ʼ��Ϊһ�������ܵ�ֵ
  HAL_Delay(50);
  // RDA5807M_Set_Volume(10);
  uint32_t locked_rf = 0;   // ��¼������RFƵ��
  float v_fm, v_am;  // fm_
  float v_mode;
  uint32_t best_rf_fm;
  uint32_t best_rf_am;
//  MODE = 2;
  // const int32_t correction = 978;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  if (!found)
  {
    if (1)
    {
      // FMģʽ����ɨƵ
      for (uint32_t rf = F_START; rf <= F_END; rf += 300000UL) 
      {
        uint32_t lo_fm = (rf > 10700000UL) ? (rf - 10700000UL) : 0;
        DDS_Output_Channel(CH0, lo_fm); // ֻ���FMͨ��
        Compute_Dual_Voltage(&v_fm, &v_am);
        
        // FM��ɨ�жϣ�ֻ�ж�FM��ѹ
        if (v_fm <= 0.10f) {
          found = 1;
          locked_rf = rf;
          printf("FM Found at RF=%lu Hz, FM=%lu Hz, Vfm=%.3f V\r\n", rf, (rf - 10700000UL), v_fm);
          // --- FM��ɨ ---
          float min_vfm = v_fm;
          best_rf_fm = rf;
          // �ڴ�ɨ���е�Ƶ�ʡ�500kHz��Χ�ڣ�����50kHz��ϸɨ��
          for (uint32_t rf_fine = (rf > 0 ? rf : F_START); rf_fine <= rf + 500000UL && rf_fine <= F_END; rf_fine += 50000UL) {
            uint32_t lo_fm_fine = (rf_fine > 10700000UL) ? (rf_fine - 10700000UL) : 0;
            DDS_Output_Channel(CH0, lo_fm_fine); // ֻ���FMͨ��
            HAL_Delay(15); // �ȴ�DDS����ȶ�
            float v_fm_fine = Compute_Single_Voltage(); // ʹ�õ�ͨ���ɼ�
            if (v_fm_fine < min_vfm) {
              min_vfm = v_fm_fine;
              best_rf_fm = rf_fine;
            }
          }
          printf("[FineScan] Best FM at RF=%lu Hz, FM=%lu Hz, Vfm=%.3f V\r\n", best_rf_fm, (best_rf_fm - 10700000UL), min_vfm);
          DDS_Output_Channel(CH0, best_rf_fm > 10700000UL ? (best_rf_fm - 10700000UL) : 0); // �������Ƶ��
          HAL_Delay(50);
          HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_AM_GPIO_Port, LED_AM_Pin, GPIO_PIN_SET);

          v_mode = MODE_Voltage();        // ADC3 = PC2
          printf("Current Mode Voltage: V=%.4f V\r\n", v_mode);

          if (v_mode < 0.005f)
          {
            HAL_GPIO_WritePin(Get_AM_GPIO_Port, Get_AM_Pin, GPIO_PIN_RESET);
            HAL_Delay(50);
            best_rf_am = best_rf_fm;
            printf("[FineScan] Best AM at RF=%lu Hz, AM=%lu Hz, Vam=%.3f V\r\n", best_rf_am, (best_rf_am > 455000UL ? best_rf_am - 455000UL : 0));
            DDS_Output_Channel(CH0, best_rf_am > 455000UL ? (best_rf_am - 455000UL) : 0); // �������Ƶ��
            HAL_Delay(50);
              HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_AM_GPIO_Port, LED_AM_Pin, GPIO_PIN_RESET);
              break;
          }
            }
          }
        }
    }
    
    if (!found) {
      printf("No signal detected\r\n");
      HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_AM_GPIO_Port, LED_AM_Pin, GPIO_PIN_RESET);
    }
  
  // DDS_Output_Channel(CH0, best_rf_am > 455000UL ? (best_rf_am - 455000UL) : 0); // �������Ƶ��
  // printf("Best AM at RF=%lu Hz, AM=%lu Hz, Vam=%.3f V\r\n", best_rf_am, (best_rf_am > 455000UL ? best_rf_am - 455000UL : 0), v_am);
  HAL_Delay(300);
  // float current_v = Compute_Single_Voltage(); // ʹ�õ�ͨ���ɼ�
  // printf("��ǰ����ѹ: V=%.4f V\r\n", current_v);
    //  // �����жϣ������ѹ������ֵ�������źŶ�ʧ
    //  if (found && current_v > 0.2f)
    //  {
    //    // �ٴ�ȷ�ϣ��ų�����
    //    HAL_Delay(50);
    //    current_v = Compute_Single_Voltage();
    //    if (current_v > 0.2f)
    //    {
    //      found = 0; // ����״̬���´�ѭ����������
    //      printf("�źŶ�ʧ����ѹ=%.4f V\r\n", current_v);
    //    }
    //  }
  // printf("MODE=%d\r\n", MODE);

  /* USER CODE END WHILE */
}
    /* USER CODE BEGIN 3 */

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
