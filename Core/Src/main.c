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
#define FRAME_LENGTH 8
#define DETECT_BUF_LEN 25

#define F_START    88000000UL   // ��ʼ RF Ƶ�� 88 MHz
#define F_END      108400000UL  // ���� RF Ƶ�� 108 MHz
#define F_STEP     100000UL     // �������� 100 kHz

// DDS ͨ������
#define DDS_CH_FM       CH0            // AD9959 ͨ�� 0 ���� FM LO
#define DDS_CH_AM       CH1            // AD9959 ͨ�� 1 ���� AM LO
#define DDS_AMP         128         // ������ֵ��10-bit��
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)detect_buf2, DETECT_BUF_LEN);
    // ���� TIM3 ������· ADC
    HAL_TIM_Base_Start(&htim3);
    // �ȴ���·���
    while (!(DetectConvEnd1 && DetectConvEnd2));
    // ֹͣ������ DMA
    // HAL_TIM_Base_Stop(&htim3);
    // HAL_ADC_Stop_DMA(&hadc1);
    // HAL_ADC_Stop_DMA(&hadc2);

    // ���� FM ƽ����ѹ
    uint32_t sum1 = 0;
    for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum1 += detect_buf1[i];
    *v_fm = ((float)sum1 / DETECT_BUF_LEN) * (3.3f / 4095.0f);
    // ���� AM ƽ����ѹ
    uint32_t sum2 = 0;
    for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum2 += detect_buf2[i];
    *v_am = ((float)sum2 / DETECT_BUF_LEN) * (3.3f / 4095.0f);
}

uint32_t SearchFMStations(void)
{
    printf("��ʼ����FM��̨...\r\n");
    
    // ʹ���Ż������㷨�ҵ����Ƶ��                  M  10KHz
    uint32_t best_freq = RDA5807M_Advanced_Search(8800, 10800, 25);    // 88.00MHz - 108.00MHz, ��С�ź�ǿ��25
    
    if (best_freq > 0)
    {// best_freq�ĵ�λ��0.01 MHz����10 kHz����
        uint8_t signal = RDA5807M_Read_Signal_Intensity();
        printf("�ҵ���ѵ�̨: %d.%dMHz, �ź�ǿ��:%d\r\n", best_freq/100, best_freq%100, signal);
        
        // ����IFƵ��: best_freq * 10000(Hz) - 455000(Hz) = best_freq * 10 - 455(kHz)
        uint32_t if_freq = best_freq * 10000 - 455000; // ��λHz��455kHz��Ƶ
        // ���AM��Ƶ��DDS
        DDS_Output_Channel(DDS_CH_AM, if_freq);
        printf("���AM��Ƶ: %d.%03d MHz\r\n", if_freq/1000000, (if_freq%1000000)/1000);
        
        return best_freq;
    }
    else
    {
        printf("δ�ҵ���Ч��̨\r\n");
        return 0;
    }
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
  /* USER CODE BEGIN 2 */
  delay_init(168);                            /* ��ʱ��ʼ�� */
  HAL_Delay(100); 
  AD9959_Init();
  // ADF4351Init();
  // RDA5807M_init();    // ��ʼ�� RDA5807M
  // initRingBuffer();		//��ʼ�����λ�����
  // HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);	//�򿪴��ڽ����ж�

  // char str[100];
  // uint32_t nowtime = HAL_GetTick();
  // uint32_t last_m = 0xFFFFFFFF;  // ��ʼ��Ϊһ�������ܵ�ֵ
  HAL_Delay(500);
  // RDA5807M_Set_Volume(10);
  uint8_t found = 0;        // ��ʾ�Ƿ��Ѿ���Ƶ
  uint32_t locked_rf = 0;   // ��¼������RFƵ��
  float v_fm, v_am;
  const int32_t correction = 978;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //      Compute_Dual_Voltage(&v_fm, &v_am);
    //  if(v_fm > 1.7f && v_am > 0.9f)
    //  {
    //     found = 0; // �����·��ѹ��������ֵ����ʾδ����ɹ�
    //     printf("Signal lost, re-scanning...\r\n");
    //  }
       if (!found)
       {
         for (uint32_t rf = F_START; rf <= F_END; rf += 200000UL) {
             // ������Ƶ LO
             uint32_t lo_fm = (rf > 10700000UL) ? (rf - 10700000UL) : 0;
             uint32_t lo_am = (rf > 455000UL)   ? (rf - 455000UL)   : 0;
             DDS_Output_Two(lo_fm, lo_am);
             // printf("FM: %d Hz, AM: %d Hz\r\n", lo_fm, lo_am);
             Compute_Dual_Voltage(&v_fm, &v_am);

             // ����һ·��ѹ < 1.7f ��ʾ����ɹ���ֹͣɨƵ
             if (v_fm <= 1.7f || v_am <= 0.10f) 
             {
                 found = 1;
                 locked_rf = rf;
                 // �жϽ��ģʽ
               if (v_fm <= 1.7f && v_am >= 0.10f) 
               {
                   printf("FM Found at RF=%lu Hz, FM=%lu Hz, Vfm=%.3f V\r\n", rf, (rf - 10700000UL), v_fm);
               } 
               else if (v_am <= 0.10f && v_fm >= 1.7f) 
               {
                   printf("AM Found at RF=%lu Hz, AM=%lu Hz, Vam=%.3f V\r\n", rf, (rf - 455000UL), v_am);
                   // --- ��ɨ�߼� ---
                   float min_vam = v_am;
                   uint32_t min_rf = rf;
                   for (uint32_t rf_fine = (rf > 0 ? rf : F_START); rf_fine <= rf + 2000000UL && rf_fine <= F_END; rf_fine += 100000UL) 
                   {
                       uint32_t lo_am_fine = (rf_fine > 455000UL) ? (rf_fine - 455000UL) : 0;
                       DDS_Output_Channel(DDS_CH_AM, lo_am_fine);
                       

                       float v_fm_fine, v_am_fine;
                       Compute_Dual_Voltage(&v_fm_fine, &v_am_fine);
                       if (v_am_fine < min_vam) {
                           min_vam = v_am_fine;
                           min_rf = rf_fine;
                       }
                   }
                   printf("[FineScan] Best AM at RF=%lu Hz, AM=%lu Hz, Vam=%.3f V\r\n", min_rf, (min_rf - 455000UL), min_vam);
                   DDS_Output_Channel(DDS_CH_AM, min_rf > 455000UL ? (min_rf - 455000UL) : 0);
                  //  HAL_Delay(100); // �ȴ�����ȶ�
               } else 
               {
                   printf("Both FM & AM Found at RF=%lu Hz, Vfm=%.3f V, Vam=%.3f V\r\n", rf, v_fm, v_am);
               }
                 // ���� LED ״̬
                 HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin,
                     (v_fm <= 1.7f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
                 HAL_GPIO_WritePin(LED_AM_GPIO_Port, LED_AM_Pin,
                     (v_am <= 0.10f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
                 // ���ֵ�ǰ LO ���
                 break;
             }
             else{printf("RF=%lu Hz, Vfm=%.3f V, AM = %lu Hz Vam=%.3f V\r\n", rf, v_fm, (rf - 455000UL), v_am);}
             
         }
         if (!found) {
             // ȫƵ��������һ·<1.7V����Ƶ�ز������ź�
             printf("Single-tone detected (no FM/AM)\r\n");
             HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_RESET);
             HAL_GPIO_WritePin(LED_AM_GPIO_Port, LED_AM_Pin, GPIO_PIN_RESET);
         }
       }
      	// si5351_Init(correction);
        // si5351_SetupCLK0(88000000, SI5351_DRIVE_STRENGTH_4MA);
        // si5351_EnableOutputs((1<<0));     // ʹ��CLK0������CLK1��CLK2
      // δ��Ƶ״̬����Ҫ����Ƶ������
      // Compute_Dual_Voltage(&v_fm, &v_am);
      // printf("Vfm=%.4f V, Vam=%.4f V\r\n", v_fm, v_am);
      // HAL_Delay(1000);
    // if (!found) 
    // {
    //     // ����SearchFMStations�������FMƵ��
    //     uint32_t best_freq = SearchFMStations();
        
    //     if (best_freq > 0) 
    //     {
    //         // �ҵ���ЧƵ�ʣ����Ϊ����Ƶ
    //         found = 1;
    //         locked_rf = best_freq;
            
    //         // ��ӡ��Ƶ��Ϣ
    //         printf("��Ƶ�ɹ�: %d.%d MHz\r\n", best_freq/100, best_freq%100);
            
    //         // ����״̬LEDָʾ�ƣ��͵�ƽ������
    //         HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_RESET);
    //     }
    //     else 
    //     {
    //         // δ�ҵ���ЧƵ�ʣ��ȴ�һ��ʱ�������
    //         // Ϩ��LED���ߵ�ƽϨ��
    //         HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_SET);
    //         HAL_Delay(100);
    //     }
    // }
    // // ��Ƶ״̬����Ҫ����ź�����
    // else 
    // {
    //     // ÿ100ms���һ���ź�����
    //     static uint32_t last_check_time = 0;
    //     uint32_t current_time = HAL_GetTick();
        
    //     if ((current_time - last_check_time) > 100) 
    //     {
    //         last_check_time = current_time;
            
    //         // ��ȡAM/FM�����ѹ
    //         Compute_Dual_Voltage(&v_fm, &v_am);
            
    //         // ���RDA5807�Ƿ���ʶ��Ϊ��̨
    //         uint8_t is_station = RDA5807M_Radio_Instructions();
    //         uint8_t signal_strength = RDA5807M_Read_Signal_Intensity();
            
    //         // �ж��ź��Ƿ�ʧ��
    //         // 1. AM�����⣺v_amС��0.6V��ʾAM����ɹ�
    //         // 2. FM��⣺RDA5807��ʶ��Ϊ��̨���ź�ǿ�ȹ���
    //         uint8_t am_demod_ok = (v_am < 0.6f); // AM����ɹ��ж���ֵ
    //         uint8_t fm_signal_lost = (!is_station || signal_strength < 20);
            
    //         if (v_am > 1.6f && fm_signal_lost) // AM���ʧ�ܻ�FM�źŶ�ʧ
    //         {
    //             printf("�źŶ�ʧ AM��ѹ=%.2f(��<0.6V), FM�ź�ǿ��=%d(��>20), ��̨״̬=%d\r\n", 
    //                    v_am, signal_strength, is_station);
                
    //             // ��������ʼ��һ������
    //             found = 0;
    //             locked_rf = 0;
    //             // Ϩ��LED���ߵ�ƽϨ��
    //             HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_SET);
    //         }
    //         else 
    //         {
    //             // �ź���Ȼ��Ч����˸LEDָʾ��������
    //             HAL_GPIO_TogglePin(LED_FM_GPIO_Port, LED_FM_Pin);
                
    //             // ÿ5�����һ�ε�ǰ״̬��Ϣ
    //             static uint8_t report_counter = 0;
    //             if (++report_counter >= 10) 
    //             {
    //                 report_counter = 0;
    //                 printf("��Ƶ�ȶ�: Ƶ��=%d.%d MHz, �ź�ǿ��=%d, ��ѹFM=%.2f AM=%.2f\r\n", 
    //                        locked_rf/100, locked_rf%100, signal_strength, v_fm, v_am);
    //             }
    //         }
    //     }
    // }
    
      // for (float rf = 88; rf <= 108; rf += 0.1)
      // {
      //       ADF4351WriteFreq(rf);
      //       HAL_Delay(500);
      // }

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
