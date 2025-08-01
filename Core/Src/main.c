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
uint8_t found = 0;        // ��ʾ�Ƿ��Ѿ���Ƶ

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
    delay_us(100);
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
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);

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

    uint16_t start_freq = 8800;  // 88MHz
    uint16_t end_freq = 10800;   // 108MHz
    uint8_t threshold = 5;      // ��С�ź�ǿ����ֵ

    uint16_t best_freq = 0;
    uint8_t max_signal = 0;

    printf("��ɨ����: %d.%02d-%d.%02d MHz\r\n",
           start_freq/100, start_freq%100, end_freq/100, end_freq%100);

    // ֻ��������㷨��ֻ���ź�ǿ������Ҹ�����ֵ��Ƶ��
    for(uint16_t freq = start_freq; freq <= end_freq; freq += 20)
    {
        RDA5807M_Set_Freq(freq);
        HAL_Delay(20);

        uint8_t signal = RDA5807M_Read_Signal_Intensity();
        // if(freq % 50 == 0)
        printf("Ƶ��: %d.%02d MHz, ǿ��: %d\r\n",
                  freq/100, freq%100, signal);

        if(signal >= threshold && signal > max_signal)
        {
            max_signal = signal;
            best_freq = freq;
        }
    }

    if(best_freq == 0)
    {
        printf("δ�ҵ���Ч��̨\r\n");
        return 0;
    }

    // ��ɨ���Դ�ɨ����Ϊ���ģ���400kHz��Χ������100kHz
    uint16_t fine_start = (best_freq > 40) ? (best_freq - 40) : start_freq;
    uint16_t fine_end = (best_freq + 40 < end_freq) ? (best_freq + 40) : end_freq;
    uint8_t final_max_signal = 0;
    uint16_t final_best_freq = best_freq;
    printf("��ϸɨ��: %d.%02d-%d.%02d MHz\r\n", fine_start/100, fine_start%100, fine_end/100, fine_end%100);
    for(uint16_t freq = fine_start; freq <= fine_end; freq += 10)
    {
        RDA5807M_Set_Freq(freq);
        HAL_Delay(25);
        uint8_t signal = RDA5807M_Read_Signal_Intensity();
        if(signal > final_max_signal)
        {
            final_max_signal = signal;
            final_best_freq = freq;
        }
    }

    final_best_freq = final_best_freq + 10;

    RDA5807M_Set_Freq(final_best_freq);
    HAL_Delay(50);
    uint8_t final_signal = RDA5807M_Read_Signal_Intensity();
    printf("������ѵ�̨: %d.%02d MHz, ǿ��: %d\r\n",
          final_best_freq/100, final_best_freq%100, final_signal);
    uint32_t if_freq = final_best_freq * 10000 - 455000;
    DDS_Output_Channel(DDS_CH_AM, if_freq);
    printf("���AM��Ƶ: %d.%03d MHz\r\n", if_freq/1000000, (if_freq%1000000)/1000);
    return final_best_freq;
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
  /* USER CODE BEGIN 2 */
  delay_init(168);                            /* ��ʱ��ʼ�� */
  HAL_Delay(50); 
  // AD9959_Init();
  // ADF4351Init();
  RDA5807M_init();    // ��ʼ�� RDA5807M
  // initRingBuffer();		//��ʼ�����λ�����
  // HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);	//�򿪴��ڽ����ж�

  // char str[100];
  // uint32_t nowtime = HAL_GetTick();
  // uint32_t last_m = 0xFFFFFFFF;  // ��ʼ��Ϊһ�������ܵ�ֵ
  HAL_Delay(500);
  RDA5807M_Set_Volume(10);



  uint32_t locked_rf = 8000;   // ��¼������RFƵ��
  float v_fm, v_am;
  // const int32_t correction = 978;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {   
    if (!found) 
    {
        // ����SearchFMStations�������FMƵ��
        printf("��ʼѲ���ź�...\r\n");
        uint32_t search_start = HAL_GetTick();
        uint32_t best_freq = SearchFMStations();
        uint32_t search_time = HAL_GetTick() - search_start;
        
        if (best_freq > 0) 
        {
            // �ҵ���ЧƵ�ʣ����Ϊ����Ƶ
            found = 1;
            locked_rf = best_freq;
            
            // ��ӡ��Ƶ��ϸ��Ϣ
            printf("���ճɹ�: %d.%02d MHz, ����ʱ��: %dms\r\n", 
                  best_freq/100, best_freq%100, search_time);
            
            // ��ȡ��ǰ�ź�ǿ�Ⱥ͵�̨״̬
            uint8_t final_signal = RDA5807M_Read_Signal_Intensity();
            
            // ��ȡAM/FM�����ѹ
            Compute_Dual_Voltage(&v_fm, &v_am);
            printf("�ź�״̬: ǿ��=%d, FM��ѹ=%.3fV, AM��ѹ=%.3fV\r\n",
                  final_signal, v_fm, v_am);
            
            // ����״̬LEDָʾ�ƣ��͵�ƽ������
            HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_RESET);
        }
        else 
        {
            // δ�ҵ���ЧƵ�ʣ��ȴ�һ��ʱ�������
            printf("����ʧ��, ��ʱ=%dms, %dms���ٴ�����\r\n", 
                  search_time, 200);
                  
            // ��ӡ���ܵ�ԭ��
            printf("����ԭ��: ��������/�ź�������/����̫��\r\n");
            
            // Ϩ��LED���ߵ�ƽϨ��
            HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_SET);
            HAL_Delay(200);  // �ȴ�500ms������
        }
    }
    // ��Ƶ״̬����Ҫ����ź�����
    else 
    {
        // ÿ100ms���һ���ź�����
        static uint32_t last_check_time = 0;
        uint32_t current_time = HAL_GetTick();
        
        if ((current_time - last_check_time) > 100) 
        {
            last_check_time = current_time;
            
            // ��ȡAM/FM�����ѹ
            Compute_Dual_Voltage(&v_fm, &v_am);
            
            // ���RDA5807�Ƿ���ʶ��Ϊ��̨
            uint8_t signal_strength = RDA5807M_Read_Signal_Intensity();
            
            // �ж��ź��Ƿ�ʧ��
            // 2. FM��⣺RDA5807��ʶ��Ϊ��̨���ź�ǿ�ȹ���
            uint8_t fm_signal_lost = (signal_strength < 20);
            
            if (fm_signal_lost) // AM���ʧ�ܻ�FM�źŶ�ʧ
            {
                printf("�źŶ�ʧ: AM��ѹ=%.3fV(>1.6V), FMǿ��=%d(<20), ��̨״̬=%d\r\n", 
                       v_am, signal_strength);
                
                // ��������ʼ��һ������
                found = 0;
                locked_rf = 0;
                // Ϩ��LED���ߵ�ƽϨ��
                HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_SET);
                printf("������������...\r\n");
            }
            else 
            {
                // �ź���Ȼ��Ч����˸LEDָʾ��������
                HAL_GPIO_TogglePin(LED_FM_GPIO_Port, LED_FM_Pin);

                // ÿ5�����һ�ε�ǰ״̬��Ϣ(50 * 100ms = 5s)
                static uint8_t report_counter = 0;
                if (++report_counter >= 50) 
                {
                    report_counter = 0;
                    printf("��Ƶ����: Ƶ��=%d.%02d MHz, ǿ��=%d, AM=%.3fV, FM=%.3fV\r\n", 
                           locked_rf/100, locked_rf%100, signal_strength, v_am, v_fm);
                    
                    // ����ź�����
                    if (signal_strength > 40)
                        printf("�ź�״̬: �ǳ�ǿ\r\n");
                    else if (signal_strength > 30)
                        printf("�ź�״̬: ����\r\n");
                    else
                        printf("�ź�״̬: ��, ���ܲ��ȶ�\r\n");
                }
            }
        }
    }
  }
  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */
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
