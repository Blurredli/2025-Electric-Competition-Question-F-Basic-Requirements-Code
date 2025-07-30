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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "AD9959.h"
#include "tjc_usart_hmi.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"
#include <math.h>      // 提供 fabsf, roundf, fmaxf 等数学函数
#include <stdlib.h>    // 提供 abs (整型)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FRAME_LENGTH 8
#define DETECT_BUF_LEN 25

#define F_START    88000000UL   // 起始 RF 频率 88 MHz
#define F_END      108000000UL  // 结束 RF 频率 108 MHz
#define F_STEP     100000UL     // 搜索步进 100 kHz

// DDS 通道定义
#define DDS_CH_FM       CH0            // AD9959 通道 0 用于 FM LO
#define DDS_CH_AM       CH1            // AD9959 通道 1 用于 AM LO
#define DDS_AMP         1023         // 最大幅度值（10-bit）
#define DDS_PHASE       0            // 相位校正值 (0)

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
DetectConvEnd1用来检测ADC是否采集完毕
0：没有采集完毕
1：采集完毕，在stm32f1xx_it里的DMA完成中断进行修改
 */
volatile uint8_t  DetectConvEnd1 = 0;
volatile uint8_t  DetectConvEnd2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == KEY2_Pin)
    {
        // 简单的消抖处理
        static uint32_t last_time = 0;
        uint32_t current_time = HAL_GetTick();
        
        if((current_time - last_time) > 50) // 50ms消抖
        {
            // 确认按键状态
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
            {
                // 按键按下处理
//                printf("Button pressed!\n");
                // m++;
                
            }
        }
        last_time = current_time;
    }
        if(GPIO_Pin == KEY0_Pin)
    {
        // 简单的消抖处理
        static uint32_t last_time = 0;
        uint32_t current_time = HAL_GetTick();
        
        if((current_time - last_time) > 50) // 50ms消抖
        {
            // 确认按键状态
            if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
            {
                // 按键按下处理
//                printf("Button pressed!\n");
                // m--;
                
            }
        }
        last_time = current_time;
    }
}

/**
 * @brief 通过指定 DDS 通道输出正弦波并等待稳定
 */
static void DDS_Output_Channel(uint8_t ch, uint32_t freq)
{
    AD9959_Set_Fre(ch,   freq);
    AD9959_Set_Amp(ch,   DDS_AMP);
    AD9959_Set_Phase(ch, DDS_PHASE);
    IO_Update();
    HAL_Delay(5);
}

/**
 * @brief 同时配置两通道 DDS 输出
 */
static void DDS_Output_Two(uint32_t lo_fm, uint32_t lo_am)
{
    DDS_Output_Channel(DDS_CH_FM, lo_fm);
    DDS_Output_Channel(DDS_CH_AM, lo_am);
}

/**
 * @brief 同时使用 TIM3 触发 ADC1/ADC2 + DMA 采集两路检测电压
 * @param[out] v_fm 输出 FM 通道平均电压 (V)
 * @param[out] v_am 输出 AM 通道平均电压 (V)
 */
static void Compute_Dual_Voltage(float *v_fm, float *v_am)
{
    DetectConvEnd1 = DetectConvEnd2 = 0;
    // 启动 DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)detect_buf1, DETECT_BUF_LEN);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)detect_buf2, DETECT_BUF_LEN);
    // 启动 TIM3 触发两路 ADC
    HAL_TIM_Base_Start(&htim3);
    // 等待两路完成
    while (!(DetectConvEnd1 && DetectConvEnd2)) {}
    // 停止触发和 DMA
    // HAL_TIM_Base_Stop(&htim3);
    // HAL_ADC_Stop_DMA(&hadc1);
    // HAL_ADC_Stop_DMA(&hadc2);

    // 计算 FM 平均电压
    uint32_t sum1 = 0;
    for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum1 += detect_buf1[i];
    *v_fm = ((float)sum1 / DETECT_BUF_LEN) * (3.3f / 4095.0f);
    // 计算 AM 平均电压
    uint32_t sum2 = 0;
    for (uint16_t i = 0; i < DETECT_BUF_LEN; i++) sum2 += detect_buf2[i];
    *v_am = ((float)sum2 / DETECT_BUF_LEN) * (3.3f / 4095.0f);
}

// /**
//  * @brief 自动搜索并检测 FM/AM 解调输出，使用双 LO+双 ADC
//  * @return 找到 单路 or 双路 的标志
//  */
// uint8_t AutoScanAndDetect_Dual(void)
// {
//     const uint32_t IF_FM = 10700000UL;
//     const uint32_t IF_AM =   455000UL;
//     uint8_t found_fm = 0, found_am = 0;
//     uint32_t found_rf_fm = 0, found_rf_am = 0;

//     for (uint32_t rf = F_START; rf <= F_END; rf += F_STEP) {
//         uint32_t lo_fm = (rf > IF_FM) ? (rf - IF_FM) : 0;
//         uint32_t lo_am = (rf > IF_AM) ? (rf - IF_AM) : 0;
//         // 同时输出两路 LO
//         DDS_Output_Two(lo_fm, lo_am);
//         // 同步采集两路电压
//         float v_fm, v_am;
//         Compute_Dual_Voltage(&v_fm, &v_am);

//         if (!found_fm && v_fm < 1.6f) {
//             found_fm = 1;
//             found_rf_fm = rf;
//             printf("FM Locked RF=%lu Hz, LO=%lu Hz, V=%.3f V\r\n", rf, lo_fm, v_fm);
//         }
//         if (!found_am && v_am < 1.6f) {
//             found_am = 1;
//             found_rf_am = rf;
//             printf("AM Locked RF=%lu Hz, LO=%lu Hz, V=%.3f V\r\n", rf, lo_am, v_am);
//         }
//         if (found_fm && found_am) break;
//     }
//     if (!found_fm) printf("FM not detected\r\n");
//     if (!found_am) printf("AM not detected\r\n");
//     return (found_fm << 1) | found_am;
// }

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
  /* USER CODE BEGIN 2 */
  delay_init(168);                            /* 延时初始化 */
  HAL_Delay(100); 
  AD9959_Init();
  initRingBuffer();		//初始化环形缓冲区
  HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);	//打开串口接收中断

  // char str[100];
  // uint32_t nowtime = HAL_GetTick();
  // uint32_t last_m = 0xFFFFFFFF;  // 初始化为一个不可能的值
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
                uint8_t found = 0;
        for (uint32_t rf = F_START; rf <= F_END; rf += F_STEP) {
            // 计算中频 LO
            uint32_t lo_fm = (rf > 10700000UL) ? (rf - 10700000UL) : 0;
            uint32_t lo_am = (rf > 455000UL)   ? (rf - 455000UL)   : 0;
            DDS_Output_Two(lo_fm, lo_am);
            printf("FM: %d Hz, AM: %d Hz\r\n", lo_fm, lo_am);
            float v_fm, v_am;
            Compute_Dual_Voltage(&v_fm, &v_am);

            // 任意一路电压 >1.6V 表示解调成功，停止扫频
            if (v_fm > 1.6f || v_am > 1.6f) {
                found = 1;
                // 判断解调模式
                if (v_fm > 1.6f && v_am <= 1.6f) {
                    printf("FM Found at RF=%lu Hz, Vfm=%.3f V\r\n", rf, v_fm);
                } else if (v_am > 1.6f && v_fm <= 1.6f) {
                    printf("AM Found at RF=%lu Hz, Vam=%.3f V\r\n", rf, v_am);
                } else {
                    printf("Both FM & AM Found at RF=%lu Hz, Vfm=%.3f V, Vam=%.3f V\r\n", rf, v_fm, v_am);
                }
                // 更新 LED 状态
                HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin,
                    (v_fm > 1.6f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_AM_GPIO_Port, LED_AM_Pin,
                    (v_am > 1.6f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
                // 保持当前 LO 输出
                break;
            }
            else{printf("Both FM & AM Found at RF=%lu Hz, Vfm=%.3f V, Vam=%.3f V\r\n", rf, v_fm, v_am);}
        }
        if (!found) {
            // 全频段无任意一路>1.6V，单频载波或无信号
            printf("Single-tone detected (no FM/AM)\r\n");
            HAL_GPIO_WritePin(LED_FM_GPIO_Port, LED_FM_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_AM_GPIO_Port, LED_AM_Pin, GPIO_PIN_RESET);
        }
        HAL_Delay(50);

        // if (HAL_GetTick() - nowtime >= 500)
        // {
        //     nowtime = HAL_GetTick(); 
        // }
        // for (uint16_t i = 0; i < LEN; i++)
        // {
        //     float voltage = adc_buff[i] * 3.3f / 4095.0f;
        //     printf("ADC actual value[%3d] = %.3f V\n", i, voltage);
        //     sum += adc_buff[i];
        // }
        // // 循环结束后计算平均
        // float avg_voltage = (float)sum * 3.3f / (4095.0f * (float)LEN);
        // float Jp = Compute_J_raw();
        // float avg_voltage = (float)Jp * 3.3f / 4095.0f;

        // printf("ADC average voltage = %.3f V\n", avg_voltage);

        // sprintf(str, "t2.txt=\"%.3f\"", avg_voltage);
        // tjc_send_string(str);
        
        // sprintf(str, "t3.txt=\"%.3f\"", (float)RES * 39.0625);
        // tjc_send_string(str);

        // RES++;
        //串口数据格式：
        //串口数据帧长度：8字节
        //帧头     低位在前(4字节)  帧尾
        //0x55       u4字节               0xffffff
        //当参数是01时
        //帧头     参数1    参数2   参数3       帧尾
        //0x55     01     led编号  led状态    0xffffff
        //例子1：上位机代码  printh 55 01 01 00 ff ff ff  含义：1号led关闭
        //例子2：上位机代码  printh 55 01 04 01 ff ff ff  含义：4号led打开
        //例子3：上位机代码  printh 55 01 00 01 ff ff ff  含义：0号led打开
        //例子4：上位机代码  printh 55 01 04 00 ff ff ff  含义：4号led关闭
        //当参数是02或03时
        //帧头     参数1    参数2   参数3       帧尾
        //0x55     02/03   滑动值    00    0xffffff
        //例子1：上位机代码  printh 55 02 64 00 ff ff ff  含义：h0.val=100
        //例子2：上位机代码  printh 55 02 00 00 ff ff ff  含义：h0.val=0
        //例子3：上位机代码  printh 55 03 64 00 ff ff ff  含义：h1.val=100
        //例子4：上位机代码  printh 55 03 00 00 ff ff ff  含义：h1.val=0
        // 当串口缓冲区大于等于一帧的长度时
        // while (usize >= FRAME_LENGTH)
        // {
        //     // 校验帧头帧尾是否匹配
        //     if (usize >= FRAME_LENGTH && u(0) == 0x55 && u(5) == 0xff && u(6) == 0xff && u(7) == 0xff)
        //     {
        //         m =  (uint32_t)u(1)
        //             | ((uint32_t)u(2) << 8)
        //             | ((uint32_t)u(3) << 16)
        //             | ((uint32_t)u(4) << 24);
        //         udelete(FRAME_LENGTH); // 删除解析过的数据
        //     } else
        //     {
        //         // 不匹配删除1字节
        //         udelete(1);
        //         break;
        //     }
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
