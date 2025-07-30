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
#define LEN 50

#define F_START    88000000UL   // 起始 RF 频率 88 MHz
#define F_END      108000000UL  // 结束 RF 频率 108 MHz
#define F_STEP     100000UL     // 搜索步进 100 kHz

// 牛顿迭代法

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 全局变量：保存 Newton 最终结果
static float   g_final_J = 0.0f;    // 收敛时的 RMS 原始码值
static uint8_t g_final_phi = 0;   // 收敛时的数字电位器码值

static uint32_t m = 1000; // 用于控制 AD9959 的频率

uint16_t adc_buff[LEN];//存放ADC采集的数据
/* 
AdcConvEnd用来检测ADC是否采集完毕
0：没有采集完毕
1：采集完毕，在stm32f1xx_it里的DMA完成中断进行修改
 */
__IO uint8_t AdcConvEnd = 0;

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

// —— 读取 ADC 平均电压值 —— 
static float Compute_J_raw(void)
{
    AdcConvEnd = 0;
    HAL_TIM_Base_Start(&htim3);                                 // 5 kHz 触发
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buff, LEN);
    while (!AdcConvEnd);
    HAL_TIM_Base_Stop(&htim3);

    uint32_t sum = 0;
    for (uint16_t i = 0; i < LEN; i++) 
    {
    
        sum += adc_buff[i];
    }
    // 返回平均原始码
    float avg_raw =  (float)sum / (float)LEN;
    return avg_raw * (3.3f / 4095.0f);
}

/**
 * @brief 利用 Newton_Minimize_Raw 已计算的值，打印电压和阻值。
 */
void Print_Debug_Info(void)
{
    char str[64];
    // 原始码值转换为电压
    float avg_voltage = g_final_J * 3.3f / 4095.0f;
    // 数字电位器实际阻值
    float res_ohm = (float)g_final_phi * 39.0625f;

    // 串口打印
    printf("ADC average voltage = %.3f V\n", avg_voltage);
    printf("Pot Resistance = %.2f Ω\n", res_ohm);

    // TFT 显示 t2=电压, t3=阻值
    sprintf(str, "t2.txt=\"%.3f\"", avg_voltage);
    tjc_send_string(str);
    sprintf(str, "t3.txt=\"%.2f\"", res_ohm);
    tjc_send_string(str);
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
  /* USER CODE BEGIN 2 */
  delay_init(168);                            /* 延时初始化 */
  HAL_Delay(100); 
  AD9959_Init();
  initRingBuffer();		//初始化环形缓冲区
  HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);	//打开串口接收中断

  // char str[100];
  uint32_t nowtime = HAL_GetTick();
  uint32_t last_m = 0xFFFFFFFF;  // 初始化为一个不可能的值
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        // if (HAL_GetTick() - nowtime >= 500)
        // {
        //     nowtime = HAL_GetTick(); 
            Newton_Minimize_Raw();
            Print_Debug_Info();
        // }
        if ((m != last_m) && (HAL_GetTick() - nowtime >= 1500))
        {
            // m 变化了，才执行更新
            last_m = m;              // 记录本次值
            nowtime = HAL_GetTick(); 

            // 以下这段只在 m 变化时跑一次：
            AD9959_Set_Fre(CH0, m);
            AD9959_Set_Amp(CH0, 1023);
            AD9959_Set_Phase(CH0, 0);

            AD9959_Set_Fre(CH1, m);
            AD9959_Set_Amp(CH1, 1023);
            AD9959_Set_Phase(CH1, 0);
            
            IO_Update();
            HAL_Delay(10);
        }
        // HAL_TIM_Base_Start(&htim3);                           //开启定时器3 定时器为100KHZ
        // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, LEN); //ADC由TIM3的溢出事件控制(采样率100KHZ)让ADC1去采集200个数，存放到adc_buff[LEN]数组里

        // while (!AdcConvEnd);                                  //等待转换完毕
        // uint32_t sum = 0;

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
