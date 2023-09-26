/**
  ******************************************************************
  * @file    main.c
  * @author  Leo
  * @version V1.0
  * @date    2023-09-18
  * @brief   步进电机-GPIO中断模拟脉冲控制
  ******************************************************************
  * @attention
  ******************************************************************
  */  
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h"
#include "./delay/core_delay.h" 
#include "./usart_test/bsp_usart_test.h"
#include ".\key\bsp_key.h" 
#include "./bsp_bldc/bsp_bldc_control.h"
#include <stdio.h>
#include "./step_v2/step2.h"


//int dir_val=0;
//int motor1_en_flag = 0, motor2_en_flag = 0, motor3_en_flag = 0, motor4_en_flag = 0;
//int i=0,j=0,k=0,l=0;
int pulse_num=0;
	
void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}	
	

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  __IO uint16_t BLDC_ChannelPulse = 5500/10;
	uint8_t BLDC_en_flag = 0;
	
//	int dir_val=0;
	int motor1_en_flag = 0, motor2_en_flag = 0, motor3_en_flag = 0, motor4_en_flag = 0;
//int i=0,j=0,k=0,l=0;
	
//  uint8_t size = sizeof(step_motor)/sizeof(Stepper_TypeDef);
	
  /* HAL 初始化 */
  HAL_Init();
	
	/* 初始化系统时钟为480MHz */
	SystemClock_Config();
	
	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	USART_Config();
	
	/* LED 灯初始化 */
	LED_GPIO_Config();
	
	/*按键中断初始化*/
	Key_GPIO_Config();	
	
	/*步进电机初始化*/
	stepper_Init_test();	
	
  /* 电机初始化 */
  bldcm_init();
	
	for(int i=0; i<4;i++)
	{
		stepper_Stop(step_motor[i].pul_channel);
	}

  while(1)
  {    
		receiving_process();
//		/* 扫描KEY1 */
//    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
//    {
//			
//    }
//		
//		if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
//    {
//      /*改变使能*/
//		  (motor1_en_flag > 0) ? stepper_Start(step_motor[0].pul_channel) : stepper_Stop(step_motor[0].pul_channel);
//      motor1_en_flag = !motor1_en_flag;
//    }
//		
//		if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
//    {
//      /*改变使能*/
//			(motor2_en_flag > 0) ? stepper_Start(step_motor[1].pul_channel) : stepper_Stop(step_motor[1].pul_channel);
//      motor2_en_flag = !motor2_en_flag;
//    }		
//		
//		if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON  )
//    {
//      /*改变使能*/
//			(motor3_en_flag > 0) ? stepper_Start(step_motor[2].pul_channel) : stepper_Stop(step_motor[2].pul_channel);
//      motor3_en_flag = !motor3_en_flag;
//    }	

//		if( Key_Scan(KEY5_GPIO_PORT,KEY5_PIN) == KEY_ON  )
//    {
//      /*改变使能*/
//			(motor4_en_flag > 0) ? stepper_Start(step_motor[3].pul_channel) : stepper_Stop(step_motor[3].pul_channel);
//      motor4_en_flag = !motor4_en_flag;
//    }					
  }
}  

/**
  * @brief  System Clock 配置
  *         system Clock 配置如下: 
	*            System Clock source  = PLL (HSE)
	*            SYSCLK(Hz)           = 480000000 (CPU Clock)
	*            HCLK(Hz)             = 240000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  120MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  120MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  120MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  120MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 192
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** 启用电源配置更新
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** 配置主内稳压器输出电压
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** 初始化CPU、AHB和APB总线时钟
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
		while(1);
  }
  /** 初始化CPU、AHB和APB总线时钟
  */
	/* 选择PLL作为系统时钟源并配置总线时钟分频器 */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  | \
																 RCC_CLOCKTYPE_HCLK    | \
																 RCC_CLOCKTYPE_D1PCLK1 | \
																 RCC_CLOCKTYPE_PCLK1   | \
                                 RCC_CLOCKTYPE_PCLK2   | \
																 RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4)!= HAL_OK)
  {
    while(1) { ; }
  }
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
		while(1);
  }
}
/****************************END OF FILE***************************/
