/**
  ******************************************************************************
  * @file    bsp_stepper_init.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   步进电机初始化
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 H743 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_init.h"
#include "./delay/core_delay.h"   
#include "stm32h7xx.h"
#include <stdio.h>

void TIM_SetTIMxCompare(TIM_TypeDef *TIMx,uint32_t channel,uint32_t compare);
void TIM_SetPWM_period(TIM_TypeDef* TIMx,uint32_t TIM_period);

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void Stepper_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*开启Motor1相关的GPIO外设时钟*/
	MOTOR1_DIR_GPIO_CLK_ENABLE();
	MOTOR1_PUL_GPIO_CLK_ENABLE();
	MOTOR1_EN_GPIO_CLK_ENABLE();
	/*开启Motor2相关的GPIO外设时钟*/
	MOTOR2_DIR_GPIO_CLK_ENABLE();
	MOTOR2_PUL_GPIO_CLK_ENABLE();
	MOTOR2_EN_GPIO_CLK_ENABLE();
	/*开启Motor3相关的GPIO外设时钟*/
	MOTOR3_DIR_GPIO_CLK_ENABLE();
	MOTOR3_PUL_GPIO_CLK_ENABLE();
	MOTOR3_EN_GPIO_CLK_ENABLE();
	/*开启Motor4相关的GPIO外设时钟*/
	MOTOR4_DIR_GPIO_CLK_ENABLE();
	MOTOR4_PUL_GPIO_CLK_ENABLE();
	MOTOR4_EN_GPIO_CLK_ENABLE();
	

	/*设置引脚的输出类型为推挽输出*/
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP; 
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	/*设置引脚速率为高速 */   
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*Motor 方向引脚 初始化*/
	
	/*对电机1方向的GPIO引脚*/															   
	GPIO_InitStruct.Pin = MOTOR1_DIR_PIN;	
	HAL_GPIO_Init(MOTOR1_DIR_GPIO_PORT, &GPIO_InitStruct);	
	
	/*对电机2方向的GPIO引脚*/															   
	GPIO_InitStruct.Pin = MOTOR2_DIR_PIN;	
	HAL_GPIO_Init(MOTOR2_DIR_GPIO_PORT, &GPIO_InitStruct);
	
	/*对电机3方向的GPIO引脚*/															   
	GPIO_InitStruct.Pin = MOTOR3_DIR_PIN;	
	HAL_GPIO_Init(MOTOR3_DIR_GPIO_PORT, &GPIO_InitStruct);
	
	/*对电机4方向的GPIO引脚*/															   
	GPIO_InitStruct.Pin = MOTOR4_DIR_PIN;	
	HAL_GPIO_Init(MOTOR4_DIR_GPIO_PORT, &GPIO_InitStruct);
	
	
	/*对电机1使能的GPIO引脚*/	
	GPIO_InitStruct.Pin = MOTOR1_EN_PIN;	
	HAL_GPIO_Init(MOTOR1_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/*对电机2使能的GPIO引脚*/	
	GPIO_InitStruct.Pin = MOTOR2_EN_PIN;	
	HAL_GPIO_Init(MOTOR2_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/*对电机3使能的GPIO引脚*/	
	GPIO_InitStruct.Pin = MOTOR3_EN_PIN;	
	HAL_GPIO_Init(MOTOR3_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/*对电机4使能的GPIO引脚*/	
	GPIO_InitStruct.Pin = MOTOR4_EN_PIN;	
	HAL_GPIO_Init(MOTOR4_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/* 定时器通道1功能引脚IO初始化 */
	/*设置输出类型*/
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	/*设置引脚速率 */ 
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*设置复用*/
	GPIO_InitStruct.Alternate = MOTOR1_PUL_GPIO_AF;
	/*设置复用*/
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	
	/*电机1的脉冲通道GPIO引脚*/	
	GPIO_InitStruct.Pin = GENERAL1_TIM_CH1_PIN;
	/*电机1脉冲引脚初始化*/
	HAL_GPIO_Init(GENERAL1_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);	
	
	/*电机2的脉冲通道GPIO引脚*/	
	GPIO_InitStruct.Pin = GENERAL2_TIM_CH2_PIN;
	/*电机2脉冲引脚初始化*/
	HAL_GPIO_Init(GENERAL2_TIM_CH2_GPIO_PORT, &GPIO_InitStruct);
	
	/*电机3的脉冲通道GPIO引脚*/	
	GPIO_InitStruct.Pin = GENERAL3_TIM_CH3_PIN;
	/*电机3脉冲引脚初始化*/
	HAL_GPIO_Init(GENERAL3_TIM_CH3_GPIO_PORT, &GPIO_InitStruct);
	
	/*电机4的脉冲通道GPIO引脚*/	
	GPIO_InitStruct.Pin = GENERAL4_TIM_CH4_PIN;
	/*电机4脉冲引脚初始化*/
	HAL_GPIO_Init(GENERAL4_TIM_CH4_GPIO_PORT, &GPIO_InitStruct);		
}


/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */

/*对电机1的定时器进行初始化*/
TIM_OC_InitTypeDef  TIM_OCInitStructure1;  
TIM_HandleTypeDef  TIM_TimeBaseStructure1;	
//int tim_prescaler = 112;
static void TIM_PWMOUTPUT1_Config(void)
{
	
	int tim_per=1000;//定时器周期

	/*使能定时器*/
	MOTOR1_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure1.Instance = MOTOR1_PUL_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/                                                                                                        		
	//当定时器从0计数到10000，即为10000次，为一个定时周期
	TIM_TimeBaseStructure1.Init.Period = tim_per;
    //定时器时钟源TIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure1.Init.Prescaler = 240 -1;	

	/*计数方式*/
	TIM_TimeBaseStructure1.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*采样时钟分频*/
	TIM_TimeBaseStructure1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*初始化定时器*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure1);                                                                                                                                                                                           

	/*PWM模式配置*/
	TIM_OCInitStructure1.OCMode = TIM_OCMODE_PWM1;//配置为PWM模式1 
	TIM_OCInitStructure1.Pulse = tim_per/2;//默认占空比为50%
	TIM_OCInitStructure1.OCFastMode = TIM_OCFAST_DISABLE;
	/*当定时器计数值小于CCR1_Val时为高电平*/
	TIM_OCInitStructure1.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*配置PWM通道*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure1, &TIM_OCInitStructure1,  MOTOR1_PUL_CHANNEL_x);
	/*开始输出PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure1,MOTOR1_PUL_CHANNEL_x);	
}
/************************************************************/

/************************************************************/
/*对电机2的定时器进行初始化*/
TIM_OC_InitTypeDef  TIM_OCInitStructure2;  
TIM_HandleTypeDef  TIM_TimeBaseStructure2;	
static void TIM_PWMOUTPUT2_Config(void)
{
	
	int tim_per=1000;//定时器周期

	/*使能定时器*/
	MOTOR2_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure2.Instance = MOTOR2_PUL_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/                                                                                                        		
	//当定时器从0计数到10000，即为10000次，为一个定时周期
	TIM_TimeBaseStructure2.Init.Period = tim_per;
    //定时器时钟源TIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure2.Init.Prescaler = 240 - 1;	

	/*计数方式*/
	TIM_TimeBaseStructure2.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*采样时钟分频*/
	TIM_TimeBaseStructure2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*初始化定时器*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure2);                                                                                                                                                                                           

	/*PWM模式配置*/
	TIM_OCInitStructure2.OCMode = TIM_OCMODE_PWM1;//配置为PWM模式1 
	TIM_OCInitStructure2.Pulse = tim_per/2;//默认占空比为50%
	TIM_OCInitStructure2.OCFastMode = TIM_OCFAST_DISABLE;
	/*当定时器计数值小于CCR1_Val时为高电平*/
	TIM_OCInitStructure2.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*配置PWM通道*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure2, &TIM_OCInitStructure2,  MOTOR2_PUL_CHANNEL_x);
	/*开始输出PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure2,MOTOR2_PUL_CHANNEL_x);	
}
/************************************************************/

/************************************************************/
/*对电机3的定时器进行初始化*/
TIM_OC_InitTypeDef  TIM_OCInitStructure3;  
TIM_HandleTypeDef  TIM_TimeBaseStructure3;	
static void TIM_PWMOUTPUT3_Config(void)
{
	
	int tim_per=1000;//定时器周期

	/*使能定时器*/
	MOTOR3_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure3.Instance = MOTOR3_PUL_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/                                                                                                        		
	//当定时器从0计数到10000，即为10000次，为一个定时周期
	TIM_TimeBaseStructure3.Init.Period = tim_per;
    //定时器时钟源TIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure3.Init.Prescaler = 240 - 1;	

	/*计数方式*/
	TIM_TimeBaseStructure3.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*采样时钟分频*/
	TIM_TimeBaseStructure3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*初始化定时器*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure3);                                                                                                                                                                                           

	/*PWM模式配置*/
	TIM_OCInitStructure3.OCMode = TIM_OCMODE_PWM1;//配置为PWM模式1 
	TIM_OCInitStructure3.Pulse = tim_per/2;//默认占空比为50%
	TIM_OCInitStructure3.OCFastMode = TIM_OCFAST_DISABLE;
	/*当定时器计数值小于CCR1_Val时为高电平*/
	TIM_OCInitStructure3.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*配置PWM通道*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure3, &TIM_OCInitStructure3,  MOTOR3_PUL_CHANNEL_x);
	/*开始输出PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure3,MOTOR3_PUL_CHANNEL_x);	
}
/************************************************************/

/************************************************************/
/*对电机4的定时器进行初始化*/
TIM_OC_InitTypeDef  TIM_OCInitStructure4;  
TIM_HandleTypeDef  TIM_TimeBaseStructure4;	
static void TIM_PWMOUTPUT4_Config(void)
{
	
	int tim_per=1000;//定时器周期

	/*使能定时器*/
	MOTOR4_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure4.Instance = MOTOR4_PUL_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/                                                                                                        		
	//当定时器从0计数到10000，即为10000次，为一个定时周期
	TIM_TimeBaseStructure4.Init.Period = tim_per;
    //定时器时钟源TIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure4.Init.Prescaler = 240 - 1;	

	/*计数方式*/
	TIM_TimeBaseStructure4.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*采样时钟分频*/
	TIM_TimeBaseStructure4.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*初始化定时器*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure4);                                                                                                                                                                                           

	/*PWM模式配置*/
	TIM_OCInitStructure4.OCMode = TIM_OCMODE_PWM1;//配置为PWM模式1 
	TIM_OCInitStructure4.Pulse = tim_per/2;//默认占空比为50%
	TIM_OCInitStructure4.OCFastMode = TIM_OCFAST_DISABLE;
	/*当定时器计数值小于CCR1_Val时为高电平*/
	TIM_OCInitStructure4.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*配置PWM通道*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure4, &TIM_OCInitStructure4,  MOTOR4_PUL_CHANNEL_x);
	/*开始输出PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure4,MOTOR4_PUL_CHANNEL_x);	
}
/************************************************************/


/**
  * @brief  设置TIM通道的占空比
	* @param  channel		通道	（1,2,3,4）
	* @param  compare		占空比
	*	@note 	无
  * @retval 无
  */
void TIM2_SetPWM_pulse(int channel,int compare)
{
	switch(channel)
	{
		case 1:   __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure1,TIM_CHANNEL_1,compare);break;
		case 2:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure1,TIM_CHANNEL_2,compare);break;
		case 3:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure1,TIM_CHANNEL_3,compare);break;
		case 4:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure1,TIM_CHANNEL_4,compare);break;
	}
}

/**
  * @brief  增大步进电机的转速,调节PWM的频率
	* @param  NULL
	*	@note 	通过更改定时器的预分频计数器对转速进行更改；计算公式 rpm = (fsys_clk * 60) / (psc * Cperiod * C32 * 200)
  * @retval 无
  */
void STEPMOTOR_SPEED_UP()
{
	int current_prescaler = TIM8->PSC;
	int target_prescaler;
	if(current_prescaler > 11)
	{
		target_prescaler = current_prescaler - 1;
		TIM8->PSC = target_prescaler-1;
		TIM8->EGR = TIM_EGR_UG;
	}
	else
	{
		printf("the speed have reach the max");
	}
}

/**
  * @brief  减少步进电机的转速,调节PWM的频率
	* @param  NULL
	*	@note 	通过更改定时器的预分频计数器对转速进行更改；计算公式 rpm = (fsys_clk * 60) / (psc * Cperiod * C32 * 200)
  * @retval 无
  */
void STEPMOTOR_SPEED_DOWN()
{
	int current_prescaler = TIM8->PSC;
	int target_prescaler = current_prescaler + 10;
	TIM8->PSC = target_prescaler-1;
  TIM8->EGR = TIM_EGR_UG;
}

/**
  * @brief  引脚初始化
  * @retval 无
  */
void stepper_Init()
{
	Stepper_GPIO_Config();

	TIM_PWMOUTPUT1_Config();
	TIM_PWMOUTPUT2_Config();
	TIM_PWMOUTPUT3_Config();
	TIM_PWMOUTPUT4_Config();
						
}



















