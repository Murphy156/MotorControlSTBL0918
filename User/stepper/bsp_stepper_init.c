/**
  ******************************************************************************
  * @file    bsp_stepper_init.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ���������ʼ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H743 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
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
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void Stepper_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*����Motor1��ص�GPIO����ʱ��*/
	MOTOR1_DIR_GPIO_CLK_ENABLE();
	MOTOR1_PUL_GPIO_CLK_ENABLE();
	MOTOR1_EN_GPIO_CLK_ENABLE();
	/*����Motor2��ص�GPIO����ʱ��*/
	MOTOR2_DIR_GPIO_CLK_ENABLE();
	MOTOR2_PUL_GPIO_CLK_ENABLE();
	MOTOR2_EN_GPIO_CLK_ENABLE();
	/*����Motor3��ص�GPIO����ʱ��*/
	MOTOR3_DIR_GPIO_CLK_ENABLE();
	MOTOR3_PUL_GPIO_CLK_ENABLE();
	MOTOR3_EN_GPIO_CLK_ENABLE();
	/*����Motor4��ص�GPIO����ʱ��*/
	MOTOR4_DIR_GPIO_CLK_ENABLE();
	MOTOR4_PUL_GPIO_CLK_ENABLE();
	MOTOR4_EN_GPIO_CLK_ENABLE();
	

	/*�������ŵ��������Ϊ�������*/
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP; 
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	/*������������Ϊ���� */   
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*Motor �������� ��ʼ��*/
	
	/*�Ե��1�����GPIO����*/															   
	GPIO_InitStruct.Pin = MOTOR1_DIR_PIN;	
	HAL_GPIO_Init(MOTOR1_DIR_GPIO_PORT, &GPIO_InitStruct);	
	
	/*�Ե��2�����GPIO����*/															   
	GPIO_InitStruct.Pin = MOTOR2_DIR_PIN;	
	HAL_GPIO_Init(MOTOR2_DIR_GPIO_PORT, &GPIO_InitStruct);
	
	/*�Ե��3�����GPIO����*/															   
	GPIO_InitStruct.Pin = MOTOR3_DIR_PIN;	
	HAL_GPIO_Init(MOTOR3_DIR_GPIO_PORT, &GPIO_InitStruct);
	
	/*�Ե��4�����GPIO����*/															   
	GPIO_InitStruct.Pin = MOTOR4_DIR_PIN;	
	HAL_GPIO_Init(MOTOR4_DIR_GPIO_PORT, &GPIO_InitStruct);
	
	
	/*�Ե��1ʹ�ܵ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR1_EN_PIN;	
	HAL_GPIO_Init(MOTOR1_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/*�Ե��2ʹ�ܵ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR2_EN_PIN;	
	HAL_GPIO_Init(MOTOR2_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/*�Ե��3ʹ�ܵ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR3_EN_PIN;	
	HAL_GPIO_Init(MOTOR3_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/*�Ե��4ʹ�ܵ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR4_EN_PIN;	
	HAL_GPIO_Init(MOTOR4_EN_GPIO_PORT, &GPIO_InitStruct);	
	
	/* ��ʱ��ͨ��1��������IO��ʼ�� */
	/*�����������*/
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	/*������������ */ 
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*���ø���*/
	GPIO_InitStruct.Alternate = MOTOR1_PUL_GPIO_AF;
	/*���ø���*/
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	
	/*���1������ͨ��GPIO����*/	
	GPIO_InitStruct.Pin = GENERAL1_TIM_CH1_PIN;
	/*���1�������ų�ʼ��*/
	HAL_GPIO_Init(GENERAL1_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);	
	
	/*���2������ͨ��GPIO����*/	
	GPIO_InitStruct.Pin = GENERAL2_TIM_CH2_PIN;
	/*���2�������ų�ʼ��*/
	HAL_GPIO_Init(GENERAL2_TIM_CH2_GPIO_PORT, &GPIO_InitStruct);
	
	/*���3������ͨ��GPIO����*/	
	GPIO_InitStruct.Pin = GENERAL3_TIM_CH3_PIN;
	/*���3�������ų�ʼ��*/
	HAL_GPIO_Init(GENERAL3_TIM_CH3_GPIO_PORT, &GPIO_InitStruct);
	
	/*���4������ͨ��GPIO����*/	
	GPIO_InitStruct.Pin = GENERAL4_TIM_CH4_PIN;
	/*���4�������ų�ʼ��*/
	HAL_GPIO_Init(GENERAL4_TIM_CH4_GPIO_PORT, &GPIO_InitStruct);		
}


/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У��������У�������ʱ����
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������(������ʱ��)
 * TIM_RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */

/*�Ե��1�Ķ�ʱ�����г�ʼ��*/
TIM_OC_InitTypeDef  TIM_OCInitStructure1;  
TIM_HandleTypeDef  TIM_TimeBaseStructure1;	
//int tim_prescaler = 112;
static void TIM_PWMOUTPUT1_Config(void)
{
	
	int tim_per=1000;//��ʱ������

	/*ʹ�ܶ�ʱ��*/
	MOTOR1_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure1.Instance = MOTOR1_PUL_TIM;
	/* �ۼ� TIM_Period�������һ�����»����ж�*/                                                                                                        		
	//����ʱ����0������10000����Ϊ10000�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure1.Init.Period = tim_per;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure1.Init.Prescaler = 240 -1;	

	/*������ʽ*/
	TIM_TimeBaseStructure1.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*����ʱ�ӷ�Ƶ*/
	TIM_TimeBaseStructure1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*��ʼ����ʱ��*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure1);                                                                                                                                                                                           

	/*PWMģʽ����*/
	TIM_OCInitStructure1.OCMode = TIM_OCMODE_PWM1;//����ΪPWMģʽ1 
	TIM_OCInitStructure1.Pulse = tim_per/2;//Ĭ��ռ�ձ�Ϊ50%
	TIM_OCInitStructure1.OCFastMode = TIM_OCFAST_DISABLE;
	/*����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ*/
	TIM_OCInitStructure1.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*����PWMͨ��*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure1, &TIM_OCInitStructure1,  MOTOR1_PUL_CHANNEL_x);
	/*��ʼ���PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure1,MOTOR1_PUL_CHANNEL_x);	
}
/************************************************************/

/************************************************************/
/*�Ե��2�Ķ�ʱ�����г�ʼ��*/
TIM_OC_InitTypeDef  TIM_OCInitStructure2;  
TIM_HandleTypeDef  TIM_TimeBaseStructure2;	
static void TIM_PWMOUTPUT2_Config(void)
{
	
	int tim_per=1000;//��ʱ������

	/*ʹ�ܶ�ʱ��*/
	MOTOR2_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure2.Instance = MOTOR2_PUL_TIM;
	/* �ۼ� TIM_Period�������һ�����»����ж�*/                                                                                                        		
	//����ʱ����0������10000����Ϊ10000�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure2.Init.Period = tim_per;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure2.Init.Prescaler = 240 - 1;	

	/*������ʽ*/
	TIM_TimeBaseStructure2.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*����ʱ�ӷ�Ƶ*/
	TIM_TimeBaseStructure2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*��ʼ����ʱ��*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure2);                                                                                                                                                                                           

	/*PWMģʽ����*/
	TIM_OCInitStructure2.OCMode = TIM_OCMODE_PWM1;//����ΪPWMģʽ1 
	TIM_OCInitStructure2.Pulse = tim_per/2;//Ĭ��ռ�ձ�Ϊ50%
	TIM_OCInitStructure2.OCFastMode = TIM_OCFAST_DISABLE;
	/*����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ*/
	TIM_OCInitStructure2.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*����PWMͨ��*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure2, &TIM_OCInitStructure2,  MOTOR2_PUL_CHANNEL_x);
	/*��ʼ���PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure2,MOTOR2_PUL_CHANNEL_x);	
}
/************************************************************/

/************************************************************/
/*�Ե��3�Ķ�ʱ�����г�ʼ��*/
TIM_OC_InitTypeDef  TIM_OCInitStructure3;  
TIM_HandleTypeDef  TIM_TimeBaseStructure3;	
static void TIM_PWMOUTPUT3_Config(void)
{
	
	int tim_per=1000;//��ʱ������

	/*ʹ�ܶ�ʱ��*/
	MOTOR3_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure3.Instance = MOTOR3_PUL_TIM;
	/* �ۼ� TIM_Period�������һ�����»����ж�*/                                                                                                        		
	//����ʱ����0������10000����Ϊ10000�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure3.Init.Period = tim_per;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure3.Init.Prescaler = 240 - 1;	

	/*������ʽ*/
	TIM_TimeBaseStructure3.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*����ʱ�ӷ�Ƶ*/
	TIM_TimeBaseStructure3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*��ʼ����ʱ��*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure3);                                                                                                                                                                                           

	/*PWMģʽ����*/
	TIM_OCInitStructure3.OCMode = TIM_OCMODE_PWM1;//����ΪPWMģʽ1 
	TIM_OCInitStructure3.Pulse = tim_per/2;//Ĭ��ռ�ձ�Ϊ50%
	TIM_OCInitStructure3.OCFastMode = TIM_OCFAST_DISABLE;
	/*����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ*/
	TIM_OCInitStructure3.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*����PWMͨ��*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure3, &TIM_OCInitStructure3,  MOTOR3_PUL_CHANNEL_x);
	/*��ʼ���PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure3,MOTOR3_PUL_CHANNEL_x);	
}
/************************************************************/

/************************************************************/
/*�Ե��4�Ķ�ʱ�����г�ʼ��*/
TIM_OC_InitTypeDef  TIM_OCInitStructure4;  
TIM_HandleTypeDef  TIM_TimeBaseStructure4;	
static void TIM_PWMOUTPUT4_Config(void)
{
	
	int tim_per=1000;//��ʱ������

	/*ʹ�ܶ�ʱ��*/
	MOTOR4_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure4.Instance = MOTOR4_PUL_TIM;
	/* �ۼ� TIM_Period�������һ�����»����ж�*/                                                                                                        		
	//����ʱ����0������10000����Ϊ10000�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure4.Init.Period = tim_per;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4                                                                
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure4.Init.Prescaler = 240 - 1;	

	/*������ʽ*/
	TIM_TimeBaseStructure4.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*����ʱ�ӷ�Ƶ*/
	TIM_TimeBaseStructure4.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*��ʼ����ʱ��*/
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure4);                                                                                                                                                                                           

	/*PWMģʽ����*/
	TIM_OCInitStructure4.OCMode = TIM_OCMODE_PWM1;//����ΪPWMģʽ1 
	TIM_OCInitStructure4.Pulse = tim_per/2;//Ĭ��ռ�ձ�Ϊ50%
	TIM_OCInitStructure4.OCFastMode = TIM_OCFAST_DISABLE;
	/*����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ*/
	TIM_OCInitStructure4.OCPolarity = TIM_OCPOLARITY_HIGH;	

	/*����PWMͨ��*/
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure4, &TIM_OCInitStructure4,  MOTOR4_PUL_CHANNEL_x);
	/*��ʼ���PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure4,MOTOR4_PUL_CHANNEL_x);	
}
/************************************************************/


/**
  * @brief  ����TIMͨ����ռ�ձ�
	* @param  channel		ͨ��	��1,2,3,4��
	* @param  compare		ռ�ձ�
	*	@note 	��
  * @retval ��
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
  * @brief  ���󲽽������ת��,����PWM��Ƶ��
	* @param  NULL
	*	@note 	ͨ�����Ķ�ʱ����Ԥ��Ƶ��������ת�ٽ��и��ģ����㹫ʽ rpm = (fsys_clk * 60) / (psc * Cperiod * C32 * 200)
  * @retval ��
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
  * @brief  ���ٲ��������ת��,����PWM��Ƶ��
	* @param  NULL
	*	@note 	ͨ�����Ķ�ʱ����Ԥ��Ƶ��������ת�ٽ��и��ģ����㹫ʽ rpm = (fsys_clk * 60) / (psc * Cperiod * C32 * 200)
  * @retval ��
  */
void STEPMOTOR_SPEED_DOWN()
{
	int current_prescaler = TIM8->PSC;
	int target_prescaler = current_prescaler + 10;
	TIM8->PSC = target_prescaler-1;
  TIM8->EGR = TIM_EGR_UG;
}

/**
  * @brief  ���ų�ʼ��
  * @retval ��
  */
void stepper_Init()
{
	Stepper_GPIO_Config();

	TIM_PWMOUTPUT1_Config();
	TIM_PWMOUTPUT2_Config();
	TIM_PWMOUTPUT3_Config();
	TIM_PWMOUTPUT4_Config();
						
}



















