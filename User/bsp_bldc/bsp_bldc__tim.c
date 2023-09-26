/*
  ******************************************************************************
  * @file    bsp_bldc_tim.c
  * @author  LEO
  * @version V1.0
  * @date    2023-09-07
  * @brief   BLDC��ض�ʱ������
  ******************************************************************************
*/

#include "./bsp_bldc/bsp_bldc_tim.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_debug_usart.h"

TIM_HandleTypeDef htimx_bldcm;
TIM_OC_InitTypeDef TIM_OCInitStructure;

/* ������������ض�ʱ����ʼ�� */
TIM_HandleTypeDef htimx_hall;

static uint16_t bldcm_pulse = 0;

/**
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void TIMx_GPIO_Config(void)
{
	/* ����һ��GPIO_InitTypeDef */
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* ������ʱ����ص�GPIO����ʱ�� */
	BLDC_OCPWM1_GPIO_CLK_ENABLE();
	BLDC_OCNPWM1_GPIO_CLK_ENABLE();
	BLDC_OCPWM2_GPIO_CLK_ENABLE();
	BLDC_OCNPWM2_GPIO_CLK_ENABLE();
	BLDC_OCPWM3_GPIO_CLK_ENABLE();
	BLDC_OCNPWM3_GPIO_CLK_ENABLE();
	
	/* ��ʱ���������ų�ʼ�� */
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; //�������ģʽ
	
	GPIO_InitStructure.Pin = BLDC_OCNPWM1_PIN;
	HAL_GPIO_Init(BLDC_OCNPWM1_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = BLDC_OCNPWM2_PIN;
	HAL_GPIO_Init(BLDC_OCNPWM2_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = BLDC_OCNPWM3_PIN;
	HAL_GPIO_Init(BLDC_OCNPWM3_GPIO_PORT, &GPIO_InitStructure);
	
	/* ͨ��2 */
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	
	GPIO_InitStructure.Pin = BLDC_OCPWM1_PIN;
	GPIO_InitStructure.Alternate = BLDC_OCPWM1_AF;
	HAL_GPIO_Init(BLDC_OCPWM1_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = BLDC_OCPWM2_PIN;
	GPIO_InitStructure.Alternate = BLDC_OCPWM2_AF;
	HAL_GPIO_Init(BLDC_OCPWM2_GPIO_PORT,&GPIO_InitStructure);
	
	/* ͨ��3 */
	GPIO_InitStructure.Pin = BLDC_OCPWM3_PIN;
	GPIO_InitStructure.Alternate = BLDC_OCPWM3_AF;
	HAL_GPIO_Init(BLDC_OCPWM3_GPIO_PORT,&GPIO_InitStructure);
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
static void TIM_Mode_Config(void)
{
  // ����TIMx_CLK,x[1,8] 
	BLDC_TIM_CLK_ENABLE();
	/* ���嶨ʱ���ľ����ȷ����ʱ���Ĵ����Ļ���ַ*/
	htimx_bldcm.Instance = BLDC_TIM;
	/* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
	htimx_bldcm.Init.Period = PWM_PERIOD_COUNT - 1;
  // �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK=216MHz 
  // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
	htimx_bldcm.Init.Prescaler = PWM_PRESCALER_COUNT - 1;
	// ����ʱ�ӷ�Ƶ
	htimx_bldcm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	// ������ʽ
	htimx_bldcm.Init.CounterMode = TIM_COUNTERMODE_UP;
	// �ظ�������
	htimx_bldcm.Init.RepetitionCounter = 0;
	// ��ʼ����ʱ��TIMx��x[1,8]
	HAL_TIM_PWM_Init(&htimx_bldcm);
	
  /*PWMģʽ����*/
  //����ΪPWMģʽ1
  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  TIM_OCInitStructure.Pulse = 0;
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
  TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	HAL_TIM_PWM_ConfigChannel(&htimx_bldcm,&TIM_OCInitStructure,TIM_CHANNEL_1);    // ��ʼ��ͨ�� 1 ��� PWM 
  HAL_TIM_PWM_ConfigChannel(&htimx_bldcm,&TIM_OCInitStructure,TIM_CHANNEL_2);    // ��ʼ��ͨ�� 2 ��� PWM
  HAL_TIM_PWM_ConfigChannel(&htimx_bldcm,&TIM_OCInitStructure,TIM_CHANNEL_3);    // ��ʼ��ͨ�� 3 ��� PWM
	
	/* ���ô���Դ */
	HAL_TIMEx_ConfigCommutationEvent(&htimx_bldcm, TIM_COM_TS_ITRx, TIM_COMMUTATION_SOFTWARE);
	
	/* ������ʱ��ͨ��1���PWM */
	HAL_TIM_PWM_Start(&htimx_bldcm,TIM_CHANNEL_1);
	
	/* ������ʱ��ͨ��2���PWM */
	HAL_TIM_PWM_Start(&htimx_bldcm,TIM_CHANNEL_2);
	
	/* ������ʱ��ͨ��3���PWM */
	HAL_TIM_PWM_Start(&htimx_bldcm,TIM_CHANNEL_3);
}

/**
  * @brief  ֹͣpwm���
  * @param  ��
  * @retval ��
  */
void stop_pwm_output(void)
{
  /* �رն�ʱ��ͨ��1���PWM */
  __HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);

  /* �رն�ʱ��ͨ��2���PWM */
  __HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);
  
  /* �رն�ʱ��ͨ��3���PWM */
  __HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);
  
  HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
}

/**
  * @brief  ����pwm�����ռ�ձ�
  * @param  pulse:Ҫ���õ�ռ�ձ�
  * @retval ��
  */
void set_pwm_pulse(uint16_t pulse)
{
  /* ���ö�ʱ��ͨ����� PWM ��ռ�ձ� */
	bldcm_pulse = pulse;
}

/**
  * @brief  ��ʼ���߼����ƶ�ʱ��
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
	TIMx_GPIO_Config();
	TIM_Mode_Config();
}

/**
  * @brief  �������������ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void hall_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	HALL_INPUTU_GPIO_CLK_ENABLE();
	HALL_INPUTV_GPIO_CLK_ENABLE();
	HALL_INPUTW_GPIO_CLK_ENABLE();
	
	/* ��ʱ��ͨ�� 1 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = HALL_INPUTU_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = HALL_INPUTU_AF;
	HAL_GPIO_Init(HALL_INPUTU_GPIO_PORT, &GPIO_InitStruct);
	
	/* ��ʱ��ͨ�� 2 ���ų�ʼ�� */
	GPIO_InitStruct.Pin = HALL_INPUTV_PIN;
	HAL_GPIO_Init(HALL_INPUTV_GPIO_PORT, &GPIO_InitStruct);
	
	/* ��ʱ��ͨ�� 3 ���ų�ʼ�� */
	GPIO_InitStruct.Pin = HALL_INPUTW_PIN;
	HAL_GPIO_Init(HALL_INPUTW_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  ������������ʱ����ʼ��
  * @param  ��
  * @retval ��
  */
static void hall_tim_init(void)
{
	TIM_HallSensor_InitTypeDef hall_sensor_onfig;
	
	/* ������ʱ������ʱ��ʹ�� */
	HALL_TIM_CLK_ENABLE();
	
  /* ��ʱ�������������� */
  htimx_hall.Instance = HALL_TIM;
  htimx_hall.Init.Prescaler = HALL_PRESCALER_COUNT - 1;       // Ԥ��Ƶ
  htimx_hall.Init.CounterMode = TIM_COUNTERMODE_UP;           // ���ϼ���
  htimx_hall.Init.Period = HALL_PERIOD_COUNT - 1;             // ��������
  htimx_hall.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     // ʱ�ӷ�Ƶ
	
	hall_sensor_onfig.IC1Prescaler = TIM_ICPSC_DIV1;            // ���벶���Ƶ
  hall_sensor_onfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;    // ���벶����
  hall_sensor_onfig.IC1Filter = 10;                           // �����˲�
  hall_sensor_onfig.Commutation_Delay = 0U;                   // ��ʹ���ӳٴ���
	
	HAL_TIMEx_HallSensor_Init(&htimx_hall,&hall_sensor_onfig);
	
	HAL_NVIC_SetPriority(HALL_TIM_IRQn, 0, 0);    // �����ж����ȼ�
  HAL_NVIC_EnableIRQ(HALL_TIM_IRQn);            // ʹ���ж�
}


/**
  * @brief  ʹ�ܻ���������
  * @param  ��
  * @retval ��
  */
void hall_enable(void)
{
  /* ʹ�ܻ����������ӿ� */
  __HAL_TIM_ENABLE_IT(&htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_ENABLE_IT(&htimx_hall, TIM_IT_UPDATE);
	
  HAL_TIMEx_HallSensor_Start(&htimx_hall);
//  LED1_OFF;  
  HAL_TIM_TriggerCallback(&htimx_hall);   // ִ��һ�λ���
}

/**
  * @brief  ���û���������
  * @param  ��
  * @retval ��
  */
void hall_disable(void)
{
  /* ���û����������ӿ� */
  __HAL_TIM_DISABLE_IT(&htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_DISABLE_IT(&htimx_hall, TIM_IT_UPDATE);
  HAL_TIMEx_HallSensor_Stop(&htimx_hall);
}

uint8_t get_hall_state(void)
{
  uint8_t state = 0;
  
#if 1
  /* ��ȡ���������� U ��״̬ */
  if(HAL_GPIO_ReadPin(HALL_INPUTU_GPIO_PORT, HALL_INPUTU_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 0;
  }
  
  /* ��ȡ���������� V ��״̬ */
  if(HAL_GPIO_ReadPin(HALL_INPUTV_GPIO_PORT, HALL_INPUTV_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 1;
  }
  
  /* ��ȡ���������� W ��״̬ */
  if(HAL_GPIO_ReadPin(HALL_INPUTW_GPIO_PORT, HALL_INPUTW_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 2;
  }
#else
  state = (GPIOH->IDR >> 10) & 7;    // �� 3 ��������������״̬
#endif

  return state;    // ���ش�����״̬
}

/**
  * @brief  ��ʼ��������������ʱ��
  * @param  ��
  * @retval ��
  */
void hall_tim_config(void)
{
	hall_gpio_init();	    // ��ʼ������
	hall_tim_init();      // ��ʼ����ʱ��
}

int update = 0;     // ��ʱ�����¼���

/**
  * @brief  ���������������ص�����
  * @param  htim:��ʱ�����
  * @retval ��
  */
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
	/* ��ȡ��������������״̬,��Ϊ��������� */
  uint8_t step = 0;
  step = get_hall_state();
	
	if(get_BLDC_direction() == MOTOR_FWD)
		{
			switch(step)
			{
				case 1:    /* U+ W- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 2 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, bldcm_pulse);                  // ͨ�� 1 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
					break;
				
				case 2:     /* V+ U- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 3 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, bldcm_pulse);                  // ͨ�� 2 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
				
					break;
				
				case 3:    /* V+ W- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
					
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, bldcm_pulse);                  // ͨ�� 2 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
					break;
				
				case 4:     /* W+ V- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
		 
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, bldcm_pulse);                  // ͨ�� 3 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű� 
					break;
				
				case 5:     /* U+  V -*/
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 3 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, bldcm_pulse);                  // ͨ�� 1 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�
					break;
				
				case 6:     /* W+ U- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 2 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, bldcm_pulse);                  // ͨ�� 3 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
					break;
			}
		}
		else
		{
			switch(step)
			{
				case 1:   /* W+ U- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 2 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, bldcm_pulse);                  // ͨ�� 3 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
					break;
				
				case 2:    /* U+  V -*/
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 3 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, bldcm_pulse);                  // ͨ�� 1 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�
					break;
				
				case 3:   /* W+ V- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
		 
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, bldcm_pulse);                  // ͨ�� 3 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�        

					break;
				
				case 4:    /* V+ W- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
					
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, bldcm_pulse);                  // ͨ�� 2 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
					break;
				
				case 5:    /* V+ U- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 3 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, bldcm_pulse);                  // ͨ�� 2 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
					break;
				
				case 6:    /* U+ W- */
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // ͨ�� 2 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM2_GPIO_PORT, BLDC_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
				
					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // ͨ�� 1 ����Ϊ 0
					HAL_GPIO_WritePin(BLDC_OCNPWM1_GPIO_PORT, BLDC_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

					__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, bldcm_pulse);                  // ͨ�� 1 ���õ�ռ�ձ�
					HAL_GPIO_WritePin(BLDC_OCNPWM3_GPIO_PORT, BLDC_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
					break;
			}
		}
  
  HAL_TIM_GenerateEvent(&htimx_bldcm, TIM_EVENTSOURCE_COM);    // �������������¼�����ʱ�Ž�����д��

  update = 0;
	
}

/**
  * @brief  ��ʱ�������жϻص�����
  * @param  htim:��ʱ�����
  * @retval ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (update++ > 1)    // ��һ���ڲ��������ж�ǰ����������û�в���ֵ
  {
    printf("��ת��ʱ\r\n");
    update = 0;
    
    LED1_ON;     // ����LED1��ʾ��ת��ʱֹͣ
    
    /* ��ת��ʱֹͣ PWM ��� */
    hall_disable();       // ���û����������ӿ�
    stop_pwm_output();    // ֹͣ PWM ���
  }
}

/*********************************************END OF FILE**********************/