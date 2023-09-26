/**
  ******************************************************************************
  * @file    bsp_bldc_control.c
  * @author  Leo
  * @version V1.0
  * @date    2023-09-07
  * @brief   
  ******************************************************************************
  */ 

#include ".\bsp_bldc\bsp_bldc_control.h"
#include "./led/bsp_led.h"

/* ˽�б��� */
static bldcm_data_t bldcm_data;

/* �ֲ����� */
static void sd_gpio_config(void);

/**
  * @brief  �����ʼ��
  * @param  ��
  * @retval ��
  */
void bldcm_init(void)
{
  TIMx_Configuration();    // ������ƶ�ʱ�������ų�ʼ��
  hall_tim_config();       // ������������ʼ��
  sd_gpio_config();
}

static void sd_gpio_config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
	BLDC_SHUTDOWN_GPIO_CLK_ENABLE();
  
  /* ����IO��ʼ�� */
	/*�����������*/                                                
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*������������ */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = BLDC_SHUTDOWN_PIN;
  
	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
  HAL_GPIO_Init(BLDC_SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  ���õ���ٶ�
  * @param  v: �ٶȣ�ռ�ձȣ�
  * @retval ��
  */
void set_BLDC_speed(uint16_t v)
{
  bldcm_data.dutyfactor = v;
  
  set_pwm_pulse(v);     // �����ٶ�
}


/**
  * @brief  ���õ������
  * @param  ��
  * @retval ��
  */
void set_BLDC_direction(motor_dir_t dir)
{
  bldcm_data.direction = dir;
}

/**
  * @brief  ��ȡ�����ǰ����
  * @param  ��
  * @retval ��
  */
motor_dir_t get_BLDC_direction(void)
{
  return bldcm_data.direction;
}


/**
  * @brief  ʹ�ܵ��
  * @param  ��
  * @retval ��
  */
void set_BLDC_enable(void)
{
  BLDC_ENABLE_SD();
	/* �ӳ�50ms��H743��Cache���ٶȻ���죬δ����SD����ʱ�򡣼������ϵͳʱ�����ע�������ʱ�����Ƿ���Ч���������� */
	HAL_Delay(50);
  hall_enable();
}

/**
  * @brief  ���õ��
  * @param  ��
  * @retval ��
  */
void set_BLDC_disable(void)
{
  /* ���û����������ӿ� */
  hall_disable();
  
  /* ֹͣ PWM ��� */
  stop_pwm_output();
  
  /* �ر� MOS �� */
  BLDC_DISABLE_SD();
}


