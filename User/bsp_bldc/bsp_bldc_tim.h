#ifndef __BSP_BLDC_TIM_H
#define	__BSP_BLDC_TIM_H

#include "stm32h7xx.h"
#include ".\bsp_bldc\bsp_bldc_control.h"

/* ������ƶ�ʱ�� */
#define BLDC_TIM												TIM1
#define BLDC_TIM_CLK_ENABLE()						__TIM1_CLK_ENABLE()
extern TIM_HandleTypeDef	htimx_bldcm;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������5599����Ϊ5600�Σ�Ϊһ����ʱ���� */
#define PWM_PERIOD_COUNT     (5600)

#define PWM_MAX_PERIOD_COUNT    (PWM_PERIOD_COUNT - 100)

/* ��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
					PCLK1 = HCLK / 2
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 14.28KHz*/
#define PWM_PRESCALER_COUNT     (3)

/* TIM1ͨ��1�������*/
#define BLDC_OCPWM1_PIN									GPIO_PIN_8
#define BLDC_OCPWM1_GPIO_PORT						GPIOA
#define BLDC_OCPWM1_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#define BLDC_OCPWM1_AF									GPIO_AF1_TIM1

/* TIM1ͨ��2�������*/
#define BLDC_OCPWM2_PIN									GPIO_PIN_9
#define BLDC_OCPWM2_GPIO_PORT						GPIOA
#define BLDC_OCPWM2_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#define BLDC_OCPWM2_AF									GPIO_AF1_TIM1

/* TIM1ͨ��3�������*/
#define BLDC_OCPWM3_PIN									GPIO_PIN_10
#define BLDC_OCPWM3_GPIO_PORT						GPIOA
#define BLDC_OCPWM3_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#define BLDC_OCPWM3_AF									GPIO_AF1_TIM1

/* TIM1ͨ��1�����������*/
#define BLDC_OCNPWM1_PIN								GPIO_PIN_13
#define BLDC_OCNPWM1_GPIO_PORT					GPIOB
#define BLDC_OCNPWM1_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#define BLDC_OCNPWM1_AF									GPIO_AF1_TIM1

/* TIM1ͨ��1�����������*/
#define BLDC_OCNPWM2_PIN								GPIO_PIN_14
#define BLDC_OCNPWM2_GPIO_PORT					GPIOB
#define BLDC_OCNPWM2_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#define BLDC_OCNPWM2_AF									GPIO_AF1_TIM1

/* TIM1ͨ��1�����������*/
#define BLDC_OCNPWM3_PIN								GPIO_PIN_15
#define BLDC_OCNPWM3_GPIO_PORT					GPIOB
#define BLDC_OCNPWM3_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#define BLDC_OCNPWM3_AF									GPIO_AF1_TIM1

#define TIM_COM_TS_ITRx                 TIM_TS_ITR2    // �ڲ���������(TIM1->ITR2->TIM3)

/* ������������ʱ�� */
#define HALL_TIM												TIM3
#define HALL_TIM_CLK_ENABLE()						__TIM3_CLK_ENABLE()

extern TIM_HandleTypeDef 								htimx_hall;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������9999����Ϊ10000�Σ�Ϊһ����ʱ���� */
#define HALL_PERIOD_COUNT     (10000)

/* ��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
					PCLK1 = HCLK / 2 
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 �趨��ʱ��Ƶ��Ϊ = TIMxCLK / (PWM_PRESCALER_COUNT + 1) / PWM_PERIOD_COUNT = 10Hz
   ���� T = 100ms */
#define HALL_PRESCALER_COUNT     (2400)

/* TIM3 ͨ�� 1 ���� */
#define HALL_INPUTU_PIN									GPIO_PIN_6
#define HALL_INPUTU_GPIO_PORT						GPIOC
#define HALL_INPUTU_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#define HALL_INPUTU_AF									GPIO_AF2_TIM3

/* TIM3 ͨ�� 2 ���� */
#define HALL_INPUTV_PIN									GPIO_PIN_7
#define HALL_INPUTV_GPIO_PORT						GPIOC
#define HALL_INPUTV_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#define HALL_INPUTV_AF									GPIO_AF2_TIM3

/* TIM3 ͨ�� 3 ���� */
#define HALL_INPUTW_PIN									GPIO_PIN_8
#define HALL_INPUTW_GPIO_PORT						GPIOC
#define HALL_INPUTW_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#define HALL_INPUTW_AF									GPIO_AF2_TIM3

#define HALL_TIM_IRQn                    TIM3_IRQn
#define HALL_TIM_IRQHandler              TIM3_IRQHandler

extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);
void stop_pwm_output(void);
void start_pwm_output(void);
void set_pwm_pulse(uint16_t pulse);

void hall_enable(void);
void hall_disable(void);
void hall_tim_config(void);


#endif /* __BSP_MOTOR_TIM_H */

