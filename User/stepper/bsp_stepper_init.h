#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

/*�궨��*/
/*�������---1*/
/*******************************************************/
//Motor1 ���� 
#define MOTOR1_DIR_PIN                  	GPIO_PIN_1   
#define MOTOR1_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR1_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor1 ʹ�� 
#define MOTOR1_EN_PIN                  	GPIO_PIN_0
#define MOTOR1_EN_GPIO_PORT            	GPIOE                       
#define MOTOR1_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor1 ����		
#define MOTOR1_PUL_TIM                   TIM8
#define MOTOR1_PUL_CLK_ENABLE()  				__TIM8_CLK_ENABLE()

#define GENERAL1_TIM_CH1_GPIO_PORT       GPIOI
#define GENERAL1_TIM_CH1_PIN             GPIO_PIN_5
#define MOTOR1_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR1_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR1_PUL_CHANNEL_x             TIM_CHANNEL_1


/*�������---2*/
/*******************************************************/
//Motor2 ����
#define MOTOR2_DIR_PIN										GPIO_PIN_8
#define MOTOR2_DIR_GPIO_PORT            	GPIOI 
#define MOTOR2_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOI_CLK_ENABLE()

//Motor2 ʹ��
#define MOTOR2_EN_PIN                  	GPIO_PIN_4
#define MOTOR2_EN_GPIO_PORT            	GPIOE
#define MOTOR2_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor2 ����
#define MOTOR2_PUL_TIM                   TIM8
#define MOTOR2_PUL_CLK_ENABLE()  		__TIM8_CLK_ENABLE()

#define GENERAL2_TIM_CH2_GPIO_PORT       GPIOI
#define GENERAL2_TIM_CH2_PIN             GPIO_PIN_6
#define MOTOR2_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR2_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR2_PUL_CHANNEL_x             TIM_CHANNEL_2


/*�������---3*/
/*******************************************************/
//Motor3 ����
#define MOTOR3_DIR_PIN										GPIO_PIN_11
#define MOTOR3_DIR_GPIO_PORT            	GPIOI 
#define MOTOR3_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOI_CLK_ENABLE()

//Motor3 ʹ��
#define MOTOR3_EN_PIN                  	GPIO_PIN_10
#define MOTOR3_EN_GPIO_PORT            	GPIOI
#define MOTOR3_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOI_CLK_ENABLE()

//Motor3 ����
#define MOTOR3_PUL_TIM                   TIM8
#define MOTOR3_PUL_CLK_ENABLE()  				__TIM8_CLK_ENABLE()

#define GENERAL3_TIM_CH3_GPIO_PORT       GPIOI
#define GENERAL3_TIM_CH3_PIN             GPIO_PIN_7
#define MOTOR3_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR3_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR3_PUL_CHANNEL_x             TIM_CHANNEL_3


/*�������---4*/
/*******************************************************/
//Motor4 ����
#define MOTOR4_DIR_PIN										GPIO_PIN_2
#define MOTOR4_DIR_GPIO_PORT            	GPIOF
#define MOTOR4_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOF_CLK_ENABLE()

//Motor4 ʹ��
#define MOTOR4_EN_PIN                  	GPIO_PIN_1
#define MOTOR4_EN_GPIO_PORT            	GPIOF
#define MOTOR4_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOF_CLK_ENABLE()

//Motor4 ����
#define MOTOR4_PUL_TIM                   TIM8
#define MOTOR4_PUL_CLK_ENABLE()  				__TIM8_CLK_ENABLE()
	
#define GENERAL4_TIM_CH4_GPIO_PORT       GPIOC
#define GENERAL4_TIM_CH4_PIN             GPIO_PIN_9
#define MOTOR4_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOC_CLK_ENABLE()

#define MOTOR4_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR4_PUL_CHANNEL_x             TIM_CHANNEL_4


extern void TIMx_Configuration(void);
extern void TIM2_SetPWM_pulse(int channel,int compare);
extern void STEPMOTOR_SPEED_UP(void);
extern void STEPMOTOR_SPEED_DOWN(void);
	
/************************************************************/
#define HIGH GPIO_PIN_SET       //�ߵ�ƽ
#define LOW  GPIO_PIN_RESET     //�͵�ƽ

#define ON  LOW                 //��
#define OFF HIGH                //��

#define CW  HIGH                //˳ʱ��
#define CCW LOW                 //��ʱ��




//����ʹ������
/* ���κ꣬��������������һ��ʹ�� */
/*���1ʹ�ܡ������������*/
#define MOTOR1_EN(x)				HAL_GPIO_WritePin(MOTOR1_EN_GPIO_PORT,MOTOR1_EN_PIN,x)
//#define MOTOR_PLU(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR1_DIR(x)				HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_PORT,MOTOR1_DIR_PIN,x)
/************************************************************/
/*���2ʹ�ܡ������������*/
#define MOTOR2_EN(x)				HAL_GPIO_WritePin(MOTOR2_EN_GPIO_PORT,MOTOR2_EN_PIN,x)
#define MOTOR2_DIR(x)				HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_PORT,MOTOR2_DIR_PIN,x)
/************************************************************/
/*���3ʹ�ܡ������������*/
#define MOTOR3_EN(x)				HAL_GPIO_WritePin(MOTOR3_EN_GPIO_PORT,MOTOR3_EN_PIN,x)
#define MOTOR3_DIR(x)				HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_PORT,MOTOR3_DIR_PIN,x)
/************************************************************/
/*���4ʹ�ܡ������������*/
#define MOTOR4_EN(x)				HAL_GPIO_WritePin(MOTOR4_EN_GPIO_PORT,MOTOR4_EN_PIN,x)
#define MOTOR4_DIR(x)				HAL_GPIO_WritePin(MOTOR4_DIR_GPIO_PORT,MOTOR4_DIR_PIN,x)
/************************************************************/


extern void stepper_Init(void);
extern void stepper_turn(int tim,float angle,float subdivide,uint8_t dir);
#endif /* __STEP_MOTOR_INIT_H */
