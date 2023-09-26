#ifndef __BSP_BLDCM_CONTROL_H
#define	__BSP_BLDCM_CONTROL_H

#include "stm32h7xx.h"
#include "./bsp_bldc/bsp_bldc_tim.h"
#include "main.h"

//���Ŷ���
/*******************************************************/
// ����������� SD ��
#define BLDC_SHUTDOWN_PIN                  GPIO_PIN_12
#define BLDC_SHUTDOWN_GPIO_PORT            GPIOG
#define BLDC_SHUTDOWN_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE()

/* ��� SD or EN ʹ�ܽ� */
#define BLDC_ENABLE_SD()                     HAL_GPIO_WritePin(BLDC_SHUTDOWN_GPIO_PORT, BLDC_SHUTDOWN_PIN, GPIO_PIN_SET)      // �ߵ�ƽ��-�ߵ�ƽʹ�� 
#define BLDC_DISABLE_SD()                    HAL_GPIO_WritePin(BLDC_SHUTDOWN_GPIO_PORT, BLDC_SHUTDOWN_PIN, GPIO_PIN_RESET)    // �͵�ƽ�ض�-�͵�ƽ����

/* ����������ö�� */
typedef enum
{
  MOTOR_FWD = 0,
  MOTOR_REV,
}motor_dir_t;

typedef struct
{
  motor_dir_t direction;    // �������
  uint16_t dutyfactor;      // PWM ���ռ�ձ�
  uint8_t is_enable;        // ʹ�ܵ��
  uint32_t lock_timeout;    // �����ת��ʱ
}bldcm_data_t;

void bldcm_init(void);


void set_BLDC_speed(uint16_t v);
void set_BLDC_direction(motor_dir_t dir);
motor_dir_t get_BLDC_direction(void);
void set_BLDC_enable(void);
void set_BLDC_disable(void);

#endif /* __BSP_BLDCM_CONTROL_H */

