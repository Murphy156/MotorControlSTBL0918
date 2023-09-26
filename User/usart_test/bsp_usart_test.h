#ifndef __BSP_USART_H
#define	__BSP_USART_H

#include "stm32h7xx.h"
#include <stdio.h>

//���ڲ�����
#define USART_BAUDRATE					115200

/* ����ͷ�ṹ�� */
typedef __packed struct
{
  uint16_t head;    	// ��ͷ
  uint8_t cmd;      	// ����
	uint32_t para;			// ����
	uint8_t parity;			// У���
}packet_head_t;

//���Ŷ���
/*************************************/
#define USART_SEL																USART1
#define USART_CLK_ENABLE()											__USART1_CLK_ENABLE();

#define USART_RX_GPIO_PORT                			GPIOB
#define USART_RX_GPIO_CLK_ENABLE()        			__HAL_RCC_GPIOB_CLK_ENABLE()
#define USART_RX_PIN                      			GPIO_PIN_6
#define USART_RX_AF                       			GPIO_AF7_USART1

#define USART_TX_GPIO_PORT                			GPIOB
#define USART_TX_GPIO_CLK_ENABLE()       				__HAL_RCC_GPIOB_CLK_ENABLE()
#define USART_TX_PIN                      			GPIO_PIN_7
#define USART_TX_AF                       			GPIO_AF7_USART1

#define USART_IRQHandler                  			USART1_IRQHandler
#define USART_IRQ                 		    			USART1_IRQn


/* ���ݽ��ջ�������С */
#define		min_protocol_length	 8
#define		max_protocol_length	 16
#define	 max_length		(min_protocol_length * max_protocol_length) //�˷�һ��Ҫ�����ţ�16*8 = 128

/* ֡ͷ */
#define PACK_HEADER						0x55AA		

/* ָ�� ����λ�� -> ��λ����*/
#define EnablePump1_CMD				0x01			//ʹ�ܱ�1��ָ��,����1
#define DisablePump1_CMD			0x02			//ʧ�ܱ�1��ָ��,����1
#define EnablePump2_CMD				0x03			//ʹ�ܱ�2��ָ��,����2
#define DisablePump2_CMD			0x04			//ʧ�ܱ�2��ָ��,����2
#define EnablePump3_CMD				0x05			//ʹ�ܱ�3��ָ��,����3
#define DisablePump3_CMD			0x06			//ʧ�ܱ�3��ָ��,����3
#define EnablePump4_CMD				0x07			//ʹ�ܱ�4��ָ��,����4
#define DisablePump4_CMD			0x08			//ʧ�ܱ�4��ָ��,����4
#define EnablePump5_CMD				0x09			//ʹ�ܱ�5��ָ��,BLDC
#define DisablePump5_CMD			0x0A			//ʧ�ܱ�5��ָ��,BLDC

#define ChangePump1Speed_CMD  0x11			//�ı��1ת�ٵ�ָ��,����1
#define ChangePump2Speed_CMD  0x12			//�ı��2ת�ٵ�ָ��,����2
#define ChangePump3Speed_CMD  0x13			//�ı��3ת�ٵ�ָ��,����3
#define ChangePump4Speed_CMD  0x14			//�ı��4ת�ٵ�ָ��,����4
#define ChangePump5Speed_CMD  0x15			//�ı��5ת�ٵ�ָ��,BLDC

/* ��ָ�� */
#define CMD_NONE             	0xFF     // ��ָ��

void USART_Config(void);
uint8_t PushArr(uint8_t *arr,uint8_t data);
uint8_t PopArr(uint8_t *arr ,uint8_t *data);
extern uint8_t	data_buff[max_length];
extern UART_HandleTypeDef UartHandle;
uint8_t calculateChecksum(unsigned char *data);


int8_t receiving_process(void);
uint16_t calculateCRC16(uint8_t arr[7]);
static uint8_t recvBuff_Find_Header(void);
uint16_t mergeBytesToUint16(uint8_t data[2]);
uint32_t mergeParametersToUint32(void);
static uint8_t protocol_Check_header_CRC(void);
void parseData(uint32_t input,uint8_t motor_chose);
void parseBLDC(uint32_t input);

#endif /* __USART1_H */

