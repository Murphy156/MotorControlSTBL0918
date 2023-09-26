#include "./usart_test/bsp_usart_test.h"
#include ".\bsp_bldc\bsp_bldc_control.h"
#include "./step_v2/step2.h"
#include "./led/bsp_led.h"

UART_HandleTypeDef UartHandle;

uint8_t	 data_buff[max_length];
uint8_t	 rd_p,wr_p;

/**
  * @brief  ������ʼ��USART���ڵ����ã����У�����GPIO�ڣ�����UASRT�Ĺ���
	*	@note 	��
  * @retval ��
  */
void USART_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	USART_CLK_ENABLE();
	
	USART_RX_GPIO_CLK_ENABLE();
	USART_TX_GPIO_CLK_ENABLE();
	
/**USART1 GPIO Configuration    
  PA9     ------> USART1_TX
  PA10    ------> USART1_RX 
  */
  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStruct.Pin = USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = USART_TX_AF;
  HAL_GPIO_Init(USART_TX_GPIO_PORT, &GPIO_InitStruct);	
	
  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStruct.Pin = USART_RX_PIN;
  GPIO_InitStruct.Alternate = USART_RX_AF;
  HAL_GPIO_Init(USART_RX_GPIO_PORT, &GPIO_InitStruct);
	
  /* ���ô���USART ģʽ */
	UartHandle.Instance	=	USART_SEL;
	UartHandle.Init.WordLength 	=	UART_WORDLENGTH_8B; 	//��USART_CR1�е�Mλ���б���Զ����ֳ�
	UartHandle.Init.BaudRate = USART_BAUDRATE;					//ѡ������Ĳ�����
	UartHandle.Init.StopBits = UART_STOPBITS_1;					//��USART_CR2�е�ֹͣλ�������б��
	UartHandle.Init.Parity	=	UART_PARITY_NONE;
	UartHandle.Init.Mode	=	 UART_MODE_TX_RX;	
	HAL_UART_Init(&UartHandle);													//ֻ�ǿ�����USART_UE
    /*���ô��ڽ����ж� */
  __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);  
	HAL_NVIC_SetPriority(USART_IRQ, 0, 2);
	HAL_NVIC_EnableIRQ(USART_IRQ);
}

/**
  * @brief  ���������������uasrt��⵽�������ƽ�data_buff����
	*	@param 	��һ������ָ��data_buff
	*	@param 	�ڶ�������ָ��RDR�Ĵ����ռ���һ��byte������
	*	@note 	��
  * @retval ����1ʱ��˵���˴�������ݺͶ���������ͬ���������ˣ�������������ݣ��Ѿ�����������ݣ�
						����0ʱ��˵���˴��������ݵģ���û�н���Щ��������ݽ�����
  */
uint8_t PushArr(uint8_t *arr,uint8_t data)
{
	uint8_t index;
	index = (wr_p+1)%max_length;
	if(index == rd_p)return 1;
	arr[wr_p]=data;
	wr_p = index;
	return 0;
}

/**
  * @brief  ���������������data_buff��֡���ݶ�ȡ������Э������ĺ�����
	*	@param 	��һ������ָ��data_buff
	*	@param 	�ڶ�������ָ���������һ֡���ݵ�������
	*	@note 	��
  * @retval ����1ʱ��˵���˴�������ݺͶ���������ͬ���������ˣ�������������ݾ��Ѿ������������һ֡һ֡�ش���
						����0ʱ��˵���˴��������ݵģ���û�н���Щ��������ݽ�����
  */
uint8_t PopArr(uint8_t *arr ,uint8_t *data)
{
	if(rd_p == wr_p)return 1;
	*data = arr[rd_p];
	rd_p= (rd_p+1)%max_length;
	return 0;
}

/**
  * @brief  ���������������ȡ����֡��У���
	*	@param 	����
	*	@param 	�����С
  * @retval ����1��ʾ������1������0��ʾż����1
  */
uint8_t calculateChecksum(unsigned char *data) {
    int onesCount = 0;

    for (int i = 0; i < 7; i++) {
        unsigned char byte = data[i];

        for (int j = 0; j < 8; j++) {
            onesCount += (byte >> j) & 1;
        }
    }

    return onesCount % 2;  // ����1��ʾ������1������0��ʾż����1
} 




/**
  * @brief  ���������������ȡ����֡��CRC-16У��ֵ
	*	@param 	7���ֽڴ�С������
  * @retval CRCУ��ֵ
  */

uint16_t calculateCRC16(uint8_t arr[7]) {
    uint16_t crc = 0xFFFF;
    uint16_t poly = 0x8005;

    for (int i = 0; i < 7; i++) {
        crc ^= (arr[i] << 8);

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc & 0xFFFF;
}


/**
 * @brief   ����֡ͷ
 *          ֱ�Ӳ�����ָ��
 * @return  0��û���ҵ�֡ͷ��1:�ҵ���֡ͷ�����ҵ�ǰrd_pָ��0x55����һ��λ�ü��������
 */
static uint8_t recvBuff_Find_Header(void)
{
    uint8_t index;
		index = rd_p;
    //�����ȣ����ָ���дָ��ָ��ͬһλ��,����Ҫ�Ƚ���ע�͵�
    if(rd_p == wr_p) return 0;

	if(data_buff[index] != 0xAA)
	{
		rd_p = (index +1) % max_length;
		return 0;
	}
	else 
	{
        //��Ӧ����ҵ�aa�󣬽�ָ���1ȥ����һ��λ
		index = (rd_p + 1) % max_length;
		if(data_buff[index] != 0x55)
		{
      //��ǰλ�Ҳ�������1Ѱ��ָ����һ��
			rd_p = (index +1) % max_length;
			return 0;
		}
		else
		{
            //�ҵ��˵�ǰ55�����԰�ͷ��λ������������ָ��ָ������λ
            rd_p = (index +1) % max_length;
            return 1;
		}
	}	
}

/**
 * @brief   �������ڰ�����ϲ������ֽ�Ϊһ��16λ����
 * @return  ����һ�������ֽڵ�����
 */
uint16_t mergeBytesToUint16(uint8_t data[2]) {
    uint16_t mergedData = 0;

    for (int i = 0; i < 4; i++) {
        mergedData |= ((uint16_t)data[i] << (i * 8));
    }
    return mergedData;
}


/**
 * @brief   ����һ��ָ�����ݣ�������������ĺ�������ָ���Ӧ��������ȡ�ĸ��ֽڳ��������ص����ٶ�ֵ�������и�����
 * @para    �����ָ������
 * @return  ����һ�����ֽڵ����ݣ�
 */
uint32_t mergeParametersToUint32(void) {
    // ��ȡ�ĸ��ֽڲ����룬ע������ת��
    uint32_t mergedData = 0;
    for (int i = 0; i < 4; i++) {
        rd_p = (rd_p + 1) % 128;  // ���� rd_p
        //�Ƚ�ȡ������һ���ֽڵ�������չ��4���ֽڵ����ݣ�Ȼ���ٸ�����4���ֽ��е�λ������ƽ�ƣ�ÿ��ƽ��ʱ��Ҫ����32λ�Ļ�������Ӷ���֤λ��һ��
        mergedData |= ((uint32_t)data_buff[rd_p] << (8 * i));
    }
    return mergedData;
}

/**
 * @brief   �����ٶ����ݣ�Ҫע����ת���Ƿ�ת
 * @param   ����ֵ��32λ��4���ֽڵ�����
 * @return  void
 */
void parseData(uint32_t input,uint8_t motor_chose) {
    int32_t set;
    uint16_t speed_pulse;   //����ת��ֵ
    // ������32λ�����ݵ����λ�Ƿ�Ϊ1�����Ϊ1��Ϊ����
    if (input & 0x80000000) {
        //�����������һ�������Ĳ��룬Ҫ����ת��һ�£���ԭ��
        set = (~input) + 1;
			
				//����ǵõ�����ķ���
				MOTOR_DIR(step_motor[motor_chose].dir_port, step_motor[motor_chose].dir_pin, CCW);
			
        //�õ���ת�ٵļ���ֵ
        speed_pulse = 37500 / set;
				
			  // д���µıȽ�ֵ
				step_motor[motor_chose].oc_pulse_num = speed_pulse;
    } else {
        //�����������һ������
        set = input;
			
				//����ǵõ�����ķ���
				MOTOR_DIR(step_motor[motor_chose].dir_port, step_motor[motor_chose].dir_pin, CW);
			
				//�õ���ת�ٵļ���ֵ
        speed_pulse = 37500 / set;
				
				// д���µıȽ�ֵ
				step_motor[motor_chose].oc_pulse_num = speed_pulse;
    }
    return ;
}


/**
 * @brief   ����BLDC�ٶ����ݣ�Ҫע����ת���Ƿ�ת
 * @param   ����ֵ��32λ��4���ֽڵ�����
 * @return  void
 */
void parseBLDC(uint32_t input){
	int32_t set;
	uint16_t speed_pulse;   //����ת��ֵ
	    // ������32λ�����ݵ����λ�Ƿ�Ϊ1�����Ϊ1��Ϊ����
    if (input & 0x80000000) {
        //�����������һ�������Ĳ��룬Ҫ����ת��һ�£���ԭ��
        set = (~input) + 1;
			
				//����ǵõ�����ķ���
				set_BLDC_direction(MOTOR_REV);
			
				speed_pulse = set;
			
				set_BLDC_speed(speed_pulse);
				
    } else {
        //�����������һ������
        set = input;
			
				//����ǵõ�����ķ���
				set_BLDC_direction(MOTOR_FWD);
			
				//�õ���ת�ٵļ���ֵ
        speed_pulse = set;
				
				// д���µıȽ�ֵ
				set_BLDC_speed(speed_pulse);
    }
    return ;
}


/**
 * @brief   ����֡ͷ,��֤���ݵ���ȷ�Լ���֤CRC��ֵ
 * @return  ����һ����־״̬��״̬�����ҵ�֡ͷ�Լ�������ȷ����ͨ����1����ͨ����0�������
 */
static uint8_t protocol_Check_header_CRC(void)
{
    uint8_t tmp_arr[7];
    uint8_t tmp_crc[2];
    uint16_t cal_crc16;
    uint16_t send_crc16;
    uint8_t pack_header_flag;
    uint8_t header_CRC_Ready_flat;
    pack_header_flag = recvBuff_Find_Header();
    uint8_t tmp_ptr;
    //û�ҵ�����ͷ
    if(0 == pack_header_flag)
    {
        return header_CRC_Ready_flat = 0;
    }
    else
    {
        //��ָ������ĵ�rd_p�������������������ֱ��ʹ��
        tmp_ptr = rd_p;
        tmp_arr[0] = 0xAA;
        tmp_arr[1] = 0x55;
        //������λ�÷���������8λ����Ϊ��Ѱ�Ұ�ͷ��ʱ��Ѱ�ҽ�����ǰָ��ָ���������λ��
        tmp_arr[2] = data_buff[tmp_ptr];

        //�����ݽ���crcУ�飬���У������ȷ��ȡ�������Լ�����
        //for��init;condition;increment ��ѭ���Ŀ������ǣ�������ִ��init��ֵ��Ȼ������������жϣ��ж�ͨ����ִ������for�ڵ���ͬ��Ȼ���ٵ�increment�н��в���
        for(int i = 3; i < 7; i++)
        {
            tmp_ptr = (tmp_ptr+1) % max_length;
            tmp_arr[i] = data_buff[tmp_ptr];
        }
        //�������forѭ���󣬷��ص��ǲ��������һ��ֵ

        //��ԭʼ�����ݰ�ͷ�������롢����ֵ���м���CRC��
        cal_crc16 = calculateCRC16(tmp_arr); 


        //ȡ��CRCУ����
        for (int j = 0; j < 2; j++)
        {
            tmp_ptr = (tmp_ptr+1) % max_length;
            tmp_crc[j] = data_buff[tmp_ptr];
        }
        //���tmp_ptrָ��У�����λ
        //���䷴��ϲ���16λ������
        send_crc16 = mergeBytesToUint16(tmp_crc);
        //������͹�����У��ֵ���������յ����ݺ�����У��ֵ������֡����Ϊ��������ݣ�����
        if (send_crc16 != cal_crc16)
        {
            // ������ָ����һ������
            tmp_ptr = (tmp_ptr+1) % max_length;
            rd_p = tmp_ptr;
            return 0;
        }
        else
        {
            //ָ����֡���ݵ������ֽڣ�����ǰ�����Ѿ���rd_p�������������ִ�е���������������֤ͨ��
            return 1;
        }
    }
}

/**
 * @brief   ���յ����ݴ���
 * @param   void
 * @return  -1��û���ҵ�һ����ȷ������.
 */
int8_t receiving_process(void)
{
    uint8_t cmd_type = CMD_NONE;
    uint8_t header_CRC_Ready_flat;
    uint32_t tmp_para;            //ע�⣬�������п����Ǹ���������Ǹ���ʱ����������������ǲ���


    while (1)
    {  
        header_CRC_Ready_flat = protocol_Check_header_CRC();
        if (1 != header_CRC_Ready_flat)
        {
            return -1;
        }
        //�Ѿ��ҵ���֡ͷ�Լ�У����CRC,��ʱ��rd_pָ��ǰ����֡������λ
        else
        {   
						
            cmd_type = data_buff[rd_p];
            switch (cmd_type)
            {
							
            case EnablePump1_CMD:
            {
							 LED1_ON;
							 //��ȡ����ֵ
							 tmp_para = mergeParametersToUint32();
							 //¼���ٶ�ֵ
							 parseData(tmp_para,0);
							 //�����������
							 stepper_Start(step_motor[0].pul_channel);
               //������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
               rd_p = (rd_p+3) % max_length;
            }
                break;
						
						case DisablePump1_CMD:
						{
							LED1_OFF;
							stepper_Stop(step_motor[0].pul_channel);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+7) % max_length;
						}
								break;
						
						case EnablePump2_CMD:
						{
							LED2_ON;
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							//¼���ٶ�ֵ
							parseData(tmp_para,1);
							//�����������
							stepper_Start(step_motor[1].pul_channel);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case DisablePump2_CMD:
						{
							LED2_OFF;
							stepper_Stop(step_motor[1].pul_channel);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+7) % max_length;
						}
								break;
						
						case EnablePump3_CMD:
						{
							LED3_ON;
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							//¼���ٶ�ֵ
							parseData(tmp_para,2);
							//�����������
							stepper_Start(step_motor[2].pul_channel);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case DisablePump3_CMD:
						{
							LED3_OFF;
							stepper_Stop(step_motor[2].pul_channel);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+7) % max_length;
						}
								break;
						
						case EnablePump4_CMD:
						{
							LED4_ON;
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							//¼���ٶ�ֵ
							parseData(tmp_para,3);
							//�����������
							stepper_Start(step_motor[3].pul_channel);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case DisablePump4_CMD:
						{
							LED4_OFF;
							stepper_Stop(step_motor[3].pul_channel);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+7) % max_length;
						}
								break;
						
						case ChangePump1Speed_CMD:
						{
							LED1_TOGGLE;
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							//¼���ٶ�ֵ
							parseData(tmp_para,0);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case ChangePump2Speed_CMD:
						{
							LED2_TOGGLE;
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							//¼���ٶ�ֵ
							parseData(tmp_para,1);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case ChangePump3Speed_CMD:
						{
							LED3_TOGGLE;
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							//¼���ٶ�ֵ
							parseData(tmp_para,2);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case ChangePump4Speed_CMD:
						{
							LED4_TOGGLE;
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							//¼���ٶ�ֵ
							parseData(tmp_para,3);
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case EnablePump5_CMD:
						{
							
							HAL_NVIC_EnableIRQ(HALL_TIM_IRQn);            // ʹ���ж�
							
							//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							
							parseBLDC(tmp_para);
							
							set_BLDC_enable();
							
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
								break;
						
						case DisablePump5_CMD:
						{
							set_BLDC_disable();
							
							HAL_NVIC_DisableIRQ(HALL_TIM_IRQn);
							
							rd_p = (rd_p+7) % max_length;
						}
								break;
						
						case ChangePump5Speed_CMD:
						{
														//��ȡ����ֵ
							tmp_para = mergeParametersToUint32();
							
							parseBLDC(tmp_para);
							
							//������һ֡���ݺ�rd_p�Ƶ���һ������֡��ͷ 
              rd_p = (rd_p+3) % max_length;
						}
						
            default:
                return -1;
            }

        }   
    }
}
