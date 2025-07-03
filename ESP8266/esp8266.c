/* esp8266.c - ESP8266����ʵ���ļ� */
#include "esp8266.h"
#include <string.h>


/* ���ջ����� */
static char esp8266_rx_buffer[ESP8266_RX_BUFFER_SIZE];
static volatile rt_uint16_t rx_buffer_index = 0;
static rt_mutex_t esp8266_mutex;

/* ESP8266���ڳ�ʼ�� */
void ESP8266_Init(void)
{
	  //GPIO��USART���ж�
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* ʹ��ʱ�� */
    RCC_APB2PeriphClockCmd(ESP8266_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(ESP8266_USART_CLK, ENABLE);
    
    /* ����TX����,������� */
    GPIO_InitStructure.GPIO_Pin = ESP8266_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(ESP8266_TX_GPIO_PORT, &GPIO_InitStructure);
    
    /* ����RX���ţ��������� */
    GPIO_InitStructure.GPIO_Pin = ESP8266_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ESP8266_RX_GPIO_PORT, &GPIO_InitStructure);
    
    /* ����USART�������ʡ�����λ��У��λ�� */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//����λ��8 λ����λ������������ã��ɴ���һ���ֽڣ�0-255����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//ֹͣλ��1 λֹͣλ�����ڱ�ʶ����֡�Ľ���
    USART_InitStructure.USART_Parity = USART_Parity_No;//У��λ����У��λ����������żУ�飬����ߴ���Ч�ʡ�
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ�������ƣ���ʹ�� RTS/CTS Ӳ���������źţ���������������С����ջ������㹻��ĳ�����
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//����ģʽ��ͬʱ���ý��գ�Rx���ͷ��ͣ�Tx��ģʽ��֧��ȫ˫��ͨ�š�
    USART_Init(ESP8266_USART, &USART_InitStructure);
    
    /* ����NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = ESP8266_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* ʹ���жϺʹ��� */
    USART_ITConfig(ESP8266_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(ESP8266_USART, ENABLE);
    
    /* ���������� */
    esp8266_mutex = rt_mutex_create("esp8266_mutex", RT_IPC_FLAG_PRIO);
    
    /* ��ս��ջ����������ñ�׼c�⺯����������������ȫ����ʼ��Ϊ0 */
    memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
    rx_buffer_index = 0;
    
    rt_kprintf("ESP8266 UART initialized\n");
}

/* ESP8266�����жϴ����� */
void USART2_IRQHandler(void)
{
    rt_interrupt_enter();
    //rtthread����ϵͳ�ṩ���жϱ�������ʶ�Ž����ж�������
	//���ڼ�� USART ���ջ������ǿ��жϵ�״̬��USART_IT��ָ��Ҫ�����ж�Դ�������� USART_IT_RXNE�����ջ������ǿ��жϣ���
	//����ֵ��SET���жϷ����ҹ����������㣩
    if (USART_GetITStatus(ESP8266_USART, USART_IT_RXNE) != RESET)
    {
        char ch = USART_ReceiveData(ESP8266_USART);
        
        if (rx_buffer_index < ESP8266_RX_BUFFER_SIZE - 1)
        {
            esp8266_rx_buffer[rx_buffer_index++] = ch;
            esp8266_rx_buffer[rx_buffer_index] = '\0';
        }
        else
        {
            /* �������������� */
            rx_buffer_index = 0;
            memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
        }
        
        USART_ClearITPendingBit(ESP8266_USART, USART_IT_RXNE);
    }
    
    rt_interrupt_leave();
}
//�����������������ݵ�esp8266�ĺ�����һ����ȡesp8266������Ϣ�ĺ���
/* �����ַ�����ESP8266���� ESP8266 �����ַ��� */
void ESP8266_SendString(const char* str)
{
    while (*str)
    {
			  //���ڼ�� USART ������ɱ�־λ��״̬
        while (USART_GetFlagStatus(ESP8266_USART, USART_FLAG_TC) == RESET);
        USART_SendData(ESP8266_USART, *str++);
    }
}

/* ����AT����ȴ���Ӧ */
//ͨ���ú������͵�ָ���ͨ������1�ڴ��������ϴ�ӡ����������Ϣ�ͽ�����Ϣ
rt_bool_t ESP8266_SendCmd(const char* cmd, const char* ack, rt_uint32_t timeout)
{
    rt_uint32_t start_time;
    rt_bool_t result = RT_FALSE;
    
    if (rt_mutex_take(esp8266_mutex, RT_WAITING_FOREVER) != RT_EOK)
        return RT_FALSE;
    
    /* ��ս��ջ����� */
		__disable_irq();  // ����ȫ���ж�
    memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
    rx_buffer_index = 0;
     __enable_irq();   // ����ȫ���ж�
		
    /* �������� */
    ESP8266_SendString(cmd);
    ESP8266_SendString("\r\n");
    
    rt_kprintf("Send: %s\n", cmd);
    
    /* �ȴ���Ӧ */
		//�Ƿ���յ���������Ӧ�ַ�����ack��
    //�Ƿ���յ�������Ϣ��error��
    start_time = rt_tick_get();
    while ((rt_tick_get() - start_time) < rt_tick_from_millisecond(timeout))
    {
        if (strstr(esp8266_rx_buffer, ack) != RT_NULL)
					//�ַ����������������ڼ���� esp8266_rx_buffer ���������Ƿ�������ַ��� ack
				//ESP8266��ָ��й涨��������ĳ��ָ��ʱ��Ԥ���յ�ĳ����Ӧָ��ظ�
        {
            result = RT_TRUE;//��Ϊ ESP8266 �ѳɹ���Ӧ����������� RT_TRUE��
            break;
        }
        
        if (strstr(esp8266_rx_buffer, "ERROR") != RT_NULL)
        {
            break;
        }
        
        rt_thread_delay(10);
    }
    
    rt_kprintf("Recv: %s\n", esp8266_rx_buffer);
    rt_mutex_release(esp8266_mutex);
    
    return result;
}

/* �������ݵ�ESP8266 */
rt_bool_t ESP8266_SendData(const char* data, const char* ack, rt_uint32_t timeout)
{
    rt_uint32_t start_time;
    rt_bool_t result = RT_FALSE;
    
    /* ��ս��ջ����� */
		__disable_irq();  // ����ȫ���ж�
    memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
    rx_buffer_index = 0;
	 __enable_irq();   // ����ȫ���ж�
    
    /* �������� */
    ESP8266_SendString(data);
    
    /* �ȴ���Ӧ */
    start_time = rt_tick_get();
    while ((rt_tick_get() - start_time) < rt_tick_from_millisecond(timeout))
    {
        if (strstr(esp8266_rx_buffer, ack) != RT_NULL)
        {
            result = RT_TRUE;
            break;
        }
        
        if (strstr(esp8266_rx_buffer, "ERROR") != RT_NULL)
        {
            break;
        }
        
        rt_thread_delay(10);
    }
    
    return result;
}

/* ��ȡ�������ݣ�����Ľ��ջ�������stm32����2����esp8266���ص����� */
rt_bool_t ESP8266_GetRecvData(char* buffer, rt_uint16_t buffer_size)
{
    if (rx_buffer_index > 0)
    {

        strncpy(buffer, esp8266_rx_buffer, buffer_size - 1);
			//�ַ������ƺ������ã����ڽ����ջ����� esp8266_rx_buffer �е����ݸ��Ƶ��û��ṩ�Ļ����� buffer ��
			//���Ƶ�����ַ������������һ���ֽ������ֶ�����ַ��������� '\0'
        buffer[buffer_size - 1] = '\0';
        
        /* ����ڲ������� */
				__disable_irq();  // ����ȫ���ж�
        memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
        rx_buffer_index = 0;
        __enable_irq();   // ����ȫ���ж�
        
        return RT_TRUE;
    }
    
    return RT_FALSE;
}
//����WiFi�ȵ�
rt_bool_t esp8266_init(void)
{
	  char ap_cmd[64];
	 char server_cmd[32];
    rt_kprintf("ESP8266 Initializing...\n");
    
    /* 1. ��λESP8266 */
	//���Թ۲쵽�����������ָ���мӱ�ʶ�����߾�����ֵ������ֱ�ӷ�ָ��������ַ�����ʽ������
    if (!ESP8266_SendCmd("AT+RST", "OK", 2000))
    {
        rt_kprintf("ESP8266 reset failed\n");
        return RT_FALSE;
    }
    rt_thread_delay(2000);
    
    /* 2. ����ΪAP+STAģʽ */
    if (!ESP8266_SendCmd("AT+CWMODE=3", "OK", 1000))
    {
        rt_kprintf("Set WiFi mode failed\n");
        return RT_FALSE;
    }
    
    /* 3. ����AP�ȵ� */
    // 5 ��ʾʹ�õ� 5 �ŵ���2.4GHz Ƶ�Σ�,WiFi �ŵ��ţ�1-13����ָ�� AP ʹ�õ�����Ƶ�Ρ�
		//3�Ǽ��ܷ�ʽ
    rt_sprintf(ap_cmd, "AT+CWSAP=\"%s\",\"%s\",5,3", WIFI_SSID, WIFI_PASSWORD);
		//����һ������ ESP8266 ģ�� AP ģʽ���ȵ�ģʽ�����ø�ʽ �� AT ָ���ַ���
    if (!ESP8266_SendCmd(ap_cmd, "OK", 3000))
    {
        rt_kprintf("Configure AP failed\n");
        return RT_FALSE;
    }
    
    /* 4. ���ö����� */
    if (!ESP8266_SendCmd("AT+CIPMUX=1", "OK", 1000))
    {
        rt_kprintf("Enable multiple connections failed\n");
        return RT_FALSE;
    }
    
    /* 5. ����TCP������ */
   //1������TCP��������������Ƕ˿ں�
    rt_sprintf(server_cmd, "AT+CIPSERVER=1,%s", TCP_SERVER_PORT);
    if (!ESP8266_SendCmd(server_cmd, "OK", 2000))
    {
        rt_kprintf("Create TCP server failed\n");
        return RT_FALSE;
    }
    
    /* 6. ��ѯ����IP */
    ESP8266_SendCmd("AT+CIFSR", "OK", 1000);
    
    rt_kprintf("ESP8266 TCP Server initialized successfully\n");
    rt_kprintf("WiFi SSID: %s\n", WIFI_SSID);
    rt_kprintf("TCP Port: %s\n", TCP_SERVER_PORT);
    
    return RT_TRUE;
}
