#ifndef __ESP8266_H
#define __ESP8266_H

#include <rtthread.h>
#include <stm32f10x.h>

/* ESP8266ʹ�õĴ������� (����ʹ��USART2) */
#define ESP8266_USART           USART2
#define ESP8266_USART_CLK       RCC_APB1Periph_USART2
#define ESP8266_USART_IRQn      USART2_IRQn

#define ESP8266_GPIO_CLK        RCC_APB2Periph_GPIOA
#define ESP8266_TX_GPIO_PORT    GPIOA
#define ESP8266_TX_GPIO_PIN     GPIO_Pin_2
#define ESP8266_RX_GPIO_PORT    GPIOA
#define ESP8266_RX_GPIO_PIN     GPIO_Pin_3

/* ���ջ�������С */
#define ESP8266_RX_BUFFER_SIZE  512

#define WIFI_SSID       "STM32_ESP8266"     // WiFi�ȵ�����
#define WIFI_PASSWORD   "12345678"          // WiFi����
#define TCP_SERVER_PORT "8877"              // TCP�������˿�


/* �������� */
void ESP8266_Init(void);//���ڳ�ʼ��
rt_bool_t ESP8266_SendCmd(const char* cmd, const char* ack, rt_uint32_t timeout);//����ָ��
rt_bool_t ESP8266_SendData(const char* data, const char* ack, rt_uint32_t timeout);//��������
rt_bool_t ESP8266_GetRecvData(char* buffer, rt_uint16_t buffer_size);//��ȡesp8266��������
void ESP8266_SendString(const char* str);//�����ķ����ַ�����esp8266����
rt_bool_t esp8266_init(void);//����wifi�ȵ�
#endif