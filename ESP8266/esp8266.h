#ifndef __ESP8266_H
#define __ESP8266_H

#include <rtthread.h>
#include <stm32f10x.h>

/* ESP8266使用的串口配置 (假设使用USART2) */
#define ESP8266_USART           USART2
#define ESP8266_USART_CLK       RCC_APB1Periph_USART2
#define ESP8266_USART_IRQn      USART2_IRQn

#define ESP8266_GPIO_CLK        RCC_APB2Periph_GPIOA
#define ESP8266_TX_GPIO_PORT    GPIOA
#define ESP8266_TX_GPIO_PIN     GPIO_Pin_2
#define ESP8266_RX_GPIO_PORT    GPIOA
#define ESP8266_RX_GPIO_PIN     GPIO_Pin_3

/* 接收缓冲区大小 */
#define ESP8266_RX_BUFFER_SIZE  512

#define WIFI_SSID       "STM32_ESP8266"     // WiFi热点名称
#define WIFI_PASSWORD   "12345678"          // WiFi密码
#define TCP_SERVER_PORT "8877"              // TCP服务器端口


/* 函数声明 */
void ESP8266_Init(void);//串口初始化
rt_bool_t ESP8266_SendCmd(const char* cmd, const char* ack, rt_uint32_t timeout);//发送指令
rt_bool_t ESP8266_SendData(const char* data, const char* ack, rt_uint32_t timeout);//发送数据
rt_bool_t ESP8266_GetRecvData(char* buffer, rt_uint16_t buffer_size);//获取esp8266返回数据
void ESP8266_SendString(const char* str);//基本的发送字符串到esp8266函数
rt_bool_t esp8266_init(void);//生成wifi热点
#endif