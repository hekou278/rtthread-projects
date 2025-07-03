/* esp8266.c - ESP8266驱动实现文件 */
#include "esp8266.h"
#include <string.h>


/* 接收缓冲区 */
static char esp8266_rx_buffer[ESP8266_RX_BUFFER_SIZE];
static volatile rt_uint16_t rx_buffer_index = 0;
static rt_mutex_t esp8266_mutex;

/* ESP8266串口初始化 */
void ESP8266_Init(void)
{
	  //GPIO、USART、中断
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(ESP8266_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(ESP8266_USART_CLK, ENABLE);
    
    /* 配置TX引脚,推挽输出 */
    GPIO_InitStructure.GPIO_Pin = ESP8266_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(ESP8266_TX_GPIO_PORT, &GPIO_InitStructure);
    
    /* 配置RX引脚，浮空输入 */
    GPIO_InitStructure.GPIO_Pin = ESP8266_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ESP8266_RX_GPIO_PORT, &GPIO_InitStructure);
    
    /* 配置USART：波特率、数据位、校验位等 */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//数据位：8 位数据位，是最常见的配置，可传输一个字节（0-255）。
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//停止位：1 位停止位，用于标识数据帧的结束
    USART_InitStructure.USART_Parity = USART_Parity_No;//校验位：无校验位，不进行奇偶校验，可提高传输效率。
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制：不使用 RTS/CTS 硬件流控制信号，适用于数据量较小或接收缓冲区足够大的场景。
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//工作模式：同时启用接收（Rx）和发送（Tx）模式，支持全双工通信。
    USART_Init(ESP8266_USART, &USART_InitStructure);
    
    /* 配置NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = ESP8266_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 使能中断和串口 */
    USART_ITConfig(ESP8266_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(ESP8266_USART, ENABLE);
    
    /* 创建互斥量 */
    esp8266_mutex = rt_mutex_create("esp8266_mutex", RT_IPC_FLAG_PRIO);
    
    /* 清空接收缓冲区，调用标准c库函数，将缓冲区数据全部初始化为0 */
    memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
    rx_buffer_index = 0;
    
    rt_kprintf("ESP8266 UART initialized\n");
}

/* ESP8266串口中断处理函数 */
void USART2_IRQHandler(void)
{
    rt_interrupt_enter();
    //rtthread操作系统提供的中断保护，标识着进入中断上下文
	//用于检查 USART 接收缓冲区非空中断的状态，USART_IT：指定要检查的中断源，这里是 USART_IT_RXNE（接收缓冲区非空中断）。
	//返回值：SET：中断发生且挂起（条件满足）
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
            /* 缓冲区满，重置 */
            rx_buffer_index = 0;
            memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
        }
        
        USART_ClearITPendingBit(ESP8266_USART, USART_IT_RXNE);
    }
    
    rt_interrupt_leave();
}
//定义了三个发送数据到esp8266的函数、一个获取esp8266返回信息的函数
/* 发送字符串到ESP8266，向 ESP8266 发送字符串 */
void ESP8266_SendString(const char* str)
{
    while (*str)
    {
			  //用于检查 USART 发送完成标志位的状态
        while (USART_GetFlagStatus(ESP8266_USART, USART_FLAG_TC) == RESET);
        USART_SendData(ESP8266_USART, *str++);
    }
}

/* 发送AT命令并等待响应 */
//通过该函数发送的指令，会通过串口1在串口助手上打印出来发送信息和接收信息
rt_bool_t ESP8266_SendCmd(const char* cmd, const char* ack, rt_uint32_t timeout)
{
    rt_uint32_t start_time;
    rt_bool_t result = RT_FALSE;
    
    if (rt_mutex_take(esp8266_mutex, RT_WAITING_FOREVER) != RT_EOK)
        return RT_FALSE;
    
    /* 清空接收缓冲区 */
		__disable_irq();  // 禁用全局中断
    memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
    rx_buffer_index = 0;
     __enable_irq();   // 启用全局中断
		
    /* 发送命令 */
    ESP8266_SendString(cmd);
    ESP8266_SendString("\r\n");
    
    rt_kprintf("Send: %s\n", cmd);
    
    /* 等待响应 */
		//是否接收到期望的响应字符串（ack）
    //是否接收到错误信息（error）
    start_time = rt_tick_get();
    while ((rt_tick_get() - start_time) < rt_tick_from_millisecond(timeout))
    {
        if (strstr(esp8266_rx_buffer, ack) != RT_NULL)
					//字符串搜索操作，用于检查在 esp8266_rx_buffer 缓冲区中是否存在子字符串 ack
				//ESP8266的指令集中规定，当发送某个指令时，预期收到某个对应指令回复
        {
            result = RT_TRUE;//认为 ESP8266 已成功响应命令，函数返回 RT_TRUE。
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

/* 发送数据到ESP8266 */
rt_bool_t ESP8266_SendData(const char* data, const char* ack, rt_uint32_t timeout)
{
    rt_uint32_t start_time;
    rt_bool_t result = RT_FALSE;
    
    /* 清空接收缓冲区 */
		__disable_irq();  // 禁用全局中断
    memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
    rx_buffer_index = 0;
	 __enable_irq();   // 启用全局中断
    
    /* 发送数据 */
    ESP8266_SendString(data);
    
    /* 等待响应 */
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

/* 获取接收数据，这里的接收缓冲区是stm32串口2接收esp8266返回的数据 */
rt_bool_t ESP8266_GetRecvData(char* buffer, rt_uint16_t buffer_size)
{
    if (rx_buffer_index > 0)
    {

        strncpy(buffer, esp8266_rx_buffer, buffer_size - 1);
			//字符串复制函数调用，用于将接收缓冲区 esp8266_rx_buffer 中的数据复制到用户提供的缓冲区 buffer 中
			//复制的最大字符数。保留最后一个字节用于手动添加字符串结束符 '\0'
        buffer[buffer_size - 1] = '\0';
        
        /* 清空内部缓冲区 */
				__disable_irq();  // 禁用全局中断
        memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_SIZE);
        rx_buffer_index = 0;
        __enable_irq();   // 启用全局中断
        
        return RT_TRUE;
    }
    
    return RT_FALSE;
}
//生成WiFi热点
rt_bool_t esp8266_init(void)
{
	  char ap_cmd[64];
	 char server_cmd[32];
    rt_kprintf("ESP8266 Initializing...\n");
    
    /* 1. 复位ESP8266 */
	//可以观察到，如果不用在指令中加标识，或者具体数值，可以直接发指令，而不用字符串格式化函数
    if (!ESP8266_SendCmd("AT+RST", "OK", 2000))
    {
        rt_kprintf("ESP8266 reset failed\n");
        return RT_FALSE;
    }
    rt_thread_delay(2000);
    
    /* 2. 设置为AP+STA模式 */
    if (!ESP8266_SendCmd("AT+CWMODE=3", "OK", 1000))
    {
        rt_kprintf("Set WiFi mode failed\n");
        return RT_FALSE;
    }
    
    /* 3. 配置AP热点 */
    // 5 表示使用第 5 信道（2.4GHz 频段）,WiFi 信道号（1-13），指定 AP 使用的无线频段。
		//3是加密方式
    rt_sprintf(ap_cmd, "AT+CWSAP=\"%s\",\"%s\",5,3", WIFI_SSID, WIFI_PASSWORD);
		//生成一个符合 ESP8266 模块 AP 模式（热点模式）配置格式 的 AT 指令字符串
    if (!ESP8266_SendCmd(ap_cmd, "OK", 3000))
    {
        rt_kprintf("Configure AP failed\n");
        return RT_FALSE;
    }
    
    /* 4. 启用多连接 */
    if (!ESP8266_SendCmd("AT+CIPMUX=1", "OK", 1000))
    {
        rt_kprintf("Enable multiple connections failed\n");
        return RT_FALSE;
    }
    
    /* 5. 创建TCP服务器 */
   //1是启用TCP服务器，后面的是端口号
    rt_sprintf(server_cmd, "AT+CIPSERVER=1,%s", TCP_SERVER_PORT);
    if (!ESP8266_SendCmd(server_cmd, "OK", 2000))
    {
        rt_kprintf("Create TCP server failed\n");
        return RT_FALSE;
    }
    
    /* 6. 查询本机IP */
    ESP8266_SendCmd("AT+CIFSR", "OK", 1000);
    
    rt_kprintf("ESP8266 TCP Server initialized successfully\n");
    rt_kprintf("WiFi SSID: %s\n", WIFI_SSID);
    rt_kprintf("TCP Port: %s\n", TCP_SERVER_PORT);
    
    return RT_TRUE;
}
