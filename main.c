#include <string.h>
//rtthread操作系统相关
#include <rtthread.h>
//stm32标准库相关
#include <system.h>
#include "SysTick.h"
//驱动
#include "led.h"
#include "usart.h"
#include "key.h"
#include "24cxx.h"
#include "ds18b20.h"
#include "lsens.h"
#include "tftlcd.h"
#include <pwm.h>
#include <data_type.h>
#include <esp8266.h>

static void Draw_Light_History_Curve(void);
static void Draw_Temp_History_Curve(void);
static void get_ordered_history_data(rt_uint8_t *light_data, float *temp_data, rt_uint8_t count);

/* 全局变量 */
static rt_thread_t data_collect_thread = RT_NULL;
static rt_thread_t data_process_thread = RT_NULL;
static rt_thread_t display_thread = RT_NULL;
static rt_thread_t storage_thread = RT_NULL;


//若用全局变量：需频繁加锁保护（如代码中的互斥量），且可能因线程调度导致数据不一致
//若用邮箱：邮箱适合传递指针（如字符串、缓冲区地址）
//但当前数据结构（sensor_data_t、processed_data_t）体积较小（分别为 8 字节和 12 字节），直接复制数据更简单可靠
//邮箱本身不具备缓存能力（仅存储最新一条消息），若处理线程处理速度慢于采集线程，会导致旧数据被覆盖，无法满足 “不丢失数据” 的需求。

/* 消息队列：原始数据队列和处理后的数据队列 */
static rt_mq_t raw_data_queue = RT_NULL;
static rt_mq_t processed_data_queue = RT_NULL;


/* 信号量 */
static rt_sem_t display_sem = RT_NULL;

/* 互斥量保护共享资源 */
static rt_mutex_t history_mutex = RT_NULL;

/* 全局数据 */
static history_data_t g_history_data = {0};//所有成员初始化为0
//static processed_data_t g_current_data = {0};//存储当前处理后的数据
static minute_accumulator_t g_minute_acc = {0}; // 一分钟数据累计器
static rt_bool_t client_connected = RT_FALSE;  // 客户端连接状态
static int flag=1;
/* EEPROM起始地址定义 */
#define EEPROM_HISTORY_ADDR     0x00

/* 获取历史数据用于绘图（考虑循环缓冲区的顺序） */
//参数是指针，会同步改变原参的值
static void get_ordered_history_data(rt_uint8_t *light_data, float *temp_data, rt_uint8_t count)
{
    rt_uint8_t i;
    rt_uint8_t start_index;
    
    if (count == HISTORY_DATA_COUNT)
    {
        /* 缓冲区已满，从最老的数据开始（实现左移效果） */
			//例如第11次写入数据时（这个数据的索引值是0），索引值为1（写入后，索引自增，对应数组2号位），2号位为最早记录的数据
        start_index = g_history_data.write_index;
    }
    else
    {
        /* 缓冲区未满，从第一个数据开始 */
        start_index = 0;
    }
    
    /* 按时间顺序重新排列数据 */
    for (i = 0; i < count; i++)
    {
        rt_uint8_t index = (start_index + i) % HISTORY_DATA_COUNT;
        light_data[i] = g_history_data.light_history[index];
        temp_data[i] = g_history_data.temp_history[index];
    }
}

/* 数据采集任务 */
static void data_collect_task(void *parameter)
{   
    sensor_data_t sensor_data;
    rt_uint8_t k;
    rt_kprintf("Data collect task started\n");
    
    while (1)
    {    
        /* 读取传感器数据 */
        sensor_data.timestamp = rt_tick_get();
        sensor_data.light_value = Lsens_Get_Val();           // 读取光敏传感器
        sensor_data.temperature = DS18B20_GetTemperture();   // 读取温度传感器

			  k=(100-sensor_data.light_value)*500/100;//光照强度归一化到0-500区间
		    TIM_SetCompare2(TIM3,k);//k的值直接写入 TIM3 的 CCR2 寄存器，k的增减变化反应PWM波的占空比（看例程容易理解）
        /* 发送数据到消息队列 */
        if (rt_mq_send(raw_data_queue, &sensor_data, sizeof(sensor_data_t)) != RT_EOK)
        {
            rt_kprintf("Raw data queue send failed\n");
        }
        else
        {
            printf("Collected: Light=%d%%, Temp=%.2f°C\n", 
                       sensor_data.light_value, 
                       sensor_data.temperature);
        }
        
        /* 等待采集周期 */
        rt_thread_delay(1000);
    }
}

/* 数据处理任务 */
static void data_process_task(void *parameter)
{
    sensor_data_t raw_data;
    processed_data_t proc_data;
    static rt_uint8_t light_buffer[5] = {0};
    static float temp_buffer[5] = {0};//存储5个数据的缓冲区
    static rt_uint8_t buffer_index = 0;
    rt_uint32_t light_sum = 0;
    float temp_sum = 0;
    rt_uint8_t i;
		
		 char tcp_buffer[128];
    char send_cmd[160];
		
    rt_kprintf("Data process task started\n");
    
    //初始化一分钟累计器 ，返回自系统启动以来经过的滴答数（tick）。
    g_minute_acc.start_time = rt_tick_get();
    
    while (1)
    {
        //从消息队列接收原始数据 ，消息队列发送的是数据本身，这不是指针
        //消息队列：基于数据复制（发送接收，前后两次复制），消息队列创建后，系统会分配两个空间，即消息缓冲区和消息队列控制区
			
        if (rt_mq_recv(raw_data_queue, &raw_data, sizeof(sensor_data_t), RT_WAITING_FOREVER) == RT_EOK)
        {
            /* 将数据存入滑动窗口缓冲区 */
            light_buffer[buffer_index] = raw_data.light_value;
            temp_buffer[buffer_index] = raw_data.temperature;
            buffer_index = (buffer_index + 1) % 5;
            
            /* 计算平均值（简单滤波） */
            light_sum = 0;
            temp_sum = 0;
            for (i = 0; i < 5; i++)
            {
                light_sum += light_buffer[i];
                temp_sum += temp_buffer[i];
            }
            
            proc_data.timestamp = raw_data.timestamp;
            proc_data.light_avg = light_sum / 5;
            proc_data.temp_avg = temp_sum / 5.0;
            proc_data.data_valid = 1;
						
            client_connected=ESP8266_SendCmd("AT+CIPSTATUS","STATUS:",500);
            if (client_connected)
            {
                /* 格式化数据 */
                sprintf(tcp_buffer, 
							"temperature:%.2f,light:%d\r\n",
                          proc_data.temp_avg, 
                          proc_data.light_avg 
                         );
                
                /* 发送数据到客户端 */
                rt_sprintf(send_cmd, "AT+CIPSEND=0,%d", rt_strlen(tcp_buffer));
                //ESP8266 在收到 AT+CIPSEND 指令并确认长度合法后，会返回 > 作为应答，此时才能发送实际数据。
                if (ESP8266_SendCmd(send_cmd, ">", 2000))
                {
									//SEND OK是模块本身规定的返回值
                    if (ESP8266_SendData(tcp_buffer, "SEND OK", 2000))
                    {
                        rt_kprintf("Data sent: %s", tcp_buffer);
                    }
                    else
                    {
                        rt_kprintf("Data send failed\n");
                        client_connected = RT_FALSE;
                    }
                }
                else
                {
                    rt_kprintf("CIPSEND command failed\n");
                    client_connected = RT_FALSE;
                }
            }
            
            /* 累计一分钟内的数据 */
            g_minute_acc.light_sum += proc_data.light_avg;
            g_minute_acc.temp_sum += proc_data.temp_avg;
            g_minute_acc.count++;
            
            /* 发送处理后的数据到显示队列 */
            if (rt_mq_send(processed_data_queue, &proc_data, sizeof(processed_data_t)) != RT_EOK)
            {
                rt_kprintf("Processed data queue send failed\n");
            }
            
            /* 通知显示任务更新 */
            rt_sem_release(display_sem);
            
            printf("Processed: Light_avg=%d%%, Temp_avg=%.2f°C\n", 
                       proc_data.light_avg, 
                       proc_data.temp_avg);
        }
        rt_thread_delay(1000);
    }
}

/* 显示任务 */
static void display_task(void *parameter)
{
    processed_data_t display_data;
    rt_uint8_t key_val;
    char str_buf[32];
char p[32];
char *k;
	
    rt_kprintf("Display task started\n");
    
    while (1)
    {
			ESP8266_GetRecvData(p,32);
			rt_kprintf(p);
			k=strstr(p, "4321");
        //检查按键,按键检测不灵敏的原因，除了线程调度，高优先级的抢占外，还有在本线程中有一个超时等待时间
        key_val = KEY_Scan(0);
//字符串相等时，返回0
			//||(k!=RT_NULL)  加入这个条件判断，可以通过labview发送数据到STM32进行菜单控制
			//现在的问题是加入这个判断，会导致温度数据读取异常，可能影响了时序
        if (key_val == KEY1_PRESS||(k!=RT_NULL))
        {   
            LCD_Clear(WHITE);
					 memset(p, 0,32);
            flag=flag+1;
            if(flag>4) flag=1;
        }
        /* 显示标题 */
        LCD_ShowString(10, 10, 200, 16, 16, "Sensor Monitor System");
        LCD_ShowString(10, 30, 200, 16, 16, "KEY1:Mode KEY,Next");
        if (client_connected)
				LCD_ShowString(200, 10, 200, 16, 16, "TCP_connected");
				else LCD_ShowString(200, 10, 200, 16, 16, "TCP_connected_failed");
        /* 显示当前模式 */
        switch(flag)
        {
            case 1:
                LCD_ShowString(10, 50, 200, 16, 16, "Mode: Light Real-time");
                break;
            case 2:
                LCD_ShowString(10, 50, 200, 16, 16, "Mode: Temp Real-time");
                break;
            case 3:
                LCD_ShowString(10, 50, 200, 16, 16, "Mode: Light History");
                break;
            case 4:
                LCD_ShowString(10, 50, 200, 16, 16, "Mode: Temp History");
                break;
        }
        
        //等待数据更新信号或超时,rt_tick_from_millisecond，将毫秒时间转换为系统滴答数（tick），即等待超时时间
        if (rt_sem_take(display_sem, rt_tick_from_millisecond(DISPLAY_UPDATE_PERIOD)) == RT_EOK)
        {
            //接收处理后的数据，使用 RT_WAITING_NO 参数指定非阻塞模式，即若队列为空则立即返回
            if (rt_mq_recv(processed_data_queue, &display_data, sizeof(processed_data_t), RT_WAITING_NO) == RT_EOK)
            {
                switch(flag)
                {
                    case 1:
                        /* 显示实时光照数据 */
                        LCD_ShowString(10, 80, 200, 16, 16, "Light Intensity:");
                        sprintf(str_buf, "%d Lux    ", display_data.light_avg);
                        LCD_ShowString(10, 110, 200, 24, 24, str_buf);
                        
                        sprintf(str_buf, "Time: %ds  ", display_data.timestamp/1000);
                        LCD_ShowString(10, 150, 200, 16, 16, str_buf);
                        break;
                        
                    case 2:
                        /* 显示实时温度数据 */
                        LCD_ShowString(10, 80, 200, 16, 16, "Temperature:");
                        sprintf(str_buf, "%.1f    ", display_data.temp_avg);
                        LCD_ShowString(10, 110, 200, 24, 24, str_buf);
                        LCD_ShowFontHZ(60,110,"℃");
                    
                        sprintf(str_buf, "Time: %ds  ", display_data.timestamp/1000);
                        LCD_ShowString(10, 150, 200, 16, 16, str_buf);
                        break;
                        
                    case 3:
                        /* 显示光照历史曲线 */
                        Draw_Light_History_Curve();
                        break;
                        
                    case 4:
                        /* 显示温度历史曲线 */
                        Draw_Temp_History_Curve();
                        break;
                }
            }
        }
    }
}

/* 存储管理任务 */
static void storage_task(void *parameter)
{
    rt_uint32_t current_time;
    
    rt_kprintf("Storage task started\n");
    
    while (1)
    {
        current_time = rt_tick_get();//当前系统tick数
        
        //检查是否到了一分钟保存周期，启动时间在数据处理线程初始化，在下面进行更新赋值
        if ((current_time - g_minute_acc.start_time) >= rt_tick_from_millisecond(HISTORY_SAVE_PERIOD))
        {
            if (g_minute_acc.count > 0)
            {
                /* 获取互斥量保护历史数据 */
                rt_mutex_take(history_mutex, RT_WAITING_FOREVER);
                
                /* 计算过去一分钟的平均值并保存到历史缓冲区 */
                g_history_data.light_history[g_history_data.write_index] = g_minute_acc.light_sum / g_minute_acc.count;
                g_history_data.temp_history[g_history_data.write_index] = g_minute_acc.temp_sum / g_minute_acc.count;
                g_history_data.timestamp = current_time;
                
                g_history_data.write_index = (g_history_data.write_index + 1) % HISTORY_DATA_COUNT;
							//在这个写入函数中，当10个数据已满，会从索引0开始覆盖
                if (g_history_data.data_count < HISTORY_DATA_COUNT)
                {
                    g_history_data.data_count++;
                }
                
                //保存到EEPROM， g_history_data 结构体中的数据写入到 AT24CXX 系列 EEPROM 芯片的指定地址。
                AT24CXX_Write(EEPROM_HISTORY_ADDR, (rt_uint8_t*)&g_history_data, sizeof(history_data_t));
                
                rt_mutex_release(history_mutex);
                
                rt_kprintf("History data saved, count: %d, light_avg: %d, temp_avg: %.2f\n", 
                          g_history_data.data_count,
                          g_minute_acc.light_sum / g_minute_acc.count,
                          g_minute_acc.temp_sum / g_minute_acc.count);
                
                /* 重置累计器 */
                g_minute_acc.light_sum = 0;
                g_minute_acc.temp_sum = 0;
                g_minute_acc.count = 0;
                g_minute_acc.start_time = current_time;
            }
        }
        
        rt_thread_delay(60000);
    }
}

/* 绘制光照历史数据曲线 */
static void Draw_Light_History_Curve(void)
{
    char str_buf[32];
    rt_uint16_t i;
    rt_uint16_t x1, y1, x2, y2;
    rt_uint8_t light_max = 0, light_min = 100;
    rt_uint8_t light_range;
    rt_uint8_t ordered_light_data[HISTORY_DATA_COUNT];
    float ordered_temp_data[HISTORY_DATA_COUNT];  // 虽然不用，但函数需要
    
    if (g_history_data.data_count < 2) 
    {
        LCD_ShowString(50, 120, 200, 16, 16, "No enough data");
        return;
    }
    
    /* 获取互斥量保护历史数据 */
    rt_mutex_take(history_mutex, RT_WAITING_FOREVER);
    
    /* 获取按时间顺序排列的数据（实现左移效果） */
    get_ordered_history_data(ordered_light_data, ordered_temp_data, g_history_data.data_count);
    
    /* 清除曲线区域 */
    LCD_Fill(10, 80, 310, 230, WHITE);
    
    /* 显示标题 */
    LCD_ShowString(10, 80, 200, 16, 16, "Light History (10min)");
    
    /* 找出实际数据范围 */
    for (i = 0; i < g_history_data.data_count; i++)
    {
        if (ordered_light_data[i] > light_max) 
            light_max = ordered_light_data[i];
        if (ordered_light_data[i] < light_min) 
            light_min = ordered_light_data[i];
    }   
    /* 确保范围合理 */
    if (light_max > 100) light_max = 100;
    if (light_min < 0) light_min = 0;
    
    light_range = light_max - light_min;
    if (light_range == 0) light_range = 1;
    
    /* 绘制坐标轴（原点位置30，210） */
    LCD_DrawLine_Color(30, 110, 30, 210, BLACK);     // Y轴
    LCD_DrawLine_Color(30, 210, 300, 210, BLACK);    // X轴
    
    /* 绘制光照强度曲线 */
    for (i = 0; i < g_history_data.data_count - 1; i++)
    {
        // 计算点的位置
        //30：X 轴起点偏移；270：X 轴有效宽度（300-30）；g_history_data.data_count - 1：数据点间隔数（N 个点有 N-1 个间隔）
        //效果：数据点在 X 轴上等间距分布
        x1 = 30 + i * 270 / (g_history_data.data_count - 1);
        //210：Y 轴起点偏移（原点位置）；100：Y 轴有效高度（210-110）
        //(ordered_light_data[i] - light_min) / light_range，把数据归一化到0-1区间
        y1 = 210 - ((ordered_light_data[i] - light_min) * 100) / light_range;
        x2 = 30 + (i + 1) * 270 / (g_history_data.data_count - 1);
        y2 = 210 - ((ordered_light_data[i + 1] - light_min) * 100) / light_range;
        
        /* 确保坐标在合理范围内 */
        if (y1 < 110) y1 = 110;
        if (y1 > 210) y1 = 210;
        if (y2 < 110) y2 = 110;
        if (y2 > 210) y2 = 210;
        
        /* 绘制连接线 */
        LCD_DrawLine_Color(x1, y1, x2, y2, BLUE);
        
        /* 绘制数据点 */
        LCD_Fill(x1-1, y1-1, x1+2, y1+2, RED);  // 画数据点
        
        /* 为最后一个点也画上标记，i == g_history_data.data_count - 2时绘制最后一条线段的终点，前面是绘制起点 */
        if (i == g_history_data.data_count - 2)  // 最后一个线段
        {
            LCD_Fill(x2-1, y2-1, x2+2, y2+2, RED); 
        }
    }
  
    /* 显示Y轴数值范围 */
    sprintf(str_buf, "%d", light_max);
    LCD_ShowString(5, 105, 25, 12, 12, str_buf);
    sprintf(str_buf, "%d", light_min);
    LCD_ShowString(5, 205, 25, 12, 12, str_buf);
    
    /* 显示单位和数据信息 */
    LCD_ShowString(5, 220, 50, 12, 12, "Lux");
    sprintf(str_buf, "Points: %d/%d", g_history_data.data_count, HISTORY_DATA_COUNT);
    LCD_ShowString(200, 230, 100, 12, 12, str_buf);
    
    /* 显示最新数值 */
    sprintf(str_buf, "Latest: %d", ordered_light_data[g_history_data.data_count - 1]);
    LCD_ShowString(50, 100, 120, 12, 12, str_buf);
    rt_mutex_release(history_mutex);
}

/* 绘制温度历史数据曲线 */
static void Draw_Temp_History_Curve(void)
{
    char str_buf[32];
    rt_uint16_t i;
    rt_uint16_t x1, y1, x2, y2;
    float temp_max = -50.0, temp_min = 100.0;
    float temp_range;
    rt_uint8_t ordered_light_data[HISTORY_DATA_COUNT];  // 两个数组的值，在函数调用内部赋
    float ordered_temp_data[HISTORY_DATA_COUNT];
    
    if (g_history_data.data_count < 2) 
    {
        LCD_ShowString(50, 120, 200, 16, 16, "No enough data");
        return;
    }
    
    /* 获取互斥量保护历史数据 */
    rt_mutex_take(history_mutex, RT_WAITING_FOREVER);
    
    /* 获取按时间顺序排列的数据（实现左移效果） */
    get_ordered_history_data(ordered_light_data, ordered_temp_data, g_history_data.data_count);
    
    /* 清除曲线区域 */
    LCD_Fill(10, 80, 310, 230, WHITE);
    
    /* 显示标题 */
    LCD_ShowString(10, 80, 200, 16, 16, "Temp History (10min)");
    
    /* 找出实际数据范围 */
    for (i = 0; i < g_history_data.data_count; i++)
    {
        if (ordered_temp_data[i] > temp_max) 
            temp_max = ordered_temp_data[i];
        if (ordered_temp_data[i] < temp_min) 
            temp_min = ordered_temp_data[i];
    }
    
    temp_range = temp_max - temp_min;
    if (temp_range < 0.1) temp_range = 0.1;  // 避免除零
    
    /* 绘制坐标轴 */
    LCD_DrawLine_Color(30, 110, 30, 210, BLACK);     // Y轴
    LCD_DrawLine_Color(30, 210, 300, 210, BLACK);    // X轴
    
    /* 绘制温度曲线 */
    for (i = 0; i < g_history_data.data_count - 1; i++)
    {
        // 计算点的位置
        x1 = 30 + i * 270 / (g_history_data.data_count - 1);
        y1 = 210 - (rt_uint16_t)((ordered_temp_data[i] - temp_min) * 100.0 / temp_range);
        x2 = 30 + (i + 1) * 270 / (g_history_data.data_count - 1);
        y2 = 210 - (rt_uint16_t)((ordered_temp_data[i + 1] - temp_min) * 100.0 / temp_range);
        
        /* 确保坐标在合理范围内 */
        if (y1 < 110) y1 = 110;
        if (y1 > 210) y1 = 210;
        if (y2 < 110) y2 = 110;
        if (y2 > 210) y2 = 210;
        
        /* 绘制连接线 */
        LCD_DrawLine_Color(x1, y1, x2, y2, BLUE);
        
        /* 绘制数据点 */
        LCD_Fill(x1-1, y1-1, x1+2, y1+2, RED);  // 画数据点（稍微大一点）
        
        /* 为最后一个点也画上标记 */
        if (i == g_history_data.data_count - 2)  // 最后一个线段
        {
            LCD_Fill(x2-1, y2-1, x2+2, y2+2, RED); 
        }
    }
    
    /* 显示Y轴数值范围 */
    sprintf(str_buf, "%.1f", temp_max);
    LCD_ShowString(5, 105, 25, 12, 12, str_buf);
    sprintf(str_buf, "%.1f", temp_min);
    LCD_ShowString(5, 205, 25, 12, 12, str_buf);
    
    /* 显示单位和数据信息 */
    LCD_ShowString(5, 220, 50, 12, 12, "C");
    sprintf(str_buf, "Points: %d/%d", g_history_data.data_count, HISTORY_DATA_COUNT);
    LCD_ShowString(200, 230, 100, 12, 12, str_buf);
    
    /* 显示最新数值 */
    sprintf(str_buf, "Latest: %.1f", ordered_temp_data[g_history_data.data_count - 1]);
    LCD_ShowString(50, 100, 120, 12, 12, str_buf);
		LCD_ShowString(30, 230, 100, 50, 12, "Temp_Max_Set:29");
    if(ordered_temp_data[g_history_data.data_count - 1]>29)
		LCD_Fill(130, 230, 140, 240, RED); 
		else LCD_Fill(130, 230, 140, 240, GRAY); 
    rt_mutex_release(history_mutex);
}



/* 系统初始化函数 */
int main(void)
{
    /* 硬件初始化 */
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    LED_Init();
    KEY_Init();
    USART1_Init(115200);
    Lsens_Init();
    TFTLCD_Init();	
    TIM3_CH2_PWM_Init(500,72-1); 
    ESP8266_Init();
	  esp8266_init();
	
    while(DS18B20_Init())
    {
        rt_kprintf("DS18B20 Error\n");
        rt_thread_delay(200);
    }

    rt_kprintf("DS18B20 Ready\n");
    
    /* 初始化EEPROM并读取历史数据 */
    AT24CXX_Init();
    AT24CXX_Read(EEPROM_HISTORY_ADDR, (rt_uint8_t*)&g_history_data, sizeof(history_data_t));
    
    /* 简单验证读取的数据是否合理 */
    if (g_history_data.data_count > HISTORY_DATA_COUNT)
    {
        rt_kprintf("EEPROM data invalid, resetting\n");
        rt_memset(&g_history_data, 0, sizeof(history_data_t));
    }
    else
    {
        rt_kprintf("History data loaded, count: %d\n", g_history_data.data_count);
    }

    //创建消息队列 ，消息深度为10，先进先出模式
    raw_data_queue = rt_mq_create("raw_data", sizeof(sensor_data_t), 10, RT_IPC_FLAG_FIFO);
    processed_data_queue = rt_mq_create("proc_data", sizeof(processed_data_t), 10, RT_IPC_FLAG_FIFO);
    
    if (!raw_data_queue || !processed_data_queue)
    {
        rt_kprintf("Message queue create failed\n");
        return -1;
    }
    
    /* 创建信号量 */
    display_sem = rt_sem_create("display", 0, RT_IPC_FLAG_FIFO);
    
    if (!display_sem)
    {
        rt_kprintf("Semaphore create failed\n");
        return -1;
    }
    
    /* 创建互斥量 */
    history_mutex = rt_mutex_create("history", RT_IPC_FLAG_FIFO);
    if (!history_mutex)
    {
        rt_kprintf("Mutex create failed\n");
        return -1;
    }
    
    /* 创建任务 */
    data_collect_thread = rt_thread_create("data_collect",
                                           data_collect_task,
                                           RT_NULL,
                                           512,
                                           10,
                                           20);
    
    data_process_thread = rt_thread_create("data_process",
                                           data_process_task,
                                           RT_NULL,
                                          768,
                                           11,
                                           20);
    
    display_thread = rt_thread_create("display",
                                      display_task,
                                      RT_NULL,
                                      768,
                                      12,
                                      20);
    
    storage_thread = rt_thread_create("storage",
                                      storage_task,
                                      RT_NULL,
                                      512,
                                      11,
                                      20);
	
 //除去存储历史数据线程外，其余线程栈空间分配为256均会导致显示卡死，硬件错误 
//当更改历史数据上限为20时，256栈空间会不足； 导致线程卡死
    if (!data_collect_thread || !data_process_thread || !display_thread || !storage_thread)
    {
        rt_kprintf("Thread create failed\n");
        return -1;
    }
    
    /* 启动任务 */
    rt_thread_startup(data_collect_thread);
    rt_thread_startup(data_process_thread);
    rt_thread_startup(display_thread);
    rt_thread_startup(storage_thread);
    
    rt_kprintf("Sensor system initialized successfully\n");
}