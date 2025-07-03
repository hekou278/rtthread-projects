#include <string.h>
//rtthread����ϵͳ���
#include <rtthread.h>
//stm32��׼�����
#include <system.h>
#include "SysTick.h"
//����
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

/* ȫ�ֱ��� */
static rt_thread_t data_collect_thread = RT_NULL;
static rt_thread_t data_process_thread = RT_NULL;
static rt_thread_t display_thread = RT_NULL;
static rt_thread_t storage_thread = RT_NULL;


//����ȫ�ֱ�������Ƶ������������������еĻ����������ҿ������̵߳��ȵ������ݲ�һ��
//�������䣺�����ʺϴ���ָ�루���ַ�������������ַ��
//����ǰ���ݽṹ��sensor_data_t��processed_data_t�������С���ֱ�Ϊ 8 �ֽں� 12 �ֽڣ���ֱ�Ӹ������ݸ��򵥿ɿ�
//���䱾���߱��������������洢����һ����Ϣ�����������̴߳����ٶ����ڲɼ��̣߳��ᵼ�¾����ݱ����ǣ��޷����� ������ʧ���ݡ� ������

/* ��Ϣ���У�ԭʼ���ݶ��кʹ��������ݶ��� */
static rt_mq_t raw_data_queue = RT_NULL;
static rt_mq_t processed_data_queue = RT_NULL;


/* �ź��� */
static rt_sem_t display_sem = RT_NULL;

/* ����������������Դ */
static rt_mutex_t history_mutex = RT_NULL;

/* ȫ������ */
static history_data_t g_history_data = {0};//���г�Ա��ʼ��Ϊ0
//static processed_data_t g_current_data = {0};//�洢��ǰ����������
static minute_accumulator_t g_minute_acc = {0}; // һ���������ۼ���
static rt_bool_t client_connected = RT_FALSE;  // �ͻ�������״̬
static int flag=1;
/* EEPROM��ʼ��ַ���� */
#define EEPROM_HISTORY_ADDR     0x00

/* ��ȡ��ʷ�������ڻ�ͼ������ѭ����������˳�� */
//������ָ�룬��ͬ���ı�ԭ�ε�ֵ
static void get_ordered_history_data(rt_uint8_t *light_data, float *temp_data, rt_uint8_t count)
{
    rt_uint8_t i;
    rt_uint8_t start_index;
    
    if (count == HISTORY_DATA_COUNT)
    {
        /* �����������������ϵ����ݿ�ʼ��ʵ������Ч���� */
			//�����11��д������ʱ��������ݵ�����ֵ��0��������ֵΪ1��д���������������Ӧ����2��λ����2��λΪ�����¼������
        start_index = g_history_data.write_index;
    }
    else
    {
        /* ������δ�����ӵ�һ�����ݿ�ʼ */
        start_index = 0;
    }
    
    /* ��ʱ��˳�������������� */
    for (i = 0; i < count; i++)
    {
        rt_uint8_t index = (start_index + i) % HISTORY_DATA_COUNT;
        light_data[i] = g_history_data.light_history[index];
        temp_data[i] = g_history_data.temp_history[index];
    }
}

/* ���ݲɼ����� */
static void data_collect_task(void *parameter)
{   
    sensor_data_t sensor_data;
    rt_uint8_t k;
    rt_kprintf("Data collect task started\n");
    
    while (1)
    {    
        /* ��ȡ���������� */
        sensor_data.timestamp = rt_tick_get();
        sensor_data.light_value = Lsens_Get_Val();           // ��ȡ����������
        sensor_data.temperature = DS18B20_GetTemperture();   // ��ȡ�¶ȴ�����

			  k=(100-sensor_data.light_value)*500/100;//����ǿ�ȹ�һ����0-500����
		    TIM_SetCompare2(TIM3,k);//k��ֱֵ��д�� TIM3 �� CCR2 �Ĵ�����k�������仯��ӦPWM����ռ�ձȣ�������������⣩
        /* �������ݵ���Ϣ���� */
        if (rt_mq_send(raw_data_queue, &sensor_data, sizeof(sensor_data_t)) != RT_EOK)
        {
            rt_kprintf("Raw data queue send failed\n");
        }
        else
        {
            printf("Collected: Light=%d%%, Temp=%.2f��C\n", 
                       sensor_data.light_value, 
                       sensor_data.temperature);
        }
        
        /* �ȴ��ɼ����� */
        rt_thread_delay(1000);
    }
}

/* ���ݴ������� */
static void data_process_task(void *parameter)
{
    sensor_data_t raw_data;
    processed_data_t proc_data;
    static rt_uint8_t light_buffer[5] = {0};
    static float temp_buffer[5] = {0};//�洢5�����ݵĻ�����
    static rt_uint8_t buffer_index = 0;
    rt_uint32_t light_sum = 0;
    float temp_sum = 0;
    rt_uint8_t i;
		
		 char tcp_buffer[128];
    char send_cmd[160];
		
    rt_kprintf("Data process task started\n");
    
    //��ʼ��һ�����ۼ��� ��������ϵͳ�������������ĵδ�����tick����
    g_minute_acc.start_time = rt_tick_get();
    
    while (1)
    {
        //����Ϣ���н���ԭʼ���� ����Ϣ���з��͵������ݱ����ⲻ��ָ��
        //��Ϣ���У��������ݸ��ƣ����ͽ��գ�ǰ�����θ��ƣ�����Ϣ���д�����ϵͳ����������ռ䣬����Ϣ����������Ϣ���п�����
			
        if (rt_mq_recv(raw_data_queue, &raw_data, sizeof(sensor_data_t), RT_WAITING_FOREVER) == RT_EOK)
        {
            /* �����ݴ��뻬�����ڻ����� */
            light_buffer[buffer_index] = raw_data.light_value;
            temp_buffer[buffer_index] = raw_data.temperature;
            buffer_index = (buffer_index + 1) % 5;
            
            /* ����ƽ��ֵ�����˲��� */
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
                /* ��ʽ������ */
                sprintf(tcp_buffer, 
							"temperature:%.2f,light:%d\r\n",
                          proc_data.temp_avg, 
                          proc_data.light_avg 
                         );
                
                /* �������ݵ��ͻ��� */
                rt_sprintf(send_cmd, "AT+CIPSEND=0,%d", rt_strlen(tcp_buffer));
                //ESP8266 ���յ� AT+CIPSEND ָ�ȷ�ϳ��ȺϷ��󣬻᷵�� > ��ΪӦ�𣬴�ʱ���ܷ���ʵ�����ݡ�
                if (ESP8266_SendCmd(send_cmd, ">", 2000))
                {
									//SEND OK��ģ�鱾��涨�ķ���ֵ
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
            
            /* �ۼ�һ�����ڵ����� */
            g_minute_acc.light_sum += proc_data.light_avg;
            g_minute_acc.temp_sum += proc_data.temp_avg;
            g_minute_acc.count++;
            
            /* ���ʹ��������ݵ���ʾ���� */
            if (rt_mq_send(processed_data_queue, &proc_data, sizeof(processed_data_t)) != RT_EOK)
            {
                rt_kprintf("Processed data queue send failed\n");
            }
            
            /* ֪ͨ��ʾ������� */
            rt_sem_release(display_sem);
            
            printf("Processed: Light_avg=%d%%, Temp_avg=%.2f��C\n", 
                       proc_data.light_avg, 
                       proc_data.temp_avg);
        }
        rt_thread_delay(1000);
    }
}

/* ��ʾ���� */
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
        //��鰴��,������ⲻ������ԭ�򣬳����̵߳��ȣ������ȼ�����ռ�⣬�����ڱ��߳�����һ����ʱ�ȴ�ʱ��
        key_val = KEY_Scan(0);
//�ַ������ʱ������0
			//||(k!=RT_NULL)  ������������жϣ�����ͨ��labview�������ݵ�STM32���в˵�����
			//���ڵ������Ǽ�������жϣ��ᵼ���¶����ݶ�ȡ�쳣������Ӱ����ʱ��
        if (key_val == KEY1_PRESS||(k!=RT_NULL))
        {   
            LCD_Clear(WHITE);
					 memset(p, 0,32);
            flag=flag+1;
            if(flag>4) flag=1;
        }
        /* ��ʾ���� */
        LCD_ShowString(10, 10, 200, 16, 16, "Sensor Monitor System");
        LCD_ShowString(10, 30, 200, 16, 16, "KEY1:Mode KEY,Next");
        if (client_connected)
				LCD_ShowString(200, 10, 200, 16, 16, "TCP_connected");
				else LCD_ShowString(200, 10, 200, 16, 16, "TCP_connected_failed");
        /* ��ʾ��ǰģʽ */
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
        
        //�ȴ����ݸ����źŻ�ʱ,rt_tick_from_millisecond��������ʱ��ת��Ϊϵͳ�δ�����tick�������ȴ���ʱʱ��
        if (rt_sem_take(display_sem, rt_tick_from_millisecond(DISPLAY_UPDATE_PERIOD)) == RT_EOK)
        {
            //���մ��������ݣ�ʹ�� RT_WAITING_NO ����ָ��������ģʽ����������Ϊ������������
            if (rt_mq_recv(processed_data_queue, &display_data, sizeof(processed_data_t), RT_WAITING_NO) == RT_EOK)
            {
                switch(flag)
                {
                    case 1:
                        /* ��ʾʵʱ�������� */
                        LCD_ShowString(10, 80, 200, 16, 16, "Light Intensity:");
                        sprintf(str_buf, "%d Lux    ", display_data.light_avg);
                        LCD_ShowString(10, 110, 200, 24, 24, str_buf);
                        
                        sprintf(str_buf, "Time: %ds  ", display_data.timestamp/1000);
                        LCD_ShowString(10, 150, 200, 16, 16, str_buf);
                        break;
                        
                    case 2:
                        /* ��ʾʵʱ�¶����� */
                        LCD_ShowString(10, 80, 200, 16, 16, "Temperature:");
                        sprintf(str_buf, "%.1f    ", display_data.temp_avg);
                        LCD_ShowString(10, 110, 200, 24, 24, str_buf);
                        LCD_ShowFontHZ(60,110,"��");
                    
                        sprintf(str_buf, "Time: %ds  ", display_data.timestamp/1000);
                        LCD_ShowString(10, 150, 200, 16, 16, str_buf);
                        break;
                        
                    case 3:
                        /* ��ʾ������ʷ���� */
                        Draw_Light_History_Curve();
                        break;
                        
                    case 4:
                        /* ��ʾ�¶���ʷ���� */
                        Draw_Temp_History_Curve();
                        break;
                }
            }
        }
    }
}

/* �洢�������� */
static void storage_task(void *parameter)
{
    rt_uint32_t current_time;
    
    rt_kprintf("Storage task started\n");
    
    while (1)
    {
        current_time = rt_tick_get();//��ǰϵͳtick��
        
        //����Ƿ���һ���ӱ������ڣ�����ʱ�������ݴ����̳߳�ʼ������������и��¸�ֵ
        if ((current_time - g_minute_acc.start_time) >= rt_tick_from_millisecond(HISTORY_SAVE_PERIOD))
        {
            if (g_minute_acc.count > 0)
            {
                /* ��ȡ������������ʷ���� */
                rt_mutex_take(history_mutex, RT_WAITING_FOREVER);
                
                /* �����ȥһ���ӵ�ƽ��ֵ�����浽��ʷ������ */
                g_history_data.light_history[g_history_data.write_index] = g_minute_acc.light_sum / g_minute_acc.count;
                g_history_data.temp_history[g_history_data.write_index] = g_minute_acc.temp_sum / g_minute_acc.count;
                g_history_data.timestamp = current_time;
                
                g_history_data.write_index = (g_history_data.write_index + 1) % HISTORY_DATA_COUNT;
							//�����д�뺯���У���10�������������������0��ʼ����
                if (g_history_data.data_count < HISTORY_DATA_COUNT)
                {
                    g_history_data.data_count++;
                }
                
                //���浽EEPROM�� g_history_data �ṹ���е�����д�뵽 AT24CXX ϵ�� EEPROM оƬ��ָ����ַ��
                AT24CXX_Write(EEPROM_HISTORY_ADDR, (rt_uint8_t*)&g_history_data, sizeof(history_data_t));
                
                rt_mutex_release(history_mutex);
                
                rt_kprintf("History data saved, count: %d, light_avg: %d, temp_avg: %.2f\n", 
                          g_history_data.data_count,
                          g_minute_acc.light_sum / g_minute_acc.count,
                          g_minute_acc.temp_sum / g_minute_acc.count);
                
                /* �����ۼ��� */
                g_minute_acc.light_sum = 0;
                g_minute_acc.temp_sum = 0;
                g_minute_acc.count = 0;
                g_minute_acc.start_time = current_time;
            }
        }
        
        rt_thread_delay(60000);
    }
}

/* ���ƹ�����ʷ�������� */
static void Draw_Light_History_Curve(void)
{
    char str_buf[32];
    rt_uint16_t i;
    rt_uint16_t x1, y1, x2, y2;
    rt_uint8_t light_max = 0, light_min = 100;
    rt_uint8_t light_range;
    rt_uint8_t ordered_light_data[HISTORY_DATA_COUNT];
    float ordered_temp_data[HISTORY_DATA_COUNT];  // ��Ȼ���ã���������Ҫ
    
    if (g_history_data.data_count < 2) 
    {
        LCD_ShowString(50, 120, 200, 16, 16, "No enough data");
        return;
    }
    
    /* ��ȡ������������ʷ���� */
    rt_mutex_take(history_mutex, RT_WAITING_FOREVER);
    
    /* ��ȡ��ʱ��˳�����е����ݣ�ʵ������Ч���� */
    get_ordered_history_data(ordered_light_data, ordered_temp_data, g_history_data.data_count);
    
    /* ����������� */
    LCD_Fill(10, 80, 310, 230, WHITE);
    
    /* ��ʾ���� */
    LCD_ShowString(10, 80, 200, 16, 16, "Light History (10min)");
    
    /* �ҳ�ʵ�����ݷ�Χ */
    for (i = 0; i < g_history_data.data_count; i++)
    {
        if (ordered_light_data[i] > light_max) 
            light_max = ordered_light_data[i];
        if (ordered_light_data[i] < light_min) 
            light_min = ordered_light_data[i];
    }   
    /* ȷ����Χ���� */
    if (light_max > 100) light_max = 100;
    if (light_min < 0) light_min = 0;
    
    light_range = light_max - light_min;
    if (light_range == 0) light_range = 1;
    
    /* ���������ᣨԭ��λ��30��210�� */
    LCD_DrawLine_Color(30, 110, 30, 210, BLACK);     // Y��
    LCD_DrawLine_Color(30, 210, 300, 210, BLACK);    // X��
    
    /* ���ƹ���ǿ������ */
    for (i = 0; i < g_history_data.data_count - 1; i++)
    {
        // ������λ��
        //30��X �����ƫ�ƣ�270��X ����Ч��ȣ�300-30����g_history_data.data_count - 1�����ݵ�������N ������ N-1 �������
        //Ч�������ݵ��� X ���ϵȼ��ֲ�
        x1 = 30 + i * 270 / (g_history_data.data_count - 1);
        //210��Y �����ƫ�ƣ�ԭ��λ�ã���100��Y ����Ч�߶ȣ�210-110��
        //(ordered_light_data[i] - light_min) / light_range�������ݹ�һ����0-1����
        y1 = 210 - ((ordered_light_data[i] - light_min) * 100) / light_range;
        x2 = 30 + (i + 1) * 270 / (g_history_data.data_count - 1);
        y2 = 210 - ((ordered_light_data[i + 1] - light_min) * 100) / light_range;
        
        /* ȷ�������ں���Χ�� */
        if (y1 < 110) y1 = 110;
        if (y1 > 210) y1 = 210;
        if (y2 < 110) y2 = 110;
        if (y2 > 210) y2 = 210;
        
        /* ���������� */
        LCD_DrawLine_Color(x1, y1, x2, y2, BLUE);
        
        /* �������ݵ� */
        LCD_Fill(x1-1, y1-1, x1+2, y1+2, RED);  // �����ݵ�
        
        /* Ϊ���һ����Ҳ���ϱ�ǣ�i == g_history_data.data_count - 2ʱ�������һ���߶ε��յ㣬ǰ���ǻ������ */
        if (i == g_history_data.data_count - 2)  // ���һ���߶�
        {
            LCD_Fill(x2-1, y2-1, x2+2, y2+2, RED); 
        }
    }
  
    /* ��ʾY����ֵ��Χ */
    sprintf(str_buf, "%d", light_max);
    LCD_ShowString(5, 105, 25, 12, 12, str_buf);
    sprintf(str_buf, "%d", light_min);
    LCD_ShowString(5, 205, 25, 12, 12, str_buf);
    
    /* ��ʾ��λ��������Ϣ */
    LCD_ShowString(5, 220, 50, 12, 12, "Lux");
    sprintf(str_buf, "Points: %d/%d", g_history_data.data_count, HISTORY_DATA_COUNT);
    LCD_ShowString(200, 230, 100, 12, 12, str_buf);
    
    /* ��ʾ������ֵ */
    sprintf(str_buf, "Latest: %d", ordered_light_data[g_history_data.data_count - 1]);
    LCD_ShowString(50, 100, 120, 12, 12, str_buf);
    rt_mutex_release(history_mutex);
}

/* �����¶���ʷ�������� */
static void Draw_Temp_History_Curve(void)
{
    char str_buf[32];
    rt_uint16_t i;
    rt_uint16_t x1, y1, x2, y2;
    float temp_max = -50.0, temp_min = 100.0;
    float temp_range;
    rt_uint8_t ordered_light_data[HISTORY_DATA_COUNT];  // ���������ֵ���ں��������ڲ���
    float ordered_temp_data[HISTORY_DATA_COUNT];
    
    if (g_history_data.data_count < 2) 
    {
        LCD_ShowString(50, 120, 200, 16, 16, "No enough data");
        return;
    }
    
    /* ��ȡ������������ʷ���� */
    rt_mutex_take(history_mutex, RT_WAITING_FOREVER);
    
    /* ��ȡ��ʱ��˳�����е����ݣ�ʵ������Ч���� */
    get_ordered_history_data(ordered_light_data, ordered_temp_data, g_history_data.data_count);
    
    /* ����������� */
    LCD_Fill(10, 80, 310, 230, WHITE);
    
    /* ��ʾ���� */
    LCD_ShowString(10, 80, 200, 16, 16, "Temp History (10min)");
    
    /* �ҳ�ʵ�����ݷ�Χ */
    for (i = 0; i < g_history_data.data_count; i++)
    {
        if (ordered_temp_data[i] > temp_max) 
            temp_max = ordered_temp_data[i];
        if (ordered_temp_data[i] < temp_min) 
            temp_min = ordered_temp_data[i];
    }
    
    temp_range = temp_max - temp_min;
    if (temp_range < 0.1) temp_range = 0.1;  // �������
    
    /* ���������� */
    LCD_DrawLine_Color(30, 110, 30, 210, BLACK);     // Y��
    LCD_DrawLine_Color(30, 210, 300, 210, BLACK);    // X��
    
    /* �����¶����� */
    for (i = 0; i < g_history_data.data_count - 1; i++)
    {
        // ������λ��
        x1 = 30 + i * 270 / (g_history_data.data_count - 1);
        y1 = 210 - (rt_uint16_t)((ordered_temp_data[i] - temp_min) * 100.0 / temp_range);
        x2 = 30 + (i + 1) * 270 / (g_history_data.data_count - 1);
        y2 = 210 - (rt_uint16_t)((ordered_temp_data[i + 1] - temp_min) * 100.0 / temp_range);
        
        /* ȷ�������ں���Χ�� */
        if (y1 < 110) y1 = 110;
        if (y1 > 210) y1 = 210;
        if (y2 < 110) y2 = 110;
        if (y2 > 210) y2 = 210;
        
        /* ���������� */
        LCD_DrawLine_Color(x1, y1, x2, y2, BLUE);
        
        /* �������ݵ� */
        LCD_Fill(x1-1, y1-1, x1+2, y1+2, RED);  // �����ݵ㣨��΢��һ�㣩
        
        /* Ϊ���һ����Ҳ���ϱ�� */
        if (i == g_history_data.data_count - 2)  // ���һ���߶�
        {
            LCD_Fill(x2-1, y2-1, x2+2, y2+2, RED); 
        }
    }
    
    /* ��ʾY����ֵ��Χ */
    sprintf(str_buf, "%.1f", temp_max);
    LCD_ShowString(5, 105, 25, 12, 12, str_buf);
    sprintf(str_buf, "%.1f", temp_min);
    LCD_ShowString(5, 205, 25, 12, 12, str_buf);
    
    /* ��ʾ��λ��������Ϣ */
    LCD_ShowString(5, 220, 50, 12, 12, "C");
    sprintf(str_buf, "Points: %d/%d", g_history_data.data_count, HISTORY_DATA_COUNT);
    LCD_ShowString(200, 230, 100, 12, 12, str_buf);
    
    /* ��ʾ������ֵ */
    sprintf(str_buf, "Latest: %.1f", ordered_temp_data[g_history_data.data_count - 1]);
    LCD_ShowString(50, 100, 120, 12, 12, str_buf);
		LCD_ShowString(30, 230, 100, 50, 12, "Temp_Max_Set:29");
    if(ordered_temp_data[g_history_data.data_count - 1]>29)
		LCD_Fill(130, 230, 140, 240, RED); 
		else LCD_Fill(130, 230, 140, 240, GRAY); 
    rt_mutex_release(history_mutex);
}



/* ϵͳ��ʼ������ */
int main(void)
{
    /* Ӳ����ʼ�� */
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
    
    /* ��ʼ��EEPROM����ȡ��ʷ���� */
    AT24CXX_Init();
    AT24CXX_Read(EEPROM_HISTORY_ADDR, (rt_uint8_t*)&g_history_data, sizeof(history_data_t));
    
    /* ����֤��ȡ�������Ƿ���� */
    if (g_history_data.data_count > HISTORY_DATA_COUNT)
    {
        rt_kprintf("EEPROM data invalid, resetting\n");
        rt_memset(&g_history_data, 0, sizeof(history_data_t));
    }
    else
    {
        rt_kprintf("History data loaded, count: %d\n", g_history_data.data_count);
    }

    //������Ϣ���� ����Ϣ���Ϊ10���Ƚ��ȳ�ģʽ
    raw_data_queue = rt_mq_create("raw_data", sizeof(sensor_data_t), 10, RT_IPC_FLAG_FIFO);
    processed_data_queue = rt_mq_create("proc_data", sizeof(processed_data_t), 10, RT_IPC_FLAG_FIFO);
    
    if (!raw_data_queue || !processed_data_queue)
    {
        rt_kprintf("Message queue create failed\n");
        return -1;
    }
    
    /* �����ź��� */
    display_sem = rt_sem_create("display", 0, RT_IPC_FLAG_FIFO);
    
    if (!display_sem)
    {
        rt_kprintf("Semaphore create failed\n");
        return -1;
    }
    
    /* ���������� */
    history_mutex = rt_mutex_create("history", RT_IPC_FLAG_FIFO);
    if (!history_mutex)
    {
        rt_kprintf("Mutex create failed\n");
        return -1;
    }
    
    /* �������� */
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
	
 //��ȥ�洢��ʷ�����߳��⣬�����߳�ջ�ռ����Ϊ256���ᵼ����ʾ������Ӳ������ 
//��������ʷ��������Ϊ20ʱ��256ջ�ռ�᲻�㣻 �����߳̿���
    if (!data_collect_thread || !data_process_thread || !display_thread || !storage_thread)
    {
        rt_kprintf("Thread create failed\n");
        return -1;
    }
    
    /* �������� */
    rt_thread_startup(data_collect_thread);
    rt_thread_startup(data_process_thread);
    rt_thread_startup(display_thread);
    rt_thread_startup(storage_thread);
    
    rt_kprintf("Sensor system initialized successfully\n");
}