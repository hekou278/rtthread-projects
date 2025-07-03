// data_type.h
#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

#include "system.h"
#include "rtthread.h"

/* ϵͳ���ò��� */
#define DATA_PROCESS_PERIOD     1000    // ���ݴ������� 1��
#define DISPLAY_UPDATE_PERIOD   500     // ��ʾ�������� 0.5��
#define HISTORY_SAVE_PERIOD     60000   // ��ʷ���ݱ������� 1����
#define HISTORY_DATA_COUNT      10      // ����10������ʷ����

/* ���������ݽṹ */
typedef struct {
    rt_uint32_t timestamp;       // ʱ���
    rt_uint8_t light_value;      // ����������ֵ (0-100)
    float temperature;           // �¶�ֵ
} sensor_data_t;

/* ���������ݽṹ */
typedef struct {
    rt_uint32_t timestamp;
    rt_uint8_t light_avg;        // ����ƽ��ֵ
    float temp_avg;              // �¶�ƽ��ֵ
    rt_uint8_t data_valid;       // ������Ч��־
} processed_data_t;

/* ��ʷ���ݽṹ */
typedef struct {
    rt_uint32_t timestamp;
    rt_uint8_t light_history[HISTORY_DATA_COUNT];
    float temp_history[HISTORY_DATA_COUNT];
    rt_uint8_t data_count;   //�Ѵ洢��������0-10
    rt_uint8_t write_index;  //д������0-9��ʼ��ָ����һ����д���λ������
} history_data_t;

/* һ�����������ۼƽṹ */
typedef struct {
    rt_uint32_t light_sum;
    float temp_sum;
    rt_uint8_t count;//����
    rt_uint32_t start_time;//����ʱ��
} minute_accumulator_t;



#endif
