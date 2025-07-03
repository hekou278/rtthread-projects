// data_type.h
#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

#include "system.h"
#include "rtthread.h"

/* 系统配置参数 */
#define DATA_PROCESS_PERIOD     1000    // 数据处理周期 1秒
#define DISPLAY_UPDATE_PERIOD   500     // 显示更新周期 0.5秒
#define HISTORY_SAVE_PERIOD     60000   // 历史数据保存周期 1分钟
#define HISTORY_DATA_COUNT      10      // 保存10分钟历史数据

/* 传感器数据结构 */
typedef struct {
    rt_uint32_t timestamp;       // 时间戳
    rt_uint8_t light_value;      // 光敏传感器值 (0-100)
    float temperature;           // 温度值
} sensor_data_t;

/* 处理后的数据结构 */
typedef struct {
    rt_uint32_t timestamp;
    rt_uint8_t light_avg;        // 光照平均值
    float temp_avg;              // 温度平均值
    rt_uint8_t data_valid;       // 数据有效标志
} processed_data_t;

/* 历史数据结构 */
typedef struct {
    rt_uint32_t timestamp;
    rt_uint8_t light_history[HISTORY_DATA_COUNT];
    float temp_history[HISTORY_DATA_COUNT];
    rt_uint8_t data_count;   //已存储数据数量0-10
    rt_uint8_t write_index;  //写入索引0-9，始终指向下一个待写入的位置索引
} history_data_t;

/* 一分钟内数据累计结构 */
typedef struct {
    rt_uint32_t light_sum;
    float temp_sum;
    rt_uint8_t count;//计数
    rt_uint32_t start_time;//启动时间
} minute_accumulator_t;



#endif
