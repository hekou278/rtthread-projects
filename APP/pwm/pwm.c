#include "pwm.h"
//PWM波的关键是占空比变化表示数字信号
/*******************************************************************************
* 函 数 名         : TIM3_CH2_PWM_Init
* 函数功能		   : TIM3通道2 PWM初始化函数
* 输    入         : per:重装载值
					 psc:分频系数
* 输    出         : 无
*******************************************************************************/
void TIM3_CH2_PWM_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
//定义三个结构体分别用于：定时器基本配置、输出比较配置、GPIO 配置	
	/* 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//将 TIM3 通道 2 的输出从默认引脚（PA7）重映射到 GPIOB.5
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);//改变指定管脚的映射	
	//定时器时钟频率 = APB1 时钟频率 × 2 / (psc + 1)
	//PWM 周期 = (per + 1) × 定时器时钟周期
	//频率：由预分频器（PSC）和自动重装载值（ARR）决定,  f_PWM = 定时器时钟频率（72MHZ） / ((PSC + 1) × (ARR + 1))
	//两个值都是这个初始化函数的输入参数
	TIM_TimeBaseInitStructure.TIM_Period=per;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);	
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//PWM 模式 1（向上计数时，计数器值小于 CCR2 时输出有效电平）
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;//低电平有效（TIM_OCPolarity_Low），低电平亮
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure); //输出比较通道2初始化
	
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable); //使能TIMx在 CCR2 上的预装载寄存器
	TIM_ARRPreloadConfig(TIM3,ENABLE);//使能预装载寄存器
	
	TIM_Cmd(TIM3,ENABLE); //使能定时器
		
}


