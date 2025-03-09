/**
  ******************************************************************************
  * @file    KEY.h
  * @brief   按键驱动模块头文件
  ******************************************************************************
  */

#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

/* 按键引脚定义 */
#define KEY1_PORT       GPIOB
#define KEY1_PIN        GPIO_Pin_1   // MODE按键 - 模式选择，从PA0改为PB1
#define KEY1_RCC        RCC_APB2Periph_GPIOB  // 时钟也需要修改

#define KEY2_PORT       GPIOB
#define KEY2_PIN        GPIO_Pin_2   // START/STOP按键，从PA1改为PB2
#define KEY2_RCC        RCC_APB2Periph_GPIOB

// 其他按键定义保持不变
#define KEY3_PORT       GPIOA
#define KEY3_PIN        GPIO_Pin_4   // UP按键
#define KEY3_RCC        RCC_APB2Periph_GPIOA

#define KEY4_PORT       GPIOA
#define KEY4_PIN        GPIO_Pin_5   // DOWN按键
#define KEY4_RCC        RCC_APB2Periph_GPIOA

#define KEY5_PORT       GPIOB
#define KEY5_PIN        GPIO_Pin_0   // ENTER/BACK按键
#define KEY5_RCC        RCC_APB2Periph_GPIOB

// ...其余代码保持不变...
/* 按键值定义 */
#define KEY_NONE        0   // 无按键按下
#define KEY_MODE        1   // 模式按键
#define KEY_START_STOP  2   // 开始/停止按键
#define KEY_UP          3   // 向上按键
#define KEY_DOWN        4   // 向下按键
#define KEY_ENTER       5   // 确认按键

/* 按键状态定义 */
#define KEY_PRESSED     0   // STM32按键按下为低电平
#define KEY_RELEASED    1   // STM32按键释放为高电平

/* 按键事件定义 */
#define KEY_EVENT_NONE      0   // 无事件
#define KEY_EVENT_SHORT     1   // 短按
#define KEY_EVENT_LONG      2   // 长按
#define KEY_EVENT_REPEAT    3   // 连续按下

/* 按键参数定义 */
#define KEY_LONG_TIME       100 // 长按时间（单位：10ms）
#define KEY_REPEAT_TIME     20  // 连按时间（单位：10ms）
#define KEY_SCAN_INTERVAL   2   // 扫描间隔（单位：ms）
#define KEY_DEBOUNCE_TIME   2   // 消抖时间（单位：扫描次数）

/* 函数声明 */
/**
  * @brief  按键初始化函数
  * @param  无
  * @retval 无
  */
void KEY_Init(void);

/**
  * @brief  按键扫描函数
  * @param  无
  * @retval uint8_t: 返回按键值
  *         0表示无按键，1-5表示对应的按键
  * @note   应在定时器中断中调用
  */
uint8_t KEY_Scan(void);

/**
  * @brief  检测按键事件
  * @param  无
  * @retval uint8_t: 按键事件类型
  *         KEY_EVENT_NONE: 无事件
  *         KEY_EVENT_SHORT: 短按事件
  *         KEY_EVENT_LONG: 长按事件
  *         KEY_EVENT_REPEAT: 连按事件
  */
uint8_t KEY_GetEvent(uint8_t key);

/**
  * @brief  检查指定按键是否按下
  * @param  key: 按键值
  * @retval uint8_t: 1表示按下，0表示未按下
  */
uint8_t KEY_IsPressed(uint8_t key);

/**
  * @brief  复位按键状态
  * @param  无
  * @retval 无
  */
void KEY_Reset(void);

#endif /* __KEY_H */
