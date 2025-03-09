/**
  ******************************************************************************
  * @file    KEY.c
  * @brief   按键驱动模块实现
  ******************************************************************************
  */

#include "KEY.h"
#include "delay.h"

/* 按键状态结构体 */
typedef struct {
    uint8_t current;          // 当前状态
    uint8_t last;             // 上次状态
    uint8_t count;            // 按下计数器
    uint8_t debounce_count;   // 消抖计数器
    uint8_t event;            // 按键事件
    uint8_t long_pressed;     // 长按标志
} KEY_Status_TypeDef;

/* 按键状态数组 */
static KEY_Status_TypeDef key_status[6]; // 索引0不使用，1-5对应5个按键

/**
  * @brief  按键初始化函数
  * @param  无
  * @retval 无
  */
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;
    
    /* 使能按键对应的GPIO时钟 */
    RCC_APB2PeriphClockCmd(KEY1_RCC | KEY2_RCC | KEY3_RCC | KEY4_RCC | KEY5_RCC, ENABLE);
    
    /* 配置按键1 - 模式选择按键 */
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY1_PORT, &GPIO_InitStructure);
    
    /* 配置按键2 - 开始/停止按键 */
    GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
    GPIO_Init(KEY2_PORT, &GPIO_InitStructure);
    
    /* 配置按键3 - 角度增加按键 */
    GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
    GPIO_Init(KEY3_PORT, &GPIO_InitStructure);
    
    /* 配置按键4 - 角度减少按键 */
    GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
    GPIO_Init(KEY4_PORT, &GPIO_InitStructure);
    
    /* 配置按键5 - 确认/返回按键 */
    GPIO_InitStructure.GPIO_Pin = KEY5_PIN;
    GPIO_Init(KEY5_PORT, &GPIO_InitStructure);
    
    /* 初始化按键状态 */
    for (i = 0; i < 6; i++) {
        key_status[i].current = KEY_RELEASED;
        key_status[i].last = KEY_RELEASED;
        key_status[i].count = 0;
        key_status[i].debounce_count = 0;
        key_status[i].event = KEY_EVENT_NONE;
        key_status[i].long_pressed = 0;
    }
}

/**
  * @brief  读取按键物理状态
  * @param  key: 按键编号（1-5）
  * @retval uint8_t: 按键状态，KEY_PRESSED 或 KEY_RELEASED
  */
static uint8_t KEY_ReadPin(uint8_t key)
{
    uint8_t state = KEY_RELEASED;
    
    switch (key) {
        case KEY_MODE:
            state = GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN);
            break;
        case KEY_START_STOP:
            state = GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN);
            break;
        case KEY_UP:
            state = GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN);
            break;
        case KEY_DOWN:
            state = GPIO_ReadInputDataBit(KEY4_PORT, KEY4_PIN);
            break;
        case KEY_ENTER:
            state = GPIO_ReadInputDataBit(KEY5_PORT, KEY5_PIN);
            break;
        default:
            break;
    }
    
    return state;
}

/**
  * @brief  按键扫描函数
  * @param  无
  * @retval uint8_t: 返回按键值
  *         0表示无按键，1-5表示对应的按键
  * @note   应在定时器中断中调用
  */
uint8_t KEY_Scan(void)
{
    uint8_t key_value = KEY_NONE;
    uint8_t i;
    uint8_t current_state;
    
    /* 扫描所有按键 */
    for (i = 1; i <= 5; i++) {
        /* 读取当前按键状态 */
        current_state = KEY_ReadPin(i);
        
        /* 按键状态变化 */
        if (current_state != key_status[i].current) {
            /* 消抖计数 */
            key_status[i].debounce_count++;
            
            if (key_status[i].debounce_count >= KEY_DEBOUNCE_TIME) {
                /* 消抖完成，更新按键状态 */
                key_status[i].last = key_status[i].current;
                key_status[i].current = current_state;
                key_status[i].debounce_count = 0;
                
                /* 按键按下 */
                if (key_status[i].current == KEY_PRESSED) {
                    key_value = i;
                    key_status[i].count = 0;
                    key_status[i].long_pressed = 0;
                    key_status[i].event = KEY_EVENT_SHORT;
                }
                /* 按键释放 */
                else {
                    if (key_status[i].long_pressed) {
                        key_status[i].event = KEY_EVENT_NONE;
                    }
                }
            }
        }
        else {
            /* 重置消抖计数器 */
            key_status[i].debounce_count = 0;
            
            /* 按键持续按下 */
            if (key_status[i].current == KEY_PRESSED) {
                key_status[i].count++;
                
                /* 长按判断 */
                if (key_status[i].count >= KEY_LONG_TIME && !key_status[i].long_pressed) {
                    key_status[i].event = KEY_EVENT_LONG;
                    key_status[i].long_pressed = 1;
                    key_value = i;
                }
                /* 连按判断 */
                else if (key_status[i].count >= KEY_LONG_TIME + KEY_REPEAT_TIME && key_status[i].long_pressed) {
                    key_status[i].event = KEY_EVENT_REPEAT;
                    key_status[i].count = KEY_LONG_TIME; // 重置计数，实现连按
                    key_value = i;
                }
            }
        }
    }
    
    return key_value;
}

/**
  * @brief  检测按键事件
  * @param  key: 按键值（1-5）
  * @retval uint8_t: 按键事件类型
  */
uint8_t KEY_GetEvent(uint8_t key)
{
    uint8_t event;
    
    if (key < 1 || key > 5) {
        return KEY_EVENT_NONE;
    }
    
    event = key_status[key].event;
    key_status[key].event = KEY_EVENT_NONE; // 读取后清除事件
    
    return event;
}

/**
  * @brief  检查指定按键是否按下
  * @param  key: 按键值（1-5）
  * @retval uint8_t: 1表示按下，0表示未按下
  */
uint8_t KEY_IsPressed(uint8_t key)
{
    if (key < 1 || key > 5) {
        return 0;
    }
    
    return (key_status[key].current == KEY_PRESSED) ? 1 : 0;
}

/**
  * @brief  复位按键状态
  * @param  无
  * @retval 无
  */
void KEY_Reset(void)
{
    uint8_t i;
    
    for (i = 1; i <= 5; i++) {
        key_status[i].count = 0;
        key_status[i].event = KEY_EVENT_NONE;
        key_status[i].long_pressed = 0;
    }
}
