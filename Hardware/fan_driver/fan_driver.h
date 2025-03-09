/**
  ******************************************************************************
  * @file    fan_driver.h
  * @brief   风扇驱动模块头文件
  ******************************************************************************
  */

#ifndef __FAN_DRIVER_H
#define __FAN_DRIVER_H

#include "stm32f10x.h"

/* 风扇方向定义 */
typedef enum {
    FAN_DIR_FORWARD = 0,  // 正方向
    FAN_DIR_REVERSE = 1   // 反方向
} FanDirection_TypeDef;

/* 风扇编号定义 */
typedef enum {
    FAN_LEFT = 0,  // 左侧风扇
    FAN_RIGHT = 1  // 右侧风扇
} FanSelect_TypeDef;

/* 风扇控制相关宏定义 */
#define FAN_PWM_FREQ         1000                  // PWM频率 (Hz)
#define FAN_MAX_DUTY         100                   // PWM最大占空比 (%)
#define FAN_PWM_PERIOD       (SystemCoreClock/FAN_PWM_FREQ) // PWM周期

/* 风扇GPIO定义 - 可根据实际硬件修改 */
// 左风扇
#define FAN_LEFT_PWM_PORT    GPIOA
#define FAN_LEFT_PWM_PIN     GPIO_Pin_1           // TIM2_CH2
#define FAN_LEFT_IN1_PORT    GPIOB
#define FAN_LEFT_IN1_PIN     GPIO_Pin_10
#define FAN_LEFT_IN2_PORT    GPIOB
#define FAN_LEFT_IN2_PIN     GPIO_Pin_11

// 右风扇
#define FAN_RIGHT_PWM_PORT   GPIOA
#define FAN_RIGHT_PWM_PIN    GPIO_Pin_2           // TIM2_CH3
#define FAN_RIGHT_IN1_PORT   GPIOB
#define FAN_RIGHT_IN1_PIN    GPIO_Pin_12
#define FAN_RIGHT_IN2_PORT   GPIOB
#define FAN_RIGHT_IN2_PIN    GPIO_Pin_13

// STBY引脚（可选）
#define FAN_STBY_PORT        GPIOB
#define FAN_STBY_PIN         GPIO_Pin_14

/* 函数声明 */
void FAN_Init(void);                              // 初始化风扇控制模块
void FAN_SetSpeed(FanSelect_TypeDef fan, uint8_t speed); // 设置风扇速度
void FAN_SetDirection(FanSelect_TypeDef fan, FanDirection_TypeDef direction); // 设置风扇方向
void FAN_Start(FanSelect_TypeDef fan);            // 启动风扇
void FAN_Stop(FanSelect_TypeDef fan);             // 停止风扇
void FAN_StartAll(void);                          // 启动所有风扇
void FAN_StopAll(void);                           // 停止所有风扇
void FAN_SetDualSpeed(uint8_t left_speed, uint8_t right_speed); // 同时设置两个风扇速度

#endif /* __FAN_DRIVER_H */
