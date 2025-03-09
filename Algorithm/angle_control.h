/**
  ******************************************************************************
  * @file    angle_control.h
  * @brief   风力板角度控制系统模块头文件
  ******************************************************************************
  */

#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#include "stm32f10x.h"
#include "pid_controller.h" 
#include "fan_driver.h"
#include "angle_sensor.h"

/* 控制系统工作模式 */
typedef enum {
    CONTROL_MODE_IDLE = 0,       // 空闲模式（不控制）
    CONTROL_MODE_SINGLE_FAN = 1, // 单风扇控制
    CONTROL_MODE_DUAL_FAN = 2,   // 双风扇控制
    CONTROL_MODE_SEQUENCE = 3    // 角度序列控制
} ControlMode_TypeDef;

/* 角度控制状态 */
typedef enum {
    ANGLE_STATE_INIT = 0,       // 初始化状态
    ANGLE_STATE_ADJUSTING = 1,  // 调整中
    ANGLE_STATE_STABLE = 2,     // 已稳定
    ANGLE_STATE_ERROR = 3       // 控制错误
} AngleState_TypeDef;

/* 角度序列控制配置 */
typedef struct {
    float angles[10];           // 角度序列
    uint8_t hold_times[10];     // 各角度保持时间(秒)
    uint8_t angle_count;        // 角度数量
    uint8_t current_index;      // 当前角度索引
    uint32_t start_time;        // 开始时间
    uint32_t stable_start_time; // 稳定开始时间
} AngleSequence_TypeDef;

/* 角度控制配置 */
typedef struct {
    ControlMode_TypeDef mode;    // 控制模式
    float target_angle;          // 目标角度
    float current_angle;         // 当前角度
    float allowed_error;         // 允许误差(角度稳定判定)
    uint16_t stable_time;        // 需要保持稳定的时间(ms)
    uint32_t stable_start_time;  // 稳定开始时间
    
    PID_TypeDef pid;             // PID控制器
    
    uint8_t fan_base_speed;      // 风扇基础速度(%)
    uint8_t dual_mode_ratio;     // 双风扇模式下的差速比例(%)
    
    AngleState_TypeDef state;    // 当前控制状态
    uint32_t system_time;        // 系统时间(ms)
    uint32_t last_update_time;   // 上次更新时间
    
    /* 序列控制设置 */
    AngleSequence_TypeDef sequence;
} AngleControl_TypeDef;

/* 函数声明 */

/**
  * @brief  角度控制系统初始化
  * @param  control: 角度控制结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void ANGLE_CONTROL_Init(AngleControl_TypeDef *control, ControlMode_TypeDef mode);

/**
  * @brief  设置目标角度
  * @param  control: 角度控制结构体指针
  * @param  angle: 目标角度(度)
  * @retval 无
  */
void ANGLE_CONTROL_SetTarget(AngleControl_TypeDef *control, float angle);

/**
  * @brief  设置控制模式
  * @param  control: 角度控制结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void ANGLE_CONTROL_SetMode(AngleControl_TypeDef *control, ControlMode_TypeDef mode);

/**
  * @brief  设置PID参数
  * @param  control: 角度控制结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @retval 无
  */
void ANGLE_CONTROL_SetPID(AngleControl_TypeDef *control, float kp, float ki, float kd);

/**
  * @brief  设置允许误差和稳定时间
  * @param  control: 角度控制结构体指针
  * @param  error: 允许误差(度)
  * @param  time: 需要稳定的时间(ms)
  * @retval 无
  */
void ANGLE_CONTROL_SetStableCondition(AngleControl_TypeDef *control, float error, uint16_t time);

/**
  * @brief  角度控制主循环，在定时器中断中调用
  * @param  control: 角度控制结构体指针
  * @retval 无
  */
void ANGLE_CONTROL_Process(AngleControl_TypeDef *control);

/**
  * @brief  判断角度是否已稳定在目标位置
  * @param  control: 角度控制结构体指针
  * @retval uint8_t: 1表示已稳定，0表示未稳定
  */
uint8_t ANGLE_CONTROL_IsStable(AngleControl_TypeDef *control);

/**
  * @brief  配置角度序列控制
  * @param  control: 角度控制结构体指针
  * @param  angles: 角度序列数组
  * @param  hold_times: 各角度保持时间数组(秒)
  * @param  count: 角度数量
  * @retval 无
  */
void ANGLE_CONTROL_ConfigSequence(AngleControl_TypeDef *control, float *angles, uint8_t *hold_times, uint8_t count);

/**
  * @brief  开始角度序列控制
  * @param  control: 角度控制结构体指针
  * @retval 无
  */
void ANGLE_CONTROL_StartSequence(AngleControl_TypeDef *control);

/**
  * @brief  设置风扇基础速度和比例
  * @param  control: 角度控制结构体指针
  * @param  base_speed: 风扇基础速度(%)
  * @param  ratio: 双风扇模式下的差速比例(%)
  * @retval 无
  */
void ANGLE_CONTROL_SetFanParameters(AngleControl_TypeDef *control, uint8_t base_speed, uint8_t ratio);

/**
  * @brief  停止所有控制
  * @param  control: 角度控制结构体指针
  * @retval 无
  */
void ANGLE_CONTROL_Stop(AngleControl_TypeDef *control);

/**
  * @brief  获取控制系统运行时间(ms)
  * @param  无
  * @retval uint32_t: 系统运行时间
  */
uint32_t ANGLE_CONTROL_GetTime(void);

/**
  * @brief  SysTick中断处理函数，更新系统时间
  * @param  无
  * @retval 无
  * @note   需要在stm32f10x_it.c中调用
  */
void ANGLE_CONTROL_TimeUpdate(void);


#endif /* __ANGLE_CONTROL_H */
