/**
  ******************************************************************************
  * @file    angle_control.c
  * @brief   风力板角度控制系统模块实现
  ******************************************************************************
  */

#include "angle_control.h"
#include "delay.h"
#include <math.h>
#include <stdio.h>

/* 控制参数默认值 */
#define DEFAULT_KP               10.0f    // 默认比例系数
#define DEFAULT_KI               0.5f     // 默认积分系数
#define DEFAULT_KD               1.0f     // 默认微分系数
#define DEFAULT_ALLOWED_ERROR    5.0f     // 默认允许误差 ±5°
#define DEFAULT_STABLE_TIME      3000     // 默认稳定时间 3秒

#define DEFAULT_FAN_BASE_SPEED   50       // 默认风扇基础速度 50%
#define DEFAULT_DUAL_MODE_RATIO  30       // 默认双风扇差速比例 30%

#define ANGLE_CONTROL_INTERVAL   10       // 控制循环间隔(ms)

/* 私有变量 */
static volatile uint32_t g_system_time = 0;  // 系统时间，由SysTick中断更新

/* 私有函数声明 */
static void ANGLE_CONTROL_UpdateTime(AngleControl_TypeDef *control);
static void ANGLE_CONTROL_ProcessSingleFan(AngleControl_TypeDef *control);
static void ANGLE_CONTROL_ProcessDualFan(AngleControl_TypeDef *control);
static void ANGLE_CONTROL_ProcessSequence(AngleControl_TypeDef *control);

/**
  * @brief  角度控制系统初始化
  * @param  control: 角度控制结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void ANGLE_CONTROL_Init(AngleControl_TypeDef *control, ControlMode_TypeDef mode)
{
    /* 初始化控制结构体 */
    control->mode = mode;
    control->target_angle = 0.0f;
    control->current_angle = 0.0f;
    control->allowed_error = DEFAULT_ALLOWED_ERROR;
    control->stable_time = DEFAULT_STABLE_TIME;
    control->stable_start_time = 0;
    control->fan_base_speed = DEFAULT_FAN_BASE_SPEED;
    control->dual_mode_ratio = DEFAULT_DUAL_MODE_RATIO;
    control->state = ANGLE_STATE_INIT;
    control->system_time = 0;
    control->last_update_time = 0;
    
    /* 初始化PID控制器 */
    PID_Init(&control->pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, PID_MODE_POSITION, 0.01f);
    PID_SetOutputLimits(&control->pid, -100.0f, 100.0f);
    
    /* 序列控制初始化 */
    control->sequence.angle_count = 0;
    control->sequence.current_index = 0;
    
    /* 初始化风扇驱动 */
    FAN_Init();
    FAN_StopAll();
    
    printf("Angle control system initialized, mode: %d\r\n", mode);
}

/**
  * @brief  设置目标角度
  * @param  control: 角度控制结构体指针
  * @param  angle: 目标角度(度)
  * @retval 无
  */
void ANGLE_CONTROL_SetTarget(AngleControl_TypeDef *control, float angle)
{
    /* 限制角度范围 */
    if (angle > 90.0f) angle = 90.0f;
    if (angle < -90.0f) angle = -90.0f;
    
    /* 更新目标角度 */
    control->target_angle = angle;
    PID_SetPoint(&control->pid, angle);
    
    /* 重置稳定状态 */
    control->state = ANGLE_STATE_ADJUSTING;
    control->stable_start_time = 0;
    
    printf("Target angle set to %.1f degrees\r\n", angle);
}

/**
  * @brief  设置控制模式
  * @param  control: 角度控制结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void ANGLE_CONTROL_SetMode(AngleControl_TypeDef *control, ControlMode_TypeDef mode)
{
    if (control->mode != mode) {
        /* 切换模式前停止所有风扇 */
        FAN_StopAll();
        
        /* 更新模式 */
        control->mode = mode;
        
        /* 重置控制状态 */
        control->state = ANGLE_STATE_INIT;
        PID_Reset(&control->pid);
        
        printf("Control mode changed to %d\r\n", mode);
    }
}

/**
  * @brief  设置PID参数
  * @param  control: 角度控制结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @retval 无
  */
void ANGLE_CONTROL_SetPID(AngleControl_TypeDef *control, float kp, float ki, float kd)
{
    PID_Tune(&control->pid, kp, ki, kd);
    printf("PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
}

/**
  * @brief  设置允许误差和稳定时间
  * @param  control: 角度控制结构体指针
  * @param  error: 允许误差(度)
  * @param  time: 需要稳定的时间(ms)
  * @retval 无
  */
void ANGLE_CONTROL_SetStableCondition(AngleControl_TypeDef *control, float error, uint16_t time)
{
    control->allowed_error = error;
    control->stable_time = time;
    printf("Stable condition updated: Error=%.1f degrees, Time=%d ms\r\n", error, time);
}

/**
  * @brief  角度控制主循环，在定时器中断中调用
  * @param  control: 角度控制结构体指针
  * @retval 无
  */
void ANGLE_CONTROL_Process(AngleControl_TypeDef *control)
{
    float error;
    
    /* 更新系统时间 */
    ANGLE_CONTROL_UpdateTime(control);
    
    /* 检查是否达到控制间隔 */
    if ((control->system_time - control->last_update_time) < ANGLE_CONTROL_INTERVAL) {
        return;
    }
    control->last_update_time = control->system_time;
    
    /* 获取当前角度 */
    control->current_angle = ANGLE_SENSOR_GetAngle();
    
    /* 根据控制模式进行处理 */
    switch (control->mode) {
        case CONTROL_MODE_IDLE:
            /* 空闲模式，停止所有风扇 */
            FAN_StopAll();
            control->state = ANGLE_STATE_INIT;
            break;
        
        case CONTROL_MODE_SINGLE_FAN:
            /* 单风扇控制 */
            ANGLE_CONTROL_ProcessSingleFan(control);
            break;
        
        case CONTROL_MODE_DUAL_FAN:
            /* 双风扇控制 */
            ANGLE_CONTROL_ProcessDualFan(control);
            break;
        
        case CONTROL_MODE_SEQUENCE:
            /* 序列控制 */
            ANGLE_CONTROL_ProcessSequence(control);
            break;
        
        default:
            /* 未知模式，停止所有风扇 */
            FAN_StopAll();
            control->state = ANGLE_STATE_ERROR;
            break;
    }
    
    /* 检查是否稳定 */
    error = fabs(control->target_angle - control->current_angle);
    
    if (error <= control->allowed_error) {
        /* 在允许误差范围内 */
        if (control->state != ANGLE_STATE_STABLE) {
            /* 第一次进入稳定状态，记录开始时间 */
            if (control->stable_start_time == 0) {
                control->stable_start_time = control->system_time;
            }
            
            /* 检查是否保持足够长的时间 */
            if ((control->system_time - control->stable_start_time) >= control->stable_time) {
                /* 稳定状态确认 */
                control->state = ANGLE_STATE_STABLE;
                printf("Angle stable at %.1f degrees\r\n", control->current_angle);
            }
        }
    } else {
        /* 不在允许误差范围内，重置稳定计时 */
        control->stable_start_time = 0;
        control->state = ANGLE_STATE_ADJUSTING;
    }
}

/**
  * @brief  单风扇控制处理
  * @param  control: 角度控制结构体指针
  * @retval 无
  * @note   私有函数
  */
static void ANGLE_CONTROL_ProcessSingleFan(AngleControl_TypeDef *control)
{
    float pid_output;
    uint8_t speed;
    
    /* 计算PID输出 */
    pid_output = PID_Calculate(&control->pid, control->current_angle);
    
    /* 
     * 单风扇控制逻辑：
     * 1. 当需要正角度时(顺时针)，使用右风扇
     * 2. 当需要负角度时(逆时针)，使用左风扇
     */
    
    /* 根据目标角度决定使用哪个风扇 */
    if (control->target_angle >= 0.0f) {
        /* 目标角度为正，使用右风扇 */
        speed = (uint8_t)(fabs(pid_output));
        if (speed > 100) speed = 100;
        
        FAN_SetSpeed(FAN_RIGHT, speed);
        FAN_SetSpeed(FAN_LEFT, 0);
        
        /* 设置风扇方向 */
        FAN_SetDirection(FAN_RIGHT, FAN_DIR_FORWARD);
    } else {
        /* 目标角度为负，使用左风扇 */
        speed = (uint8_t)(fabs(pid_output));
        if (speed > 100) speed = 100;
        
        FAN_SetSpeed(FAN_LEFT, speed);
        FAN_SetSpeed(FAN_RIGHT, 0);
        
        /* 设置风扇方向 */
        FAN_SetDirection(FAN_LEFT, FAN_DIR_FORWARD);
    }
}

/**
  * @brief  双风扇控制处理
  * @param  control: 角度控制结构体指针
  * @retval 无
  * @note   私有函数
  */
static void ANGLE_CONTROL_ProcessDualFan(AngleControl_TypeDef *control)
{
    float pid_output;
    int16_t base_speed;
    float delta;
    int16_t left_raw;
    int16_t right_raw;
    uint8_t left_speed, right_speed;
    
    /* 计算PID输出 */
    pid_output = PID_Calculate(&control->pid, control->current_angle);
    
    /* 
     * 双风扇控制逻辑：
     * 1. 基础速度为两个风扇的基本速度
     * 2. PID输出作为差速调整量
     * 3. 当需要板子顺时针转动(pid_output > 0)时，增加右风扇速度，减少左风扇速度
     * 4. 当需要板子逆时针转动(pid_output < 0)时，增加左风扇速度，减少右风扇速度
     */
    
    /* 基础速度 */
    base_speed = control->fan_base_speed;
    
    /* 差速计算 */
    delta = pid_output * control->dual_mode_ratio / 100.0f;
    
    /* 计算左右风扇速度 */
    left_raw = base_speed - (int16_t)delta;
    right_raw = base_speed + (int16_t)delta;
    
    /* 限制速度范围 */
    if (left_raw < 0) left_raw = 0;
    if (left_raw > 100) left_raw = 100;
    if (right_raw < 0) right_raw = 0;
    if (right_raw > 100) right_raw = 100;
    
    left_speed = (uint8_t)left_raw;
    right_speed = (uint8_t)right_raw;
    
    /* 设置风扇速度和方向 */
    FAN_SetSpeed(FAN_LEFT, left_speed);
    FAN_SetSpeed(FAN_RIGHT, right_speed);
    FAN_SetDirection(FAN_LEFT, FAN_DIR_FORWARD);
    FAN_SetDirection(FAN_RIGHT, FAN_DIR_FORWARD);
}

/**
  * @brief  序列控制处理
  * @param  control: 角度控制结构体指针
  * @retval 无
  * @note   私有函数
  */
static void ANGLE_CONTROL_ProcessSequence(AngleControl_TypeDef *control)
{
    float current_target;
    uint8_t hold_time;
    uint32_t stable_time;
    float next_angle;
    
    /* 检查序列是否有效 */
    if (control->sequence.angle_count == 0) {
        /* 无效序列，切换到空闲模式 */
        ANGLE_CONTROL_SetMode(control, CONTROL_MODE_IDLE);
        return;
    }
    
    /* 获取当前目标角度和保持时间 */
    current_target = control->sequence.angles[control->sequence.current_index];
    hold_time = control->sequence.hold_times[control->sequence.current_index];
    
    /* 检查当前角度是否已达到目标 */
    if (control->state == ANGLE_STATE_STABLE) {
        /* 已稳定，检查是否保持足够长的时间 */
        stable_time = (control->system_time - control->sequence.stable_start_time) / 1000; // 转换为秒
        
        if (stable_time >= hold_time) {
            /* 当前角度已保持足够长时间，切换到下一个角度 */
            control->sequence.current_index++;
            
            /* 检查是否完成所有角度 */
            if (control->sequence.current_index >= control->sequence.angle_count) {
                /* 序列完成，切换到空闲模式 */
                printf("Angle sequence completed\r\n");
                ANGLE_CONTROL_SetMode(control, CONTROL_MODE_IDLE);
                return;
            }
            
            /* 设置新的目标角度 */
            next_angle = control->sequence.angles[control->sequence.current_index];
            ANGLE_CONTROL_SetTarget(control, next_angle);
            
            /* 重置稳定计时 */
            control->sequence.stable_start_time = 0;
            printf("Moving to next angle: %.1f degrees\r\n", next_angle);
        }
    } else if (control->state == ANGLE_STATE_STABLE && control->sequence.stable_start_time == 0) {
        /* 刚稳定，记录稳定开始时间 */
        control->sequence.stable_start_time = control->system_time;
        printf("Angle %.1f degrees stable, holding for %d seconds\r\n", current_target, hold_time);
    }
    
    /* 使用双风扇控制模式处理角度 */
    ANGLE_CONTROL_ProcessDualFan(control);
}

/**
  * @brief  判断角度是否已稳定在目标位置
  * @param  control: 角度控制结构体指针
  * @retval uint8_t: 1表示已稳定，0表示未稳定
  */
uint8_t ANGLE_CONTROL_IsStable(AngleControl_TypeDef *control)
{
    return (control->state == ANGLE_STATE_STABLE) ? 1 : 0;
}

/**
  * @brief  配置角度序列控制
  * @param  control: 角度控制结构体指针
  * @param  angles: 角度序列数组
  * @param  hold_times: 各角度保持时间数组(秒)
  * @param  count: 角度数量
  * @retval 无
  */
void ANGLE_CONTROL_ConfigSequence(AngleControl_TypeDef *control, float *angles, uint8_t *hold_times, uint8_t count)
{
    uint8_t i;
    float angle;
    
    /* 检查参数有效性 */
    if (count > 10) count = 10; // 最多10个角度
    
    /* 保存序列参数 */
    control->sequence.angle_count = count;
    
    for (i = 0; i < count; i++) {
        /* 限制角度范围 */
        angle = angles[i];
        if (angle > 90.0f) angle = 90.0f;
        if (angle < -90.0f) angle = -90.0f;
        
        control->sequence.angles[i] = angle;
        control->sequence.hold_times[i] = hold_times[i];
    }
    
    printf("Angle sequence configured with %d angles\r\n", count);
}

/**
  * @brief  开始角度序列控制
  * @param  control: 角度控制结构体指针
  * @retval 无
  */
void ANGLE_CONTROL_StartSequence(AngleControl_TypeDef *control)
{
    /* 检查是否有有效序列 */
    if (control->sequence.angle_count == 0) {
        printf("Error: No valid angle sequence\r\n");
        return;
    }
    
    /* 切换到序列控制模式 */
    ANGLE_CONTROL_SetMode(control, CONTROL_MODE_SEQUENCE);
    
    /* 重置序列索引 */
    control->sequence.current_index = 0;
    control->sequence.start_time = control->system_time;
    control->sequence.stable_start_time = 0;
    
    /* 设置第一个目标角度 */
    ANGLE_CONTROL_SetTarget(control, control->sequence.angles[0]);
    
    printf("Angle sequence started\r\n");
}

/**
  * @brief  设置风扇基础速度和比例
  * @param  control: 角度控制结构体指针
  * @param  base_speed: 风扇基础速度(%)
  * @param  ratio: 双风扇模式下的差速比例(%)
  * @retval 无
  */
void ANGLE_CONTROL_SetFanParameters(AngleControl_TypeDef *control, uint8_t base_speed, uint8_t ratio)
{
    /* 限制参数范围 */
    if (base_speed > 100) base_speed = 100;
    if (ratio > 100) ratio = 100;
    
    control->fan_base_speed = base_speed;
    control->dual_mode_ratio = ratio;
    
    printf("Fan parameters updated: Base speed=%d%%, Ratio=%d%%\r\n", base_speed, ratio);
}

/**
  * @brief  停止所有控制
  * @param  control: 角度控制结构体指针
  * @retval 无
  */
void ANGLE_CONTROL_Stop(AngleControl_TypeDef *control)
{
    /* 停止所有风扇 */
    FAN_StopAll();
    
    /* 切换到空闲模式 */
    control->mode = CONTROL_MODE_IDLE;
    control->state = ANGLE_STATE_INIT;
    
    /* 重置PID控制器 */
    PID_Reset(&control->pid);
    
    printf("Angle control stopped\r\n");
}

/**
  * @brief  更新系统时间
  * @param  control: 角度控制结构体指针
  * @retval 无
  * @note   私有函数
  */
static void ANGLE_CONTROL_UpdateTime(AngleControl_TypeDef *control)
{
    control->system_time = g_system_time;
}

/**
  * @brief  获取控制系统运行时间(ms)
  * @param  无
  * @retval uint32_t: 系统运行时间
  */
uint32_t ANGLE_CONTROL_GetTime(void)
{
    return g_system_time;
}

/**
  * @brief  SysTick中断处理函数，更新系统时间
  * @param  无
  * @retval 无
  * @note   需要在stm32f10x_it.c中调用
  */
void ANGLE_CONTROL_TimeUpdate(void)
{
    g_system_time++;
}
