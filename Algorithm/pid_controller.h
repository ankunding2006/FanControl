/**
  ******************************************************************************
  * @file    pid_controller.h
  * @brief   PID控制器模块头文件
  ******************************************************************************
  */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include "stm32f10x.h"

/* PID控制器模式枚举 */
typedef enum {
    PID_MODE_POSITION = 0,   // 位置式PID
    PID_MODE_INCREMENTAL = 1 // 增量式PID
} PIDMode_TypeDef;

/* PID控制器结构体 */
typedef struct {
    /* PID参数 */
    float Kp;                // 比例系数
    float Ki;                // 积分系数
    float Kd;                // 微分系数
    
    /* PID输入输出 */
    float setPoint;          // 设定目标值
    float processValue;      // 当前过程值
    float lastError;         // 上次误差
    float prevError;         // 上上次误差
    float integral;          // 积分项
    float derivative;        // 微分项
    float output;            // 输出值
    
    /* 配置参数 */
    PIDMode_TypeDef mode;    // PID模式
    float sampleTime;        // 采样时间(s)
    float outputMax;         // 输出上限
    float outputMin;         // 输出下限
    float integralMax;       // 积分限幅值
    float integralMin;       // 积分下限
    float deadBand;          // 死区范围
    float differentiatorLPF; // 微分低通滤波系数 (0-1)
    
    /* 功能控制 */
    uint8_t enableIntegral;  // 积分使能标志
    uint8_t enableDerivative;// 微分使能标志
    uint8_t enableLPF;       // 低通滤波使能标志
    uint8_t enableAntiWindup;// 抗积分饱和使能
    uint8_t integralSeparation; // 积分分离使能
    float integralSeparationThreshold; // 积分分离阈值
} PID_TypeDef;

/* 函数声明 */
/**
  * @brief  初始化PID控制器
  * @param  pid: 指向PID结构体的指针
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @param  mode: PID模式，位置式或增量式
  * @param  sampleTime: 采样时间，单位秒
  * @retval 无
  */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, PIDMode_TypeDef mode, float sampleTime);

/**
  * @brief  计算PID输出
  * @param  pid: 指向PID结构体的指针
  * @param  nextPoint: 当前过程值
  * @retval float: PID计算输出值
  */
float PID_Calculate(PID_TypeDef *pid, float nextPoint);

/**
  * @brief  设置PID目标值
  * @param  pid: 指向PID结构体的指针
  * @param  setPoint: 目标值
  * @retval 无
  */
void PID_SetPoint(PID_TypeDef *pid, float setPoint);

/**
  * @brief  设置PID输出限幅
  * @param  pid: 指向PID结构体的指针
  * @param  min: 输出下限
  * @param  max: 输出上限
  * @retval 无
  */
void PID_SetOutputLimits(PID_TypeDef *pid, float min, float max);

/**
  * @brief  设置积分限幅
  * @param  pid: 指向PID结构体的指针
  * @param  min: 积分下限
  * @param  max: 积分上限
  * @retval 无
  */
void PID_SetIntegralLimits(PID_TypeDef *pid, float min, float max);

/**
  * @brief  设置死区
  * @param  pid: 指向PID结构体的指针
  * @param  deadBand: 死区范围
  * @retval 无
  */
void PID_SetDeadBand(PID_TypeDef *pid, float deadBand);

/**
  * @brief  设置积分分离阈值
  * @param  pid: 指向PID结构体的指针
  * @param  threshold: 积分分离阈值
  * @retval 无
  */
void PID_SetIntegralSeparationThreshold(PID_TypeDef *pid, float threshold);

/**
  * @brief  清除PID积分项
  * @param  pid: 指向PID结构体的指针
  * @retval 无
  */
void PID_ClearIntegral(PID_TypeDef *pid);

/**
  * @brief  重置PID控制器
  * @param  pid: 指向PID结构体的指针
  * @retval 无
  */
void PID_Reset(PID_TypeDef *pid);

/**
  * @brief  调整PID参数
  * @param  pid: 指向PID结构体的指针
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @retval 无
  */
void PID_Tune(PID_TypeDef *pid, float Kp, float Ki, float Kd);

/**
  * @brief  使能/禁用积分项
  * @param  pid: 指向PID结构体的指针
  * @param  enable: 使能状态 (1:使能, 0:禁用)
  * @retval 无
  */
void PID_EnableIntegral(PID_TypeDef *pid, uint8_t enable);

/**
  * @brief  使能/禁用微分项
  * @param  pid: 指向PID结构体的指针
  * @param  enable: 使能状态 (1:使能, 0:禁用)
  * @retval 无
  */
void PID_EnableDerivative(PID_TypeDef *pid, uint8_t enable);

/**
  * @brief  获取PID当前误差
  * @param  pid: 指向PID结构体的指针
  * @retval float: 当前误差值
  */
float PID_GetError(PID_TypeDef *pid);

#endif /* __PID_CONTROLLER_H */
