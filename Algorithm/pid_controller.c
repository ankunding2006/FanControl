/**
  ******************************************************************************
  * @file    pid_controller.c
  * @brief   PID控制器模块实现
  ******************************************************************************
  */

#include "pid_controller.h"
#include <math.h>

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
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, PIDMode_TypeDef mode, float sampleTime)
{
    /* 设置PID参数 */
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->mode = mode;
    pid->sampleTime = sampleTime;
    
    /* 初始化PID状态 */
    pid->setPoint = 0.0f;
    pid->processValue = 0.0f;
    pid->lastError = 0.0f;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    
    /* 设置默认配置参数 */
    pid->outputMax = 100.0f;
    pid->outputMin = -100.0f;
    pid->integralMax = 100.0f;
    pid->integralMin = -100.0f;
    pid->deadBand = 0.0f;
    pid->differentiatorLPF = 0.1f; // 默认低通滤波系数
    
    /* 功能控制 */
    pid->enableIntegral = 1;       // 默认启用积分
    pid->enableDerivative = 1;     // 默认启用微分
    pid->enableLPF = 1;            // 默认启用低通滤波
    pid->enableAntiWindup = 1;     // 默认启用抗积分饱和
    pid->integralSeparation = 1;   // 默认启用积分分离
    pid->integralSeparationThreshold = 10.0f;  // 默认积分分离阈值
}

/**
  * @brief  计算PID输出 - 位置式PID
  * @param  pid: 指向PID结构体的指针
  * @param  nextPoint: 当前过程值
  * @retval float: 位置式PID计算输出值
  */
static float PID_CalculatePosition(PID_TypeDef *pid, float nextPoint)
{
    float error, pTerm, iTerm, dTerm;
    float output;
    
    /* 更新当前过程值 */
    pid->processValue = nextPoint;
    
    /* 计算当前误差 */
    error = pid->setPoint - nextPoint;
    
    /* 死区处理 */
    if (fabs(error) <= pid->deadBand) {
        error = 0.0f;
    }
    
    /* 计算比例项 */
    pTerm = pid->Kp * error;
    
    /* 计算积分项 */
    if (pid->enableIntegral) {
        /* 积分分离 */
        if (!pid->integralSeparation || fabs(error) < pid->integralSeparationThreshold) {
            pid->integral += error * pid->sampleTime;
            
            /* 积分限幅 */
            if (pid->integral > pid->integralMax) {
                pid->integral = pid->integralMax;
            } else if (pid->integral < pid->integralMin) {
                pid->integral = pid->integralMin;
            }
        }
        iTerm = pid->Ki * pid->integral;
    } else {
        iTerm = 0.0f;
    }
    
    /* 计算微分项 */
    if (pid->enableDerivative) {
        if (pid->enableLPF) {
            /* 带低通滤波的微分项计算 */
            pid->derivative = pid->differentiatorLPF * pid->derivative + 
                             (1.0f - pid->differentiatorLPF) * ((error - pid->lastError) / pid->sampleTime);
        } else {
            /* 标准微分项计算 */
            pid->derivative = (error - pid->lastError) / pid->sampleTime;
        }
        dTerm = pid->Kd * pid->derivative;
    } else {
        dTerm = 0.0f;
    }
    
    /* 计算PID输出 */
    output = pTerm + iTerm + dTerm;
    
    /* 输出限幅 */
    if (output > pid->outputMax) {
        output = pid->outputMax;
        
        /* 抗积分饱和 */
        if (pid->enableAntiWindup && pid->enableIntegral && error > 0.0f) {
            pid->integral -= error * pid->sampleTime;
        }
    } else if (output < pid->outputMin) {
        output = pid->outputMin;
        
        /* 抗积分饱和 */
        if (pid->enableAntiWindup && pid->enableIntegral && error < 0.0f) {
            pid->integral -= error * pid->sampleTime;
        }
    }
    
    /* 保存状态 */
    pid->lastError = error;
    pid->output = output;
    
    return output;
}

/**
  * @brief  计算PID输出 - 增量式PID
  * @param  pid: 指向PID结构体的指针
  * @param  nextPoint: 当前过程值
  * @retval float: 增量式PID计算输出值
  */
static float PID_CalculateIncremental(PID_TypeDef *pid, float nextPoint)
{
    float error, deltaP, deltaI, deltaD;
    float deltaOutput;
    
    /* 更新当前过程值 */
    pid->processValue = nextPoint;
    
    /* 计算当前误差 */
    error = pid->setPoint - nextPoint;
    
    /* 死区处理 */
    if (fabs(error) <= pid->deadBand) {
        error = 0.0f;
    }
    
    /* 计算比例项增量 */
    deltaP = pid->Kp * (error - pid->lastError);
    
    /* 计算积分项增量 */
    if (pid->enableIntegral) {
        /* 积分分离 */
        if (!pid->integralSeparation || fabs(error) < pid->integralSeparationThreshold) {
            deltaI = pid->Ki * error;
        } else {
            deltaI = 0.0f;
        }
    } else {
        deltaI = 0.0f;
    }
    
    /* 计算微分项增量 */
    if (pid->enableDerivative) {
        if (pid->enableLPF) {
            /* 带低通滤波的微分项计算 */
            deltaD = pid->Kd * pid->differentiatorLPF * (error - 2 * pid->lastError + pid->prevError);
        } else {
            /* 标准微分项计算 */
            deltaD = pid->Kd * (error - 2 * pid->lastError + pid->prevError);
        }
    } else {
        deltaD = 0.0f;
    }
    
    /* 计算输出增量 */
    deltaOutput = deltaP + deltaI + deltaD;
    
    /* 更新输出 */
    pid->output += deltaOutput;
    
    /* 输出限幅 */
    if (pid->output > pid->outputMax) {
        pid->output = pid->outputMax;
    } else if (pid->output < pid->outputMin) {
        pid->output = pid->outputMin;
    }
    
    /* 保存状态 */
    pid->prevError = pid->lastError;
    pid->lastError = error;
    
    return pid->output;
}

/**
  * @brief  计算PID输出
  * @param  pid: 指向PID结构体的指针
  * @param  nextPoint: 当前过程值
  * @retval float: PID计算输出值
  */
float PID_Calculate(PID_TypeDef *pid, float nextPoint)
{
    if (pid->mode == PID_MODE_POSITION) {
        return PID_CalculatePosition(pid, nextPoint);
    } else {
        return PID_CalculateIncremental(pid, nextPoint);
    }
}

/**
  * @brief  设置PID目标值
  * @param  pid: 指向PID结构体的指针
  * @param  setPoint: 目标值
  * @retval 无
  */
void PID_SetPoint(PID_TypeDef *pid, float setPoint)
{
    pid->setPoint = setPoint;
}

/**
  * @brief  设置PID输出限幅
  * @param  pid: 指向PID结构体的指针
  * @param  min: 输出下限
  * @param  max: 输出上限
  * @retval 无
  */
void PID_SetOutputLimits(PID_TypeDef *pid, float min, float max)
{
    if(min < max) {
        pid->outputMin = min;
        pid->outputMax = max;
        
        /* 检查并调整当前输出 */
        if(pid->output > pid->outputMax) {
            pid->output = pid->outputMax;
        } else if(pid->output < pid->outputMin) {
            pid->output = pid->outputMin;
        }
    }
}

/**
  * @brief  设置积分限幅
  * @param  pid: 指向PID结构体的指针
  * @param  min: 积分下限
  * @param  max: 积分上限
  * @retval 无
  */
void PID_SetIntegralLimits(PID_TypeDef *pid, float min, float max)
{
    if(min < max) {
        pid->integralMin = min;
        pid->integralMax = max;
        
        /* 检查并调整当前积分值 */
        if(pid->integral > pid->integralMax) {
            pid->integral = pid->integralMax;
        } else if(pid->integral < pid->integralMin) {
            pid->integral = pid->integralMin;
        }
    }
}

/**
  * @brief  设置死区
  * @param  pid: 指向PID结构体的指针
  * @param  deadBand: 死区范围
  * @retval 无
  */
void PID_SetDeadBand(PID_TypeDef *pid, float deadBand)
{
    if(deadBand >= 0) {
        pid->deadBand = deadBand;
    }
}

/**
  * @brief  设置积分分离阈值
  * @param  pid: 指向PID结构体的指针
  * @param  threshold: 积分分离阈值
  * @retval 无
  */
void PID_SetIntegralSeparationThreshold(PID_TypeDef *pid, float threshold)
{
    if(threshold >= 0) {
        pid->integralSeparationThreshold = threshold;
    }
}

/**
  * @brief  清除PID积分项
  * @param  pid: 指向PID结构体的指针
  * @retval 无
  */
void PID_ClearIntegral(PID_TypeDef *pid)
{
    pid->integral = 0.0f;
}

/**
  * @brief  重置PID控制器
  * @param  pid: 指向PID结构体的指针
  * @retval 无
  */
void PID_Reset(PID_TypeDef *pid)
{
    pid->lastError = 0.0f;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
}

/**
  * @brief  调整PID参数
  * @param  pid: 指向PID结构体的指针
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @retval 无
  */
void PID_Tune(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

/**
  * @brief  使能/禁用积分项
  * @param  pid: 指向PID结构体的指针
  * @param  enable: 使能状态 (1:使能, 0:禁用)
  * @retval 无
  */
void PID_EnableIntegral(PID_TypeDef *pid, uint8_t enable)
{
    pid->enableIntegral = enable;
    
    /* 禁用积分时清除积分项 */
    if(!enable) {
        pid->integral = 0.0f;
    }
}

/**
  * @brief  使能/禁用微分项
  * @param  pid: 指向PID结构体的指针
  * @param  enable: 使能状态 (1:使能, 0:禁用)
  * @retval 无
  */
void PID_EnableDerivative(PID_TypeDef *pid, uint8_t enable)
{
    pid->enableDerivative = enable;
    
    /* 禁用微分时清除微分项 */
    if(!enable) {
        pid->derivative = 0.0f;
    }
}

/**
  * @brief  获取PID当前误差
  * @param  pid: 指向PID结构体的指针
  * @retval float: 当前误差值
  */
float PID_GetError(PID_TypeDef *pid)
{
    return pid->setPoint - pid->processValue;
}
