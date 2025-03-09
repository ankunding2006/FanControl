/**
  ******************************************************************************
  * @file    fan_driver.c
  * @brief   风扇驱动模块实现
  ******************************************************************************
  */

#include "fan_driver.h"

/* 私有变量定义 */
static uint8_t g_fan_speeds[2] = {0, 0};         // 风扇速度（左,右）
static FanDirection_TypeDef g_fan_directions[2] = {FAN_DIR_FORWARD, FAN_DIR_FORWARD}; // 风扇方向

/**
  * @brief  配置风扇控制相关的GPIO
  * @param  无
  * @retval 无
  */
static void FAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能相关时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    /* 左风扇PWM引脚配置 */
    GPIO_InitStructure.GPIO_Pin = FAN_LEFT_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(FAN_LEFT_PWM_PORT, &GPIO_InitStructure);
    
    /* 右风扇PWM引脚配置 */
    GPIO_InitStructure.GPIO_Pin = FAN_RIGHT_PWM_PIN;
    GPIO_Init(FAN_RIGHT_PWM_PORT, &GPIO_InitStructure);
    
    /* 左风扇方向控制引脚 */
    GPIO_InitStructure.GPIO_Pin = FAN_LEFT_IN1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    GPIO_Init(FAN_LEFT_IN1_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = FAN_LEFT_IN2_PIN;
    GPIO_Init(FAN_LEFT_IN2_PORT, &GPIO_InitStructure);
    
    /* 右风扇方向控制引脚 */
    GPIO_InitStructure.GPIO_Pin = FAN_RIGHT_IN1_PIN;
    GPIO_Init(FAN_RIGHT_IN1_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = FAN_RIGHT_IN2_PIN;
    GPIO_Init(FAN_RIGHT_IN2_PORT, &GPIO_InitStructure);
    
    /* STBY引脚配置 */
    GPIO_InitStructure.GPIO_Pin = FAN_STBY_PIN;
    GPIO_Init(FAN_STBY_PORT, &GPIO_InitStructure);
    
    /* 禁用STBY，激活TB6612 */
    GPIO_SetBits(FAN_STBY_PORT, FAN_STBY_PIN);
}

/**
  * @brief  配置风扇PWM输出的定时器
  * @param  无
  * @retval 无
  */
static void FAN_TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    /* 使能TIM2时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /* 基本定时器配置 */
    TIM_TimeBaseStructure.TIM_Period = FAN_PWM_PERIOD - 1;          // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;                   // 预分频值，72M/72=1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         // 时钟分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     // 向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    /* 配置PWM模式 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                              // 初始占空比为0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       // 输出极性：高电平有效
    
    /* 配置左风扇PWM通道 */
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    /* 配置右风扇PWM通道 */
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    /* 使能定时器预装载寄存器 */
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    
    /* 启动定时器 */
    TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  初始化风扇驱动模块
  * @param  无
  * @retval 无
  */
void FAN_Init(void)
{
    /* 配置GPIO */
    FAN_GPIO_Config();
    
    /* 配置定时器PWM */
    FAN_TIM_Config();
    
    /* 初始停止所有风扇 */
    FAN_StopAll();
}

/**
  * @brief  设置风扇方向
  * @param  fan: 风扇选择
  * @param  direction: 风扇旋转方向
  * @retval 无
  */
void FAN_SetDirection(FanSelect_TypeDef fan, FanDirection_TypeDef direction)
{
    GPIO_TypeDef* IN1_PORT;
    uint16_t IN1_PIN;
    GPIO_TypeDef* IN2_PORT;
    uint16_t IN2_PIN;
    
    /* 保存设置的方向 */
    g_fan_directions[fan] = direction;
    
    /* 获取对应风扇的控制引脚 */
    if(fan == FAN_LEFT) {
        IN1_PORT = FAN_LEFT_IN1_PORT;
        IN1_PIN = FAN_LEFT_IN1_PIN;
        IN2_PORT = FAN_LEFT_IN2_PORT;
        IN2_PIN = FAN_LEFT_IN2_PIN;
    } else {
        IN1_PORT = FAN_RIGHT_IN1_PORT;
        IN1_PIN = FAN_RIGHT_IN1_PIN;
        IN2_PORT = FAN_RIGHT_IN2_PORT;
        IN2_PIN = FAN_RIGHT_IN2_PIN;
    }
    
    /* 根据方向设置IN1/IN2引脚 */
    if(direction == FAN_DIR_FORWARD) {
        GPIO_SetBits(IN1_PORT, IN1_PIN);
        GPIO_ResetBits(IN2_PORT, IN2_PIN);
    } else {
        GPIO_ResetBits(IN1_PORT, IN1_PIN);
        GPIO_SetBits(IN2_PORT, IN2_PIN);
    }
}

/**
  * @brief  设置风扇速度
  * @param  fan: 风扇选择
  * @param  speed: 风扇速度 (0-100)
  * @retval 无
  */
void FAN_SetSpeed(FanSelect_TypeDef fan, uint8_t speed)
{
    uint16_t ccr;
    
    /* 限制速度范围 */
    if(speed > FAN_MAX_DUTY)
        speed = FAN_MAX_DUTY;
    
    /* 保存设置的速度 */
    g_fan_speeds[fan] = speed;
    
    /* 计算PWM占空比值 */
    ccr = (uint16_t)((uint32_t)speed * (FAN_PWM_PERIOD - 1) / 100);
    
    /* 设置PWM输出值 */
    if(fan == FAN_LEFT) {
        TIM_SetCompare2(TIM2, ccr);
    } else {
        TIM_SetCompare3(TIM2, ccr);
    }
}

/**
  * @brief  启动指定的风扇
  * @param  fan: 风扇选择
  * @retval 无
  */
void FAN_Start(FanSelect_TypeDef fan)
{
    /* 设置风扇方向 */
    FAN_SetDirection(fan, g_fan_directions[fan]);
    
    /* 应用上一次设置的速度 */
    FAN_SetSpeed(fan, g_fan_speeds[fan]);
}

/**
  * @brief  停止指定的风扇
  * @param  fan: 风扇选择
  * @retval 无
  */
void FAN_Stop(FanSelect_TypeDef fan)
{
    GPIO_TypeDef* IN1_PORT;
    uint16_t IN1_PIN;
    GPIO_TypeDef* IN2_PORT;
    uint16_t IN2_PIN;
    
    /* 获取对应风扇的控制引脚 */
    if(fan == FAN_LEFT) {
        IN1_PORT = FAN_LEFT_IN1_PORT;
        IN1_PIN = FAN_LEFT_IN1_PIN;
        IN2_PORT = FAN_LEFT_IN2_PORT;
        IN2_PIN = FAN_LEFT_IN2_PIN;
        
        /* 设置PWM为0 */
        TIM_SetCompare2(TIM2, 0);
    } else {
        IN1_PORT = FAN_RIGHT_IN1_PORT;
        IN1_PIN = FAN_RIGHT_IN1_PIN;
        IN2_PORT = FAN_RIGHT_IN2_PORT;
        IN2_PIN = FAN_RIGHT_IN2_PIN;
        
        /* 设置PWM为0 */
        TIM_SetCompare3(TIM2, 0);
    }
    
    /* 停止电机：TB6612的制动模式 */
    GPIO_ResetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
    
    /* 更新保存的速度为0 */
    g_fan_speeds[fan] = 0;
}

/**
  * @brief  启动所有风扇
  * @param  无
  * @retval 无
  */
void FAN_StartAll(void)
{
    FAN_Start(FAN_LEFT);
    FAN_Start(FAN_RIGHT);
}

/**
  * @brief  停止所有风扇
  * @param  无
  * @retval 无
  */
void FAN_StopAll(void)
{
    FAN_Stop(FAN_LEFT);
    FAN_Stop(FAN_RIGHT);
}

/**
  * @brief  同时设置两个风扇的速度
  * @param  left_speed: 左风扇速度 (0-100)
  * @param  right_speed: 右风扇速度 (0-100)
  * @retval 无
  */
void FAN_SetDualSpeed(uint8_t left_speed, uint8_t right_speed)
{
    FAN_SetSpeed(FAN_LEFT, left_speed);
    FAN_SetSpeed(FAN_RIGHT, right_speed);
}
