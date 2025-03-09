#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "oled.h"
#include "fan_driver.h"
#include "angle_sensor.h"
#include "angle_control.h"
#include "pid_controller.h"
#include <string.h>
#include <math.h>

/* 系统状态定义 */
typedef enum {
    STATE_MENU = 0,        // 菜单状态
    STATE_ANGLE_SETTING,   // 角度设置状态
    STATE_RUNNING,         // 运行状态
    STATE_PARAM_SETTING    // 参数设置状态
} SystemState_TypeDef;

/* 工作模式定义 */
typedef enum {
    MODE_IDLE = 0,            // 空闲模式
    MODE_SINGLE_FAN_45DEG,    // 单风扇45度模式
    MODE_SINGLE_FAN_ANY,      // 单风扇任意角度模式
    MODE_DUAL_FAN_ANY,        // 双风扇任意角度模式
    MODE_DUAL_FAN_SEQUENCE    // 双风扇序列模式
} WorkMode_TypeDef;

/* 全局变量 */
AngleControl_TypeDef g_angle_control;   // 角度控制结构体
static SystemState_TypeDef g_systemState;      // 系统状态
static WorkMode_TypeDef g_workMode;            // 工作模式
static float g_targetAngle = 0.0f;            // 目标角度
static uint32_t g_modeStartTime = 0;          // 模式开始时间

/* 显示缓冲区，用于存储上一次显示的内容 */
static char g_lastDisplayBuf[8][32];  // 8行显示，每行最多32个字符
static SystemState_TypeDef g_lastSystemState = STATE_MENU;  // 上一次的系统状态，初始化为菜单状态
static WorkMode_TypeDef g_lastWorkMode = MODE_IDLE;        // 上一次的工作模式，初始化为空闲模式
static float g_lastTargetAngle = -1.0f;              // 上一次显示的目标角度
static float g_lastCurrentAngle = -1.0f;             // 上一次显示的当前角度
static uint32_t g_lastElapsedTime = 0xFFFFFFFF;      // 上一次显示的经过时间

/* 函数声明 */
static void System_Init(void);
static void Timer_Init(void);
static void UserInterface_Process(void);
static void ProcessKeys(void);
static void MenuManager(void);
static void ConfigureControlMode(WorkMode_TypeDef mode);
void DisplayStatus(void);

/*
 * @brief 
 * 
 * @brief  系统初始化函数
 * @param  无
 * @retval 无
 */
static void System_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(115200);
    KEY_Init();
    
    // 初始化角度控制系统
    ANGLE_CONTROL_Init(&g_angle_control, CONTROL_MODE_IDLE);
    
    // 初始化定时器
    Timer_Init();
    
    // 初始化系统状态
    g_systemState = STATE_MENU;
    g_workMode = MODE_IDLE;
}

/**
  * @brief  定时器初始化函数
  * @param  无
  * @retval 无
  */
static void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    
    // TIM3配置 - 10ms中断
    TIM_TimeBaseStructure.TIM_Period = 9999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // TIM4配置 - 2ms中断
    TIM_TimeBaseStructure.TIM_Period = 1999;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // 配置NVIC - TIM3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 配置NVIC - TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能定时器中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    // 启动定时器
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief  主函数
  * @param  无
  * @retval int
  */
int main(void)
{
    // 系统初始化
    System_Init();
    delay_ms(100);
    OLED_Init();
	OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    // 显示欢迎信息
    OLED_ShowString(0, 0, (u8 *)"Wind Panel Control", 12, 1);
    OLED_ShowString(0, 2, (u8 *)"System Ready", 12, 1);
    OLED_Refresh();      // 确保信息显示后刷新
    printf("Wind Panel Control System Started\r\n");
    delay_ms(1000);
    
    // 主循环
    while(1)
    {
        // 用户交互处理
        UserInterface_Process();
        
        DisplayStatus();  // 显示状态更新
        
        // 延时
        delay_ms(10);
    }
}

/**
  * @brief  用户界面处理函数
  * @param  无
  * @retval 无
  */
static void UserInterface_Process(void)
{
    ProcessKeys();
    MenuManager();
}

/**
  * @brief  按键处理函数
  * @param  无
  * @retval 无
  */
static void ProcessKeys(void)
{
    uint8_t key = KEY_Scan();
    if(key == KEY_NONE) return;
    printf("Key: %d\r\n", key);
    switch(g_systemState)
    {

        case STATE_MENU:
            // 模式选择
            if(key == KEY_UP || key == KEY_DOWN)
            {
                if(key == KEY_UP) g_workMode++;
                else g_workMode--;
                
                // 删除重复的条件判断，只保留完整的逻辑
                if(g_workMode > MODE_DUAL_FAN_SEQUENCE) {
                    g_workMode = MODE_IDLE;
                } else if(g_workMode == MODE_IDLE && key == KEY_DOWN) {
                    g_workMode = MODE_DUAL_FAN_SEQUENCE;
                }
            }
            else if(key == KEY_ENTER)
            {
                // 进入角度设置或直接开始
                if(g_workMode == MODE_SINGLE_FAN_45DEG)
                {
                    // 45度模式直接开始
                    g_targetAngle = 45.0f;
                    ConfigureControlMode(g_workMode);
                    g_systemState = STATE_RUNNING;
                    g_modeStartTime = ANGLE_CONTROL_GetTime();
                }
                else if(g_workMode != MODE_IDLE)
                {
                    g_systemState = STATE_ANGLE_SETTING;
                }
            }
            break;
            
        case STATE_ANGLE_SETTING:
            // 角度设置
            if(key == KEY_UP)
            {
                g_targetAngle += 5.0f;
                if(g_workMode == MODE_SINGLE_FAN_ANY && g_targetAngle > 90.0f)
                    g_targetAngle = 90.0f;
                else if(g_workMode == MODE_DUAL_FAN_ANY && g_targetAngle > 180.0f)
                    g_targetAngle = 180.0f;
            }
            else if(key == KEY_DOWN)
            {
                g_targetAngle -= 5.0f;
                if(g_targetAngle < 0.0f) g_targetAngle = 0.0f;
            }
            else if(key == KEY_ENTER)
            {
                // 开始控制
                ConfigureControlMode(g_workMode);
                ANGLE_CONTROL_SetTarget(&g_angle_control, g_targetAngle);
                g_systemState = STATE_RUNNING;
                g_modeStartTime = ANGLE_CONTROL_GetTime();
            }
            else if(key == KEY_MODE)
            {
                // 返回菜单
                g_systemState = STATE_MENU;
            }
            break;
            
        case STATE_RUNNING:
            if(key == KEY_MODE)
            {
                // 停止控制，返回菜单
                ANGLE_CONTROL_Stop(&g_angle_control);
                g_systemState = STATE_MENU;
            }
            break;
            
        default:
            break;
    }
}

/**
  * @brief  配置控制模式
  * @param  mode: 工作模式
  * @retval 无
  */
static void ConfigureControlMode(WorkMode_TypeDef mode)
{
    switch(mode)
    {
        case MODE_SINGLE_FAN_45DEG:
            ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_SINGLE_FAN);
            ANGLE_CONTROL_SetStableCondition(&g_angle_control, 5.0f, 3000);
            ANGLE_CONTROL_SetTarget(&g_angle_control, 45.0f);
            break;
            
        case MODE_SINGLE_FAN_ANY:
            ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_SINGLE_FAN);
            ANGLE_CONTROL_SetStableCondition(&g_angle_control, 5.0f, 3000);
            break;
            
        case MODE_DUAL_FAN_ANY:
            ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_DUAL_FAN);
            ANGLE_CONTROL_SetStableCondition(&g_angle_control, 3.0f, 5000);
            break;
            
        case MODE_DUAL_FAN_SEQUENCE:
            {
                float angles[] = {45.0f, 60.0f, 90.0f, 120.0f, 135.0f};
                uint8_t times[] = {3, 3, 3, 3, 3};
                ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_SEQUENCE);
                ANGLE_CONTROL_ConfigSequence(&g_angle_control, angles, times, 5);
                ANGLE_CONTROL_SetStableCondition(&g_angle_control, 3.0f, 3000);
            }
            break;
            
        default:
            ANGLE_CONTROL_SetMode(&g_angle_control, CONTROL_MODE_IDLE);
            break;
    }
}

/**
  * @brief  显示状态更新
  * @param  无
  * @retval 无
  */
void DisplayStatus(void)
{
    char buf[8][32];  // 临时缓冲区，存储当前要显示的内容
    float current_angle = ANGLE_SENSOR_GetAngle();
    uint32_t elapsed = 0;
    uint8_t needUpdate = 0;  // 标记是否需要更新显示，0=不需要，1=需要
    uint8_t i;
    
    // 清空临时缓冲区
    for(i = 0; i < 8; i++) {
        memset(buf[i], 0, 32);
    }
    
    // 检查系统状态或工作模式是否变化
    if(g_systemState != g_lastSystemState || g_workMode != g_lastWorkMode) {
        needUpdate = 1;
    }
    
    // 检查角度是否变化（允许0.1度的误差）
    if(fabs(current_angle - g_lastCurrentAngle) > 0.1f || fabs(g_targetAngle - g_lastTargetAngle) > 0.1f) {
        needUpdate = 1;
    }
    
    // 根据系统状态填充临时缓冲区
    switch(g_systemState)
    {
        case STATE_MENU:
            strcpy(buf[0], "Select Mode:");
            switch(g_workMode)
            {
                case MODE_IDLE:
                    strcpy(buf[2], "Idle Mode");
                    break;
                case MODE_SINGLE_FAN_45DEG:
                    strcpy(buf[2], "Single Fan 45");
                    break;
                case MODE_SINGLE_FAN_ANY:
                    strcpy(buf[2], "Single Fan Any");
                    break;
                case MODE_DUAL_FAN_ANY:
                    strcpy(buf[2], "Dual Fan Any");
                    break;
                case MODE_DUAL_FAN_SEQUENCE:
                    strcpy(buf[2], "Sequence Mode");
                    break;
            }
            break;
            
        case STATE_ANGLE_SETTING:
            strcpy(buf[0], "Set Angle:");
            sprintf(buf[2], "%.1f", g_targetAngle);
            break;
            
        case STATE_RUNNING:
            strcpy(buf[0], "Running");
            sprintf(buf[2], "Cur:%.1f", current_angle);
            sprintf(buf[4], "Tar:%.1f", g_targetAngle);
            
            if(g_workMode == MODE_SINGLE_FAN_45DEG)
            {
                // 显示10秒计时
                elapsed = (ANGLE_CONTROL_GetTime() - g_modeStartTime) / 1000;
                if(elapsed <= 10)
                {
                    sprintf(buf[6], "Time:%ds", (int)elapsed);
                    if(elapsed != g_lastElapsedTime) {
                        needUpdate = 1;
                    }
                }
            }
            break;
            
        default:
            break;
    }
    
    // 比较新旧缓冲区内容，检查是否需要更新
    if(!needUpdate) {
        for(i = 0; i < 8; i++) {
            if(strcmp(buf[i], g_lastDisplayBuf[i]) != 0) {
                needUpdate = 1;
                break;
            }
        }
    }
    
    // 如果需要更新，则进行OLED显示更新
    if(needUpdate) {
        OLED_Clear();
        
        // 将缓冲区内容显示到OLED
        for(i = 0; i < 8; i += 2) {
            if(buf[i][0] != '\0') {
                OLED_ShowString(0, i, (u8 *)buf[i], 12, 1);
            }
        }
        
        // 保存当前显示内容到上一次的缓冲区
        for(i = 0; i < 8; i++) {
            strcpy(g_lastDisplayBuf[i], buf[i]);
        }
        
        // 保存当前状态
        g_lastSystemState = g_systemState;
        g_lastWorkMode = g_workMode;
        g_lastCurrentAngle = current_angle;
        g_lastTargetAngle = g_targetAngle;
        g_lastElapsedTime = elapsed;
        
        // 刷新屏幕
        OLED_Refresh();
    }
}

/**
  * @brief  菜单管理函数
  * @param  无
  * @retval 无
  */
static void MenuManager(void)
{
    // 根据系统状态管理菜单
    // 此处可以添加菜单管理逻辑
    return;
}
