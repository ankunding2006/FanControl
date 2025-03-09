#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "oled.h"
#include "fan_driver.h"
#include "angle_sensor.h"
#include "angle_control.h"
#include "pid_controller.h"

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

/* 函数声明 */
static void System_Init(void);
static void Timer_Init(void);
static void UserInterface_Process(void);
static void ProcessKeys(void);
static void MenuManager(void);
static void ConfigureControlMode(WorkMode_TypeDef mode);
static void DisplayStatus(void);

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
    OLED_Init();
    
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
    
    // 显示欢迎信息
    OLED_Clear();
    OLED_ShowString(0, 0, "Wind Panel Control", 12, 1);
    OLED_ShowString(0, 2, "System Ready", 12, 1);
    printf("Wind Panel Control System Started\r\n");
    delay_ms(1000);
    
    // 主循环
    while(1)
    {
        // 用户交互处理
        UserInterface_Process();
        
        // 状态显示更新
        DisplayStatus();
        
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
static void DisplayStatus(void)
{
    char buf[32];
    float current_angle = ANGLE_SENSOR_GetAngle();
    
    OLED_Clear();
    
    switch(g_systemState)
    {
        case STATE_MENU:
            OLED_ShowString(0, 0, (u8 *)"Select Mode:", 12, 1);
            switch(g_workMode)
            {
                case MODE_IDLE:
                    OLED_ShowString(0, 2, (u8 *)"Idle Mode", 12, 1);
                    break;
                case MODE_SINGLE_FAN_45DEG:
                    OLED_ShowString(0, 2, (u8 *)"Single Fan 45", 12, 1);
                    break;
                case MODE_SINGLE_FAN_ANY:
                    OLED_ShowString(0, 2, (u8 *)"Single Fan Any", 12, 1);
                    break;
                case MODE_DUAL_FAN_ANY:
                    OLED_ShowString(0, 2, (u8 *)"Dual Fan Any", 12, 1);
                    break;
                case MODE_DUAL_FAN_SEQUENCE:
                    OLED_ShowString(0, 2, (u8 *)"Sequence Mode", 12, 1);
                    break;
            }
            break;
            
        case STATE_ANGLE_SETTING:
            OLED_ShowString(0, 0, (u8 *)"Set Angle:", 12, 1);
            sprintf(buf, "%.1f", g_targetAngle);
            OLED_ShowString(0, 2, (u8 *)buf, 12, 1);
            break;
            
        case STATE_RUNNING:
            OLED_ShowString(0, 0, (u8 *)"Running", 12, 1);
            sprintf(buf, "Cur:%.1f", current_angle);
            OLED_ShowString(0, 2, (u8 *)buf, 12, 1);
            sprintf(buf, "Tar:%.1f", g_targetAngle);
            OLED_ShowString(0, 4, (u8 *)buf, 12, 1);
            
            if(g_workMode == MODE_SINGLE_FAN_45DEG)
            {
                // 显示10秒计时
                uint32_t elapsed = (ANGLE_CONTROL_GetTime() - g_modeStartTime) / 1000;
                if(elapsed <= 10)
                {
                    sprintf(buf, "Time:%ds", (int)elapsed);
                    OLED_ShowString(0, 6, (u8 *)buf, 12, 1);
                }
            }
            break;
            
        default:
            break;
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
