#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "oled.h"
#include "fan_driver.h"
#include "angle_sensor.h"
#include "angle_control.h"
#include "pid_controller.h"

/* ϵͳ״̬���� */
typedef enum {
    STATE_MENU = 0,        // �˵�״̬
    STATE_ANGLE_SETTING,   // �Ƕ�����״̬
    STATE_RUNNING,         // ����״̬
    STATE_PARAM_SETTING    // ��������״̬
} SystemState_TypeDef;

/* ����ģʽ���� */
typedef enum {
    MODE_IDLE = 0,            // ����ģʽ
    MODE_SINGLE_FAN_45DEG,    // ������45��ģʽ
    MODE_SINGLE_FAN_ANY,      // ����������Ƕ�ģʽ
    MODE_DUAL_FAN_ANY,        // ˫��������Ƕ�ģʽ
    MODE_DUAL_FAN_SEQUENCE    // ˫��������ģʽ
} WorkMode_TypeDef;

/* ȫ�ֱ��� */
AngleControl_TypeDef g_angle_control;   // �Ƕȿ��ƽṹ��
static SystemState_TypeDef g_systemState;      // ϵͳ״̬
static WorkMode_TypeDef g_workMode;            // ����ģʽ
static float g_targetAngle = 0.0f;            // Ŀ��Ƕ�
static uint32_t g_modeStartTime = 0;          // ģʽ��ʼʱ��

/* �������� */
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
 * @brief  ϵͳ��ʼ������
 * @param  ��
 * @retval ��
 */
static void System_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(115200);
    KEY_Init();
    OLED_Init();
    
    // ��ʼ���Ƕȿ���ϵͳ
    ANGLE_CONTROL_Init(&g_angle_control, CONTROL_MODE_IDLE);
    
    // ��ʼ����ʱ��
    Timer_Init();
    
    // ��ʼ��ϵͳ״̬
    g_systemState = STATE_MENU;
    g_workMode = MODE_IDLE;
}

/**
  * @brief  ��ʱ����ʼ������
  * @param  ��
  * @retval ��
  */
static void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // ʹ�ܶ�ʱ��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    
    // TIM3���� - 10ms�ж�
    TIM_TimeBaseStructure.TIM_Period = 9999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // TIM4���� - 2ms�ж�
    TIM_TimeBaseStructure.TIM_Period = 1999;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // ����NVIC - TIM3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // ����NVIC - TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    
    // ʹ�ܶ�ʱ���ж�
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    // ������ʱ��
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief  ������
  * @param  ��
  * @retval int
  */
int main(void)
{
    // ϵͳ��ʼ��
    System_Init();
    
    // ��ʾ��ӭ��Ϣ
    OLED_Clear();
    OLED_ShowString(0, 0, "Wind Panel Control", 12, 1);
    OLED_ShowString(0, 2, "System Ready", 12, 1);
    printf("Wind Panel Control System Started\r\n");
    delay_ms(1000);
    
    // ��ѭ��
    while(1)
    {
        // �û���������
        UserInterface_Process();
        
        // ״̬��ʾ����
        DisplayStatus();
        
        // ��ʱ
        delay_ms(10);
    }
}

/**
  * @brief  �û����洦����
  * @param  ��
  * @retval ��
  */
static void UserInterface_Process(void)
{
    ProcessKeys();
    MenuManager();
}

/**
  * @brief  ����������
  * @param  ��
  * @retval ��
  */
static void ProcessKeys(void)
{
    uint8_t key = KEY_Scan();
    if(key == KEY_NONE) return;
    
    switch(g_systemState)
    {
        case STATE_MENU:
            // ģʽѡ��
            if(key == KEY_UP || key == KEY_DOWN)
            {
                if(key == KEY_UP) g_workMode++;
                else g_workMode--;
                
                // ɾ���ظ��������жϣ�ֻ�����������߼�
                if(g_workMode > MODE_DUAL_FAN_SEQUENCE) {
                    g_workMode = MODE_IDLE;
                } else if(g_workMode == MODE_IDLE && key == KEY_DOWN) {
                    g_workMode = MODE_DUAL_FAN_SEQUENCE;
                }
            }
            else if(key == KEY_ENTER)
            {
                // ����Ƕ����û�ֱ�ӿ�ʼ
                if(g_workMode == MODE_SINGLE_FAN_45DEG)
                {
                    // 45��ģʽֱ�ӿ�ʼ
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
            // �Ƕ�����
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
                // ��ʼ����
                ConfigureControlMode(g_workMode);
                ANGLE_CONTROL_SetTarget(&g_angle_control, g_targetAngle);
                g_systemState = STATE_RUNNING;
                g_modeStartTime = ANGLE_CONTROL_GetTime();
            }
            else if(key == KEY_MODE)
            {
                // ���ز˵�
                g_systemState = STATE_MENU;
            }
            break;
            
        case STATE_RUNNING:
            if(key == KEY_MODE)
            {
                // ֹͣ���ƣ����ز˵�
                ANGLE_CONTROL_Stop(&g_angle_control);
                g_systemState = STATE_MENU;
            }
            break;
            
        default:
            break;
    }
}

/**
  * @brief  ���ÿ���ģʽ
  * @param  mode: ����ģʽ
  * @retval ��
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
  * @brief  ��ʾ״̬����
  * @param  ��
  * @retval ��
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
                // ��ʾ10���ʱ
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
  * @brief  �˵�������
  * @param  ��
  * @retval ��
  */
static void MenuManager(void)
{
    // ����ϵͳ״̬����˵�
    // �˴�������Ӳ˵������߼�
    return;
}
