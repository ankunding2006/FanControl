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

/* ��ʾ�����������ڴ洢��һ����ʾ������ */
static char g_lastDisplayBuf[8][32];  // 8����ʾ��ÿ�����32���ַ�
static SystemState_TypeDef g_lastSystemState = STATE_MENU;  // ��һ�ε�ϵͳ״̬����ʼ��Ϊ�˵�״̬
static WorkMode_TypeDef g_lastWorkMode = MODE_IDLE;        // ��һ�εĹ���ģʽ����ʼ��Ϊ����ģʽ
static float g_lastTargetAngle = -1.0f;              // ��һ����ʾ��Ŀ��Ƕ�
static float g_lastCurrentAngle = -1.0f;             // ��һ����ʾ�ĵ�ǰ�Ƕ�
static uint32_t g_lastElapsedTime = 0xFFFFFFFF;      // ��һ����ʾ�ľ���ʱ��

/* �������� */
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
    delay_ms(100);
    OLED_Init();
	OLED_ColorTurn(0);//0������ʾ��1 ��ɫ��ʾ
    OLED_DisplayTurn(0);//0������ʾ 1 ��Ļ��ת��ʾ
    // ��ʾ��ӭ��Ϣ
    OLED_ShowString(0, 0, (u8 *)"Wind Panel Control", 12, 1);
    OLED_ShowString(0, 2, (u8 *)"System Ready", 12, 1);
    OLED_Refresh();      // ȷ����Ϣ��ʾ��ˢ��
    printf("Wind Panel Control System Started\r\n");
    delay_ms(1000);
    
    // ��ѭ��
    while(1)
    {
        // �û���������
        UserInterface_Process();
        
        DisplayStatus();  // ��ʾ״̬����
        
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
    printf("Key: %d\r\n", key);
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
void DisplayStatus(void)
{
    char buf[8][32];  // ��ʱ���������洢��ǰҪ��ʾ������
    float current_angle = ANGLE_SENSOR_GetAngle();
    uint32_t elapsed = 0;
    uint8_t needUpdate = 0;  // ����Ƿ���Ҫ������ʾ��0=����Ҫ��1=��Ҫ
    uint8_t i;
    
    // �����ʱ������
    for(i = 0; i < 8; i++) {
        memset(buf[i], 0, 32);
    }
    
    // ���ϵͳ״̬����ģʽ�Ƿ�仯
    if(g_systemState != g_lastSystemState || g_workMode != g_lastWorkMode) {
        needUpdate = 1;
    }
    
    // ���Ƕ��Ƿ�仯������0.1�ȵ���
    if(fabs(current_angle - g_lastCurrentAngle) > 0.1f || fabs(g_targetAngle - g_lastTargetAngle) > 0.1f) {
        needUpdate = 1;
    }
    
    // ����ϵͳ״̬�����ʱ������
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
                // ��ʾ10���ʱ
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
    
    // �Ƚ��¾ɻ��������ݣ�����Ƿ���Ҫ����
    if(!needUpdate) {
        for(i = 0; i < 8; i++) {
            if(strcmp(buf[i], g_lastDisplayBuf[i]) != 0) {
                needUpdate = 1;
                break;
            }
        }
    }
    
    // �����Ҫ���£������OLED��ʾ����
    if(needUpdate) {
        OLED_Clear();
        
        // ��������������ʾ��OLED
        for(i = 0; i < 8; i += 2) {
            if(buf[i][0] != '\0') {
                OLED_ShowString(0, i, (u8 *)buf[i], 12, 1);
            }
        }
        
        // ���浱ǰ��ʾ���ݵ���һ�εĻ�����
        for(i = 0; i < 8; i++) {
            strcpy(g_lastDisplayBuf[i], buf[i]);
        }
        
        // ���浱ǰ״̬
        g_lastSystemState = g_systemState;
        g_lastWorkMode = g_workMode;
        g_lastCurrentAngle = current_angle;
        g_lastTargetAngle = g_targetAngle;
        g_lastElapsedTime = elapsed;
        
        // ˢ����Ļ
        OLED_Refresh();
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
