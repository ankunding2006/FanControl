#include "angle_sensor.h"
#include "math.h"
#include <stddef.h> /* 添加 NULL 定义的头文件 */

/* 私有宏定义 */
#define ADC_MIN           820     // ADC最小值
#define ADC_MID           2420    // ADC中间值（0度位置）
#define ADC_MAX           4020    // ADC最大值
#define ADC_TIMEOUT_COUNT 1000    // ADC超时计数

/* 私有变量 */
static float angle_offset = 0.0f; // 角度偏移值

/**
  * @brief  角度传感器初始化
  * @retval AngleSensorStatus_TypeDef 初始化状态
  */
AngleSensorStatus_TypeDef ANGLE_SENSOR_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    
    // 配置GPIO - 修改为PA3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置ADC
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // 修改为ADC通道3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_55Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    // ADC校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    return ANGLE_SENSOR_OK;
}
/**
  * @brief  获取当前角度值
  * @retval float: 当前角度值，范围[-90, 90]度
  */
float ANGLE_SENSOR_GetAngle(void)
{
    static uint16_t adc_buffer[8];
    uint32_t sum = 0;
    uint32_t timeout;
    uint16_t avg_adc;
    float actual_angle;
    int i;
    
    // 8次采样
    for(i=0; i<8; i++) {
        timeout = ADC_TIMEOUT_COUNT;
        while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
            if(--timeout == 0) {
                return 0.0f;  // ADC读取超时，返回0度
            }
        }
        adc_buffer[i] = ADC_GetConversionValue(ADC1);
        sum += adc_buffer[i];
    }
    
    // 计算平均值
    avg_adc = sum / 8;
    
    // 计算角度
    if(avg_adc <= ADC_MID) {
        actual_angle = (avg_adc - ADC_MID) * 90.0f / (ADC_MID - ADC_MIN);
    } else {
        actual_angle = (avg_adc - ADC_MID) * 90.0f / (ADC_MAX - ADC_MID);
    }
    
    // 限幅
    if(actual_angle > 90.0f) actual_angle = 90.0f;
    if(actual_angle < -90.0f) actual_angle = -90.0f;
    
    // 死区处理
    if(fabs(actual_angle) < 0.5f) actual_angle = 0.0f;
    
    // 应用偏移
    actual_angle += angle_offset;
    
    return actual_angle;
}

/**
  * @brief  获取角度详细数据
  * @param  angle_data: 角度数据结构体指针
  * @retval AngleSensorStatus_TypeDef: 传感器状态
  */
AngleSensorStatus_TypeDef ANGLE_SENSOR_GetData(AngleData_TypeDef *angle_data)
{
    uint16_t raw_adc;
    
    if(angle_data == NULL) return ANGLE_SENSOR_ERROR;
    
    raw_adc = ANGLE_SENSOR_ReadRaw();
    if(raw_adc < ADC_MIN || raw_adc > ADC_MAX) {
        angle_data->status = ANGLE_SENSOR_ERROR;
        return ANGLE_SENSOR_ERROR;
    }
    
    angle_data->angle = ANGLE_SENSOR_GetAngle();
    angle_data->raw_angle = angle_data->angle - angle_offset;
    angle_data->timestamp = 0; // 需要系统时间支持
    angle_data->status = ANGLE_SENSOR_OK;
    
    return ANGLE_SENSOR_OK;
}

/**
  * @brief  校准角度传感器
  * @retval AngleSensorStatus_TypeDef: 校准状态
  */
AngleSensorStatus_TypeDef ANGLE_SENSOR_Calibrate(void)
{
    float current_angle = ANGLE_SENSOR_GetAngle() - angle_offset;
    angle_offset = -current_angle; // 将当前位置校准为0度
    return ANGLE_SENSOR_OK;
}

/**
  * @brief  设置角度偏移
  * @param  offset_angle: 偏移角度值
  * @retval 无
  */
void ANGLE_SENSOR_SetOffset(float offset_angle)
{
    angle_offset = offset_angle;
}

/**
  * @brief  读取ADC原始值
  * @retval uint16_t: ADC原始读数
  */
uint16_t ANGLE_SENSOR_ReadRaw(void)
{
    uint32_t timeout = ADC_TIMEOUT_COUNT;
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        if(--timeout == 0) {
            return 0;  // 超时返回0
        }
    }
    return ADC_GetConversionValue(ADC1);
}
