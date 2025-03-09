#ifndef __ANGLE_SENSOR_H
#define __ANGLE_SENSOR_H

#include "stm32f10x.h"

/* 角度传感器状态枚举 */
typedef enum {
    ANGLE_SENSOR_OK = 0,      // 传感器正常
    ANGLE_SENSOR_ERROR = 1,   // 传感器错误
    ANGLE_SENSOR_TIMEOUT = 2  // 传感器超时
} AngleSensorStatus_TypeDef;

/* 角度数据结构体 */
typedef struct {
    float angle;              // 当前角度值 (度)
    float raw_angle;          // 原始角度读数
    uint32_t timestamp;       // 时间戳（读取时间）
    AngleSensorStatus_TypeDef status; // 传感器状态
} AngleData_TypeDef;

/* 函数声明 */
AngleSensorStatus_TypeDef ANGLE_SENSOR_Init(void);
float ANGLE_SENSOR_GetAngle(void);
AngleSensorStatus_TypeDef ANGLE_SENSOR_GetData(AngleData_TypeDef *angle_data);
AngleSensorStatus_TypeDef ANGLE_SENSOR_Calibrate(void);
void ANGLE_SENSOR_SetOffset(float offset_angle);
uint16_t ANGLE_SENSOR_ReadRaw(void);

#endif
