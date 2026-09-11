#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f1xx_hal.h"

extern float now_angle;
extern float now_pwm_duty;

void Servo_SetAngle(float angle);
void VOFA_SendTelemetry(void);
void Servo_UART_Process(void);
void Servo_UART_StartReceive(void);

#endif
