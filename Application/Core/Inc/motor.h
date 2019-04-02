#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetFrequency(float frequency);
float Motor_GetCurrentFrequency(void);
void Motor_HandleTimerUpdate(void);
void Motor_HandleTimerCapture(int interval);

#endif
