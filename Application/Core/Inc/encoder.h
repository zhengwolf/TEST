#ifndef __ENCODER_H
#define __ENCODER_H

#include <math.h>

void Encoder_Init(void);
void Encoder_HandleTimerUpdate(void);
void Encoder_HandleTimerCapture(int interval);
float Encoder_GetCurrentFrequency(void);

#endif
