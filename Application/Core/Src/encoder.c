#include "encoder.h"
#include "device.h"
#include "scan.h"

#include "stm32f4xx_hal.h"

#define PULSES_PER_REVOLUTION 64

typedef enum {
  ENCODER_HALTED, ENCODER_UNSYNCHRONIZED, ENCODER_SYNCHRONIZING, ENCODER_SYNCHRONIZED
} EncoderState;

static EncoderState state = ENCODER_HALTED;

static float current_frequency = NAN;

extern TIM_HandleTypeDef htim12;

void Encoder_Init(void)
{
  HAL_TIM_Base_Start_IT(&htim12);
  HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
}

void Encoder_HandleTimerUpdate(void)
{
  state = ENCODER_HALTED;
  current_frequency = 0;
}

void Encoder_HandleTimerCapture(int interval)
{
  static int saved_interval = -1;
  static int position = -1;
  static int time_of_revolution = -1;

  switch (state) {
    case ENCODER_HALTED: {
      state = ENCODER_UNSYNCHRONIZED;
      break;
    }

    case ENCODER_UNSYNCHRONIZED: {
      current_frequency = NAN;
      position = time_of_revolution = -1;

      saved_interval = interval;
      state = ENCODER_SYNCHRONIZING;
      break;
    }

    case ENCODER_SYNCHRONIZING: {
      if (interval > saved_interval * 2) {   //起始点
        position = time_of_revolution = 0;
        state = ENCODER_SYNCHRONIZED;
      }
      saved_interval = interval;
      break;
    }

    case ENCODER_SYNCHRONIZED: {
      time_of_revolution += interval;

      if (interval > saved_interval * 2) {//再次到达起始点，表示已经完成一圈
        position += 3;
        if (position != PULSES_PER_REVOLUTION * 2) { //检测位置是否正确
          state = ENCODER_UNSYNCHRONIZED;
          break;
        }
        else {                 //位置正确
          if (isnan(current_frequency))
            current_frequency = 1000000.0 / time_of_revolution;
          else
            current_frequency = current_frequency * 0.3 + (1000000.0 / time_of_revolution) * 0.7;
          
          DeviceState state = Device_GetCurrentState();
          if (state == DEVICE_STANDBY || state == DEVICE_ACTIVE)
            Scan_UpdateFiringInterval(time_of_revolution);
            
          position = time_of_revolution = 0;
        }
      }
      else
        position += 1;

      if (position == 80 && Scan_GetRemainingScanCount() > 0)  //位置为80且剩余扫描数大于0（剩余扫描数量最开始由用户通过命令输入）
        Scan_StartFiringSequence();//开始扫描

      saved_interval = interval;
      break;
    }
  }
}

float Encoder_GetCurrentFrequency(void)
{
  return current_frequency;
}
