#include "motor.h"
#include "device.h"

#include "stm32f4xx_hal.h"

#include "arm_math.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

typedef enum {
  MOTOR_HALTED, MOTOR_RAMPING, MOTOR_SETTLED, MOTOR_ERROR
} MotorState;

static MotorState state = MOTOR_HALTED;

static float set_frequency = 0;

static float target_frequency = 0;
static arm_pid_instance_f32 pid = { .Kp = 2, .Ki = 2, .Kd = 0 };

static float current_frequency = NAN;
extern TIM_HandleTypeDef htim4;

static void Motor_SpeedControlCallback(TimerHandle_t timer);

static void Motor_PWMOutputOn(void);
static void Motor_PWMOutputOff(void);
static void Motor_PWMSetValue(int value);

void Motor_Init(void)
{
  HAL_GPIO_WritePin(DRV11873_FR_GPIO_Port, DRV11873_FR_Pin, GPIO_PIN_SET);
  
  arm_pid_init_f32(&pid, 1);//初始化PID控制器

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

  TimerHandle_t motor_control_timer = xTimerCreate(NULL, pdMS_TO_TICKS(200), pdTRUE, NULL, Motor_SpeedControlCallback);
  xTimerStart(motor_control_timer, portMAX_DELAY);//创建一个200ms的定时器
}

void Motor_Start(void)
{
  vTaskSuspendAll();

  if (state == MOTOR_HALTED) {
    state = MOTOR_RAMPING;
    Device_MakeStateTransition(DEVICE_RAMPING);
    target_frequency = set_frequency;
    Motor_PWMSetValue(0);
    Motor_PWMOutputOn();
  }

  xTaskResumeAll();
}

void Motor_Stop(void)
{
  vTaskSuspendAll();

  state = MOTOR_RAMPING;
  Device_MakeStateTransition(DEVICE_RAMPING);
  target_frequency = 0;
  Motor_PWMOutputOff();

  xTaskResumeAll();
}

void Motor_SetFrequency(float frequency)
{
  vTaskSuspendAll();

  set_frequency = frequency;
  if (state != MOTOR_ERROR) {
    target_frequency = set_frequency;
    if (state == MOTOR_SETTLED) {
      state = MOTOR_RAMPING;
      Device_MakeStateTransition(DEVICE_RAMPING);
    }
  }

  xTaskResumeAll();
}

float Motor_GetCurrentFrequency(void)
{
  return current_frequency;
}

void Motor_HandleTimerUpdate(void)
{
  current_frequency = 0;
}

void Motor_HandleTimerCapture(int interval)
{
  current_frequency = (1000000.0 / interval) / 7;  //计算当前电机转速
}

#include "encoder.h"

static void Motor_SpeedControlCallback(TimerHandle_t timer) //200ms调用一次
{
  vTaskSuspendAll();
  
  switch (state) {
    case MOTOR_HALTED:
    case MOTOR_SETTLED:
    case MOTOR_ERROR:
      break;

    case MOTOR_RAMPING: {
      float frequency = (Encoder_GetCurrentFrequency() == 0) ? 0 : current_frequency;//当前频率
      if (isnan(frequency))
        break;

      static int in_range_count = 0;

      if (target_frequency == 0) {  //目标频率为0
        if (frequency == 0) {    //当前频率也为0
          arm_pid_init_f32(&pid, 1);
          state = MOTOR_HALTED;//电机状态为停止
          Device_MakeStateTransition(DEVICE_HALTED);
        }
      }
      else {
        float error = target_frequency - frequency;  //目标频率与当前频率的差值
        if (fabs(error) > 0.5) {   //如果差值大于0.5
          int pulse_width = (int)arm_pid_f32(&pid, error);//使用PID控制器进行调节
          pulse_width = (pulse_width < 50) ? 50 : pulse_width;  //下限
          pulse_width = (pulse_width > 400) ? 400 : pulse_width;//上限

          Motor_PWMSetValue(pulse_width); //设置频率
        }
        else { //如果差值小于0.5
          if (++in_range_count == 10) {//连续10次小于0.5
            arm_pid_init_f32(&pid, 1);
            state = MOTOR_SETTLED;//电机状态为设置完毕
            Device_MakeStateTransition(DEVICE_STANDBY);
          }
          else
            break;
        }
      }

      in_range_count = 0;
      break;
    }
  }

  xTaskResumeAll();
}

extern TIM_HandleTypeDef htim1;

static void Motor_PWMOutputOn(void)
{
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static void Motor_PWMOutputOff(void)
{
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static void Motor_PWMSetValue(int value)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value);
}
