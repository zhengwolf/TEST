#include "device.h"
#include "scan.h"
#include "ldcp.h"

#include "stm32f4xx_hal.h"

#include <stdio.h>

static char *state_names[] = { "halted", "ramping", "standby", "active", "error" };
static DeviceState state = DEVICE_HALTED;

extern TIM_HandleTypeDef htim2;

void Device_Init(void)
{
  HAL_TIM_Base_Start(&htim2);
}

void Device_MakeStateTransition(DeviceState next)
{
  DeviceState previous = state;
  state = next;
  if ((previous == DEVICE_STANDBY || previous == DEVICE_ACTIVE) && (next != DEVICE_STANDBY && next != DEVICE_ACTIVE))
    Scan_ResetFiringInterval();
  LDCP_NotifyStateTransition(previous, next);
}

int Device_BuildStateTransitionNotification(DeviceState from, DeviceState to, char *notification)
{
  int length = snprintf(notification, STATE_TRANSITION_NOTIFICATION_BUF_SIZE, "+state;%s,%s\r\n", state_names[from], state_names[to]);
  return length;
}

DeviceState Device_GetCurrentState(void)
{
  return state;
}

const char *Device_GetStateName(DeviceState state)
{
  return state_names[state];
}

uint32_t Device_GetCurrentTimestamp(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}
