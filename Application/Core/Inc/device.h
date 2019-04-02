#ifndef __DEVICE_H
#define __DEVICE_H

#include <stdint.h>

typedef enum {
  DEVICE_HALTED, DEVICE_RAMPING, DEVICE_STANDBY, DEVICE_ACTIVE, DEVICE_ERROR
} DeviceState;

void Device_Init(void);
void Device_MakeStateTransition(DeviceState next);
int Device_BuildStateTransitionNotification(DeviceState from, DeviceState to, char *notification);
DeviceState Device_GetCurrentState(void);
const char *Device_GetStateName(DeviceState state);
uint32_t Device_GetCurrentTimestamp(void);

#endif
