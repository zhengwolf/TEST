#ifndef __LDCP_H
#define __LDCP_H

#include "device.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "scan.h"
#include "request_handler.h"
#include "timers.h"

#include <stdbool.h>
#include <string.h>

#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip/ip.h"
#include "lwip/tcp.h"

#define RESPONSE_BUF_SIZE 32
#define STATE_TRANSITION_NOTIFICATION_BUF_SIZE 32
#define SCAN_AVAILABLE_NOTIFICATION_BUF_SIZE 8192

enum com_state {
  FIRST_TOKEN,
  SECOND_TOKEN,
  COMMAND,
  SUMCHECK,
  STATE_CLOSE
};

typedef struct{
  unsigned int timestamps[2];
  int distances[100];
}ScanDataBuffer;

typedef struct{
  unsigned short token;
  unsigned char cmd;
  unsigned char sumcheck;
}ComDataBuffer;

typedef struct{
  unsigned short token;
  unsigned char pstate;
  unsigned char cstate;
  unsigned char sumcheck;
}StateDataBuffer;

extern QueueHandle_t ScanDataQueue;//扫描数据缓存队列
extern TIM_HandleTypeDef htim11;
extern EventGroupHandle_t DeviceInfo;

#define CONNECT        0X01
#define DISCONNECT     0X02
#define SCANDATASEND   0X04
#define ACKDATASEND    0X08
#define STATEDATASEND  0X10
#define SCANNING       0X20

void LDCP_Init(void);
void LDCP_NotifyStateTransition(DeviceState from, DeviceState to);
void LDCP_NotifyScanAvailable(int buffer_index);

#endif
