#include "scan.h"
#include "ldcp.h"
#include "range_finder.h"
#include "util.h"
#include "device.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include "timers.h"
#include <math.h>
#include <limits.h>

#define TIMER_FREQUENCY_MHZ 160
#define FIRINGS_PER_REVOLUTION 100

static int remaining_scans = 0;

static int firing_interval = -1;
static int firing_count = 0;

static struct {
  unsigned int timestamps[2];
  int distances[FIRINGS_PER_REVOLUTION];
} scan_data_buffer[2];
static int buffer_index = 0;

extern TIM_HandleTypeDef htim11;

void Scan_UpdateFiringInterval(int time_of_revolution)
{
  float count = roundf(time_of_revolution * 0.75 * TIMER_FREQUENCY_MHZ / FIRINGS_PER_REVOLUTION);

  if (firing_interval < 0)
    firing_interval = (int)count;
  else
    firing_interval = (int)(firing_interval * 0.2 + count * 0.8);
}

void Scan_ResetFiringInterval(void)
{
  firing_interval = -1;
}

void Scan_StartFiringSequence(void)
{
  if (firing_interval < 0)
    return;

  firing_count = 0;

  __HAL_TIM_SET_AUTORELOAD(&htim11, firing_interval);
  __HAL_TIM_SET_COUNTER(&htim11, firing_interval);
  __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim11);
}

void Scan_MakeSingleShot(void)
{
  static ScanDataBuffer TempScanData;
  if (firing_count == 0)
    TempScanData.timestamps[0] = Device_GetCurrentTimestamp();//获取当前时间

  TempScanData.distances[firing_count] = firing_count;

  firing_count++;

  if (firing_count == FIRINGS_PER_REVOLUTION) 
  {  //扫描完一个周期
    BaseType_t MpxHigherPriorityTaskWoken;
    firing_count=0;   
    TempScanData.timestamps[1] = Device_GetCurrentTimestamp();//取得当前时间值
    xQueueSendFromISR(ScanDataQueue, &TempScanData,&MpxHigherPriorityTaskWoken);   //扫描数据入队
    xEventGroupSetBitsFromISR(DeviceInfo,SCANDATASEND,&MpxHigherPriorityTaskWoken);//设置扫描数据发送标记位
    portYIELD_FROM_ISR(MpxHigherPriorityTaskWoken);
      
    
  }
}

int Scan_BuildScanAvailableNotification(char *notification, int buffer_index)
{
  int length = snprintf(notification, SCAN_AVAILABLE_NOTIFICATION_BUF_SIZE, "+scan;");
  length += snprintf(notification + length, SCAN_AVAILABLE_NOTIFICATION_BUF_SIZE - length, "#%08X,#%08X;%d;d1;",
    scan_data_buffer[buffer_index].timestamps[0], scan_data_buffer[buffer_index].timestamps[1], FIRINGS_PER_REVOLUTION);
  for (int i = 0; i < FIRINGS_PER_REVOLUTION; i++)
    length += snprintf(notification + length, SCAN_AVAILABLE_NOTIFICATION_BUF_SIZE - length, "%04X", (uint16_t)scan_data_buffer[buffer_index].distances[i]);
  length += snprintf(notification + length, SCAN_AVAILABLE_NOTIFICATION_BUF_SIZE - length, ";");  
  uint16_t checksum = Util_CalculateCRC16((uint8_t *)notification, length);
  length += snprintf(notification + length, SCAN_AVAILABLE_NOTIFICATION_BUF_SIZE - length, "#%04X\r\n", checksum);
  
  return length;
}

void Scan_BeginBatchScan(int count)
{
  remaining_scans = count;
  Device_MakeStateTransition(DEVICE_ACTIVE);
}

void Scan_BeginContinuousScan(void)
{
  remaining_scans = INT_MAX;
  Device_MakeStateTransition(DEVICE_ACTIVE);
}

void Scan_EndScan(void)
{
  remaining_scans = 0;
  Device_MakeStateTransition(DEVICE_STANDBY);
}

int Scan_GetRemainingScanCount(void)
{
  return remaining_scans;
}
