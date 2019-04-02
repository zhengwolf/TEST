#include "request_handler.h"
#include "motor.h"
#include "scan.h"
#include "device.h"
#include "encoder.h"

#include <stdio.h>
#include <string.h>

void RequestHandler_HandleRequest(char *request, char *response)
{
  char *token = strtok(request, " ");
  if (token == NULL)
    goto BAD_REQUEST;
  if (strcmp(token, "device") == 0) {
    token = strtok(NULL, " ");
    if (token == NULL)
      goto BAD_REQUEST;
    if (strcmp(token, "run") == 0) {
      if (Device_GetCurrentState() != DEVICE_HALTED) {
        sprintf(response, ">c02\r\n");
        return;
      }
      else {
        Motor_Start();
        sprintf(response, ">c00\r\n");
        return;
      }
    }
    else if (strcmp(token, "halt") == 0) {
      DeviceState state = Device_GetCurrentState();
      if (state == DEVICE_HALTED || state == DEVICE_ERROR) {
        sprintf(response, ">c02\r\n");
        return;
      }
      else {
        Motor_Stop();
        sprintf(response, ">c00\r\n");
        return;
      }
    }
  }
  else if (strcmp(token, "scan") == 0) {
    token = strtok(NULL, " ");
    if (token == NULL)
      goto BAD_REQUEST;
    if (strcmp(token, "begin") == 0) {
      if (Device_GetCurrentState() != DEVICE_STANDBY) {
        sprintf(response, ">c02\r\n");
        return;
      }
      else {
        token = strtok(NULL, " ");
        if (token == NULL)
          Scan_BeginContinuousScan();
        else {
          int count = 0;
          sscanf(token, "%d", &count);
          Scan_BeginBatchScan(count);
        }
        sprintf(response, ">c00\r\n");
        return;
      }
    }
    else if (strcmp(token, "end") == 0) {
      if (Device_GetCurrentState() != DEVICE_ACTIVE) {
        sprintf(response, ">c02\r\n");
        return;
      }
      else {
        Scan_EndScan();
        sprintf(response, ">c00\r\n");
        return;
      }
    }
  }
  else if (strcmp(token, "config") == 0) {
    token = strtok(NULL, " ");
    if (token == NULL)
      goto BAD_REQUEST;
    if (strcmp(token, "set") == 0) {
      token = strtok(NULL, " ");
      if (token == NULL)
        goto BAD_REQUEST;
      if (strcmp(token, "frequency") == 0) {
        token = strtok(NULL, " ");
        if (token == NULL)
          goto BAD_REQUEST;
        float target_frequency = 0;
        sscanf(token, "%f", &target_frequency);
        Motor_SetFrequency(target_frequency);
        sprintf(response, ">c00\r\n");
        return;
      }
    }
  }
  else if (strcmp(token, "info") == 0) {
    token = strtok(NULL, " ");
    if (token == NULL)
      goto BAD_REQUEST;
    if (strcmp(token, "status") == 0) {
      token = strtok(NULL, " ");
      if (token == NULL)
        goto BAD_REQUEST;
      if (strcmp(token, "state") == 0) {
        sprintf(response, ">c00;%s\r\n", Device_GetStateName(Device_GetCurrentState()));
        return;
      }
      if (strcmp(token, "frequency") == 0) {
        float frequency = (Encoder_GetCurrentFrequency() == 0) ? 0 : Motor_GetCurrentFrequency();
        sprintf(response, ">c00;%.1f\r\n", frequency);
        return;
      }
      if (strcmp(token, "timestamp") == 0) {
        sprintf(response, ">c00;#%08X\r\n", Device_GetCurrentTimestamp());
        return;
      }
    }
  }

BAD_REQUEST:
  sprintf(response, ">c01\r\n");
  return;
}
