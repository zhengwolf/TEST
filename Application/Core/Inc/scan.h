#ifndef __SCAN_H
#define __SCAN_H

void Scan_UpdateFiringInterval(int time_of_revolution);
void Scan_ResetFiringInterval(void);
void Scan_StartFiringSequence(void);
void Scan_MakeSingleShot(void);
int Scan_BuildScanAvailableNotification(char *notification, int buffer_index);
void Scan_BeginBatchScan(int count);
void Scan_BeginContinuousScan(void);
void Scan_EndScan(void);
int Scan_GetRemainingScanCount(void);

#endif
