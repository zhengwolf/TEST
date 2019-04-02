#include "ldcp.h"




#define LDCP_PORT 2105

static QueueHandle_t state_transition_event_queue = NULL;
static QueueHandle_t scan_available_event_queue = NULL;

EventGroupHandle_t DeviceInfo;  //����һ���¼���־�飬����ָʾ����״̬
QueueHandle_t ReceiveCommandQueue;//TCP������������
QueueHandle_t ScanDataQueue;//ɨ�����ݻ������
QueueHandle_t AckDataQueue;//Ӧ�����ݻ������
QueueHandle_t StateDataQueue;//״̬���ݻ������
struct netconn *new_conn = NULL;


static void LDCP_ServerThread(void *arg);
static void LDCP_Serve(struct netconn *conn);
static void LDCP_FormRequest(struct netconn *conn, char *data, int length);
static void LDCP_SendResponse(struct netconn *conn, char *response, int length);
static void LDCP_SendStateTransitionNotification(struct netconn *conn, DeviceState from, DeviceState to);
static void LDCP_SendScanAvailableNotification(struct netconn *conn, int buffer_index);
void ConnectTask();
void TCP_ReceiveTask();
void CommandTask();
void TCP_SendDataTask();

void LDCP_Init(void)
{
  
  DeviceInfo=xEventGroupCreate();//�����¼���־������ָʾ״̬
  ReceiveCommandQueue=xQueueCreate(5,sizeof(uint8_t));//����������ջ������  
  StateDataQueue=xQueueCreate(3,sizeof(StateDataBuffer*));//����״̬���ݻ������
  AckDataQueue=xQueueCreate(3,sizeof(ComDataBuffer));//����Ӧ�����ݻ������
  ScanDataQueue=xQueueCreate(2,sizeof(ScanDataBuffer));//����ɨ�����ݻ������
  sys_thread_new("connect_task", ConnectTask, NULL, 50, 7);//����������TCP��������������"connect_task"��������ConnectTask�������ޣ���ջΪ200�����ȼ�Ϊ6
  sys_thread_new("receive_task", TCP_ReceiveTask, NULL, 100, 4);//����TCP���ݽ�������
  sys_thread_new("send_data_task", TCP_SendDataTask, NULL, 600, 6);//����TCP���ݷ�������
  sys_thread_new("command_task", CommandTask, NULL, 100, 5);//�������������
  

}

void ConnectTask()
{
  struct netconn *conn = netconn_new(NETCONN_TCP);  //����TCP����
  netconn_bind(conn, IP_ADDR_ANY, LDCP_PORT);  //�󶨱���IP��˿�2105
  netconn_listen(conn); //�����ȴ��ͻ�������
  while(1)
  {
    err_t result = netconn_accept(conn, &new_conn);  //���ӿͻ��ˣ��ú�����������ֱ���ͻ���������
    if (result == ERR_OK) 
    {
      ip_set_option(new_conn->pcb.ip, SOF_KEEPALIVE); //������������
      tcp_nagle_disable(new_conn->pcb.tcp);//��ֹnagle�㷨
     
      xEventGroupSetBits(DeviceInfo,CONNECT);//��λ����״̬λ����ʾ�Ѿ������Ͽͻ���
      xEventGroupClearBits(DeviceInfo,DISCONNECT);//����Ͽ�״̬λ����ʾ��������״̬ 
      xEventGroupWaitBits(DeviceInfo,DISCONNECT,pdTRUE,pdTRUE,portMAX_DELAY); //�����ȴ��Ͽ�״̬λ��λ 
      xEventGroupClearBits(DeviceInfo,CONNECT);//�������״̬λ����ʾδ�����Ͽͻ���     
    }
    netconn_close(new_conn);  //�ر�����
    netconn_delete(new_conn);//ɾ������
  }
}
void DataProcess(uint8_t *data, uint16_t length)    
{
  uint8_t TempBuffer[20],i=0,CommandReceived=0,Judge=FIRST_TOKEN; 
  uint16_t SumCheck=0;
  memcpy(TempBuffer, data, length);
  for(i=0;i<length;i++)
  {
    switch(Judge)
    {
      case FIRST_TOKEN:
        if(TempBuffer[i]==0x5a)
        {
          Judge=SECOND_TOKEN;
          SumCheck+=TempBuffer[i];
        }  
        else
        {
          Judge=FIRST_TOKEN;
          SumCheck=0;
        }
        break;
        
      case SECOND_TOKEN:
        if(TempBuffer[i]==0xa5)
        {
          Judge=COMMAND;
          SumCheck+=TempBuffer[i];
        }  
        else
        {
          Judge=FIRST_TOKEN;
          SumCheck=0;
        }
        break;
        
      case COMMAND:       
        Judge=SUMCHECK;
        CommandReceived=TempBuffer[i];
        SumCheck+=TempBuffer[i];       
        break;
        
      case SUMCHECK:
        if((uint8_t)SumCheck==TempBuffer[i])
        {
           xQueueSend(ReceiveCommandQueue, &CommandReceived, portMAX_DELAY);   //�������
        }
        Judge=FIRST_TOKEN;
        SumCheck=0;     
        break;
      
      default:
        Judge=FIRST_TOKEN;
        SumCheck=0;
        break;
    }
  } 
}
void TCP_ReceiveTask()
{
  struct netbuf *buf = NULL;
  while(1)
  {
    xEventGroupWaitBits(DeviceInfo,CONNECT,pdFALSE,pdTRUE,portMAX_DELAY); //�ͻ����������� 
    
    err_t result =netconn_recv(new_conn, &buf);  //�����ȴ���������
    if (result == ERR_OK) 
    {               
      void *data = NULL;
      uint16_t length = 0;        
      do 
      {
        netbuf_data(buf, &data, &length);// ȡ����
        DataProcess((uint8_t *)data, length);//��������
      }
      while (netbuf_next(buf) >= 0); //�ƶ�����һ��pbuf
      
      netbuf_delete(buf);
    }
    else   //������ֵ��ΪERR_OK�����ӶϿ�
    {
      xEventGroupSetBits(DeviceInfo,DISCONNECT);//���öϿ�״̬λ���Ͽ�����
      xEventGroupClearBits(DeviceInfo,CONNECT);//�������״̬λ����ʾδ�����Ͽͻ���  
    }
    
   // osDelay(10);
  }
}

void CommandTask()
{
  uint8_t CommandReceived;
  ComDataBuffer AckData;
  StateDataBuffer StateDataTemp;
  StateDataBuffer* pStateDataTemp;
  pStateDataTemp=&StateDataTemp;
  while(1)
  {
    xQueueReceive(ReceiveCommandQueue, &CommandReceived, portMAX_DELAY);//�Ӷ�����ȡ������
    switch(CommandReceived)
    {
      case 0x01:
        __HAL_TIM_SET_AUTORELOAD(&htim11, 8000); 
        __HAL_TIM_SET_COUNTER(&htim11, 8000);
        __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE);
        __HAL_TIM_ENABLE(&htim11);//��ʼɨ��
        xEventGroupSetBits(DeviceInfo,SCANNING);//��λɨ����λ
        AckData.token=0xa55a;
        AckData.cmd=0x55;
        AckData.sumcheck=(uint8_t)(0x5a+0xa5+0x55);
        
        xQueueSend(AckDataQueue, &AckData, 10);   //Ӧ�����
        xEventGroupSetBits(DeviceInfo,ACKDATASEND);//��λӦ�����ݷ��ͱ��λ
        break;
        
      case 0x02:
        __HAL_TIM_DISABLE(&htim11);///ֹͣɨ��
        xEventGroupClearBits(DeviceInfo,SCANNING);//���ɨ����λ
        AckData.token=0xa55a;
        AckData.cmd=0x55;
        AckData.sumcheck=(uint8_t)(0x5a+0xa5+0x55);
        
        xQueueSend(AckDataQueue, &AckData, 10);   //Ӧ�����
        xEventGroupSetBits(DeviceInfo,ACKDATASEND);//��λӦ�����ݷ��ͱ��λ
        
        break;
        
      case 0x03://����״̬��Ϣ             
        if(xEventGroupGetBits(DeviceInfo) & SCANNING)
        {
          StateDataTemp.token=0xa55a;
          StateDataTemp.pstate=0x02;
          StateDataTemp.cstate=0x01;
          StateDataTemp.sumcheck=(uint8_t)(0x5a+0xa5+0x01+0x02);
          xQueueSend(StateDataQueue, &pStateDataTemp, 10);   //Ӧ�����
          xEventGroupSetBits(DeviceInfo,STATEDATASEND);//��λӦ�����ݷ��ͱ��λ
        }
        else
        {
          StateDataTemp.token=0xa55a;
          StateDataTemp.pstate=0x01;
          StateDataTemp.cstate=0x02;
          StateDataTemp.sumcheck=(uint8_t)(0x5a+0xa5+0x02+0x01);
          xQueueSend(StateDataQueue, &pStateDataTemp, 10);   //Ӧ�����
          xEventGroupSetBits(DeviceInfo,STATEDATASEND);//��λӦ�����ݷ��ͱ��λ
        }
        
        break;
        
      default:       
        AckData.token=0xa55a;
        AckData.cmd=0xFF;
        AckData.sumcheck=(uint8_t)(0x5a+0xa5+0xFF);
        xQueueSend(AckDataQueue, &AckData, 10);   //Ӧ�����
        xEventGroupSetBits(DeviceInfo,ACKDATASEND);//��λӦ�����ݷ��ͱ��λ
        break;
    }
  }
}

void TCP_SendDataTask()
{
  ScanDataBuffer TempScanData;
  ComDataBuffer AckData;
  StateDataBuffer *pStateData;
  while(1)
  {
      xEventGroupWaitBits(DeviceInfo,SCANDATASEND|ACKDATASEND|STATEDATASEND,pdFALSE,pdFALSE,portMAX_DELAY); //�����ȴ���Ҫ���͵�����
      if(xEventGroupGetBits(DeviceInfo) & SCANDATASEND) //
      {           
          xQueueReceive(ScanDataQueue, &TempScanData, 10); //ȡ��ɨ�����ݲ�����
          netconn_write(new_conn, &TempScanData, sizeof(TempScanData), NETCONN_NOCOPY);
          xEventGroupClearBits(DeviceInfo,SCANDATASEND);//���ɨ�����ݷ��ͱ��λ
      }
      if(xEventGroupGetBits(DeviceInfo) & ACKDATASEND)
      {
          xQueueReceive(AckDataQueue, &AckData, 10); //ȡ��Ӧ�����ݲ�����
          netconn_write(new_conn, &AckData, sizeof(AckData), NETCONN_NOCOPY);
          xEventGroupClearBits(DeviceInfo,ACKDATASEND);//���Ӧ�����ݷ��ͱ��λ
      }
      if(xEventGroupGetBits(DeviceInfo) & STATEDATASEND)
      {
          xQueueReceive(StateDataQueue, &pStateData, 10); //ȡ��״̬���ݲ�����
          netconn_write(new_conn, pStateData, sizeof(StateDataBuffer)-1, NETCONN_NOCOPY);
          xEventGroupClearBits(DeviceInfo,STATEDATASEND);//���״̬���ݷ��ͱ��λ
      }
    
  }
}

static void LDCP_ServerThread(void *arg)
{
  struct netconn *conn = netconn_new(NETCONN_TCP);  //����TCP����

  netconn_bind(conn, IP_ADDR_ANY, LDCP_PORT);  //�󶨱�������IP��˿�2105
  netconn_listen(conn); //�����ȴ��ͻ�������

  struct netconn *new_conn = NULL;
  while (true) {
    err_t result = netconn_accept(conn, &new_conn);  //���ӿͻ��ˣ��ú�����������ֱ���ͻ���������
    if (result == ERR_OK) {
      ip_set_option(new_conn->pcb.ip, SOF_KEEPALIVE); //������������
      tcp_nagle_disable(new_conn->pcb.tcp);//��ֹnagle�㷨
      LDCP_Serve(new_conn);
    }
  }
}

static void LDCP_Serve(struct netconn *conn)
{
  xQueueReset(state_transition_event_queue);
  xQueueReset(scan_available_event_queue);

  netconn_set_recvtimeout(conn, 1);   //��ʱʱ��Ϊ1ms

  struct netbuf *buf = NULL;
  while (true) {
    err_t result = netconn_recv(conn, &buf);  //�����ȴ���������
    if (result == ERR_TIMEOUT) {   //��ʱ����û���յ�����
      while (uxQueueMessagesWaiting(scan_available_event_queue) > 0) {  //�����ȴ�ɨ���������
        uint32_t buffer_index;
        xQueueReceive(scan_available_event_queue, &buffer_index, 0);//�Ӷ�����ȡ������
        LDCP_SendScanAvailableNotification(conn, buffer_index);   //����
      }
      while (uxQueueMessagesWaiting(state_transition_event_queue) > 0) {//�����ȴ�״̬��Ϣ���
        DeviceState transition[2];
        xQueueReceive(state_transition_event_queue, transition, 0);//�Ӷ�����ȡ����Ϣ
        LDCP_SendStateTransitionNotification(conn, transition[0], transition[1]);//����
      }
      continue;
    }
    else if (result == ERR_OK) {               //�յ�����
      void *data = NULL;
      uint16_t length = 0;
      do {
        netbuf_data(buf, &data, &length);//��ȡ����
        LDCP_FormRequest(conn, (char *)data, length);//��������
      }
      while (netbuf_next(buf) >= 0);

      netbuf_delete(buf);
    }
    else
      break;
  }

  netconn_close(conn);
  netconn_delete(conn);
}

#define REQUEST_BUF_SIZE 32

static void LDCP_FormRequest(struct netconn *conn, char *data, int length)
{
  static char request[REQUEST_BUF_SIZE];
  static char response[RESPONSE_BUF_SIZE];
  static uint8_t index = 0;

  while (length > 0) {
    int count = (index + length < REQUEST_BUF_SIZE) ? length : (REQUEST_BUF_SIZE - index - 1);
    memcpy(request + index, data, count);
    
    data += count;
    length -= count;
    index += count;
    request[index] = '\0';

    char *context = NULL;
    char *token = strtok_r(request, "\r\n", &context);
    while (token != NULL) {
      if (token + strlen(token) == request + index) {
        memmove(request, token, strlen(token) + 1);
        index = strlen(request);
        break;
      }
      else {
        RequestHandler_HandleRequest(token, response);//��������
        LDCP_SendResponse(conn, response, strlen(response));//����
        token = strtok_r(NULL, "\r\n", &context);
      }
    }

    if (token == NULL)
      index = 0;
  }
}

static void LDCP_SendResponse(struct netconn *conn, char *response, int length)
{
  netconn_write(conn, response, length, NETCONN_COPY);
}

void LDCP_NotifyStateTransition(DeviceState from, DeviceState to)
{
  DeviceState transition[2] = { from, to };
  if (__get_IPSR() != 0)
    xQueueSendFromISR(state_transition_event_queue, transition, 0);
  else
    xQueueSend(state_transition_event_queue, transition, 0);
}

static void LDCP_SendStateTransitionNotification(struct netconn *conn, DeviceState from, DeviceState to)
{
  static char notification[STATE_TRANSITION_NOTIFICATION_BUF_SIZE];
  int length = Device_BuildStateTransitionNotification(from, to, notification);
  netconn_write(conn, notification, length, NETCONN_NOCOPY);
}

void LDCP_NotifyScanAvailable(int buffer_index)
{
  xQueueSendFromISR(scan_available_event_queue, &buffer_index, NULL);
}

static void LDCP_SendScanAvailableNotification(struct netconn *conn, int buffer_index)
{
  static char notification[SCAN_AVAILABLE_NOTIFICATION_BUF_SIZE];
  int length = Scan_BuildScanAvailableNotification(notification, buffer_index);//����ֵתΪ�ַ���
  netconn_write(conn, notification, length, NETCONN_NOCOPY);//ͨ��TCP����
}
