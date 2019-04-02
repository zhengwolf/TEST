#include "ldcp.h"




#define LDCP_PORT 2105

static QueueHandle_t state_transition_event_queue = NULL;
static QueueHandle_t scan_available_event_queue = NULL;

EventGroupHandle_t DeviceInfo;  //定义一个事件标志组，用于指示各种状态
QueueHandle_t ReceiveCommandQueue;//TCP接收命令缓存队列
QueueHandle_t ScanDataQueue;//扫描数据缓存队列
QueueHandle_t AckDataQueue;//应答数据缓存队列
QueueHandle_t StateDataQueue;//状态数据缓存队列
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
  
  DeviceInfo=xEventGroupCreate();//创建事件标志组用于指示状态
  ReceiveCommandQueue=xQueueCreate(5,sizeof(uint8_t));//创建命令接收缓存队列  
  StateDataQueue=xQueueCreate(3,sizeof(StateDataBuffer*));//创建状态数据缓存队列
  AckDataQueue=xQueueCreate(3,sizeof(ComDataBuffer));//创建应答数据缓存队列
  ScanDataQueue=xQueueCreate(2,sizeof(ScanDataBuffer));//创建扫描数据缓存队列
  sys_thread_new("connect_task", ConnectTask, NULL, 50, 7);//创建服务器TCP连接任务，任务名"connect_task"，任务函数ConnectTask，参数无，堆栈为200，优先级为6
  sys_thread_new("receive_task", TCP_ReceiveTask, NULL, 100, 4);//创建TCP数据接收任务
  sys_thread_new("send_data_task", TCP_SendDataTask, NULL, 600, 6);//创建TCP数据发送任务
  sys_thread_new("command_task", CommandTask, NULL, 100, 5);//创建命令处理任务
  

}

void ConnectTask()
{
  struct netconn *conn = netconn_new(NETCONN_TCP);  //创建TCP连接
  netconn_bind(conn, IP_ADDR_ANY, LDCP_PORT);  //绑定本地IP与端口2105
  netconn_listen(conn); //监听等待客户端连接
  while(1)
  {
    err_t result = netconn_accept(conn, &new_conn);  //连接客户端，该函数会阻塞，直到客户端连接上
    if (result == ERR_OK) 
    {
      ip_set_option(new_conn->pcb.ip, SOF_KEEPALIVE); //设置连接类型
      tcp_nagle_disable(new_conn->pcb.tcp);//禁止nagle算法
     
      xEventGroupSetBits(DeviceInfo,CONNECT);//置位连接状态位，表示已经连接上客户端
      xEventGroupClearBits(DeviceInfo,DISCONNECT);//清除断开状态位，表示处于连接状态 
      xEventGroupWaitBits(DeviceInfo,DISCONNECT,pdTRUE,pdTRUE,portMAX_DELAY); //阻塞等待断开状态位置位 
      xEventGroupClearBits(DeviceInfo,CONNECT);//清除连接状态位，表示未连接上客户端     
    }
    netconn_close(new_conn);  //关闭连接
    netconn_delete(new_conn);//删除连接
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
           xQueueSend(ReceiveCommandQueue, &CommandReceived, portMAX_DELAY);   //命令入队
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
    xEventGroupWaitBits(DeviceInfo,CONNECT,pdFALSE,pdTRUE,portMAX_DELAY); //客户端正常连接 
    
    err_t result =netconn_recv(new_conn, &buf);  //阻塞等待接收数据
    if (result == ERR_OK) 
    {               
      void *data = NULL;
      uint16_t length = 0;        
      do 
      {
        netbuf_data(buf, &data, &length);// 取数据
        DataProcess((uint8_t *)data, length);//处理数据
      }
      while (netbuf_next(buf) >= 0); //移动到下一个pbuf
      
      netbuf_delete(buf);
    }
    else   //当返回值不为ERR_OK，连接断开
    {
      xEventGroupSetBits(DeviceInfo,DISCONNECT);//设置断开状态位，断开连接
      xEventGroupClearBits(DeviceInfo,CONNECT);//清除连接状态位，表示未连接上客户端  
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
    xQueueReceive(ReceiveCommandQueue, &CommandReceived, portMAX_DELAY);//从队列中取出命令
    switch(CommandReceived)
    {
      case 0x01:
        __HAL_TIM_SET_AUTORELOAD(&htim11, 8000); 
        __HAL_TIM_SET_COUNTER(&htim11, 8000);
        __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE);
        __HAL_TIM_ENABLE(&htim11);//开始扫描
        xEventGroupSetBits(DeviceInfo,SCANNING);//置位扫描标记位
        AckData.token=0xa55a;
        AckData.cmd=0x55;
        AckData.sumcheck=(uint8_t)(0x5a+0xa5+0x55);
        
        xQueueSend(AckDataQueue, &AckData, 10);   //应答入队
        xEventGroupSetBits(DeviceInfo,ACKDATASEND);//置位应答数据发送标记位
        break;
        
      case 0x02:
        __HAL_TIM_DISABLE(&htim11);///停止扫描
        xEventGroupClearBits(DeviceInfo,SCANNING);//清除扫描标记位
        AckData.token=0xa55a;
        AckData.cmd=0x55;
        AckData.sumcheck=(uint8_t)(0x5a+0xa5+0x55);
        
        xQueueSend(AckDataQueue, &AckData, 10);   //应答入队
        xEventGroupSetBits(DeviceInfo,ACKDATASEND);//置位应答数据发送标记位
        
        break;
        
      case 0x03://发送状态信息             
        if(xEventGroupGetBits(DeviceInfo) & SCANNING)
        {
          StateDataTemp.token=0xa55a;
          StateDataTemp.pstate=0x02;
          StateDataTemp.cstate=0x01;
          StateDataTemp.sumcheck=(uint8_t)(0x5a+0xa5+0x01+0x02);
          xQueueSend(StateDataQueue, &pStateDataTemp, 10);   //应答入队
          xEventGroupSetBits(DeviceInfo,STATEDATASEND);//置位应答数据发送标记位
        }
        else
        {
          StateDataTemp.token=0xa55a;
          StateDataTemp.pstate=0x01;
          StateDataTemp.cstate=0x02;
          StateDataTemp.sumcheck=(uint8_t)(0x5a+0xa5+0x02+0x01);
          xQueueSend(StateDataQueue, &pStateDataTemp, 10);   //应答入队
          xEventGroupSetBits(DeviceInfo,STATEDATASEND);//置位应答数据发送标记位
        }
        
        break;
        
      default:       
        AckData.token=0xa55a;
        AckData.cmd=0xFF;
        AckData.sumcheck=(uint8_t)(0x5a+0xa5+0xFF);
        xQueueSend(AckDataQueue, &AckData, 10);   //应答入队
        xEventGroupSetBits(DeviceInfo,ACKDATASEND);//置位应答数据发送标记位
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
      xEventGroupWaitBits(DeviceInfo,SCANDATASEND|ACKDATASEND|STATEDATASEND,pdFALSE,pdFALSE,portMAX_DELAY); //阻塞等待需要发送的数据
      if(xEventGroupGetBits(DeviceInfo) & SCANDATASEND) //
      {           
          xQueueReceive(ScanDataQueue, &TempScanData, 10); //取得扫描数据并发送
          netconn_write(new_conn, &TempScanData, sizeof(TempScanData), NETCONN_NOCOPY);
          xEventGroupClearBits(DeviceInfo,SCANDATASEND);//清除扫描数据发送标记位
      }
      if(xEventGroupGetBits(DeviceInfo) & ACKDATASEND)
      {
          xQueueReceive(AckDataQueue, &AckData, 10); //取得应答数据并发送
          netconn_write(new_conn, &AckData, sizeof(AckData), NETCONN_NOCOPY);
          xEventGroupClearBits(DeviceInfo,ACKDATASEND);//清除应答数据发送标记位
      }
      if(xEventGroupGetBits(DeviceInfo) & STATEDATASEND)
      {
          xQueueReceive(StateDataQueue, &pStateData, 10); //取得状态数据并发送
          netconn_write(new_conn, pStateData, sizeof(StateDataBuffer)-1, NETCONN_NOCOPY);
          xEventGroupClearBits(DeviceInfo,STATEDATASEND);//清除状态数据发送标记位
      }
    
  }
}

static void LDCP_ServerThread(void *arg)
{
  struct netconn *conn = netconn_new(NETCONN_TCP);  //创建TCP连接

  netconn_bind(conn, IP_ADDR_ANY, LDCP_PORT);  //绑定本地任意IP与端口2105
  netconn_listen(conn); //监听等待客户端连接

  struct netconn *new_conn = NULL;
  while (true) {
    err_t result = netconn_accept(conn, &new_conn);  //连接客户端，该函数会阻塞，直到客户端连接上
    if (result == ERR_OK) {
      ip_set_option(new_conn->pcb.ip, SOF_KEEPALIVE); //设置连接类型
      tcp_nagle_disable(new_conn->pcb.tcp);//禁止nagle算法
      LDCP_Serve(new_conn);
    }
  }
}

static void LDCP_Serve(struct netconn *conn)
{
  xQueueReset(state_transition_event_queue);
  xQueueReset(scan_available_event_queue);

  netconn_set_recvtimeout(conn, 1);   //超时时间为1ms

  struct netbuf *buf = NULL;
  while (true) {
    err_t result = netconn_recv(conn, &buf);  //阻塞等待接收数据
    if (result == ERR_TIMEOUT) {   //超时，即没有收到命令
      while (uxQueueMessagesWaiting(scan_available_event_queue) > 0) {  //阻塞等待扫描数据入队
        uint32_t buffer_index;
        xQueueReceive(scan_available_event_queue, &buffer_index, 0);//从队列中取出数据
        LDCP_SendScanAvailableNotification(conn, buffer_index);   //发送
      }
      while (uxQueueMessagesWaiting(state_transition_event_queue) > 0) {//阻塞等待状态信息入队
        DeviceState transition[2];
        xQueueReceive(state_transition_event_queue, transition, 0);//从队列中取出信息
        LDCP_SendStateTransitionNotification(conn, transition[0], transition[1]);//发送
      }
      continue;
    }
    else if (result == ERR_OK) {               //收到命令
      void *data = NULL;
      uint16_t length = 0;
      do {
        netbuf_data(buf, &data, &length);//读取数据
        LDCP_FormRequest(conn, (char *)data, length);//处理命令
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
        RequestHandler_HandleRequest(token, response);//处理命令
        LDCP_SendResponse(conn, response, strlen(response));//发送
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
  int length = Scan_BuildScanAvailableNotification(notification, buffer_index);//将数值转为字符串
  netconn_write(conn, notification, length, NETCONN_NOCOPY);//通过TCP发送
}
