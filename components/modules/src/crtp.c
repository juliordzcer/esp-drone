#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include "esp_log.h"

/*FreeRtos includes*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "config.h"
#include "crtp.h"
#include "info.h"
#include "cfassert.h"

static bool isInit;

static int nopFunc(void);
static struct crtpLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
}; 

static struct crtpLinkOperations *link = &nopLink;

static QueueHandle_t  tmpQueue;
static QueueHandle_t  rxQueue;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 20
#define CRTP_RX_QUEUE_SIZE 2

static void crtpTxTask(void *param);
static void crtpRxTask(void *param);

static QueueHandle_t queues[CRTP_NBR_OF_PORTS];
static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];

void crtpInit(void)
{
  if(isInit)
    return;

  tmpQueue = xQueueCreate(CRTP_TX_QUEUE_SIZE, sizeof(CRTPPacket));
  rxQueue = xQueueCreate(CRTP_RX_QUEUE_SIZE, sizeof(CRTPPacket));
  /* Start Rx/Tx tasks */
    xTaskCreate(crtpTxTask, "CRTP-Tx",
                configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
    xTaskCreate(crtpRxTask, "CRTP-Rx",
                configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  
  isInit = true;
}

bool crtpTest(void)
{
  return isInit;
}

void crtpInitTaskQueue(CRTPPort portId)
{
  ASSERT(queues[portId] == NULL);
  
  queues[portId] = xQueueCreate(1, sizeof(CRTPPacket));
}

int crtpReceivePacket(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
    
  return xQueueReceive(queues[portId], p, 0);
}

int crtpReceivePacketBlock(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, portMAX_DELAY);
}


int crtpReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait) {
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, M2T(wait));
}

void crtpTxTask(void *param)
{
  CRTPPacket p;

  while (true)
  {
    if (xQueueReceive(tmpQueue, &p, portMAX_DELAY) == pdTRUE)
    {
      link->sendPacket(&p);
    }
  }
}

void crtpRxTask(void *param)
{
  CRTPPacket p;
  static unsigned int droppedPacket=0;

  while (true)
  {
    if (!link->receivePacket(&p))
    {
      if(queues[p.port])
      {
        // TODO: If full, remove one packet and then send
        xQueueSend(queues[p.port], &p, 0);
      } else {
        droppedPacket++;
      }
      
      if(callbacks[p.port])
        callbacks[p.port](&p);  //Dangerous?
    }
  }
}

void crtpRegisterPortCB(int port, CrtpCallback cb)
{
  if (port>CRTP_NBR_OF_PORTS)
    return;
  
  callbacks[port] = cb;
}

int crtpSendPacket(CRTPPacket *p)
{
  ASSERT(p); 

  return xQueueSend(tmpQueue, p, 0);
}

int crtpSendPacketBlock(CRTPPacket *p)
{
  ASSERT(p); 

  return xQueueSend(tmpQueue, p, portMAX_DELAY);
}

void crtpPacketReveived(CRTPPacket *p)
{
  portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(rxQueue, p, &xHigherPriorityTaskWoken);
}

void crtpSetLink(struct crtpLinkOperations * lk)
{
  if(link)
    link->setEnable(false);

  if (lk)
    link = lk;
  else
    link = &nopLink;

  link->setEnable(true);
}

static int nopFunc(void)
{
  return ENETDOWN;
}
