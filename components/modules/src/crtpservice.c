#include <stdbool.h>

/* FreeRtos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "crtp.h"
#include "crtpservice.h"

static bool isInit=false;

typedef enum {
  linkEcho   = 0x00,
  linkSource = 0x01,
  linkSink   = 0x02,
} LinkNbr;

void crtpserviceHandler(CRTPPacket *p);

void crtpserviceInit(void)
{
  if (isInit)
    return;

  // Register a callback to service the Link port
  crtpRegisterPortCB(CRTP_PORT_LINK, crtpserviceHandler);
  
  isInit = true;
}

bool crtpserviceTest(void)
{
  return isInit;
}

void crtpserviceHandler(CRTPPacket *p)
{
  switch (p->channel)
  {
    case linkEcho:
      crtpSendPacket(p);
      break;
    case linkSource:
      p->size = CRTP_MAX_DATA_SIZE;
      crtpSendPacket(p);
      break;
    case linkSink:
      /* Ignore packet */
      break;
    default:
      break;
  } 
}

