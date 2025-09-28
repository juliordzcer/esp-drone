#include <stdbool.h>

/*FreeRtos includes*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "crtp.h"

CRTPPacket messageToPrint;
// CORRECCIÓN: cambiado de xSemaphoreHandle a SemaphoreHandle_t
SemaphoreHandle_t synch = NULL; 

static bool isInit;

/**
 * Send the data to the client
 */
static void consoleSendMessage(void)
{
  crtpSendPacketBlock(&messageToPrint);
  messageToPrint.size = 0;
}

void consoleInit()
{
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  
  // CORRECCIÓN: Usar la función moderna xSemaphoreCreateBinary() 
  // en lugar de la macro obsoleta vSemaphoreCreateBinary()
  synch = xSemaphoreCreateBinary();
  
  if (synch != NULL) {
      // Un semáforo binario creado por xSemaphoreCreateBinary comienza vacío, 
      // por lo que le damos el 'token' para que esté disponible inmediatamente.
      xSemaphoreGive(synch); 
  }
  
  isInit = true;
}

bool consoleTest(void)
{
  return isInit;
}

int consolePutchar(int ch)
{
  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
    messageToPrint.size++;
    if (ch == '\n' || messageToPrint.size == CRTP_MAX_DATA_SIZE)
    {
      consoleSendMessage();
    }
    xSemaphoreGive(synch);
  }
  
  return (unsigned char)ch;
}

int consolePuts(char *str)
{
  int ret = 0;
  
  while(*str)
    ret |= consolePutchar(*str++);
  
  return ret;
}

void consoleFlush(void)
{
  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    consoleSendMessage();
    xSemaphoreGive(synch);
  }
}