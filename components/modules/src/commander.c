#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "commander.h"
#include "crtp.h"
#include "configblock.h"

#define MIN_THRUST  10000
#define MAX_THRUST  60000

// [CORRECCIÓN] Quitar la definición duplicada de la estructura
// La estructura ya está definida en commander.h

// [CORRECCIÓN] Definir las variables como extern (sin static)
struct CommanderCrtpValues targetVal[2];  // <-- QUITAR 'static'
int side = 0;                             // <-- QUITAR 'static'
bool hoverMode = false;                   // <-- AÑADIR definición

static bool isInit;
static uint32_t lastUpdate;
static bool isInactive;

static void commanderCrtpCB(CRTPPacket* pk);
static void commanderWatchdog(void);
static void commanderWatchdogReset(void);

void commanderInit(void)
{
  if(isInit)
    return;

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
}

bool commanderTest(void)
{
  crtpTest();
  return isInit;
}

static void commanderCrtpCB(CRTPPacket* pk)
{
  targetVal[!side] = *((struct CommanderCrtpValues*)pk->data);
  side = !side;
  commanderWatchdogReset();
}

static void commanderWatchdog(void)
{
  int usedSide = side;
  uint32_t ticktimeSinceUpdate;

  ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;

  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_STABALIZE)
  {
    targetVal[usedSide].roll = 0;
    targetVal[usedSide].pitch = 0;
    targetVal[usedSide].yaw = 0;
  }
  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    targetVal[usedSide].thrust = 0;
    isInactive = true;
  }
  else
  {
    isInactive = false;
  }
}

static void commanderWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired)
{
  int usedSide = side;

  *eulerRollDesired  = targetVal[usedSide].roll;
  *eulerPitchDesired = targetVal[usedSide].pitch;
  *eulerYawDesired   = targetVal[usedSide].yaw;
}

void commanderGetHover(bool* hover, bool* set_hover, float* hover_change) {
    int usedSide = side;
    *hover = targetVal[usedSide].hover; 
    *set_hover = !hoverMode && targetVal[usedSide].hover; 
    *hover_change = targetVal[usedSide].hover ? targetVal[usedSide].thrust : 0;
    hoverMode = targetVal[usedSide].hover;
}

void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  *rollType  = ANGLE;
  *pitchType = ANGLE;
  *yawType   = RATE;
}

// [CORRECCIÓN] Solo una definición de commanderGetThrust
void commanderGetThrust(uint16_t* thrust)
{
  int usedSide = side;
  uint16_t rawThrust = targetVal[usedSide].thrust;

  if (rawThrust > MIN_THRUST)
  {
    *thrust = rawThrust;
  }
  else
  {
    *thrust = 0;
  }

  if (rawThrust > MAX_THRUST)
  {
    *thrust = MAX_THRUST;
  }

  commanderWatchdog();
}