#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"  // <-- Agregar esto

#include "commander.h"
#include "crtp.h"
#include "configblock.h"

#define MIN_THRUST  10000
#define MAX_THRUST  60000

struct CommanderCrtpValues
{
  float roll;
  float pitch;
  float yaw;
  uint16_t thrust;
} __attribute__((packed));

static struct CommanderCrtpValues targetVal[2];
static bool isInit;
static int side=0;
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
  
  crtpInitTaskQueue(CRTP_PORT_COMMANDER);
  
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
  
  ESP_LOGI("COMMANDER", "Commander inicializado - Cola para puerto %d creada", CRTP_PORT_COMMANDER);
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
  
  // Log para debug
  ESP_LOGD("COMMANDER", "Comando recibido - Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Thrust: %d", 
           targetVal[side].roll, targetVal[side].pitch, targetVal[side].yaw, targetVal[side].thrust);
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

void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  *rollType  = ANGLE;
  *pitchType = ANGLE;
  *yawType   = RATE;
}

void commanderGetTrust(uint16_t* thrust)
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