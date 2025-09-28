#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "commander.h" // Ahora incluye la definición de struct CommanderCrtpValues
#include "crtp.h"
#include "configblock.h"

#define MIN_THRUST  10000
#define MAX_THRUST  60000

// [CORRECCIÓN] Definiciones de variables globales (sin 'static') para acceso externo (ej: log.c)
struct CommanderCrtpValues targetVal[2];
int side = 0;

// Variables internas (static)
static bool isInit;
static uint32_t lastUpdate;
static bool isInactive;
static QueueHandle_t commandQueue = NULL;
static TaskHandle_t commanderTaskHandle = NULL;

// Declaraciones de funciones internas (static)
static void commanderCrtpCB(CRTPPacket* pk);
static void commanderWatchdog(void);
static void commanderWatchdogReset(void);
static void commanderTask(void *param);

void commanderInit(void)
{
  if(isInit)
    return;

  crtpInit();
  
  // Inicializar cola CRTP para commander
  crtpInitTaskQueue(CRTP_PORT_COMMANDER);
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);

  // Crear cola interna para comandos
  // La línea con sizeof(struct CommanderCrtpValues) ahora funciona.
  commandQueue = xQueueCreate(10, sizeof(struct CommanderCrtpValues));
  if (commandQueue == NULL) {
    ESP_LOGE("COMMANDER", "Error creando cola interna de comandos");
    return;
  }

  // Crear tarea de procesamiento
  if (xTaskCreate(commanderTask, "COMMANDER_TASK", 3072, NULL, 3, &commanderTaskHandle) != pdPASS) {
    ESP_LOGE("COMMANDER", "Error creando tarea commander");
    vQueueDelete(commandQueue);
    commandQueue = NULL;
    return;
  }

  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
  
  ESP_LOGI("COMMANDER", "Commander inicializado - Cola para puerto %d creada", CRTP_PORT_COMMANDER);
}

bool commanderTest(void)
{
  return isInit && (commandQueue != NULL);
}

// Tarea de procesamiento de comandos
static void commanderTask(void *param)
{
    // La declaración local de 'cmd' ahora es válida.
    struct CommanderCrtpValues cmd;
    
    ESP_LOGI("COMMANDER", "Tarea commander iniciada");
    
    while (true) {
        // Procesar comandos de la cola interna
        if (xQueueReceive(commandQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            targetVal[!side] = cmd;
            side = !side;
            commanderWatchdogReset();
            
            // Log para debug (opcional)
            ESP_LOGD("COMMANDER", "Comando procesado - Thrust: %d", cmd.thrust);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz
    }
}

// Callback CRTP - se ejecuta cuando llega un paquete al puerto 3
static void commanderCrtpCB(CRTPPacket* pk)
{
    // sizeof() ahora es válido.
    if (pk == NULL || pk->size < sizeof(struct CommanderCrtpValues)) {
        ESP_LOGW("COMMANDER", "Paquete CRTP inválido");
        return;
    }
    
    // La variable 'newCmd' ahora es válida.
    struct CommanderCrtpValues newCmd = *((struct CommanderCrtpValues*)pk->data);
    
    // Enviar a la cola interna (no bloqueante)
    if (commandQueue != NULL) {
        if (xQueueSend(commandQueue, &newCmd, 0) != pdTRUE) {
            ESP_LOGW("COMMANDER", "Cola interna llena, comando descartado");
        }
    }
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

  if (eulerRollDesired) *eulerRollDesired = targetVal[usedSide].roll;
  if (eulerPitchDesired) *eulerPitchDesired = targetVal[usedSide].pitch;
  if (eulerYawDesired) *eulerYawDesired = targetVal[usedSide].yaw;
}

void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  if (rollType) *rollType = ANGLE;
  if (pitchType) *pitchType = ANGLE;
  if (yawType) *yawType = RATE;
}

void commanderGetTrust(uint16_t* thrust)
{
  if (thrust == NULL) return;
  
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