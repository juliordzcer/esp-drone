#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

/* FreeRtos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "debug.h"
#include "version.h"
#include "config.h"
#include "param.h"
#include "ledseq.h"
#include "pm.h"
#include "adc.h"

#include "system.h"
#include "configblock.h"
#include "worker.h"
#include "uart.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "console.h"

static const char *TAG = "SYSTEM";

/* Private variable */
static bool canFly;
static bool isInit;

/* System wide synchronisation */
SemaphoreHandle_t canStartMutex;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  BaseType_t result = xTaskCreate(systemTask, "SYSTEM", 
                                  4096, NULL, /*Priority*/2, NULL);
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create SYSTEM task!");
  } else {
    ESP_LOGI(TAG, "SYSTEM task created with stack size: 4096");
  }
}

//This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  // Crear el mutex primero
  canStartMutex = xSemaphoreCreateMutex();
  if (canStartMutex == NULL) {
    ESP_LOGE(TAG, "Failed to create canStartMutex!");
    return;
  }
  
  // Tomar el mutex inicialmente
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  ESP_LOGI(TAG, "Debug: Calling configblockInit()...");
  configblockInit();
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Debug: configblockInit() done.");
  
  ESP_LOGI(TAG, "Debug: Calling workerInit()...");
  workerInit();
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Debug: workerInit() done.");
  
  ESP_LOGI(TAG, "Debug: Calling ledseqInit()...");
  ledseqInit();
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Debug: ledseqInit() done.");
  
  // 🚀 Se mueve la inicialización de la comunicación a este punto
  // para evitar conflictos con el ADC y otros periféricos.
  ESP_LOGI(TAG, "Debug: Calling commInit()...");
  commInit();
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Debug: commInit() done.");

  // Los demás módulos se inicializan después de que la comunicación está lista
  ESP_LOGI(TAG, "Debug: Calling adcInit()...");
  adcInit();
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Debug: adcInit() done.");
  
  ESP_LOGI(TAG, "Debug: Calling pmInit()...");
  pmInit();
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Debug: pmInit() done.");
    
  isInit = true;
  ESP_LOGI(TAG, "System initialized");
}

bool systemTest()
{
  bool pass = isInit;
  
  pass &= commTest();
  pass &= adcTest();
  pass &= ledseqTest();
  pass &= pmTest();
  pass &= workerTest();
  
  return pass;
}

/* Private functions implementation */
void systemTask(void *arg)
{
  ESP_LOGI(TAG, "Step 1: System task started");
  ESP_LOGI(TAG, "Free heap: %" PRIu32, esp_get_free_heap_size());
  
  // Init the high-levels modules in the correct order
  ESP_LOGI(TAG, "Step 2: Calling systemInit()...");
  systemInit();
  ESP_LOGI(TAG, "Step 3: systemInit() completed.");
  vTaskDelay(pdMS_TO_TICKS(100));

  ESP_LOGI(TAG, "Crazyflie is up and running!");
  ESP_LOGI(TAG, "Build %s:%s (%s) %s", V_SLOCAL_REVISION,
              V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  vTaskDelay(pdMS_TO_TICKS(100));

  ESP_LOGI(TAG, "Step 4: Calling commanderInit()...");
  commanderInit();
  ESP_LOGI(TAG, "Step 5: commanderInit() completed.");
  vTaskDelay(pdMS_TO_TICKS(100));
  
  ESP_LOGI(TAG, "Step 6: Calling stabilizerInit()...");
  stabilizerInit();
  ESP_LOGI(TAG, "Step 7: stabilizerInit() completed.");
  vTaskDelay(pdMS_TO_TICKS(100));
  
  //Test the modules
  bool pass = true;
  pass &= systemTest();
  pass &= commTest();
  pass &= commanderTest();
  pass &= stabilizerTest();
  
  //Start the firmware
  if(pass)
  {
    systemStart();
    ledseqRun(LED_RED, seq_alive);
    ledseqRun(LED_GREEN, seq_testPassed);
    ESP_LOGI(TAG, "All tests passed - system starting");
  }
  else
  {
    ESP_LOGE(TAG, "System tests failed!");
    if (systemTest())
    {
      while(true)
      {
        ledseqRun(LED_RED, seq_testPassed);
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    }
    else
    {
      while(true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  }
  
  while(true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    static int counter = 0;
    if (++counter % 10 == 0) {
      ESP_LOGI(TAG, "System alive - Free heap: %" PRIu32, esp_get_free_heap_size());
    }
  }
}

/* Global system variables */
void systemStart()
{
  if (canStartMutex != NULL) {
    xSemaphoreGive(canStartMutex);
    ESP_LOGI(TAG, "System start signal given");
  }
}

void systemWaitStart(void)
{
  while(!isInit) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (canStartMutex != NULL) {
    xSemaphoreTake(canStartMutex, portMAX_DELAY);
    xSemaphoreGive(canStartMutex);
    ESP_LOGI(TAG, "System start wait completed");
  }
}

void systemSetCanFly(bool val)
{
  canFly = val;
  ESP_LOGI(TAG, "CanFly set to: %s", val ? "true" : "false");
}

bool systemCanFly(void)
{
  return canFly;
}