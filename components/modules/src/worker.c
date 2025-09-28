#include "worker.h"

#include <errno.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "console.h"
#include "esp_log.h"

static const char *TAG = "worker";

#define WORKER_QUEUE_LENGTH 5

// The worker task's stack size should be large enough to handle any scheduled work.
// 2048 is a safe starting point.
#define WORKER_TASK_STACK_SIZE 2048

struct worker_work {
  void (*function)(void*);
  void* arg;
};

// CORRECCIÓN: Cambiado de 'xQueueHandle' a 'QueueHandle_t'
static QueueHandle_t workerQueue;
static bool isInit = false;

// The worker task that will process the queue
static void workerTask(void *param) {
  struct worker_work work;
  while (1) {
    // Wait for a new work item in the queue. 
    // Ahora 'workerQueue' es del tipo correcto (QueueHandle_t).
    if (xQueueReceive(workerQueue, &work, portMAX_DELAY) == pdTRUE) {
      if (work.function) {
        ESP_LOGD(TAG, "Executing scheduled function.");
        work.function(work.arg);
      }
    }
  }
}

void workerInit()
{
  if (isInit)
    return;

  // Ahora 'workerQueue' es QueueHandle_t, la asignación es correcta.
  workerQueue = xQueueCreate(WORKER_QUEUE_LENGTH, sizeof(struct worker_work));
  if (workerQueue == NULL) {
    ESP_LOGE(TAG, "Failed to create worker queue!");
    return;
  }
  
  // Create and start the worker task
  if (xTaskCreate(workerTask, "worker", WORKER_TASK_STACK_SIZE, NULL, 5, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create worker task!");
    // Ahora 'workerQueue' es QueueHandle_t, vQueueDelete es correcto.
    vQueueDelete(workerQueue); 
    return;
  }
  
  isInit = true;
  ESP_LOGI(TAG, "Worker module initialized.");
}

bool workerTest()
{
  // Ahora la comparación es entre un puntero (QueueHandle_t) y NULL, lo cual es correcto.
  return isInit && (workerQueue != NULL);
}

int workerSchedule(void (*function)(void*), void *arg)
{
  struct worker_work work;
  
  if (!function)
    return ENOEXEC; // Invalid function pointer

  if (!isInit) {
    return ENOSYS; // Worker system not initialized
  }
  
  work.function = function;
  work.arg = arg;
  
  // Use a timeout of 0. If the queue is full, the call will fail immediately.
  // Ahora 'workerQueue' es QueueHandle_t, xQueueSend es correcto.
  if (xQueueSend(workerQueue, &work, 0) == pdFALSE) {
    ESP_LOGW(TAG, "Worker queue is full, could not schedule work.");
    return ENOMEM; // ENOMEM is acceptable here, it signals a resource issue.
  }

  return 0; 
}