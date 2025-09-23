/**
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * worker.c - Worker system that can execute asynchronous actions in tasks
 */
#include "worker.h"

#include <errno.h>
#include <stdbool.h> // Added for 'bool' type

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "console.h"
#include "esp_log.h" // Added for logging

static const char *TAG = "worker";

#define WORKER_QUEUE_LENGTH 5

// The worker task's stack size should be large enough to handle any scheduled work.
// 2048 is a safe starting point.
#define WORKER_TASK_STACK_SIZE 2048

struct worker_work {
  void (*function)(void*);
  void* arg;
};

static xQueueHandle workerQueue;
static bool isInit = false;

// The worker task that will process the queue
static void workerTask(void *param) {
  struct worker_work work;
  while (1) {
    // Wait for a new work item in the queue
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

  workerQueue = xQueueCreate(WORKER_QUEUE_LENGTH, sizeof(struct worker_work));
  if (workerQueue == NULL) {
    ESP_LOGE(TAG, "Failed to create worker queue!");
    return;
  }
  
  // Create and start the worker task
  if (xTaskCreate(workerTask, "worker", WORKER_TASK_STACK_SIZE, NULL, 5, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create worker task!");
    vQueueDelete(workerQueue);
    return;
  }
  
  isInit = true;
  ESP_LOGI(TAG, "Worker module initialized.");
}

bool workerTest()
{
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
  if (xQueueSend(workerQueue, &work, 0) == pdFALSE) {
    ESP_LOGW(TAG, "Worker queue is full, could not schedule work.");
    return ENOMEM; // ENOMEM is acceptable here, it signals a resource issue.
  }

  return 0; 
}