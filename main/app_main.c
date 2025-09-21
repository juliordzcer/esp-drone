#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h" 
#include "esp_log.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Application starting...");

    motorsInit();

    ESP_LOGI(TAG, "Running motor test...");
    motorsTest();

    ESP_LOGI(TAG, "Modules initialized.");
}


