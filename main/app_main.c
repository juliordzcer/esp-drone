#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"
#include "led.h"
#include "adc.h"
#include "esp_log.h"

static const char *TAG = "APP_MAIN";

#define LOW_BATTERY_VOLTAGE 3.3f

void app_main(void)
{
    ESP_LOGI(TAG, "Application starting...");

    motorsInit();
    ledInit();
    adcInit();

    ledSet(LED_GREEN, true);
    ESP_LOGI(TAG, "Modules initialized. Entering main loop.");

    // Variable para controlar el parpadeo del LED rojo
    bool red_led_state = false;

    while (1) {
        float battery_voltage = adcGetBatteryVoltage();
        ESP_LOGI(TAG, "Battery voltage: %.2f V", battery_voltage);

        if (battery_voltage < LOW_BATTERY_VOLTAGE && battery_voltage > 0) {
            // Invierte el estado y actualiza el LED
            red_led_state = !red_led_state;
            ledSet(LED_RED, red_led_state);
        } else {
            // Si la batería está bien, apaga el LED y resetea el estado
            ledSet(LED_RED, false);
            red_led_state = false;
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Hacemos el parpadeo un poco más rápido
    }
}