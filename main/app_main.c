#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"
#include "led.h"
#include "esp_log.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Application starting...");

    motorsInit();
    ledInit();

    // --- AJUSTE REALIZADO AQUÍ ---
    // Encender LED verde para indicar que el sistema inició correctamente.
    // Ahora 'true' realmente enciende el LED.
    ledSet(LED_GREEN, true);

    // Opcional: ejecutar la prueba de motores
    motorsTest();

    // Encender el LED rojo después de test, por ejemplo
    ledSet(LED_RED, true);

    ESP_LOGI(TAG, "Modules initialized.");
}