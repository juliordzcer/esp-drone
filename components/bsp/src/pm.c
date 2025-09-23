#include "pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "adc.h" // Tu driver de ADC personalizado

static const char *TAG = "PM";

// --- Umbrales de Voltaje ---
#define BATTERY_VOLTAGE_LOW         3.6f
#define BATTERY_VOLTAGE_CRITICAL    3.3f
#define CRITICAL_POWER_TIMEOUT_MS   5000

// --- Variables estáticas del módulo ---
static bool isInit = false;
static float batteryVoltage = 0.0f;
static float batteryVoltageMin = 99.0f; // Iniciar en un valor alto
static float batteryVoltageMax = 0.0f;  // Iniciar en un valor bajo

// --- API Pública ---

void pmInit(void) {
    if (isInit) {
        return;
    }
    ESP_LOGI(TAG, "Inicializando sistema de gestión de energía (PM)...");
    xTaskCreate(pmTask, "PWRMGNT", 3072, NULL, 5, NULL);
    isInit = true;
}

bool pmTest(void) {
    return isInit;
}

float pmGetBatteryVoltage(void) {
    return batteryVoltage;
}

float pmGetBatteryVoltageMin(void) {
    return batteryVoltageMin;
}

float pmGetBatteryVoltageMax(void) {
    return batteryVoltageMax;
}

/**
 * @brief Esta es una función "stub" o de compatibilidad.
 * En el hardware original (Crazyflie), esto controlaba un chip de carga.
 * Como el ESP32 no tiene ese hardware, esta función no hace nada,
 * pero existe para que el resto del código que la llama pueda compilar.
 */
void pmSetChargeState(PMChargeStates chgState) {
    // No se implementa en esta plataforma, pero la función existe por compatibilidad.
    ESP_LOGD(TAG, "pmSetChargeState llamado, pero no es aplicable en este hardware.");
}

// --- Lógica Interna y Tarea ---

static void system_shutdown(void) {
    ESP_LOGE(TAG, "APAGADO DE EMERGENCIA. Batería críticamente baja.");
    // Aquí iría el código para detener los motores.
    esp_deep_sleep_start();
}

void pmTask(void *param) {
    uint32_t criticalPowerTimestamp = 0;
    bool inCriticalState = false;

    ESP_LOGI(TAG, "Tarea de gestión de energía iniciada.");

    // Pequeña espera para asegurar que el ADC se estabilice tras el arranque
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        // 1. Obtener el voltaje y actualizar min/max
        float newVoltage = adcGetBatteryVoltage();
        // Solo actualizar si la lectura es válida (evita que el 0.0 inicial afecte)
        if (newVoltage > 1.0f) {
            batteryVoltage = newVoltage;
            if (batteryVoltage < batteryVoltageMin) {
                batteryVoltageMin = batteryVoltage;
            }
            if (batteryVoltage > batteryVoltageMax) {
                batteryVoltageMax = batteryVoltage;
            }
        }

        // 2. Lógica de estado de emergencia
        if (batteryVoltage < BATTERY_VOLTAGE_CRITICAL && batteryVoltage > 1.0f) {
            if (!inCriticalState) {
                // Acabamos de entrar en estado crítico
                ESP_LOGE(TAG, "Nivel de batería crítico detectado (%.2fV). Apagando en %d segundos...",
                         batteryVoltage, CRITICAL_POWER_TIMEOUT_MS / 1000);
                criticalPowerTimestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
                inCriticalState = true;
            } else {
                // Ya estábamos en estado crítico, comprobar si se agotó el tiempo
                if (xTaskGetTickCount() * portTICK_PERIOD_MS - criticalPowerTimestamp > CRITICAL_POWER_TIMEOUT_MS) {
                    system_shutdown();
                }
            }
        } else {
            // El voltaje es seguro, salir del estado crítico
            if (inCriticalState) {
                ESP_LOGI(TAG, "Nivel de batería recuperado (%.2fV).", batteryVoltage);
            }
            inCriticalState = false;
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Revisar el voltaje cada 500ms
    }
}