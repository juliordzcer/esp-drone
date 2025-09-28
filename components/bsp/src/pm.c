/**
 * pm.c - Power Management driver and functions for ESP-Drone.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "pm.h"
#include "adc.h"
#include "led.h"
#include "ledseq.h"
#include "commander.h"
#include "wifilink.h"
#include "system.h"

static const char *TAG = "PM";

#define M2T(X) ((unsigned int)(X)/ portTICK_PERIOD_MS) //ms to tick

static bool isInit = false;
static float    batteryVoltage;
static float    batteryVoltageMin = 6.0f;
static float    batteryVoltageMax = 0.0f;
static uint32_t batteryCriticalLowTimeStamp;
static bool batteryOk = false; // Variable para rastrear el estado de la batería

// --- Internal Functions ---
static void pmSystemShutdown(void)
{
#ifdef ACTIVATE_AUTO_SHUTDOWN
    ESP_LOGW(TAG, "Initiating system shutdown!");
    // Tu lógica de apagado aquí
#endif
}

static void pmSetBatteryVoltage(float voltage)
{
    batteryVoltage = voltage;
    
    // Actualizar el estado de la batería
    bool previousBatteryOk = batteryOk;
    batteryOk = (voltage >= PM_BAT_LOW_VOLTAGE); // Usar el mismo umbral que en pmTask
    
    if (batteryOk != previousBatteryOk) {
        ESP_LOGI(TAG, "Estado de batería cambiado: %s (%.2fV)", 
                batteryOk ? "OK" : "BAJA", voltage);
    }
    
    if (batteryVoltageMax < voltage)
    {
        batteryVoltageMax = voltage;
    }
    if (batteryVoltageMin > voltage)
    {
        batteryVoltageMin = voltage;
    }
}

// --- Nueva función pública ---
bool pmIsBatteryOk(void)
{
    return batteryOk && (batteryVoltage >= PM_BAT_LOW_VOLTAGE);
}

// --- Public Functions ---
void pmInit(void)
{
    if(isInit)
        return;

    // Inicializar con estado por defecto
    batteryVoltage = 0.0f;
    batteryOk = false;
    
    xTaskCreate(pmTask, "PWRMGNT", 4096, NULL, 5, NULL);
    
    isInit = true;
    ESP_LOGI(TAG, "Power Management module initialized.");
}

bool pmTest(void)
{
    return isInit;
}

float pmGetBatteryVoltage(void)
{
    return batteryVoltage;
}

float pmGetBatteryVoltageMin(void)
{
    return batteryVoltageMin;
}

float pmGetBatteryVoltageMax(void)
{
    return batteryVoltageMax;
}

void pmTask(void *param)
{
    PMStates pmState;
    PMStates pmStateOld = battery;
    uint32_t tickCount;

    // Initial state setup
    tickCount = xTaskGetTickCount();
    batteryCriticalLowTimeStamp = tickCount;
    
    // Set a small initial delay
    vTaskDelay(M2T(1000));
    
    while(1)
    {
        vTaskDelay(M2T(100)); // Run every 100ms
        tickCount = xTaskGetTickCount();
        
        // Get voltage from your ADC module
        float voltage = adcGetBatteryVoltage();
        pmSetBatteryVoltage(voltage);

        // Determine current state based on voltage
        if (voltage < PM_BAT_CRITICAL_LOW_VOLTAGE)
        {
            pmState = shutDown;
        }
        else if (voltage < PM_BAT_LOW_VOLTAGE)
        {
            pmState = lowPower;
        }
        else
        {
            pmState = battery;
            batteryCriticalLowTimeStamp = tickCount;
        }

        if (pmState != pmStateOld)
        {
            // Actions on state change
            switch (pmState)
            {
                case lowPower:
                    ESP_LOGW(TAG, "⚠️ BATERÍA BAJA: %.2fV", voltage);
                    ledseqRun(LED_RED, seq_lowbat);
                    systemSetCanFly(false);
                    break;
                case battery:
                    ESP_LOGI(TAG, "✅ BATERÍA OK: %.2fV", voltage);
                    ledseqStop(LED_RED, seq_lowbat);
                    systemSetCanFly(true);
                    wifilinkReInit();
                    break;
                case shutDown:
                    ESP_LOGE(TAG, "❌ BATERÍA CRÍTICA: %.2fV - APAGANDO", voltage);
                    ledseqStop(LED_RED, seq_lowbat);
                    pmSystemShutdown();
                    break;
                default:
                    break;
            }
            pmStateOld = pmState;
        }
        
        // Actions during state
        switch (pmState)
        {
            case shutDown:
                {
                    uint32_t batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;
                    if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT)
                    {
                        pmSystemShutdown();
                    }
                }
                break;
            case battery:
                {
                    // Assuming commanderGetInactivityTime() exists
                    if ((commanderGetInactivityTime() > PM_SYSTEM_SHUTDOWN_TIMEOUT))
                    {
                        pmSystemShutdown();
                    }
                }
                break;
            default:
                break;
        }
    }
}