/**
 * pm.c - Power Management driver and functions for ESP-Drone.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "pm.h"
#include "adc.h"
// Assuming these headers exist and have the required functions
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

// --- Internal Functions ---
static void pmSystemShutdown(void)
{
#ifdef ACTIVATE_AUTO_SHUTDOWN
    ESP_LOGW(TAG, "Initiating system shutdown!");
    // You'll need to define what your shutdown action is,
    // e.g., a specific GPIO pin to turn off power.
    // For now, this is a placeholder.
    // gpio_set_level(PM_GPIO_SYSOFF, 1);
#endif
}

static void pmSetBatteryVoltage(float voltage)
{
    batteryVoltage = voltage;
    if (batteryVoltageMax < voltage)
    {
        batteryVoltageMax = voltage;
    }
    if (batteryVoltageMin > voltage)
    {
        batteryVoltageMin = voltage;
    }
}

// --- Public Functions ---
void pmInit(void)
{
    if(isInit)
        return;

    xTaskCreate(pmTask, "PWRMGNT", 4096, NULL, 5, NULL); // Increased stack size for safety
    
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
                    ledseqRun(LED_RED, seq_lowbat);
                    systemSetCanFly(false);
                    break;
                case battery:
                    ledseqStop(LED_RED, seq_lowbat);
                    systemSetCanFly(true);
                    wifilinkReInit(); // Changed from radiolinkReInit()
                    break;
                case shutDown:
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