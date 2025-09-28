/**
 * pm.h - Power Management driver and functions for ESP-Drone.
 */

#ifndef PM_H_
#define PM_H_

// #define M2T(X) ((unsigned int)(X)/ portTICK_PERIOD_MS) //ms to tick

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Battery Voltage Thresholds ---
#define PM_BAT_CRITICAL_LOW_VOLTAGE   3.0f
#define PM_BAT_LOW_VOLTAGE            3.2f

// --- Timeouts in FreeRTOS ticks (M2T stands for Milliseconds to Ticks) ---
#define M2T(ms) pdMS_TO_TICKS(ms)
#define PM_BAT_CRITICAL_LOW_TIMEOUT   M2T(5000)  // 5 sec
#define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(300000) // 5 min

typedef enum
{
  battery,
  lowPower,
  shutDown,
} PMStates;

/**
 * @brief Initialize the power management module.
 * Starts the PM task.
 */
void pmInit(void);

/**
 * @brief Test if the PM module is initialized.
 * @return True if initialized, false otherwise.
 */
bool pmTest(void);

/**
 * @brief Power management task.
 * This task runs in the background to monitor battery and manage power states.
 */
void pmTask(void *param);

/**
 * @brief Returns the battery voltage in volts.
 * @return The current battery voltage.
 */
float pmGetBatteryVoltage(void);

/**
 * @brief Returns the minimum battery voltage recorded.
 * @return The minimum battery voltage.
 */
float pmGetBatteryVoltageMin(void);

/**
 * @brief Returns the maximum battery voltage recorded.
 * @return The maximum battery voltage.
 */
float pmGetBatteryVoltageMax(void);

bool pmIsBatteryOk(void);

#endif /* PM_H_ */