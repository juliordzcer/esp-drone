#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensor_fusion.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "esp_log.h"

static const char *TAG = "stabilizer";

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER))

#define LOGGING_ENABLED
#ifdef LOGGING_ENABLED
  #define PRIVATE
#else
  #define PRIVATE static
#endif

// Reducir frecuencia IMU para evitar stack overflow
#define IMU_UPDATE_FREQ   250  // Reducido de 500 a 250 Hz
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

PRIVATE Axis3f gyro;
PRIVATE Axis3f acc;

PRIVATE float eulerRollActual;
PRIVATE float eulerPitchActual;
PRIVATE float eulerYawActual;
PRIVATE float eulerRollDesired;
PRIVATE float eulerPitchDesired;
PRIVATE float eulerYawDesired;
PRIVATE float rollRateDesired;
PRIVATE float pitchRateDesired;
PRIVATE float yawRateDesired;
PRIVATE float fusionDt;

RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerLeft;
uint32_t motorPowerRight;
uint32_t motorPowerFront;
uint32_t motorPowerRear;

// LOG_GROUP_START(stabilizer)
// LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
// LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
// LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
// LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
// LOG_GROUP_STOP(stabilizer)

// LOG_GROUP_START(motor)
// LOG_ADD(LOG_INT32, m4, &motorPowerLeft) 
// LOG_ADD(LOG_INT32, m1, &motorPowerFront) 
// LOG_ADD(LOG_INT32, m2, &motorPowerRight) 
// LOG_ADD(LOG_INT32, m3, &motorPowerRear) 
// LOG_GROUP_STOP(motor)

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_s;

attitude_s attitude;

static bool isInit;

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu_init(); 
  sensfusion6Init();
  controllerInit();

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  // Aumentar significativamente el stack size
  xTaskCreate(stabilizerTask, "STABILIZER",
              4*configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu_test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  // Calcular frecuencia con la nueva frecuencia IMU
  TickType_t xFrequency = pdMS_TO_TICKS(1000 / IMU_UPDATE_FREQ);
  if (xFrequency == 0) {
      xFrequency = 1;
      ESP_LOGW(TAG, "xFrequency era 0, ajustado a 1 tick");
  }

  ESP_LOGI(TAG, "Stabilizer task started with frequency: %lu ticks (%d Hz)", 
           (unsigned long)xFrequency, IMU_UPDATE_FREQ);

  // Verificar stack disponible
  ESP_LOGI(TAG, "Free stack space: %d bytes", uxTaskGetStackHighWaterMark(NULL));

  // Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  ESP_LOGI(TAG, "Stabilizer loop starting...");

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, xFrequency);

    // Leer IMU
    imu_read(&gyro, &acc);

    if (imu_is_calibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;
      }

      if (rollType == RATE)
      {
        rollRateDesired = eulerRollDesired;
      }
      if (pitchType == RATE)
      {
        pitchRateDesired = eulerPitchDesired;
      }
      if (yawType == RATE)
      {
        yawRateDesired = -eulerYawDesired;
      }

      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      commanderGetTrust(&actuatorThrust);
      if (actuatorThrust > 0)
      {
#if defined(TUNE_ROLL)
        distributePower(actuatorThrust, actuatorRoll, 0, 0);
#elif defined(TUNE_PITCH)
        distributePower(actuatorThrust, 0, actuatorPitch, 0);
#elif defined(TUNE_YAW)
        distributePower(actuatorThrust, 0, 0, -actuatorYaw);
#else
        distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
#endif
      }
      else
      {
        distributePower(0, 0, 0, 0);
        controllerResetAllPID();
      }
    }

    // Monitorear stack periódicamente (cada 1000 iteraciones)
    static uint32_t stackCheckCounter = 0;
    if (++stackCheckCounter >= 1000) {
        UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);
        if (freeStack < 100) {
            ESP_LOGW(TAG, "Low stack space: %d bytes", freeStack);
        }
        stackCheckCounter = 0;
    }
  }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  int16_t temp_roll = roll >> 1;
  int16_t temp_pitch = pitch >> 1;
  motorPowerLeft =  limitThrust(thrust + temp_roll + temp_pitch - yaw);
  motorPowerRight = limitThrust(thrust - temp_roll - temp_pitch - yaw);
  motorPowerFront = limitThrust(thrust - temp_roll + temp_pitch + yaw);
  motorPowerRear =  limitThrust(thrust + temp_roll - temp_pitch + yaw);
#else
  motorPowerLeft =  limitThrust(thrust + roll - yaw);
  motorPowerRight = limitThrust(thrust - roll - yaw);
  motorPowerFront = limitThrust(thrust + pitch + yaw);
  motorPowerRear =  limitThrust(thrust - pitch + yaw);
#endif

  motorsSetRatio(MOTOR_LEFT, motorPowerLeft);
  motorsSetRatio(MOTOR_RIGHT, motorPowerRight);
  motorsSetRatio(MOTOR_FRONT, motorPowerFront);
  motorsSetRatio(MOTOR_REAR, motorPowerRear);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}