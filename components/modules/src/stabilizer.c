#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"

#include "system.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensor_fusion.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "esp_log.h"
#include "pid.h"
#include "ledseq.h"
#include "math.h"
#include "vl53l1x.h"

#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))

// LÍNEA MODIFICADA: Reducimos la frecuencia para que sea compatible con el tick por defecto de 100Hz.
// La solución ideal sigue siendo aumentar configTICK_RATE_HZ a 1000Hz en menuconfig.
#define IMU_UPDATE_FREQ   100  // Reducido de 250 a 100 Hz
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 50hz

// VL53L1x/ Hover stuff
#define HOVER_UPDATE_RATE_DIVIDER  2 // 100hz/2 = 50hz para sensor de distancia
#define HOVER_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / HOVER_UPDATE_RATE_DIVIDER))   // 50hz

#define LOGGING_ENABLED
#ifdef LOGGING_ENABLED
  #define PRIVATE
#else
  #define PRIVATE static
#endif

PRIVATE Axis3f gyro; // Gyro axis data in deg/s
PRIVATE Axis3f acc;  // Accelerometer axis data in mG

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

// ============================================================================
// VARIABLES PARA HOVER CON VL53L1X
// ============================================================================

// Sensor VL53L1X - Mide distancia en milímetros con rango hasta 4m
PRIVATE VL53L1_Dev_t vl53l1x_dev;
PRIVATE VL53L1_RangingMeasurementData_t rangingData;
PRIVATE uint8_t vl53l1x_ready = 0;

// Variables de altura (en metros)
PRIVATE float asl;      // Altura suavizada (filtrada a corto plazo)
PRIVATE float aslRaw;   // Altura cruda del sensor (en metros)
PRIVATE float aslLong;  // Altura suavizada a largo plazo (para calcular velocidad)

// Variables de control de hover
PRIVATE bool hover = false;          // Indica si estamos en modo hover
PRIVATE bool set_hover = false;      // Bandera para activar hover por primera vez
PRIVATE float hoverChange = 0;       // Cambio en altura deseada desde el control
PRIVATE float hoverTarget = 0.5f;    // Altura objetivo en metros (0.5m = 50cm)
PRIVATE float hoverErr = 0;          // Error entre altura actual y objetivo
PRIVATE float hoverPIDVal = 0;       // Salida del controlador PID

// Variables para velocidad vertical
PRIVATE float accWZ = 0.0;           // Aceleración vertical en eje Z del mundo
PRIVATE float vSpeed = 0.0;          // Velocidad vertical integrada
PRIVATE float vSpeedASL = 0.0;       // Velocidad vertical calculada del sensor
PRIVATE float vSpeedAcc = 0.0;       // Velocidad vertical calculada del acelerómetro

// Controlador PID para hover
PRIVATE PidObject hoverPID;

// Parámetros del controlador hover (ajustables)
PRIVATE float hoverKp = 2.0;         // Ganancia proporcional - responde al error actual
PRIVATE float hoverKi = 0.5;         // Ganancia integral - elimina error estacionario
PRIVATE float hoverKd = 0.1;         // Ganancia derivativa - amortigua oscilaciones
PRIVATE float hoverErrMax = 0.5f;    // Error máximo permitido (50cm)
PRIVATE float hoverChange_SENS = 200;// Sensibilidad del control de altura
PRIVATE float pidAslFac = 10000;     // Factor de conversión altura->thrust
PRIVATE float hoverBaseThrust = 43000; // Thrust base para hover
PRIVATE float hoverMaxThrust = 60000;  // Thrust máximo permitido
PRIVATE float hoverMinThrust = 10000;  // Thrust mínimo permitido

// Parámetros de filtrado y suavizado
PRIVATE float aslAlpha = 0.85;       // Factor suavizado corto plazo (0.85 = 85% valor anterior)
PRIVATE float aslAlphaLong = 0.90;   // Factor suavizado largo plazo
PRIVATE float vBiasAlpha = 0.91;     // Mezcla velocidad sensor vs acelerómetro
PRIVATE float vAccDeadband = 0.05;   // Zona muerta aceleración
PRIVATE float vSpeedLimit = 0.05;    // Límite velocidad vertical

// ============================================================================
// FIN VARIABLES HOVER
// ============================================================================

RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit;

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit();
  imu_init();
  sensfusion6Init();
  controllerInit();

  // ==========================================================================
  // INICIALIZACIÓN SENSOR VL53L1X
  // ==========================================================================
  ESP_LOGI("STABILIZER", "Inicializando sensor VL53L1X...");
  if (vl53l1xInit(&vl53l1x_dev, I2C1_DEV)) {
      vl53l1x_ready = 1;
      // Configurar sensor para modo largo (hasta 4m)
      VL53L1_SetDistanceMode(&vl53l1x_dev, VL53L1_DISTANCEMODE_LONG);
      // Tiempo de medición: 33ms (compatible con 50Hz de update)
      VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x_dev, 33000);
      // Iniciar mediciones continuas
      VL53L1_StartMeasurement(&vl53l1x_dev);
      
      // Inicializar valores de altura
      // NOTA: Usamos 0.1m como altura inicial porque el drone está en el piso
      // Esto evita comportamientos extraños al iniciar
      asl = 0.04f;     // 4cm - asumiendo que el drone está en el piso
      aslRaw = 0.04f;
      aslLong = 0.04f;
      
      ESP_LOGI("STABILIZER", "VL53L1X inicializado correctamente");
  } else {
      vl53l1x_ready = 0;
      // Valores por defecto seguros si el sensor falla
      asl = 0.1f;
      aslRaw = 0.1f;
      aslLong = 0.1f;
      ESP_LOGE("STABILIZER", "Fallo al inicializar VL53L1X");
  }
  // ==========================================================================

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  xTaskCreate(stabilizerTask, "STABILIZER",
              4*configMINIMAL_STACK_SIZE, NULL, /*Priority*/2, NULL); 

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu_test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  // Verificar sensor VL53L1X
  if (!vl53l1x_ready) {
      ESP_LOGW("STABILIZER", "VL53L1X no está listo");
      // No marcamos como fallo para permitir operación sin hover
  }

  return pass;
}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t hoverCounter = 0;  // Contador para updates de hover
  uint32_t lastWakeTime;

#ifdef configUSE_APPLICATION_TASK_TAG
  #if configUSE_APPLICATION_TASK_TAG == 1
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);
  #endif
#endif

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ));

    imu_read(&gyro, &acc);

    if (imu_is_calibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // ======================================================================
      // ACTUALIZACIÓN HOVER (50Hz)
      // ======================================================================
      if (++hoverCounter >= HOVER_UPDATE_RATE_DIVIDER) {
          hoverCounter = 0;

          // Obtener comandos de hover del piloto
          commanderGetHover(&hover, &set_hover, &hoverChange);

          // LEER SENSOR VL53L1X
          if (vl53l1x_ready) {
              uint8_t dataReady = 0;
              VL53L1_GetMeasurementDataReady(&vl53l1x_dev, &dataReady);
              
              if (dataReady) {
                  VL53L1_GetRangingMeasurementData(&vl53l1x_dev, &rangingData);
                  
                  if (rangingData.RangeStatus == 0) { // Medida válida
                      // Convertir mm a metros (sensor da distancia en mm)
                      aslRaw = (float)rangingData.RangeMilliMeter / 1000.0f;
                      
                      // Reiniciar para siguiente medición
                      VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x_dev);
                  }
                  // Si medida no válida, mantener último valor (aslRaw no cambia)
              }
          }

          // Aplicar filtros de suavizado a las lecturas
          asl     = asl     * aslAlpha     + aslRaw * (1.0f - aslAlpha);
          aslLong = aslLong * aslAlphaLong + aslRaw * (1.0f - aslAlphaLong);

          // Calcular velocidad vertical basada en cambio de altura
          vSpeedASL = asl - aslLong;

          // Integrar velocidad vertical desde acelerómetro
          vSpeedAcc += deadband(accWZ, vAccDeadband) * HOVER_UPDATE_DT;
          vSpeedAcc = constrain(vSpeedAcc, -vSpeedLimit, vSpeedLimit);

          // Fusionar ambas velocidades (sensor + acelerómetro)
          vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.0f - vBiasAlpha);

          // ACTIVACIÓN INICIAL DE HOVER
          if (set_hover) {
              // Establecer altura objetivo como altura actual
              hoverTarget = asl;
              
              // Inicializar controlador PID
              pidInit(&hoverPID, hoverTarget, hoverKp, hoverKi, hoverKd, HOVER_UPDATE_DT);
              
              ESP_LOGI("STABILIZER", "Hover activado - Altura objetivo: %.2fm", hoverTarget);
          }

          // MODO HOVER ACTIVO
          if (hover) {
              // Actualizar altura objetivo desde entrada del usuario
              hoverTarget += hoverChange / hoverChange_SENS;
              hoverTarget = constrain(hoverTarget, 0.1f, 3.5f); // Limitar entre 10cm y 3.5m
              pidSetDesired(&hoverPID, hoverTarget);

              // Calcular error (actual - objetivo) y limitarlo
              hoverErr = constrain(asl - hoverTarget, -hoverErrMax, hoverErrMax);
              pidSetError(&hoverPID, -hoverErr);

              // Obtener control del PID
              hoverPIDVal = pidUpdate(&hoverPID, asl, false);

              // Calcular thrust basado en PID + compensación velocidad
              float thrustAdjust = hoverPIDVal * pidAslFac + vSpeed * 500.0f;
              actuatorThrust = limitThrust(hoverBaseThrust + (int32_t)thrustAdjust);
              actuatorThrust = constrain(actuatorThrust, hoverMinThrust, hoverMaxThrust);

          } else {
              // MODO MANUAL: usar thrust del controlador
              commanderGetThrust(&actuatorThrust);
              hoverTarget = asl; // Seguir altura actual para transición suave
          }
      }
      // ======================================================================

      // ACTUALIZACIÓN ACTITUD (50Hz)
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual, acc.z, &accWZ);

        // Integrar velocidad vertical desde acelerómetro (para uso en hover)
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;

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

      // Control de tasa
      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      // Distribuir potencia a motores
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
  }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  roll = roll >> 1;
  pitch = pitch >> 1;
  motorPowerM4 =  limitThrust(thrust + roll + pitch - yaw);
  motorPowerM2 = limitThrust(thrust - roll - pitch - yaw);
  motorPowerM1 = limitThrust(thrust - roll + pitch + yaw);
  motorPowerM3 =  limitThrust(thrust + roll - pitch + yaw);
#else // QUAD_FORMATION_NORMAL
  motorPowerM4 =  limitThrust(thrust + roll - yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
#endif

  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
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

// ============================================================================
// FUNCIONES AUXILIARES PARA HOVER
// ============================================================================

// Limitar valor entre mínimo y máximo
static float constrain(float value, const float minVal, const float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// Aplicar zona muerta a un valor
static float deadband(float value, const float threshold) {
    if (fabs(value) < threshold) {
        return 0.0f;
    } else if (value > 0) {
        return value - threshold;
    } else {
        return value + threshold;
    }
}

// ============================================================================
// GRUPOS DE LOG PARA DEBUGGING
// ============================================================================

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m1, &motorPowerM1) 
LOG_ADD(LOG_INT32, m2, &motorPowerM2) 
LOG_ADD(LOG_INT32, m3, &motorPowerM3) 
LOG_ADD(LOG_INT32, m4, &motorPowerM4) 
LOG_GROUP_STOP(motor)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

// Nuevos grupos de log para hover y sensor
LOG_GROUP_START(hover)
LOG_ADD(LOG_FLOAT, target, &hoverTarget)
LOG_ADD(LOG_FLOAT, actual, &asl)
LOG_ADD(LOG_FLOAT, error, &hoverErr)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_UINT8, active, &hover)
LOG_GROUP_STOP(hover)

