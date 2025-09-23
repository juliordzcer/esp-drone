#include "imu.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_dev.h"
#include "mpu6050.h"
#include "filter.h" 

// =================================================================
// ============== PARÁMETROS DE CONFIGURACIÓN Y AJUSTE =============
// =================================================================
#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

// --- Configuración del Sensor MPU-6050 ---
#define IMU_GYRO_FS_CFG         MPU6050_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG     MPU6050_DEG_PER_LSB_2000
#define IMU_ACCEL_FS_CFG        MPU6050_ACCEL_FS_8
#define IMU_G_PER_LSB_CFG       MPU6050_G_PER_LSB_8

// --- Parámetros de Calibración ---
// Aumenta para más precisión (y más tiempo de espera), disminuye para calibrar más rápido.
#define GYRO_BIAS_SAMPLES       128   // Número de muestras para el giroscopio
#define ACCEL_SCALE_SAMPLES     200   // Número de muestras para el acelerómetro
#define GYRO_VARIANCE_THRESHOLD 5000  // Umbral de estabilidad (más bajo es más estricto)

// --- Parámetros del Filtro de Segundo Orden ---
#define IMU_SAMPLE_RATE_HZ      500   // Frecuencia de muestreo del IMU (ajustar si se cambia el DLPF)
#define GYRO_LPF_CUTOFF_HZ      80    // Frecuencia de corte para el giroscopio
#define ACCEL_LPF_CUTOFF_HZ     30    // Frecuencia de corte para el acelerómetro

// =================================================================
// =================== FIN DE LA CONFIGURACIÓN =====================
// =================================================================

static const char* TAG = "IMU_IMPROVED";

// Estructuras internas
typedef struct { int16_t x, y, z; } Axis3i16;
typedef struct { int32_t x, y, z; } Axis3i32;

// Variables estáticas del driver
static imu_state_t imuState = IMU_STATE_UNINITIALIZED;
static bool isI2cInitialized = false;

// Variables de calibración
static Axis3f gyroBias = {0};
static float accelScale = 1.0f;
static bool gyroBiasFound = false;
static bool accelScaleFound = false;

// Búfer para la calibración del giroscopio
static Axis3i16 gyroBuffer[GYRO_BIAS_SAMPLES];
static uint32_t gyroBufferIndex = 0;
static bool gyroBufferFilled = false;

// Acumulador para la calibración del acelerómetro
static float accelScaleSum = 0;
static uint32_t accelScaleSampleCount = 0;

// Filtros de segundo orden
static lpf2pData gyroLpf[3];
static lpf2pData accLpf[3];

// Prototipos de funciones internas
static bool configure_mpu6050(void);
static void calculate_gyro_bias(void);
static void calculate_accel_scale(int16_t ax, int16_t ay, int16_t az);

bool imu_init(void) {
    if (imuState != IMU_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "IMU ya inicializado.");
        return true;
    }
    
    ESP_LOGI(TAG, "Inicializando IMU (versión mejorada)...");
    
    if (!isI2cInitialized) {
        if (!i2cdevInit(I2C0_DEV)) {
            ESP_LOGE(TAG, "Fallo al inicializar I2C.");
            imuState = IMU_STATE_ERROR;
            return false;
        }
        isI2cInitialized = true;
    }

    mpu6050Init(I2C0_DEV);

    if (!mpu6050TestConnection()) {
        ESP_LOGE(TAG, "Fallo en la conexión con MPU6050.");
        imuState = IMU_STATE_ERROR;
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 conectado.");

    if (!configure_mpu6050()) {
        ESP_LOGE(TAG, "Fallo al configurar MPU6050.");
        imuState = IMU_STATE_ERROR;
        return false;
    }

    // Inicializar los filtros
    for (int i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], IMU_SAMPLE_RATE_HZ, GYRO_LPF_CUTOFF_HZ);
        lpf2pInit(&accLpf[i], IMU_SAMPLE_RATE_HZ, ACCEL_LPF_CUTOFF_HZ);
    }
    
    imuState = IMU_STATE_INITIALIZED;
    ESP_LOGI(TAG, "IMU inicializado y listo para calibrar.");
    return true;
}

bool imu_test(void) {
    if (imuState < IMU_STATE_INITIALIZED) return false;
    return mpu6050SelfTest();
}

bool imu_is_calibrated(void) {
    return gyroBiasFound && accelScaleFound;
}

void imu_read(Axis3f* gyro, Axis3f* acc) {
    if (imuState < IMU_STATE_INITIALIZED) {
        memset(gyro, 0, sizeof(Axis3f));
        memset(acc, 0, sizeof(Axis3f));
        return;
    }

    Axis3i16 accelRaw, gyroRaw;
    mpu6050GetMotion6(&accelRaw.x, &accelRaw.y, &accelRaw.z, 
                     &gyroRaw.x, &gyroRaw.y, &gyroRaw.z);

    // --- Lógica de Calibración (se ejecuta hasta que se completa) ---
    if (!imu_is_calibrated()) {
        imuState = IMU_STATE_CALIBRATING;

        // 1. Calibrar giroscopio
        if (!gyroBiasFound) {
            gyroBuffer[gyroBufferIndex++] = gyroRaw;
            if (gyroBufferIndex >= GYRO_BIAS_SAMPLES) {
                gyroBufferIndex = 0;
                gyroBufferFilled = true;
            }
            if (gyroBufferFilled) {
                calculate_gyro_bias();
            }
        }

        // 2. Calibrar acelerómetro
        if (!accelScaleFound) {
            calculate_accel_scale(accelRaw.x, accelRaw.y, accelRaw.z);
        }

        // 3. Comprobar si hemos terminado
        if (imu_is_calibrated()) {
            imuState = IMU_STATE_CALIBRATED;
            ESP_LOGI(TAG, "¡Calibración de Giroscopio y Acelerómetro completada!");
            ESP_LOGI(TAG, "Gyro Bias [x,y,z]: %.2f, %.2f, %.2f", gyroBias.x, gyroBias.y, gyroBias.z);
            ESP_LOGI(TAG, "Accel Scale Factor: %.4f", accelScale);
        }
    }

    // --- Aplicar Correcciones y Filtros ---
    Axis3f tempGyro, tempAcc;

    // 1. Corregir y escalar Giroscopio
    tempGyro.x = (gyroRaw.x - gyroBias.x) * IMU_DEG_PER_LSB_CFG;
    tempGyro.y = (gyroRaw.y - gyroBias.y) * IMU_DEG_PER_LSB_CFG;
    tempGyro.z = (gyroRaw.z - gyroBias.z) * IMU_DEG_PER_LSB_CFG;

    // 2. Corregir y escalar Acelerómetro
    tempAcc.x = accelRaw.x * IMU_G_PER_LSB_CFG / accelScale;
    tempAcc.y = accelRaw.y * IMU_G_PER_LSB_CFG / accelScale;
    tempAcc.z = accelRaw.z * IMU_G_PER_LSB_CFG / accelScale;

    // 3. Aplicar filtros de segundo orden y asignar solo si los punteros no son nulos
    if (gyro) {
        gyro->x = lpf2pApply(&gyroLpf[0], tempGyro.x);
        gyro->y = lpf2pApply(&gyroLpf[1], tempGyro.y);
        gyro->z = lpf2pApply(&gyroLpf[2], tempGyro.z);
    }

    if (acc) {
        acc->x = lpf2pApply(&accLpf[0], tempAcc.x);
        acc->y = lpf2pApply(&accLpf[1], tempAcc.y);
        acc->z = lpf2pApply(&accLpf[2], tempAcc.z);
    }
}

// --- Funciones Internas ---

static void calculate_gyro_bias(void) {
    int64_t sumSq[3] = {0};
    Axis3i32 sum = {0};

    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
        sum.x += gyroBuffer[i].x;
        sum.y += gyroBuffer[i].y;
        sum.z += gyroBuffer[i].z;
        sumSq[0] += (int64_t)gyroBuffer[i].x * gyroBuffer[i].x;
        sumSq[1] += (int64_t)gyroBuffer[i].y * gyroBuffer[i].y;
        sumSq[2] += (int64_t)gyroBuffer[i].z * gyroBuffer[i].z;
    }
    
    Axis3i32 variance;
    variance.x = (sumSq[0] - ((int64_t)sum.x * sum.x) / GYRO_BIAS_SAMPLES);
    variance.y = (sumSq[1] - ((int64_t)sum.y * sum.y) / GYRO_BIAS_SAMPLES);
    variance.z = (sumSq[2] - ((int64_t)sum.z * sum.z) / GYRO_BIAS_SAMPLES);

    if (variance.x < GYRO_VARIANCE_THRESHOLD &&
        variance.y < GYRO_VARIANCE_THRESHOLD &&
        variance.z < GYRO_VARIANCE_THRESHOLD)
    {
        gyroBias.x = (float)sum.x / GYRO_BIAS_SAMPLES;
        gyroBias.y = (float)sum.y / GYRO_BIAS_SAMPLES;
        gyroBias.z = (float)sum.z / GYRO_BIAS_SAMPLES;
        gyroBiasFound = true;
    }
}

static void calculate_accel_scale(int16_t ax, int16_t ay, int16_t az) {
    if (accelScaleFound) return;
    
    // Solo acumular si el giroscopio ya está estable (mejora la calidad)
    if (gyroBiasFound) {
        float magnitude = sqrtf(powf(ax * IMU_G_PER_LSB_CFG, 2) + 
                                powf(ay * IMU_G_PER_LSB_CFG, 2) + 
                                powf(az * IMU_G_PER_LSB_CFG, 2));
        accelScaleSum += magnitude;
        accelScaleSampleCount++;

        if (accelScaleSampleCount >= ACCEL_SCALE_SAMPLES) {
            accelScale = accelScaleSum / ACCEL_SCALE_SAMPLES;
            if (accelScale < 0.9f || accelScale > 1.1f) {
                ESP_LOGW(TAG, "Calibracion de escala del Accel fuera de rango (%.2f). Usando 1.0", accelScale);
                accelScale = 1.0f;
            }
            accelScaleFound = true;
        }
    }
}

static bool configure_mpu6050(void) {
    mpu6050Reset();
    vTaskDelay(pdMS_TO_TICKS(50));
    mpu6050SetSleepEnabled(false);
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu6050SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
    mpu6050SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_42); // Un buen balance para drones
    return true;
}

// Implementación de las funciones de estado
imu_state_t imu_get_state(void) {
    return imuState;
}

const char* imu_get_state_string(void) {
    switch(imuState) {
        case IMU_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case IMU_STATE_INITIALIZED:   return "INITIALIZED";
        case IMU_STATE_CALIBRATING:   return "CALIBRATING";
        case IMU_STATE_CALIBRATED:    return "CALIBRATED";
        case IMU_STATE_ERROR:         return "ERROR";
        default:                      return "UNKNOWN";
    }
}