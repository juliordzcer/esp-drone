#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "imu.h"
#include "i2c_dev.h"
#include "mpu6050.h"
#include "ledseq.h"
#include "configblock.h"

static const char* TAG = "IMU";

// Configuración por defecto
#ifndef IMU_I2C_PORT
#define IMU_I2C_PORT I2C_NUM_1
#endif

// Configuración MPU6050
#define IMU_GYRO_FS_CFG       MPU6050_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   MPU6050_DEG_PER_LSB_2000
#define IMU_ACCEL_FS_CFG      MPU6050_ACCEL_FS_8
#define IMU_G_PER_LSB_CFG     MPU6050_G_PER_LSB_8
#define IMU_1G_RAW            (int16_t)(1.0 / MPU6050_G_PER_LSB_8)

#define IMU_STARTUP_TIME_MS   1000

// Configuración giroscopio
#define GYRO_NBR_OF_AXES 3
#define GYRO_X_SIGN      (-1)
#define GYRO_Y_SIGN      (-1)
#define GYRO_Z_SIGN      (-1)
#define GYRO_MIN_BIAS_TIMEOUT_MS    (1000 / portTICK_PERIOD_MS)

#define IMU_NBR_OF_BIAS_SAMPLES  64
#define GYRO_VARIANCE_BASE        4000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

// Estructura para manejo de bias
typedef struct {
    Axis3i16   bias;
    bool       isBiasValueFound;
    bool       isBufferFilled;
    Axis3i16*  bufHead;
    Axis3i16   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;

// Variables globales
static BiasObj    gyroBias;
static BiasObj    accelBias;
static int32_t    varianceSampleTime;
static Axis3i16   gyroMpu;
static Axis3i16   accelMpu;
static Axis3i16   accelLPF;
static Axis3i16   accelLPFAligned;
static Axis3i32   accelStoredFilterValues;
static uint8_t    imuAccLpfAttFactor;

// Valores precalculados para alineamiento
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static imu_state_t imuState = IMU_STATE_UNINITIALIZED;
static bool isI2cInitialized = false;

// Prototipos de funciones internas
static void imuBiasInit(BiasObj* bias);
static bool imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut);
static bool imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);
static bool imuConfigureMpu6050(void);

static I2C_TypeDef imu_i2c_bus = 0; 

/**
 * @brief Inicializa el sistema IMU
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool imu6Init(void) {
    if (imuState >= IMU_STATE_INITIALIZED) {
        ESP_LOGW(TAG, "IMU ya está inicializada");
        return true;
    }
    
    ESP_LOGI(TAG, "Inicializando IMU...");
    imuState = IMU_STATE_CALIBRATING;

    // Esperar inicio de sensores
    vTaskDelay(pdMS_TO_TICKS(IMU_STARTUP_TIME_MS));

    // Inicializar I2C - CORREGIDO: pasar puntero
    if (!isI2cInitialized) {
        if (!i2cdevInit(&imu_i2c_bus)) {
            ESP_LOGE(TAG, "Error inicializando I2C");
            imuState = IMU_STATE_ERROR;
            return false;
        }
        isI2cInitialized = true;
    }

    // Inicializar MPU6050 - CORREGIDO: pasar puntero
    mpu6050Init(&imu_i2c_bus);

    // Test de conexión
    if (mpu6050TestConnection() != TRUE) {
        ESP_LOGE(TAG, "Conexión I2C MPU6050 falló");
        imuState = IMU_STATE_ERROR;
        return false;
    }
    ESP_LOGI(TAG, "Conexión MPU6050 I2C [OK]");

    // Configurar MPU6050
    if (!imuConfigureMpu6050()) {
        ESP_LOGE(TAG, "Error configurando MPU6050");
        imuState = IMU_STATE_ERROR;
        return false;
    }

    // Inicializar sistemas de bias
    imuBiasInit(&gyroBias);
    imuBiasInit(&accelBias);
    varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
    imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

    // Precalcular valores de alineamiento
    float pitchRad = configblockGetCalibPitch() * M_PI / 180.0f;
    float rollRad = configblockGetCalibRoll() * M_PI / 180.0f;
    cosPitch = cosf(pitchRad);
    sinPitch = sinf(pitchRad);
    cosRoll = cosf(rollRad);
    sinRoll = sinf(rollRad);

    imuState = IMU_STATE_INITIALIZED;
    ESP_LOGI(TAG, "IMU inicializada exitosamente");
    return true;
}

/**
 * @brief Configura los parámetros del MPU6050
 */
static bool imuConfigureMpu6050(void) {
    mpu6050Reset();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Activar MPU6050
    mpu6050SetSleepEnabled(FALSE);
    
    // Habilitar sensor de temperatura
    mpu6050SetTempSensorEnabled(TRUE);
    
    // Deshabilitar interrupciones
    mpu6050SetIntEnabled(FALSE);
    
    // Conectar bus I2C principal
    mpu6050SetI2CBypassEnabled(TRUE);
    
    // Configurar fuente de reloj
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    
    // Configurar rango del giroscopio
    mpu6050SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
    
    // Configurar rango del acelerómetro
    mpu6050SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);

    // Configurar filtro digital
#ifdef IMU_MPU6050_DLPF_256HZ
    mpu6050SetRate(15);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
#else
    mpu6050SetRate(1);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_188);
#endif

    return true;
}

/**
 * @brief Ejecuta self-test del MPU6050
 */
bool imu6Test(void) {
    if (imuState == IMU_STATE_UNINITIALIZED) {
        ESP_LOGE(TAG, "IMU no inicializada");
        return false;
    }

    if (imuState == IMU_STATE_ERROR) {
        ESP_LOGE(TAG, "IMU en estado de error");
        return false;
    }

    return mpu6050SelfTest();
}

/**
 * @brief Lee datos de la IMU y aplica calibración
 */
void imu6Read(Axis3f* gyroOut, Axis3f* accOut) {
    if (imuState < IMU_STATE_INITIALIZED || imuState == IMU_STATE_ERROR) {
        ESP_LOGW(TAG, "Intento de lectura con IMU no inicializada");
        if (gyroOut) memset(gyroOut, 0, sizeof(Axis3f));
        if (accOut) memset(accOut, 0, sizeof(Axis3f));
        return;
    }

    // Leer datos del sensor
    mpu6050GetMotion6(&accelMpu.x, &accelMpu.y, &accelMpu.z, 
                     &gyroMpu.x, &gyroMpu.y, &gyroMpu.z);

    // Procesar bias
    imuAddBiasValue(&gyroBias, &gyroMpu);
    if (!accelBias.isBiasValueFound) {
        imuAddBiasValue(&accelBias, &accelMpu);
    }

    // Buscar valores de bias si no están calibrados
    if (!gyroBias.isBiasValueFound) {
        if (imuFindBiasValue(&gyroBias)) {
            imuState = IMU_STATE_CALIBRATED;
            ESP_LOGI(TAG, "Giroscopio calibrado");
#ifdef LEDSEQ_ENABLED
            ledseqRun(LED_RED, seq_calibrated);
#endif
        }
    }

#ifdef IMU_TAKE_ACCEL_BIAS
    if (gyroBias.isBiasValueFound && !accelBias.isBiasValueFound) {
        Axis3i32 mean;
        if (imuCalculateBiasMean(&accelBias, &mean)) {
            accelBias.bias.x = mean.x;
            accelBias.bias.y = mean.y;
            accelBias.bias.z = mean.z - IMU_1G_RAW;
            accelBias.isBiasValueFound = true;
            ESP_LOGI(TAG, "Acelerómetro calibrado");
        }
    }
#endif

    // Aplicar filtros
    imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues,
                     (int32_t)imuAccLpfAttFactor);
    imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

    // Convertir y aplicar bias
    if (gyroOut) {
        gyroOut->x = (gyroMpu.x - gyroBias.bias.x) * IMU_DEG_PER_LSB_CFG;
        gyroOut->y = (gyroMpu.y - gyroBias.bias.y) * IMU_DEG_PER_LSB_CFG;
        gyroOut->z = (gyroMpu.z - gyroBias.bias.z) * IMU_DEG_PER_LSB_CFG;
    }

    if (accOut) {
        accOut->x = (accelLPFAligned.x - accelBias.bias.x) * IMU_G_PER_LSB_CFG;
        accOut->y = (accelLPFAligned.y - accelBias.bias.y) * IMU_G_PER_LSB_CFG;
        accOut->z = (accelLPFAligned.z - accelBias.bias.z) * IMU_G_PER_LSB_CFG;
    }
}

bool imu6IsCalibrated(void) {
    bool status = gyroBias.isBiasValueFound;
#ifdef IMU_TAKE_ACCEL_BIAS
    status &= accelBias.isBiasValueFound;
#endif
    return status && (imuState >= IMU_STATE_CALIBRATED);
}

imu_state_t imu6GetState(void) {
    return imuState;
}

const char* imu6GetStateString(void) {
    switch (imuState) {
        case IMU_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case IMU_STATE_INITIALIZED: return "INITIALIZED";
        case IMU_STATE_CALIBRATING: return "CALIBRATING";
        case IMU_STATE_CALIBRATED: return "CALIBRATED";
        case IMU_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// Funciones internas
static void imuBiasInit(BiasObj* bias) {
    if (!bias) return;
    memset(bias, 0, sizeof(BiasObj));
    bias->bufHead = bias->buffer;
}

static bool imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut) {
    if (!bias || !varOut || !meanOut || !bias->isBufferFilled) return false;

    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += (int64_t)bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += (int64_t)bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += (int64_t)bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

    meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
    meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
    meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

    return true;
}

static bool imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut) {
    if (!bias || !meanOut || !bias->isBufferFilled) return false;

    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
    }

    meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
    meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
    meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

    return true;
}

static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal) {
    if (!bias || !dVal) return;

    bias->bufHead->x = dVal->x;
    bias->bufHead->y = dVal->y;
    bias->bufHead->z = dVal->z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

static bool imuFindBiasValue(BiasObj* bias) {
    if (!bias || !bias->isBufferFilled) return false;

    Axis3i32 variance;
    Axis3i32 mean;

    if (!imuCalculateVarianceAndMean(bias, &variance, &mean)) return false;

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
        varianceSampleTime = xTaskGetTickCount();
        bias->bias.x = mean.x;
        bias->bias.y = mean.y;
        bias->bias.z = mean.z;
        bias->isBiasValueFound = true;
        return true;
    }

    return false;
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation) {
    if (!in || !out || !storedValues) return;

    out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
    out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
    out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}

static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out) {
    if (!in || !out) return;

    Axis3i16 rx, ry;

    // Rotar alrededor del eje X
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotar alrededor del eje Y
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}