#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "i2c_dev.h"
#include "mpu6050.h"
#include "configblock.h"

#ifdef LEDSEQ_ENABLED
#include "ledseq.h"
#endif

static const char* TAG = "IMU";

#define IMU_GYRO_FS_CFG       MPU6050_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   MPU6050_DEG_PER_LSB_2000
#define IMU_ACCEL_FS_CFG      MPU6050_ACCEL_FS_8
#define IMU_G_PER_LSB_CFG     MPU6050_G_PER_LSB_8
#define IMU_1G_RAW            (int16_t)(1.0f / MPU6050_G_PER_LSB_8)

#define IMU_STARTUP_DELAY_MS  1000
#define GYRO_MIN_BIAS_TIMEOUT_MS 1000
#define IMU_NBR_OF_BIAS_SAMPLES  64
#define GYRO_VARIANCE_BASE       4000
#define GYRO_VARIANCE_THRESHOLD  GYRO_VARIANCE_BASE

typedef struct {
    Axis3i16 bias;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16* bufHead;
    Axis3i16 buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static BiasObj gyroBias = {0};
static BiasObj accelBias = {0};
static int32_t varianceSampleTime = 0;
static Axis3i16 gyroMpu = {0};
static Axis3i16 accelMpu = {0};
static Axis3i16 accelLPF = {0};
static Axis3i16 accelLPFAligned = {0};
static Axis3i32 accelStoredFilterValues = {0};
static uint8_t imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

static float cosPitch = 0.0f;
static float sinPitch = 0.0f;
static float cosRoll = 0.0f;
static float sinRoll = 0.0f;

static imu_state_t imuState = IMU_STATE_UNINITIALIZED;
static bool isI2cInitialized = false;

static void imuBiasInit(BiasObj* bias);
static bool imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* var, Axis3i32* mean);
static bool imuCalculateBiasMean(BiasObj* bias, Axis3i32* mean);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* data);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* stored, int att);
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);
static bool imuConfigureMpu6050(void);

bool imu6Init(void) {
    if (imuState >= IMU_STATE_INITIALIZED) {
        ESP_LOGW(TAG, "IMU already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing IMU...");
    imuState = IMU_STATE_CALIBRATING;

    vTaskDelay(pdMS_TO_TICKS(IMU_STARTUP_DELAY_MS));

    if (!isI2cInitialized) {
        if (!i2cdevInit(I2C0_DEV)) {
            ESP_LOGE(TAG, "I2C initialization failed");
            imuState = IMU_STATE_ERROR;
            return false;
        }
        isI2cInitialized = true;
    }

    mpu6050Init(I2C0_DEV);

    if (!mpu6050TestConnection()) {
        ESP_LOGE(TAG, "MPU6050 connection failed");
        imuState = IMU_STATE_ERROR;
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 connected");

    if (!imuConfigureMpu6050()) {
        ESP_LOGE(TAG, "MPU6050 configuration failed");
        imuState = IMU_STATE_ERROR;
        return false;
    }

    imuBiasInit(&gyroBias);
    imuBiasInit(&accelBias);
    varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;

    float pitchRad = configblockGetCalibPitch() * (float)M_PI / 180.0f;
    float rollRad = configblockGetCalibRoll() * (float)M_PI / 180.0f;
    cosPitch = cosf(pitchRad);
    sinPitch = sinf(pitchRad);
    cosRoll = cosf(rollRad);
    sinRoll = sinf(rollRad);

    imuState = IMU_STATE_INITIALIZED;
    ESP_LOGI(TAG, "IMU initialized successfully");
    return true;
}

static bool imuConfigureMpu6050(void) {
    mpu6050Reset();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    mpu6050SetSleepEnabled(false);
    mpu6050SetTempSensorEnabled(true);
    mpu6050SetIntEnabled(false);
    mpu6050SetI2CBypassEnabled(true);
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu6050SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
    mpu6050SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);

#ifdef IMU_MPU6050_DLPF_256HZ
    mpu6050SetRate(15);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
#else
    mpu6050SetRate(1);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_188);
#endif

    return true;
}

bool imu6Test(void) {
    if (imuState == IMU_STATE_UNINITIALIZED) {
        ESP_LOGE(TAG, "IMU not initialized");
        return false;
    }

    if (imuState == IMU_STATE_ERROR) {
        ESP_LOGE(TAG, "IMU in error state");
        return false;
    }

    return mpu6050SelfTest();
}

void imu6Read(Axis3f* gyroOut, Axis3f* accOut) {
    if (imuState < IMU_STATE_INITIALIZED || imuState == IMU_STATE_ERROR) {
        if (gyroOut) memset(gyroOut, 0, sizeof(Axis3f));
        if (accOut) memset(accOut, 0, sizeof(Axis3f));
        return;
    }

    mpu6050GetMotion6(&accelMpu.x, &accelMpu.y, &accelMpu.z, 
                     &gyroMpu.x, &gyroMpu.y, &gyroMpu.z);

    imuAddBiasValue(&gyroBias, &gyroMpu);
    
    if (!accelBias.isBiasValueFound) {
        imuAddBiasValue(&accelBias, &accelMpu);
    }

    if (!gyroBias.isBiasValueFound && imuFindBiasValue(&gyroBias)) {
        imuState = IMU_STATE_CALIBRATED;
        ESP_LOGI(TAG, "Gyroscope calibrated");
#ifdef LEDSEQ_ENABLED
        ledseqRun(LED_RED, seq_calibrated);
#endif
    }

#ifdef IMU_TAKE_ACCEL_BIAS
    if (gyroBias.isBiasValueFound && !accelBias.isBiasValueFound) {
        Axis3i32 mean;
        if (imuCalculateBiasMean(&accelBias, &mean)) {
            accelBias.bias = (Axis3i16){mean.x, mean.y, mean.z - IMU_1G_RAW};
            accelBias.isBiasValueFound = true;
            ESP_LOGI(TAG, "Accelerometer calibrated");
        }
    }
#endif

    imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues, imuAccLpfAttFactor);
    imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

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
    bool calibrated = gyroBias.isBiasValueFound;
#ifdef IMU_TAKE_ACCEL_BIAS
    calibrated &= accelBias.isBiasValueFound;
#endif
    return calibrated && (imuState >= IMU_STATE_CALIBRATED);
}

imu_state_t imu6GetState(void) {
    return imuState;
}

const char* imu6GetStateString(void) {
    static const char* states[] = {
        "UNINITIALIZED", "INITIALIZED", "CALIBRATING", "CALIBRATED", "ERROR"
    };
    return (imuState < sizeof(states)/sizeof(states[0])) ? states[imuState] : "UNKNOWN";
}

static void imuBiasInit(BiasObj* bias) {
    if (!bias) return;
    memset(bias, 0, sizeof(BiasObj));
    bias->bufHead = bias->buffer;
}

static bool imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* var, Axis3i32* mean) {
    if (!bias || !var || !mean || !bias->isBufferFilled) return false;

    int32_t sum[3] = {0};
    int64_t sumSq[3] = {0};

    for (uint32_t i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += (int64_t)bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += (int64_t)bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += (int64_t)bias->buffer[i].z * bias->buffer[i].z;
    }

    var->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
    var->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
    var->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

    mean->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
    mean->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
    mean->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

    return true;
}

static bool imuCalculateBiasMean(BiasObj* bias, Axis3i32* mean) {
    if (!bias || !mean || !bias->isBufferFilled) return false;

    int32_t sum[3] = {0};
    for (uint32_t i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
    }

    mean->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
    mean->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
    mean->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

    return true;
}

static void imuAddBiasValue(BiasObj* bias, Axis3i16* data) {
    if (!bias || !data) return;

    *bias->bufHead = *data;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

static bool imuFindBiasValue(BiasObj* bias) {
    if (!bias || !bias->isBufferFilled) return false;

    Axis3i32 variance, mean;
    if (!imuCalculateVarianceAndMean(bias, &variance, &mean)) return false;

    if (variance.x < GYRO_VARIANCE_THRESHOLD &&
        variance.y < GYRO_VARIANCE_THRESHOLD &&
        variance.z < GYRO_VARIANCE_THRESHOLD &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
        varianceSampleTime = xTaskGetTickCount();
        bias->bias = (Axis3i16){mean.x, mean.y, mean.z};
        bias->isBiasValueFound = true;
        return true;
    }

    return false;
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* stored, int att) {
    if (!in || !out || !stored) return;
    
    out->x = iirLPFilterSingle(in->x, att, &stored->x);
    out->y = iirLPFilterSingle(in->y, att, &stored->y);
    out->z = iirLPFilterSingle(in->z, att, &stored->z);
}

static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out) {
    if (!in || !out) return;

    // Rotate around X axis
    int16_t rx_y = in->y * cosRoll - in->z * sinRoll;
    int16_t rx_z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around Y axis
    out->x = in->x * cosPitch - rx_z * sinPitch;
    out->y = rx_y;
    out->z = -in->x * sinPitch + rx_z * cosPitch;
}