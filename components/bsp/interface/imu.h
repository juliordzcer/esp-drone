#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include "filter.h"

// Configuración por defecto
#ifndef IMU_UPDATE_FREQ
#define IMU_UPDATE_FREQ   500
#endif

#ifndef IMU_UPDATE_RATE_MS
#define IMU_UPDATE_RATE_MS 100
#endif

#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

// Configuración del filtro LPF
#ifndef IMU_ACC_WANTED_LPF_CUTOFF_HZ
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  10
#endif

#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)

// Estructuras de datos
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Axis3i16;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} Axis3i32;

typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

// Estados de la IMU
typedef enum {
    IMU_STATE_UNINITIALIZED = 0,
    IMU_STATE_INITIALIZED,
    IMU_STATE_CALIBRATING,
    IMU_STATE_CALIBRATED,
    IMU_STATE_ERROR
} imu_state_t;

// Prototipos de funciones
bool imu6Init(void);
bool imu6Test(void);
void imu6Read(Axis3f* gyro, Axis3f* acc);
bool imu6IsCalibrated(void);
imu_state_t imu6GetState(void);
const char* imu6GetStateString(void);

#endif /* IMU_H_ */