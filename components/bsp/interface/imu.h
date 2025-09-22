#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include "filter.h"

#ifndef IMU_UPDATE_FREQ
#define IMU_UPDATE_FREQ   500
#endif

#ifndef IMU_UPDATE_RATE_MS
#define IMU_UPDATE_RATE_MS 100
#endif

#define IMU_UPDATE_DT     (1.0f / IMU_UPDATE_FREQ)

#ifndef IMU_ACC_WANTED_LPF_CUTOFF_HZ
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  10
#endif

#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415f * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  ((int)((1 << IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION + 0.5f))

typedef struct { int16_t x, y, z; } Axis3i16;
typedef struct { int32_t x, y, z; } Axis3i32;
typedef struct { float x, y, z; } Axis3f;

typedef enum {
    IMU_STATE_UNINITIALIZED,
    IMU_STATE_INITIALIZED,
    IMU_STATE_CALIBRATING,
    IMU_STATE_CALIBRATED,
    IMU_STATE_ERROR
} imu_state_t;

bool imu6Init(void);
bool imu6Test(void);
void imu6Read(Axis3f* gyro, Axis3f* acc);
bool imu6IsCalibrated(void);
imu_state_t imu6GetState(void);
const char* imu6GetStateString(void);

#endif