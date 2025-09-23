#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>

// Estructuras de datos para los ejes (puedes moverlas a un imu_types.h si prefieres)
typedef struct { float x, y, z; } Axis3f;

// Estados del IMU
typedef enum {
    IMU_STATE_UNINITIALIZED,
    IMU_STATE_INITIALIZED,
    IMU_STATE_CALIBRATING,
    IMU_STATE_CALIBRATED,
    IMU_STATE_ERROR
} imu_state_t;

/**
 * @brief Inicializa el sensor MPU-6050 y el driver.
 * @return true si la inicialización fue exitosa.
 */
bool imu_init(void);

/**
 * @brief Realiza una auto-prueba de hardware del MPU-6050.
 * @return true si la prueba fue exitosa.
 */
bool imu_test(void);

/**
 * @brief Lee los datos del giroscopio y acelerómetro.
 *
 * Durante las primeras llamadas, esta función realizará la calibración.
 * Una vez calibrado, devolverá lecturas corregidas y filtradas.
 *
 * @param gyro Puntero para almacenar los datos del giroscopio en grados/segundo.
 * @param acc Puntero para almacenar los datos del acelerómetro en g's.
 */
void imu_read(Axis3f* gyro, Axis3f* acc);

/**
 * @brief Comprueba si el IMU ha completado la calibración.
 * @return true si la calibración de giroscopio y acelerómetro ha finalizado.
 */
bool imu_is_calibrated(void);

/**
 * @brief Obtiene el estado actual del driver del IMU.
 * @return El estado actual como un enum imu_state_t.
 */
imu_state_t imu_get_state(void);

/**
 * @brief Obtiene el estado actual del driver como una cadena de texto.
 * @return Una cadena de texto que describe el estado actual.
 */
const char* imu_get_state_string(void);

#endif // IMU_H_