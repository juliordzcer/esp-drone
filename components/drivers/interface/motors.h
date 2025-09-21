#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

/******** Defines ********/

// Mapeo de pines para tu ESP32
#define MOTOR_FRONT_GPIO    5   // Tu MOT_1
#define MOTOR_RIGHT_GPIO    6   // Tu MOT_2
#define MOTOR_LEFT_GPIO     3   // Tu MOT_3
#define MOTOR_REAR_GPIO     4   // Tu MOT_4

// Configuración del PWM con el periférico LEDC del ESP32
// El original usaba 9 bits, lo cual es perfecto para el LEDC.
#define MOTORS_PWM_BITS     9
// El período se calcula automáticamente con los bits de resolución
#define MOTORS_PWM_PERIOD   ((1 << MOTORS_PWM_BITS) - 1)

// IDs de los motores (se mantienen igual)
#define MOTOR_LEFT  0
#define MOTOR_REAR  1
#define MOTOR_RIGHT 2
#define MOTOR_FRONT 3

// Defines para la función de prueba
#define MOTORS_TEST_RATIO         (uint16_t)(0.2 * 65535) // 20% del duty cycle para el test
#define MOTORS_TEST_ON_TIME       200 // ms
#define MOTORS_TEST_DELAY_TIME    100 // ms

/*** Interfaz pública ***/

/**
 * @brief Inicializa el periférico LEDC para controlar los 4 motores.
 * Configura los timers y canales PWM.
 */
void motorsInit();

/**
 * @brief Prueba de los motores. Gira cada motor brevemente en secuencia.
 * @return true si el driver fue inicializado, false en caso contrario.
 */
bool motorsTest(void);

/**
 * @brief Establece la velocidad (duty cycle) de un motor específico.
 * @param id El identificador del motor (MOTOR_LEFT, MOTOR_RIGHT, etc.).
 * @param ratio Un valor de 16 bits (0 a 65535) que representa la velocidad de 0% a 100%.
 */
void motorsSetRatio(int id, uint16_t ratio);

/**
 * @brief Obtiene la velocidad actual (duty cycle) de un motor.
 * @param id El identificador del motor.
 * @return La velocidad como un valor de 16 bits, o -1 si el ID es inválido.
 */
int motorsGetRatio(int id);

/**
 * @brief Tarea de FreeRTOS para probar los motores con una secuencia.
 */
void motorsTestTask(void* params);

#endif /* __MOTORS_H__ */