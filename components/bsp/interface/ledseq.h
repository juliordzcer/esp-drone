#ifndef __LEDSEQ_H__
#define __LEDSEQ_H__

#include <stdbool.h>
#include "led.h" 
#include <stdint.h>

/*
 * Un 'ledseq' (secuencia de LED) es una lista de acciones.
 * Cada acción contiene el estado del LED (encendido/apagado) y un tiempo de espera
 * o un comando especial como LOOP (repetir) o STOP (detener).
 *
 * Las secuencias se organizan por prioridad en el archivo .c.
 * Para cada LED, solo se ejecuta la secuencia activa de mayor prioridad.
 */

// --- Definiciones de acciones para las secuencias ---

// Convierte un valor en milisegundos a una acción de espera
#define LEDSEQ_WAITMS(X) (X)
// Acción para detener la secuencia y ceder el control a una de menor prioridad
#define LEDSEQ_STOP      -1
// Acción para reiniciar la secuencia desde el principio
#define LEDSEQ_LOOP      -2

// Estructura que define un paso en una secuencia de LED
typedef struct {
  bool value; // Estado del LED (true = ON, false = OFF)
  int  action;  // Duración en ms, o un comando (STOP/LOOP)
} ledseq_t;

// --- API Pública ---

/**
 * @brief Inicializa el sistema de secuencias de LED.
 * Debe llamarse una vez al arranque.
 */
void ledseqInit(void);

/**
 * @brief Ejecuta una prueba del módulo de secuencia de LEDs.
 * @return true si la prueba se ejecuta, false en caso de fallo.
 */
bool ledseqTest(void);

/**
 * @brief Inicia la ejecución de una secuencia en un LED específico.
 *
 * @param led El LED a controlar (LED_RED, LED_GREEN, LED_BLUE).
 * @param sequence Puntero a la secuencia a ejecutar.
 */
void ledseqRun(led_t led, ledseq_t * sequence);

/**
 * @brief Detiene una secuencia específica en un LED.
 * Si había una secuencia de menor prioridad en espera, esta se activará.
 *
 * @param led El LED afectado.
 * @param sequence Puntero a la secuencia a detener.
 */
void ledseqStop(led_t led, ledseq_t * sequence);

/**
 * @brief Modifica dinámicamente los tiempos de una secuencia.
 * Útil para secuencias simples de dos pasos (ON/OFF).
 *
 * @param sequence Puntero a la secuencia a modificar.
 * @param onTime Nuevo tiempo de encendido en ms.
 * @param offTime Nuevo tiempo de apagado en ms.
 */
void ledseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime);

// --- Secuencias predefinidas (declaradas como externas) ---
extern ledseq_t seq_armed[];
extern ledseq_t seq_calibrated[];
extern ledseq_t seq_alive[];
extern ledseq_t seq_lowbat[];
extern ledseq_t seq_linkup[];
extern ledseq_t seq_hover[];
extern ledseq_t seq_charged[];
extern ledseq_t seq_charging[];
extern ledseq_t seq_bootloader[];
extern ledseq_t seq_testPassed[];

#endif // __LEDSEQ_H__