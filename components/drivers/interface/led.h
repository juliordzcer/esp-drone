#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>

// --- Configuración de Hardware para ESP32 ---

// Mapeo de pines
#define LED_RED_GPIO    8
#define LED_GREEN_GPIO  9
#define LED_BLUE_GPIO   7

// Polaridad del LED (cómo se enciende)
// LED_POL_POS = Positiva (GPIO en HIGH enciende el LED)
// LED_POL_NEG = Negativa (GPIO en LOW enciende el LED)
#define LED_POL_POS 0
#define LED_POL_NEG 1

// --- AJUSTE REALIZADO AQUÍ ---
// Se cambia la polaridad para que coincida con el hardware real.
#define LED_RED_POL     LED_POL_POS
#define LED_GREEN_POL   LED_POL_POS
#define LED_BLUE_POL    LED_POL_POS
// ---------------------------------------------

// Se actualiza el número de LEDs
#define LED_NUM 3

// Enum para identificar cada LED
typedef enum {
    LED_RED = 0,
    LED_GREEN,
    LED_BLUE
} led_t;

/**
 * @brief Inicializa los pines GPIO para los LEDs.
 */
void ledInit();

/**
 * @brief Enciende o apaga un LED específico.
 * * @param led El LED a controlar (LED_RED, LED_GREEN, o LED_BLUE).
 * @param value true para encender, false para apagar.
 */
void ledSet(led_t led, bool value);

#endif // __LED_H__