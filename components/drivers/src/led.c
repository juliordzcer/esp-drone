#include "led.h"

// Includes de ESP-IDF
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Incluimos el driver de motores para la funcionalidad de prueba condicional
#include <motors.h>

// Flag para asegurar que la inicialización solo se ejecute una vez
static bool isInit = false;

// Arrays para mapear el enum 'led_t' a la configuración de hardware
static const uint8_t led_pin[] = {
    [LED_RED]   = LED_RED_GPIO,
    [LED_GREEN] = LED_GREEN_GPIO,
    [LED_BLUE]  = LED_BLUE_GPIO,
};

static const int led_polarity[] = {
    [LED_RED]   = LED_RED_POL,
    [LED_GREEN] = LED_GREEN_POL,
    [LED_BLUE]  = LED_BLUE_POL,
};

void ledInit() {
    if (isInit) {
        return;
    }

    // Configuración de los pines GPIO para los LEDs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO) | (1ULL << LED_BLUE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Apagar todos los LEDs al iniciar
    ledSet(LED_RED, false);
    ledSet(LED_GREEN, false);
    ledSet(LED_BLUE, false);
  
    isInit = true;
}

void ledSet(led_t led, bool value) {
    if (led >= LED_NUM) {
        return;
    }
  
    // Esta lógica ahora se saltará la inversión
    if (led_polarity[led] == LED_POL_NEG) {
        value = !value;
    }
  
    gpio_set_level(led_pin[led], value);

    #ifdef MOTORS_TEST    
    if (led == LED_RED) {
        static int step = 0;
        uint16_t ratio_on = 0x3FFF;
        
        if (!value) {
            motorsSetRatio(step, ratio_on);
            step++;
            if (step > 3) step = 0;
        } else {
            for (int i=0; i<4; i++) {
                motorsSetRatio(i, 0);
            }
        }
    }
    #endif
}