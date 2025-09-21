#include <stdbool.h>
#include "motors.h"

// Includes de ESP-IDF y FreeRTOS
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Tag para los logs
static const char *TAG = "MOTORS";

// Macro para convertir el valor de 16 bits a la resolución del PWM (9 bits)
// Es un simple desplazamiento a la derecha de (16 - 9) = 7 bits.
#define C_16_TO_BITS(X) ((X) >> (16 - MOTORS_PWM_BITS))
// Macro para convertir la resolución del PWM (9 bits) de vuelta a 16 bits
#define C_BITS_TO_16(X) ((X) << (16 - MOTORS_PWM_BITS))

// Mapeo de los motores a los canales del periférico LEDC
#define MOTOR_LEFT_CH   LEDC_CHANNEL_0
#define MOTOR_REAR_CH   LEDC_CHANNEL_1
#define MOTOR_RIGHT_CH  LEDC_CHANNEL_2
#define MOTOR_FRONT_CH  LEDC_CHANNEL_3

// Variable para asegurar que la inicialización solo ocurra una vez
static bool isInit = false;

void motorsInit() {
    if (isInit) {
        return;
    }

    ESP_LOGI(TAG, "Initializing motors...");

    // 1. Configurar el Timer del LEDC
    // Usaremos un solo timer para los 4 canales, así todos tendrán la misma frecuencia.
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE, // Modo de baja velocidad es suficiente
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = MOTORS_PWM_BITS, // 9 bits de resolución (0-511)
        .freq_hz          = 5000, // 5 kHz es una frecuencia común para ESCs
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // 2. Configurar los Canales del LEDC (uno por cada motor)
    ledc_channel_config_t ledc_channel[] = {
        { // Motor Izquierdo (MOT_3 -> GPIO3)
            .gpio_num   = MOTOR_LEFT_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = MOTOR_LEFT_CH,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        },
        { // Motor Trasero (MOT_4 -> GPIO4)
            .gpio_num   = MOTOR_REAR_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = MOTOR_REAR_CH,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        },
        { // Motor Derecho (MOT_2 -> GPIO6)
            .gpio_num   = MOTOR_RIGHT_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = MOTOR_RIGHT_CH,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        },
        { // Motor Frontal (MOT_1 -> GPIO5)
            .gpio_num   = MOTOR_FRONT_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = MOTOR_FRONT_CH,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        },
    };

    // Aplicar la configuración para cada canal
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
    }
    
    ESP_LOGI(TAG, "Motors initialized successfully!");
    isInit = true;
}

bool motorsTest(void) {
    if (!isInit) return false;

    ESP_LOGI(TAG, "Running motor test sequence...");
    motorsSetRatio(MOTOR_FRONT, MOTORS_TEST_RATIO);
    vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_ON_TIME));
    motorsSetRatio(MOTOR_FRONT, 0);
    vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_DELAY_TIME));

    motorsSetRatio(MOTOR_RIGHT, MOTORS_TEST_RATIO);
    vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_ON_TIME));
    motorsSetRatio(MOTOR_RIGHT, 0);
    vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_DELAY_TIME));

    motorsSetRatio(MOTOR_REAR, MOTORS_TEST_RATIO);
    vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_ON_TIME));
    motorsSetRatio(MOTOR_REAR, 0);
    vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_DELAY_TIME));

    motorsSetRatio(MOTOR_LEFT, MOTORS_TEST_RATIO);
    vTaskDelay(pdMS_TO_TICKS(MOTORS_TEST_ON_TIME));
    motorsSetRatio(MOTOR_LEFT, 0);
    ESP_LOGI(TAG, "Motor test finished.");

    return isInit;
}

void motorsSetRatio(int id, uint16_t ratio) {
    if (!isInit) return;

    // Escalar el valor de 16 bits al valor de 9 bits (0-511)
    uint32_t duty = C_16_TO_BITS(ratio);
    ledc_channel_t channel;

    switch(id) {
        case MOTOR_LEFT:  channel = MOTOR_LEFT_CH;  break;
        case MOTOR_REAR:  channel = MOTOR_REAR_CH;  break;
        case MOTOR_RIGHT: channel = MOTOR_RIGHT_CH; break;
        case MOTOR_FRONT: channel = MOTOR_FRONT_CH; break;
        default: return; // ID de motor no válido
    }
    
    // Establecer el nuevo duty cycle y actualizar la salida
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

int motorsGetRatio(int id) {
    if (!isInit) return -1;
    
    ledc_channel_t channel;
    switch(id) {
        case MOTOR_LEFT:  channel = MOTOR_LEFT_CH;  break;
        case MOTOR_REAR:  channel = MOTOR_REAR_CH;  break;
        case MOTOR_RIGHT: channel = MOTOR_RIGHT_CH; break;
        case MOTOR_FRONT: channel = MOTOR_FRONT_CH; break;
        default: return -1; // ID de motor no válido
    }

    uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, channel);
    // Convertir el valor de 9 bits de vuelta a 16 bits
    return C_BITS_TO_16(duty);
}

// Tarea de FreeRTOS para probar los motores
void motorsTestTask(void* params) {
    ESP_LOGI(TAG, "Starting motor test task...");
    
    // Esperar 3 segundos antes de iniciar la secuencia
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    static const uint16_t sequence[] = {
        (uint16_t)(0.10 * 65535), // 10%
        (uint16_t)(0.15 * 65535), // 15%
        (uint16_t)(0.20 * 65535), // 20%
        (uint16_t)(0.25 * 65535)  // 25%
    };
    int step = 0;

    while(1) {
        motorsSetRatio(MOTOR_LEFT,  sequence[step % 4]);
        motorsSetRatio(MOTOR_REAR,  sequence[(step + 1) % 4]);
        motorsSetRatio(MOTOR_RIGHT, sequence[(step + 2) % 4]);
        motorsSetRatio(MOTOR_FRONT, sequence[(step + 3) % 4]);

        step++;
        if (step >= 4) {
            step = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}