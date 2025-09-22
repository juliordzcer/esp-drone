#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "led.h"
#include "adc.h"
#include "motors.h"

static const char *TAG = "MAIN";

// Definimos los ejes de la IMU para facilitar la lectura en los logs.
#define IMU_AXIS(label, value) label ": %+6.2f"

void app_main(void)
{
    ESP_LOGI(TAG, "Application starting...");
    
    // --- 1. INICIALIZACIÓN DE MÓDULOS DEL SISTEMA ---
    motorsInit();
    ledInit();
    adcInit();

    // --- 2. INICIALIZACIÓN DE LA IMU ---
    // La función imu6Init() se encarga de todo el proceso:
    // inicializar I2C, configurar el MPU6050 y empezar la calibración.
    if (!imu6Init()) {
        ESP_LOGE(TAG, "IMU initialization failed. Stopping.");
        ledSet(LED_RED, true);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    // --- 3. BUCLE PRINCIPAL ---
    Axis3f gyro_data;
    Axis3f accel_data;

    ESP_LOGI(TAG, "Entering main loop.");
    
    while (1) {
        // La función imu6Read() lee los datos, los filtra y los calibra.
        imu6Read(&gyro_data, &accel_data);
        
        // Verificamos si la IMU ha terminado el proceso de calibración.
        if (imu6IsCalibrated()) {
            // Imprimimos los valores calibrados y filtrados.
            ESP_LOGI(TAG, "IMU Data | "
                     IMU_AXIS("Gyro-X", gyro_data.x) " | "
                     IMU_AXIS("Gyro-Y", gyro_data.y) " | "
                     IMU_AXIS("Gyro-Z", gyro_data.z) " | "
                     IMU_AXIS("Accel-X", accel_data.x) " | "
                     IMU_AXIS("Accel-Y", accel_data.y) " | "
                     IMU_AXIS("Accel-Z", accel_data.z),
                     gyro_data.x, gyro_data.y, gyro_data.z,
                     accel_data.x, accel_data.y, accel_data.z);
                     
            ledSet(LED_GREEN, true); // Indicamos que el sistema está listo.
        } else {
            // Si no está calibrada, mostramos el estado actual.
            ESP_LOGW(TAG, "IMU is %s...", imu6GetStateString());
            ledSet(LED_RED, true); // Puedes usar el LED rojo para indicar que está calibrando.
        }

        // Leer voltaje de batería
        float battery_voltage = adcGetBatteryVoltage();
        if (battery_voltage < 3.3f && battery_voltage > 0) {
            // Lógica de parpadeo para batería baja
            static bool red_led_state = false;
            red_led_state = !red_led_state;
            ledSet(LED_RED, red_led_state);
        } else {
            // Apaga el LED rojo si la batería está bien y la IMU está calibrada
            if (imu6IsCalibrated()) {
                ledSet(LED_RED, false);
            }
        }
        
        // Espera para controlar la frecuencia del bucle
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}