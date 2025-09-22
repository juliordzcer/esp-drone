#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Incluye los drivers para tus periféricos
#include "motors.h"
#include "led.h"
#include "adc.h"

// Incluye el driver del MPU6050 (que a su vez incluye el de i2c_dev)
#include "mpu6050.h"

static const char *TAG = "APP_MAIN";

#define LOW_BATTERY_VOLTAGE 3.3f

// Declaramos la variable que actuará como "handle" o identificador para el bus I2C.
// Es un 'int' porque así lo definimos en mpu6050.h (con 'typedef int I2C_TypeDef;')
// El valor 0 corresponde a I2C_NUM_0 en nuestro driver i2c_dev.
static int i2c_sensor_bus = 0;

void app_main(void)
{
    ESP_LOGI(TAG, "Application starting...");

    // --- 1. INICIALIZACIÓN DE MÓDULOS ---
    motorsInit();
    ledInit();
    adcInit();

    // Primero, inicializa el driver del bus I2C.
    // Le pasamos la dirección de nuestra variable handle.
    i2cdevInit(&i2c_sensor_bus);

    // Luego, inicializa el driver del MPU6050, que usará el bus I2C ya configurado.
    mpu6050Init(&i2c_sensor_bus);
    
    // --- 2. VERIFICACIÓN DE CONEXIÓN ---
    // Es una buena práctica asegurarse de que la comunicación con la IMU funciona.
    if (mpu6050TestConnection()) {
        ESP_LOGI(TAG, "MPU6050 connection successful!");
        ledSet(LED_GREEN, true); // Encendemos el LED verde para indicar éxito.
    } else {
        ESP_LOGE(TAG, "MPU6050 connection FAILED!");
        ledSet(LED_RED, true); // Encendemos el LED rojo para indicar un error crítico.
        // Detenemos la ejecución si la IMU no funciona, ya que es un componente esencial.
        while(1) { 
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        }
    }

    ESP_LOGI(TAG, "All modules initialized. Entering main loop.");
    
    // Variables para almacenar los datos crudos (raw) de la IMU
    int16_t ax, ay, az; // Acelerómetro
    int16_t gx, gy, gz; // Giroscopio

    // Variable para controlar el parpadeo del LED de batería baja
    bool red_led_state = false;

    // --- 3. BUCLE PRINCIPAL ---
    while (1) {
        // Leer los datos de los 6 ejes del sensor
        mpu6050GetMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // Imprimir los valores en la consola
        ESP_LOGI(TAG, "IMU RAW -> Accel[X:%6d, Y:%6d, Z:%6d] | Gyro[X:%6d, Y:%6d, Z:%6d]", 
                 ax, ay, az, gx, gy, gz);

        // Comprobar el voltaje de la batería y hacer parpadear el LED rojo si es bajo
        float battery_voltage = adcGetBatteryVoltage();
        if (battery_voltage < LOW_BATTERY_VOLTAGE && battery_voltage > 0) {
            red_led_state = !red_led_state;
            ledSet(LED_RED, red_led_state);
        } else {
            // Si la batería está bien, el LED rojo debe estar apagado.
            ledSet(LED_RED, false);
            red_led_state = false;
        }

        // Espera para controlar la frecuencia del bucle
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}