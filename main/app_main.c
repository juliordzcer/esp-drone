#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_dev.h"
#include "i2c_drv.h"
#include "vl53l1x.h"

static const char *TAG = "MAIN_APP";
static VL53L1_Dev_t vl53l1x_dev;

void app_main(void) {

    // <<--- SE ELIMINA ESTE BLOQUE
    // Ya no inicializamos el bus aquí, porque el driver vl53l1xInit() lo hará por nosotros.
    /*
    ESP_LOGI(TAG, "Inicializando I2C Bus para el sensor ToF...");
    if (!i2cdevInit(I2C1_DEV)) {
        ESP_LOGE(TAG, "Fallo al inicializar I2C_BUS_DECK. Deteniendo.");
        return;
    }
    */

    // Directamente llamamos a la inicialización del sensor.
    // Esta función ahora es responsable de inicializar tanto el sensor como el bus I2C.
    ESP_LOGI(TAG, "Inicializando Lidar Sensor VL53L1X...");
    if (vl53l1xInit(&vl53l1x_dev, I2C1_DEV)) {
        ESP_LOGI(TAG,"Lidar Sensor VL53L1X [OK]");
    } else {
        ESP_LOGE(TAG,"Lidar Sensor VL53L1X [FAIL]");
        return;
    }

    // El resto del código para configurar y leer el sensor no cambia.
    ESP_LOGI(TAG, "Configurando sensor para medición...");
    VL53L1_SetDistanceMode(&vl53l1x_dev, VL53L1_DISTANCEMODE_LONG);
    VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x_dev, 33000); // 33ms es un buen valor
    VL53L1_StartMeasurement(&vl53l1x_dev);
    ESP_LOGI(TAG, "Iniciando bucle de mediciones.");

    while(1) {
        uint8_t dataReady = 0;
        
        // Espera a que el dato esté listo
        while (dataReady == 0) {
            VL53L1_GetMeasurementDataReady(&vl53l1x_dev, &dataReady);
            vTaskDelay(pdMS_TO_TICKS(5)); 
        }

        VL53L1_RangingMeasurementData_t rangingData;
        VL53L1_GetRangingMeasurementData(&vl53l1x_dev, &rangingData);
        
        // Limpia la interrupción para la siguiente medida
        VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x_dev);

        if (rangingData.RangeStatus == 0) { // Status 0 es una medida válida
             ESP_LOGI(TAG, "Distancia: %d mm", rangingData.RangeMilliMeter);
        } else {
             // Opcional: Imprimir un warning si la medida no es válida
             ESP_LOGW(TAG, "Medida fuera de rango o inválida (Status: %d)", rangingData.RangeStatus);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Espera un poco entre medidas
    }
}