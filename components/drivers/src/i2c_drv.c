/**
 * @file i2c_drv.c
 * @brief Implementación del driver de inicialización I2C para ESP32.
 */

#include "i2c_drv.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h" // Necesario para los identificadores GPIO_NUM_...

// Etiqueta para los mensajes de log
static const char *TAG = "I2CDRV";

// --- Definiciones de los Buses I2C del Sistema ---

// Configuración para el bus de sensores (I2C0)
static const I2cDef sensorBusDef = {
    .i2cPort        = I2C_NUM_0,
    .i2cClockSpeed  = 400000, // 400 KHz
    // CORRECCIÓN: Usamos los números de GPIO directamente.
    .gpioSCLPin     = GPIO_NUM_10,
    .gpioSDAPin     = GPIO_NUM_11,
    .gpioPullup     = GPIO_PULLUP_DISABLE,
};

// Instancia del driver para el bus de sensores
I2cDrv sensorsBus = {
    .def            = &sensorBusDef,
    .isBusFreeMutex = NULL,
    .is_initialized = false,
};

// Configuración para el bus de expansión (I2C1)
static const I2cDef deckBusDef = {
    .i2cPort        = I2C_NUM_1,
    .i2cClockSpeed  = 100000, // 100 KHz
    // CORRECCIÓN: Usamos los números de GPIO directamente.
    .gpioSCLPin     = GPIO_NUM_41,
    .gpioSDAPin     = GPIO_NUM_40,
    .gpioPullup     = GPIO_PULLUP_ENABLE,
};

// Instancia del driver para el bus de expansión
I2cDrv deckBus = {
    .def            = &deckBusDef,
    .isBusFreeMutex = NULL,
    .is_initialized = false,
};


esp_err_t i2cdrvInit(I2cDrv *i2c)
{
    if (i2c->is_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2C port %d", i2c->def->i2cPort);

    if (i2c->isBusFreeMutex == NULL) {
        i2c->isBusFreeMutex = xSemaphoreCreateMutex();
        if (i2c->isBusFreeMutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex for I2C port %d", i2c->def->i2cPort);
            return ESP_FAIL;
        }
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c->def->gpioSDAPin,
        .sda_pullup_en = i2c->def->gpioPullup,
        .scl_io_num = i2c->def->gpioSCLPin,
        .scl_pullup_en = i2c->def->gpioPullup,
        .master.clk_speed = i2c->def->i2cClockSpeed,
    };

    esp_err_t err = i2c_param_config(i2c->def->i2cPort, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C#%d param config failed. Error: %s", i2c->def->i2cPort, esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c->def->i2cPort, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C#%d driver install failed. Error: %s", i2c->def->i2cPort, esp_err_to_name(err));
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "I2C#%d driver was already installed.", i2c->def->i2cPort);
        } else {
            return err;
        }
    }

    ESP_LOGI(TAG, "I2C#%d driver installed successfully", i2c->def->i2cPort);
    i2c->is_initialized = true;

    return ESP_OK;
}