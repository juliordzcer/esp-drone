#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "i2c_drv.h"

static const char *TAG = "I2CDRV";

#define I2C_DEFAULT_SENSORS_CLOCK_SPEED   400000
#define I2C_DEFAULT_DECK_CLOCK_SPEED      100000
#define I2C_OPERATION_TIMEOUT_MS          100
#define I2C_RETRY_DELAY_MS                5

static const I2cDef sensorBusDef = {
    .i2cPort        = I2C_NUM_0,
    .i2cClockSpeed  = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPin     = GPIO_NUM_10,
    .gpioSDAPin     = GPIO_NUM_11,
    .gpioPullup     = GPIO_PULLUP_DISABLE,
};

static const I2cDef deckBusDef = {
    .i2cPort        = I2C_NUM_1,
    .i2cClockSpeed  = I2C_DEFAULT_DECK_CLOCK_SPEED,
    .gpioSCLPin     = GPIO_NUM_41,
    .gpioSDAPin     = GPIO_NUM_40,
    .gpioPullup     = GPIO_PULLUP_ENABLE,
};

I2cDrv sensorsBus = { .def = &sensorBusDef, .is_initialized = false };
I2cDrv deckBus = { .def = &deckBusDef, .is_initialized = false };

esp_err_t i2cdrvInit(I2cDrv *i2c)
{
    if (i2c->is_initialized) {
        return ESP_OK;
    }

    if (i2c->isBusFreeMutex == NULL) {
        i2c->isBusFreeMutex = xSemaphoreCreateMutex();
        if (i2c->isBusFreeMutex == NULL) {
            ESP_LOGE(TAG, "Mutex creation failed for I2C port %d", i2c->def->i2cPort);
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
        ESP_LOGE(TAG, "I2C#%d config error: %s", i2c->def->i2cPort, esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c->def->i2cPort, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C#%d install error: %s", i2c->def->i2cPort, esp_err_to_name(err));
        return err;
    }

    i2c->is_initialized = true;
    ESP_LOGI(TAG, "I2C#%d initialized", i2c->def->i2cPort);
    return ESP_OK;
}

bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message)
{
    if (!i2c->is_initialized || i2c->isBusFreeMutex == NULL) {
        ESP_LOGE(TAG, "I2C#%d not initialized", i2c->def->i2cPort);
        message->status = i2cNack;
        return false;
    }

    if (xSemaphoreTake(i2c->isBusFreeMutex, pdMS_TO_TICKS(I2C_OPERATION_TIMEOUT_MS)) == pdFALSE) {
        ESP_LOGE(TAG, "I2C#%d bus busy", i2c->def->i2cPort);
        message->status = i2cNack;
        return false;
    }

    esp_err_t err = ESP_FAIL;
    message->status = i2cNack;

    for (int attempt = 0; attempt <= message->nbrOfRetries; attempt++) {
        if (message->direction == i2cWrite) {
            err = i2c_master_write_to_device(i2c->def->i2cPort, message->slaveAddress,
                                           message->buffer, message->messageLength,
                                           pdMS_TO_TICKS(I2C_OPERATION_TIMEOUT_MS));
        } else {
            err = i2c_master_read_from_device(i2c->def->i2cPort, message->slaveAddress,
                                            message->buffer, message->messageLength,
                                            pdMS_TO_TICKS(I2C_OPERATION_TIMEOUT_MS));
        }

        if (err == ESP_OK) {
            message->status = i2cAck;
            break;
        }

        if (attempt < message->nbrOfRetries) {
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
        }
    }

    xSemaphoreGive(i2c->isBusFreeMutex);

    if (err != ESP_OK) {
        ESP_LOGD(TAG, "I2C#%d transfer failed to 0x%02X: %s",
                i2c->def->i2cPort, message->slaveAddress, esp_err_to_name(err));
    }

    return message->status == i2cAck;
}

void i2cdrvCreateMessage(I2cMessage *message,
                         uint8_t slaveAddress,
                         I2cDirection direction,
                         uint32_t length,
                         uint8_t *buffer)
{
    memset(message, 0, sizeof(I2cMessage));
    message->slaveAddress = slaveAddress;
    message->direction = direction;
    message->messageLength = length;
    message->buffer = buffer;
    message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
    message->nbrOfRetries = 1;
    message->status = i2cNack;
}

void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                                uint8_t slaveAddress,
                                bool isInternal16,
                                uint16_t intAddress,
                                I2cDirection direction,
                                uint32_t length,
                                uint8_t *buffer)
{
    i2cdrvCreateMessage(message, slaveAddress, direction, length, buffer);
    message->isInternal16bit = isInternal16;
    message->internalAddress = intAddress;
}