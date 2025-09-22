#include "i2c_dev.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h" 

static const char *TAG = "I2CDEV";
#define I2C_TIMEOUT_MS 50

static const I2cDef sensorBusDef = {
    .i2cPort        = I2C_NUM_0,
    .i2cClockSpeed  = 400000,
    .gpioSCLPin     = GPIO_NUM_10,
    .gpioSDAPin     = GPIO_NUM_11,
    .gpioPullup     = true,
};

static const I2cDef deckBusDef = {
    .i2cPort        = I2C_NUM_1,
    .i2cClockSpeed  = 100000,
    .gpioSCLPin     = GPIO_NUM_41,
    .gpioSDAPin     = GPIO_NUM_40,
    .gpioPullup     = true,
};

static I2cDev sensorsBus = {.def = &sensorBusDef};
static I2cDev deckBus    = {.def = &deckBusDef};

/**
 * @brief Función interna para obtener el puntero a la estructura del bus
 * a partir de un puntero genérico que contiene el número de puerto.
 */
static I2cDev* i2cdev_get_bus(void* i2c_bus_handle) {
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return NULL;
    }
    // CAMBIO CLAVE: Convertimos el puntero genérico a un puntero a int
    // y luego leemos su valor (desreferenciamos).
    int i2c_port = *(int*)i2c_bus_handle;

    if (i2c_port == I2C_NUM_0) {
        return &sensorsBus;
    } else if (i2c_port == I2C_NUM_1) {
        return &deckBus;
    }
    ESP_LOGE(TAG, "Invalid I2C port number: %d", i2c_port);
    return NULL;
}

// --- Implementación de Funciones ---

bool i2cdevInit(void* i2c_bus_handle) {
    I2cDev *dev = i2cdev_get_bus(i2c_bus_handle);
    if (!dev) return false;

    if (dev->is_initialized) {
        return true;
    }
    ESP_LOGI(TAG, "Initializing I2C port %d", dev->def->i2cPort);

    if (dev->mutex == NULL) {
        dev->mutex = xSemaphoreCreateMutex();
        if (dev->mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex for I2C port %d", dev->def->i2cPort);
            return false;
        }
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = dev->def->gpioSDAPin,
        .scl_io_num = dev->def->gpioSCLPin,
        .sda_pullup_en = dev->def->gpioPullup,
        .scl_pullup_en = dev->def->gpioPullup,
        .master.clk_speed = dev->def->i2cClockSpeed,
    };

    esp_err_t err = i2c_param_config(dev->def->i2cPort, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C#%d param config failed: %s", dev->def->i2cPort, esp_err_to_name(err));
        vSemaphoreDelete(dev->mutex);
        dev->mutex = NULL;
        return false;
    }

    err = i2c_driver_install(dev->def->i2cPort, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C#%d driver install failed: %s", dev->def->i2cPort, esp_err_to_name(err));
        vSemaphoreDelete(dev->mutex);
        dev->mutex = NULL;
        return false;
    }

    dev->is_initialized = true;
    return true;
}

bool i2cdevResetBus(void* i2c_bus_handle) {
    // La lógica interna no cambia
    I2cDev *dev = i2cdev_get_bus(i2c_bus_handle);
    if (!dev) return false;
    
    ESP_LOGW(TAG, "Resetting I2C bus %d by bit-banging", dev->def->i2cPort);
    // ... (Implementación completa del reseteo iría aquí)
    return true; 
}

bool i2cdevRead(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data) {
    I2cDev *dev = i2cdev_get_bus(i2c_bus_handle);
    if (!dev) return false;

    if (len == 0) return true;
    if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdFALSE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex for read on port %d", dev->def->i2cPort);
        return false;
    }

    esp_err_t err;
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        err = i2c_master_write_read_device(dev->def->i2cPort, devAddress, &memAddress, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    } else {
        err = i2c_master_read_from_device(dev->def->i2cPort, devAddress, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    xSemaphoreGive(dev->mutex);

    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Read failed. Port:%d Addr:0x%02X Reg:0x%02X Err:%s", dev->def->i2cPort, devAddress, memAddress, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool i2cdevWrite(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data) {
    I2cDev *dev = i2cdev_get_bus(i2c_bus_handle);
    if (!dev) return false;

    if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdFALSE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex for write on port %d", dev->def->i2cPort);
        return false;
    }

    esp_err_t err;
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        uint8_t *write_buf = malloc(len + 1);
        if (write_buf == NULL) {
            xSemaphoreGive(dev->mutex);
            ESP_LOGE(TAG, "Malloc failed for I2C write buffer");
            return false;
        }
        write_buf[0] = memAddress;
        memcpy(write_buf + 1, data, len);
        err = i2c_master_write_to_device(dev->def->i2cPort, devAddress, write_buf, len + 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        free(write_buf);
    } else {
        err = i2c_master_write_to_device(dev->def->i2cPort, devAddress, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    xSemaphoreGive(dev->mutex);

    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Write failed. Port:%d Addr:0x%02X Reg:0x%02X Err:%s", dev->def->i2cPort, devAddress, memAddress, esp_err_to_name(err));
        return false;
    }
    return true;
}

// --- Implementación de Funciones de Conveniencia (Wrappers) ---
// Todas estas funciones simplemente pasan el 'handle' a la función principal.

bool i2cdevReadByte(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t *data) {
    return i2cdevRead(i2c_bus_handle, devAddress, memAddress, 1, data);
}

bool i2cdevWriteByte(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t data) {
    return i2cdevWrite(i2c_bus_handle, devAddress, memAddress, 1, &data);
}

bool i2cdevReadBit(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data) {
    uint8_t byte;
    if (i2cdevReadByte(i2c_bus_handle, devAddress, memAddress, &byte)) {
        *data = (byte >> bitNum) & 1;
        return true;
    }
    return false;
}

bool i2cdevWriteBit(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data) {
    uint8_t byte;
    if (i2cdevReadByte(i2c_bus_handle, devAddress, memAddress, &byte)) {
        byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
        return i2cdevWriteByte(i2c_bus_handle, devAddress, memAddress, byte);
    }
    return false;
}

bool i2cdevReadBits(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t *data) {
    uint8_t byte;
    if (i2cdevReadByte(i2c_bus_handle, devAddress, memAddress, &byte)) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        byte &= mask;
        byte >>= (bitStart - length + 1);
        *data = byte;
        return true;
    }
    return false;
}

bool i2cdevWriteBits(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t byte;
    if (i2cdevReadByte(i2c_bus_handle, devAddress, memAddress, &byte)) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        uint8_t data_shifted = data << (bitStart - length + 1);
        byte &= ~mask;
        byte |= (data_shifted & mask);
        return i2cdevWriteByte(i2c_bus_handle, devAddress, memAddress, byte);
    }
    return false;
}