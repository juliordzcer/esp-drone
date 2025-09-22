#ifndef I2C_DEV_H
#define I2C_DEV_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "freertos/semphr.h"

// --- CAPA DE COMPATIBILIDAD para drivers antiguos ---
#ifndef TRUE
#define TRUE true
#endif
#ifndef FALSE
#define FALSE false
#endif
// ----------------------------------------------------

#define I2CDEV_NO_MEM_ADDR 0xFF 

typedef struct {
    const i2c_port_t      i2cPort;
    const uint32_t        i2cClockSpeed;
    const int             gpioSCLPin;
    const int             gpioSDAPin;
    const bool            gpioPullup;
} I2cDef;

typedef struct {
    const I2cDef* def;
    SemaphoreHandle_t mutex;
    bool            is_initialized;
} I2cDev;

// --- API de Control del Bus ---
// CAMBIO CLAVE: El primer argumento ahora es 'void* i2c_bus_handle' para aceptar cualquier puntero
// como los drivers originales esperan.

bool i2cdevInit(void* i2c_bus_handle);
bool i2cdevResetBus(void* i2c_bus_handle);

// --- API Principal de Lectura/Escritura ---

bool i2cdevRead(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);
bool i2cdevWrite(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);

// --- Funciones de Conveniencia (Wrappers) ---

bool i2cdevReadByte(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t *data);
bool i2cdevReadBit(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data);
bool i2cdevReadBits(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t *data);

bool i2cdevWriteByte(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t data);
bool i2cdevWriteBit(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data);
bool i2cdevWriteBits(void* i2c_bus_handle, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

#endif // I2C_DEV_H