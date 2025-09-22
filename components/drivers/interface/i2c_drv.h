#ifndef __I2C_DRV_H__
#define __I2C_DRV_H__

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_err.h"

// Define la estructura para la configuración de un bus I2C
typedef struct {
    i2c_port_t      i2cPort;
    uint32_t        i2cClockSpeed;
    gpio_num_t      gpioSCLPin;
    gpio_num_t      gpioSDAPin;
    gpio_pullup_t   gpioPullup;
} I2cDef;

// Define la estructura para el driver de un bus I2C
typedef struct {
    const I2cDef    *def;
    SemaphoreHandle_t isBusFreeMutex;
    bool            is_initialized;
} I2cDrv;

// =======================================================================
// ESTA ES LA LÍNEA QUE FALTABA Y SOLUCIONA EL ERROR
// Se crea un alias llamado 'I2C_Dev' que es igual a 'I2cDrv'.
typedef I2cDrv    I2C_Dev;
// =======================================================================

// Instancias globales de los buses
extern I2cDrv sensorsBus;
extern I2cDrv deckBus;

// Prototipo de la función de inicialización
esp_err_t i2cdrvInit(I2cDrv *i2c);

#endif // __I2C_DRV_H__
