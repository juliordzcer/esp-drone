#ifndef I2C_DRV_H
#define I2C_DRV_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

#define I2C_NO_INTERNAL_ADDRESS   0xFFFF

typedef enum {
    i2cAck,
    i2cNack
} I2cStatus;

typedef enum {
    i2cWrite,
    i2cRead
} I2cDirection;

typedef struct {
    uint32_t        messageLength;
    uint8_t         slaveAddress;
    uint8_t         nbrOfRetries;
    I2cDirection    direction;
    I2cStatus       status;
    xQueueHandle    clientQueue;
    bool            isInternal16bit;
    uint16_t        internalAddress;
    uint8_t         *buffer;
} I2cMessage;

typedef struct {
    i2c_port_t      i2cPort;
    uint32_t        i2cClockSpeed;
    gpio_num_t      gpioSCLPin;
    gpio_num_t      gpioSDAPin;
    gpio_pullup_t   gpioPullup;
} I2cDef;

typedef struct {
    const I2cDef    *def;
    SemaphoreHandle_t isBusFreeMutex;
    bool            is_initialized;
} I2cDrv;

extern I2cDrv deckBus;
extern I2cDrv sensorsBus;

esp_err_t i2cdrvInit(I2cDrv *i2c);
bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message);
void i2cdrvCreateMessage(I2cMessage *message,
                         uint8_t slaveAddress,
                         I2cDirection direction,
                         uint32_t length,
                         uint8_t *buffer);
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                                uint8_t slaveAddress,
                                bool isInternal16,
                                uint16_t intAddress,
                                I2cDirection direction,
                                uint32_t length,
                                uint8_t *buffer);

#endif