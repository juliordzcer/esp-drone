#ifndef CONFIGBLOCK_H_
#define CONFIGBLOCK_H_

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_BLOCK_MAGIC 0x43427830
#define CONFIG_BLOCK_VERSION 1

typedef struct {
    // Header
    uint32_t magic;
    uint8_t version;
    
    // Datos de calibración
    float calibPitch;
    float calibRoll;
    
    // Checksum
    uint8_t cksum;
} __attribute__((packed)) config_block_t;

int configblockInit(void);
float configblockGetCalibPitch(void);
float configblockGetCalibRoll(void);
bool configblockSetCalibData(float pitch, float roll);
bool configblockIsValid(void);

#endif /* CONFIGBLOCK_H_ */