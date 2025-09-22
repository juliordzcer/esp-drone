#include "configblock.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "CONFIGBLOCK";

static config_block_t config_data;
static bool config_initialized = false;
static bool config_valid = false;

static uint8_t calculate_cksum(const void* data, size_t len) {
    const uint8_t* c = data;
    uint8_t cksum = 0;
    
    for (size_t i = 0; i < len - 1; i++) { // Excluir el checksum
        cksum += c[i];
    }
    
    return cksum;
}

int configblockInit(void) {
    if (config_initialized) {
        return 0;
    }

    // Valores por defecto
    config_data.magic = CONFIG_BLOCK_MAGIC;
    config_data.version = CONFIG_BLOCK_VERSION;
    config_data.calibPitch = 0.0f;
    config_data.calibRoll = 0.0f;
    config_data.cksum = calculate_cksum(&config_data, sizeof(config_data));
    
    config_valid = true;
    config_initialized = true;
    
    ESP_LOGI(TAG, "Config block inicializado con valores por defecto");
    ESP_LOGI(TAG, "Pitch: %.2f°, Roll: %.2f°", config_data.calibPitch, config_data.calibRoll);
    
    return 0;
}

float configblockGetCalibPitch(void) {
    if (!config_initialized) {
        configblockInit();
    }
    return config_data.calibPitch;
}

float configblockGetCalibRoll(void) {
    if (!config_initialized) {
        configblockInit();
    }
    return config_data.calibRoll;
}

bool configblockSetCalibData(float pitch, float roll) {
    if (!config_initialized) {
        configblockInit();
    }
    
    config_data.calibPitch = pitch;
    config_data.calibRoll = roll;
    config_data.cksum = calculate_cksum(&config_data, sizeof(config_data));
    
    config_valid = true;
    ESP_LOGI(TAG, "Datos de calibración actualizados: Pitch: %.2f°, Roll: %.2f°", pitch, roll);
    
    return true;
}

bool configblockIsValid(void) {
    if (!config_initialized) {
        configblockInit();
    }
    return config_valid;
}