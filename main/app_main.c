#include <stdio.h>
#include <string.h>  // <-- AGREGAR ESTA LÍNEA
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/event_groups.h" // Se asume que este se necesita para el EventGroup

#include "crtp.h"
#include "wifilink.h"

// Definir los bits del EventGroup si no están en un header global
#define WIFI_CONNECTED_BIT BIT0
extern EventGroupHandle_t wifi_event_group;
// Definir los parámetros de la red para que sean visibles aquí
#define WIFI_SSID      "INFINITUM0361"  // <-- AGREGAR ESTA LÍNEA

static const char *TAG = "APP_MAIN";

// Handler para paquetes CRTP
static void consoleHandler(CRTPPacket *pk) {
    if (pk == NULL) return;
    
    ESP_LOGI("CRTP_RX", "Paquete recibido - Port: %d, Channel: %d, Size: %d", 
             pk->port, pk->channel, pk->size);
    
    if (pk->size > 0) {
        char data_str[CRTP_MAX_DATA_SIZE + 1];
        int copy_size = (pk->size < CRTP_MAX_DATA_SIZE) ? pk->size : CRTP_MAX_DATA_SIZE;
        memcpy(data_str, pk->data, copy_size);
        data_str[copy_size] = '\0';
        
        ESP_LOGI("CRTP_RX", "Datos: %s", data_str);
    }
    
    // Responder
    CRTPPacket response;
    response.port = pk->port;
    response.channel = pk->channel;
    
    const char *ack_msg = "ACK from ESP32 Drone";
    response.size = strlen(ack_msg);
    memcpy(response.data, ack_msg, response.size);
    
    if (crtpSendPacket(&response) == pdTRUE) {
        ESP_LOGI("CRTP_TX", "Respuesta enviada");
    }
}

static void init_crtp_handlers(void) {
    crtpInitTaskQueue(0);
    crtpRegisterPortCB(0, consoleHandler);
    ESP_LOGI(TAG, "Handlers CRTP inicializados");
}

static void monitor_task(void *param) {
    while (1) {
        if (!wifilinkTest()) {
            ESP_LOGW(TAG, "Problema con enlace Wi-Fi. Sistema en espera...");
        } else {
            ESP_LOGI(TAG, "✅ Sistema funcionando correctamente");
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Verificar cada 10 segundos
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Iniciando ESP-Drone Controller");
    ESP_LOGI(TAG, "Modo: Wi-Fi Station (Cliente)");
    ESP_LOGI(TAG, "Conectando a: %s", WIFI_SSID);
    ESP_LOGI(TAG, "==========================================");
    
    // 1. Inicializar Wi-Fi
    ESP_LOGI(TAG, "Inicializando Wi-Fi...");
    wifilinkInit();
    
    // 2. Esperar a que la conexión Wi-Fi esté lista de forma segura
    ESP_LOGI(TAG, "Esperando conexión Wi-Fi de forma segura...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    
    // El programa solo continuará cuando se haya obtenido una IP
    
    // 3. Inicializar CRTP
    ESP_LOGI(TAG, "Inicializando CRTP...");
    crtpInit();
    
    // 4. Configurar enlace CRTP
    ESP_LOGI(TAG, "Configurando enlace CRTP...");
    crtpSetLink(wifilinkGetLink());
    
    // 5. Inicializar handlers
    init_crtp_handlers();
    
    ESP_LOGI(TAG, "✅ Sistema inicializado correctamente");
    ESP_LOGI(TAG, "📍 El drone está listo para recibir comandos CRTP");
    
    // 6. Tarea de monitoreo
    xTaskCreate(monitor_task, "monitor_task", 4096, NULL, 1, NULL);
    
    // Bucle principal (se ejecuta continuamente)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}