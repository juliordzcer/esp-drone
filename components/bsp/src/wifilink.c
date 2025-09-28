/**
 * ESP32 Wi-Fi implementation of the CRTP link.
 */

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "wifilink.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "crtp.h"

// =================================================================
// ============== PARÁMETROS DE CONFIGURACIÓN DE RED ===============
// =================================================================
#define WIFI_SSID      "INFINITUM0361"
#define WIFI_PASSWORD  "cx4WSwVdRV"
#define UDP_PORT       2390 // Puerto para la comunicación CRTP

// Event group para sincronización Wi-Fi
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// --- Tiempos de espera y tamaños de cola ---
#define WIFI_TIMEOUT_MS         10000 // 10 segundos para conectar
#define WIFI_RECONNECT_DELAY_MS 5000  // 5 segundos entre reconexiones
#define INCOMING_QUEUE_SIZE     10   // Capacidad de la cola de paquetes recibidos

// =================================================================

static const char *TAG = "WIFILINK";

// --- Variables estáticas del módulo ---
static bool isInit = false;
static QueueHandle_t rxQueue;
static int sock = -1;
static struct sockaddr_in server_addr;
static bool is_connected = false;
static uint32_t lastPacketTimestamp = 0;
static EventGroupHandle_t wifi_event_group;
static esp_netif_ip_info_t ip_info;
static bool socket_ready = false;

// --- Prototipos de funciones internas ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifilinkTask(void *param);
static int sendPacket(CRTPPacket *pk);
static int receivePacket(CRTPPacket *pk);
static bool isConnected(void);
static void setEnable(bool enable);
static void print_ip_address(void);

// La estructura de operaciones que define la API del enlace
static struct crtpLinkOperations link_ops = {
  .setEnable    = setEnable,
  .sendPacket   = sendPacket,
  .receivePacket = receivePacket,
};

// --- Handler de eventos Wi-Fi ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi STA iniciado, conectando...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi desconectado, intentando reconectar...");
        is_connected = false;
        socket_ready = false;
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_DELAY_MS));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        memcpy(&ip_info, &event->ip_info, sizeof(ip_info));
        ESP_LOGI(TAG, "Conectado! IP obtenida: " IPSTR, IP2STR(&ip_info.ip));
        is_connected = true;
        lastPacketTimestamp = xTaskGetTickCount(); // Inicializar timestamp
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        print_ip_address();
    }
}

// --- Función para imprimir la IP asignada ---
static void print_ip_address(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "✅ WI-FI CONECTADO EXITOSAMENTE");
    ESP_LOGI(TAG, "📶 SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "🌐 IP del Drone: " IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "📡 Gateway: " IPSTR, IP2STR(&ip_info.gw));
    ESP_LOGI(TAG, "🔌 Máscara: " IPSTR, IP2STR(&ip_info.netmask));
    ESP_LOGI(TAG, "🎯 Puerto UDP: %d", UDP_PORT);
    ESP_LOGI(TAG, "==========================================");
}

// --- Función setEnable ---
static void setEnable(bool enable) {
    ESP_LOGI(TAG, "Wi-Fi link %s", enable ? "enabled" : "disabled");
}

// --- Tarea principal para recibir datos por Wi-Fi ---
static void wifilinkTask(void *param) {
    uint8_t rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    ESP_LOGI(TAG, "Tarea de recepción Wi-Fi iniciada.");

    // Esperar a que Wi-Fi esté conectado
    ESP_LOGI(TAG, "Esperando conexión Wi-Fi...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, 
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, 
        pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi conectado, iniciando socket UDP...");
        
        // Configurar dirección del servidor (en este caso somos servidor UDP)
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(UDP_PORT);
        
        // Crear socket UDP
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "No se pudo crear el socket: errno %d", errno);
            return;
        }
        
        // Configurar socket para no bloquear
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
        
        // Configurar opciones del socket
        int enable = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
        
        // Enlazar socket
        if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            ESP_LOGE(TAG, "Fallo al enlazar el socket: errno %d", errno);
            close(sock);
            sock = -1;
            return;
        }
        
        socket_ready = true;
        ESP_LOGI(TAG, "Socket UDP creado y enlazado en puerto %d", UDP_PORT);
        ESP_LOGI(TAG, "✅ Listo para recibir conexiones CRTP");
        ESP_LOGI(TAG, "📍 Conectate a la misma red y usa IP: " IPSTR ":%d", 
                IP2STR(&ip_info.ip), UDP_PORT);

    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "No se pudo conectar al Wi-Fi");
        return;
    }

    // Bucle principal de recepción
    while (1) {
        if (sock < 0 || !socket_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                          (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            // Actualizar timestamp del último paquete
            lastPacketTimestamp = xTaskGetTickCount();

            ESP_LOGD(TAG, "Paquete UDP recibido de %s:%d, tamaño: %d bytes", 
                     inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port), len);

            // Procesar paquete CRTP
            if (len > 0 && len <= (CRTP_MAX_DATA_SIZE + 1)) {
                CRTPPacket packet;
                packet.size = len - 1;
                packet.port = (rx_buffer[0] >> 4) & 0x0F;
                packet.channel = rx_buffer[0] & 0x0F;
                
                if (packet.size > 0) {
                    memcpy(packet.data, rx_buffer + 1, packet.size);
                }

                // Guardar dirección del cliente para responder
                memcpy(&server_addr, &source_addr, sizeof(source_addr));

                if (xQueueSend(rxQueue, &packet, pdMS_TO_TICKS(10)) != pdPASS) {
                    ESP_LOGW(TAG, "La cola de recepción está llena. Paquete descartado.");
                }
            }
        } else if (len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "Error en recvfrom: errno %d", errno);
                // Reintentar después de un error
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        // Pequeña pausa para no saturar la CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- Implementación de las funciones de la API del enlace ---
static int sendPacket(CRTPPacket *pk) {
    if (!isInit || sock < 0 || !socket_ready || pk == NULL) {
        return -1;
    }

    if (pk->size > CRTP_MAX_DATA_SIZE) {
        return -2;
    }

    // Preparar buffer con cabecera CRTP
    uint8_t tx_buffer[CRTP_MAX_DATA_SIZE + 1];
    tx_buffer[0] = (pk->port << 4) | (pk->channel & 0x0F);
    if (pk->size > 0) {
        memcpy(tx_buffer + 1, pk->data, pk->size);
    }

    int err = sendto(sock, tx_buffer, pk->size + 1, 0, 
                    (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error al enviar paquete UDP: errno %d", errno);
        return -1;
    }
    
    ESP_LOGD(TAG, "Paquete enviado: port=%d, channel=%d, size=%d", 
             pk->port, pk->channel, pk->size);
    return 0;
}

static int receivePacket(CRTPPacket *pk) {
    if (pk == NULL) {
        return -1;
    }
    
    if (xQueueReceive(rxQueue, pk, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGD(TAG, "Paquete recibido de la cola: port=%d, channel=%d, size=%d", 
                 pk->port, pk->channel, pk->size);
        return 0;
    }
    return -1;
}

static bool isConnected(void) {
    return isInit && is_connected && socket_ready;
}

// --- Función pública para verificar conexión ---
bool wifilinkIsConnected(void)
{
    if (!isInit || !is_connected || !socket_ready) {
        ESP_LOGD(TAG, "Conexión no lista: isInit=%d, is_connected=%d, socket_ready=%d", 
                 isInit, is_connected, socket_ready);
        return false;
    }
    
    // Para debug, mostrar estado detallado periódicamente
    static uint32_t lastDebugTime = 0;
    uint32_t currentTime = xTaskGetTickCount();
    if (currentTime - lastDebugTime > pdMS_TO_TICKS(5000)) {
        ESP_LOGI(TAG, "✅ Estado Wi-Fi: Conectado, Socket listo");
        lastDebugTime = currentTime;
    }
    
    return true;
}

// --- Funciones públicas de inicialización ---
void wifilinkInit() {
    if (isInit) {
        ESP_LOGW(TAG, "Wi-Fi link ya inicializado");
        return;
    }

    ESP_LOGI(TAG, "Inicializando enlace Wi-Fi (modo Station)...");

    // Inicializar variables
    is_connected = false;
    socket_ready = false;
    lastPacketTimestamp = xTaskGetTickCount();

    // 1. Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "NVS flash corrupto, borrando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Inicializar TCP/IP y Event Loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 3. Crear event group
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Error creando event group");
        return;
    }
    
    // 4. Configurar interfaz de red en modo Station
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // 5. Configurar Wi-Fi en modo Station
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 6. Registrar event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // 7. Configurar y conectar
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Desactivar power saving para mejor rendimiento
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi Station iniciado, conectando a: %s", WIFI_SSID);

    // 8. Crear cola de recepción
    rxQueue = xQueueCreate(INCOMING_QUEUE_SIZE, sizeof(CRTPPacket));
    if (rxQueue == NULL) {
        ESP_LOGE(TAG, "Error creando la cola de recepción");
        vEventGroupDelete(wifi_event_group);
        return;
    }

    // 9. Crear tarea de recepción
    if (xTaskCreate(wifilinkTask, "wifilinkTask", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Error creando la tarea Wi-Fi");
        vQueueDelete(rxQueue);
        vEventGroupDelete(wifi_event_group);
        return;
    }

    isInit = true;
    ESP_LOGI(TAG, "Wi-Fi link inicializado correctamente");
}

bool wifilinkTest() {
    bool result = isInit && is_connected && socket_ready;
    ESP_LOGI(TAG, "Test Wi-Fi: %s", result ? "PASS" : "FAIL");
    return result;
}

struct crtpLinkOperations* wifilinkGetLink() {
    return &link_ops;
}

void wifilinkReInit(void) {
    ESP_LOGI(TAG, "Reinicializando enlace Wi-Fi...");
    
    if (sock >= 0) {
        close(sock);
        sock = -1;
    }
    
    is_connected = false;
    socket_ready = false;
    isInit = false;
    
    // Limpiar event group
    if (wifi_event_group) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    wifilinkInit();
}

/**
 * @brief Espera a que la conexión Wi-Fi se establezca.
 * @param timeout Tiempo máximo en ticks del sistema para esperar.
 * @return Verdadero si la conexión fue exitosa, falso si falló o hubo timeout.
 */
bool wifilinkWaitForConnection(TickType_t timeout) {
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "El EventGroup de Wi-Fi no ha sido creado.");
        return false;
    }

    ESP_LOGI(TAG, "Esperando conexión Wi-Fi (timeout: %d ms)...", (int)pdTICKS_TO_MS(timeout));
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, // No limpiar bits al salir
                                           pdFALSE, // Esperar al menos uno de los bits
                                           timeout);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Conexión Wi-Fi exitosa.");
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ Fallo en la conexión Wi-Fi.");
        return false;
    } else {
        ESP_LOGE(TAG, "❌ Timeout en la conexión Wi-Fi.");
        return false;
    }
}

// --- Función para obtener información de conexión ---
void wifilinkGetConnectionInfo(char* ssid, char* ip, int ip_len) {
    if (ssid) {
        strncpy(ssid, WIFI_SSID, 32);
    }
    if (ip && is_connected) {
        snprintf(ip, ip_len, IPSTR, IP2STR(&ip_info.ip));
    }
}

// --- Función para obtener estadísticas ---
void wifilinkGetStats(uint32_t* packets_received, uint32_t* last_packet_time) {
    if (packets_received) {
        // Esta implementación necesitaría llevar un contador de paquetes
        *packets_received = 0; // Placeholder
    }
    if (last_packet_time) {
        *last_packet_time = lastPacketTimestamp;
    }
}

// En wifilink.c - función para enviar paquetes de log
int wifilinkSendLogPacket(CRTPPacket *pk) {
    if (!isInit || sock < 0 || !socket_ready || pk == NULL) {
        return -1;
    }

    if (pk->size > CRTP_MAX_DATA_SIZE) {
        return -2;
    }

    // Preparar buffer con cabecera CRTP para LOG (puerto 5)
    uint8_t tx_buffer[CRTP_MAX_DATA_SIZE + 1];
    tx_buffer[0] = (5 << 4) | (pk->channel & 0x0F); // Puerto LOG = 5
    if (pk->size > 0) {
        memcpy(tx_buffer + 1, pk->data, pk->size);
    }

    // Enviar a la misma dirección de donde vinieron los comandos
    int err = sendto(sock, tx_buffer, pk->size + 1, 0, 
                    (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error al enviar paquete LOG: errno %d", errno);
        return -1;
    }
    
    ESP_LOGD(TAG, "Paquete LOG enviado: channel=%d, size=%d", pk->channel, pk->size);
    return 0;
}