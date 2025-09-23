/**
 * uart.c - Adaptación para ESP32 Wi-Fi del enlace CRTP.
 */
#include "uart.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// --- Configuración de Red ---
#define WIFI_SSID      "INFINITUM0361"
#define WIFI_PASSWORD  "cx4WSwVdRV"
#define UDP_PORT       1998 // Puerto de comunicación

static const char *TAG = "UART_WIFI_LINK";

// --- Variables estáticas del módulo ---
static bool isInit = false;
static QueueHandle_t packetDelivery; // Cola para paquetes entrantes (de Wi-Fi a la app)
static int sock = -1;
static struct sockaddr_in last_client_addr;
static bool has_client_addr = false;

// --- Prototipos de funciones internas ---
static void uartRxTask(void *param);
static int uartSendCRTPPacket(CRTPPacket *p);
static int uartReceiveCRTPPacket(CRTPPacket *p);

// --- La estructura de operaciones que define la API del enlace ---
static struct crtpLinkOperations uartOp = {
  .sendPacket    = uartSendCRTPPacket,
  .receivePacket = uartReceiveCRTPPacket,
};


// --- Tarea principal para recibir paquetes UDP ---
void uartRxTask(void *param) {
    uint8_t rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            // Guardamos la dirección del cliente para poder responderle
            memcpy(&last_client_addr, &source_addr, sizeof(last_client_addr));
            has_client_addr = true;

            // Empaquetamos los datos en una CRTPPacket y los enviamos a la cola de la aplicación
            if (len <= (CRTP_MAX_DATA_SIZE + 1)) {
                CRTPPacket packet;
                packet.size = len - 1;
                memcpy(&packet.raw, rx_buffer, len);

                if (xQueueSend(packetDelivery, &packet, pdMS_TO_TICKS(10)) != pdPASS) {
                    ESP_LOGW(TAG, "La cola de paquetes de entrada (packetDelivery) está llena.");
                }
            }
        }
    }
}

// --- Implementación de las funciones CRTP ---
static int uartSendCRTPPacket(CRTPPacket *p) {
    if (!isInit || sock < 0 || !has_client_addr) {
        return -1;
    }
    // El paquete UDP se envía con el byte de cabecera (size + 1)
    return sendto(sock, p->raw, p->size + 1, 0, (struct sockaddr *)&last_client_addr, sizeof(last_client_addr));
}

static int uartReceiveCRTPPacket(CRTPPacket *p) {
    if (xQueueReceive(packetDelivery, p, portMAX_DELAY) == pdTRUE) {
        return 0; // Paquete recibido con éxito
    }
    return -1; // Error
}

// --- Implementación de la API Pública ---
void uartInit(void) {
    if (isInit) return;
    ESP_LOGI(TAG, "Inicializando enlace UART sobre Wi-Fi...");

    // 1. Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Configurar Wi-Fi en modo Access Point
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = { .ap = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD, .authmode = WIFI_AUTH_WPA2_PSK, .max_connection = 1 }};
    strcpy((char*)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.ap.password, WIFI_PASSWORD);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 3. Crear Socket UDP
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE(TAG, "Fallo al enlazar el socket UDP.");
        return;
    }

    // 4. Crear la cola y la tarea de recepción
    packetDelivery = xQueueCreate(5, sizeof(CRTPPacket));
    xTaskCreate(uartRxTask, "uartRxWifiTask", 4096, NULL, 5, NULL);

    isInit = true;
    ESP_LOGI(TAG, "Enlace Wi-Fi listo. Conéctate a la red '%s'", WIFI_SSID);
}

bool uartTest(void) {
    return isInit;
}

struct crtpLinkOperations * uartGetLink() {
    return &uartOp;
}

// --- Implementación de las Funciones "Stub" ---
void uartSendData(uint32_t size, uint8_t* data) {
    ESP_LOGW(TAG, "uartSendData() no es soportado en el enlace Wi-Fi. Use el sistema CRTP.");
}

int uartPutchar(int ch) {
    ESP_LOGW(TAG, "uartPutchar() no es soportado en el enlace Wi-Fi.");
    return ch;
}

void uartSendDataDma(uint32_t size, uint8_t* data) {
    ESP_LOGW(TAG, "uartSendDataDma() no es soportado en el enlace Wi-Fi.");
}