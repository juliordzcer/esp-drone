/**
 * eskylink.c: Adaptación para ESP32 Wi-Fi del enlace E-Sky.
 */
#include "eskylink.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// --- Configuración de Red ---
#define WIFI_SSID      "ESP32_Esky_Link"
#define WIFI_PASSWORD  "dronecontrol"
#define UDP_PORT       1999 // Puerto para la comunicación

// --- Constantes del Protocolo E-Sky (conservadas del original) ---
#define PPM_ZERO 1500
#define PPM_RANGE 500
#define PPM_MIN 1000
#define PPM_MAX 2000
#define ESKY_PACKET_SIZE 13 // Tamaño esperado del paquete de datos

static const char *TAG = "ESKYLINK_WIFI";

// --- Variables estáticas ---
static bool isInit = false;
static QueueHandle_t rxQueue; // Cola para paquetes CRTP ya decodificados
static int sock = -1;

// --- Prototipos de funciones internas ---
static void eskylinkTask(void *param);
static int sendPacket(CRTPPacket *pk);
static int receivePacket(CRTPPacket *pk);
static void eskylinkDecode(uint8_t* packet);

// --- La estructura de operaciones que define la API del enlace ---
static struct crtpLinkOperations eskyOp = {
  .sendPacket    = sendPacket,
  .receivePacket = receivePacket,
};

// --- Lógica de Decodificación (conservada del driver original) ---
static void eskylinkDecode(uint8_t* packet) {
    CRTPPacket crtpPacket;
    float pitch, roll, yaw;
    uint16_t thrust;

    // El paquete de 13 bytes contiene 6 canales de 16 bits y un byte de checksum
    uint16_t channels[6];
    memcpy(channels, packet, 12);

    // Mapeo de canales a funciones (puede variar según el control remoto)
    roll   = channels[0];
    pitch  = channels[1];
    thrust = channels[2];
    yaw    = channels[3];

    // Escalar los valores a los rangos esperados por el firmware
    roll   = (roll - PPM_ZERO) * 20.0f / PPM_RANGE;
    pitch  = (pitch - PPM_ZERO) * 20.0f / PPM_RANGE;
    yaw    = (yaw - PPM_ZERO) * 200.0f / PPM_RANGE;
    thrust = (thrust < PPM_MIN) ? 0 : (thrust - PPM_MIN);
    thrust = (thrust > (2 * PPM_RANGE)) ? (2 * PPM_RANGE) : thrust;
    thrust = (uint16_t)((float)thrust * 65535.0f / (2.0f * PPM_RANGE));
    
    // Construir el paquete CRTP de tipo "setpoint"
    crtpPacket.header = CRTP_HEADER(CRTP_PORT_COMMANDER, 0);
    crtpPacket.size = sizeof(roll) + sizeof(pitch) + sizeof(yaw) + sizeof(thrust);
    memcpy(&crtpPacket.data[0], &roll, sizeof(roll));
    memcpy(&crtpPacket.data[4], &pitch, sizeof(pitch));
    memcpy(&crtpPacket.data[8], &yaw, sizeof(yaw));
    memcpy(&crtpPacket.data[12], &thrust, sizeof(thrust));

    // Enviar el paquete procesado a la cola para la aplicación principal
    if (xQueueSend(rxQueue, &crtpPacket, 0) != pdPASS) {
        ESP_LOGW(TAG, "La cola de recepción está llena (rxQueue).");
    }
}

// --- Tarea principal para recibir paquetes UDP ---
static void eskylinkTask(void *arg) {
    uint8_t rx_buffer[64];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    ESP_LOGI(TAG, "Tarea de recepción E-Sky Wi-Fi iniciada.");

    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);

        if (len == ESKY_PACKET_SIZE) {
            // Si el paquete tiene el tamaño correcto, lo decodificamos
            eskylinkDecode(rx_buffer);
        } else if (len > 0) {
            ESP_LOGW(TAG, "Paquete recibido con tamaño incorrecto: %d bytes (esperado: %d)", len, ESKY_PACKET_SIZE);
        }
    }
}

// --- Implementación de las funciones de la API del enlace ---
static int sendPacket(CRTPPacket *pk) {
    // El enlace E-Sky es solo de recepción, por lo que esta función no hace nada.
    return 0;
}

static int receivePacket(CRTPPacket *pk) {
    // La aplicación llama a esta función para obtener los comandos ya decodificados.
    if (xQueueReceive(rxQueue, pk, portMAX_DELAY) == pdTRUE) {
        return 0;
    }
    return -1;
}

// --- Funciones públicas ---
void eskylinkInit() {
    if (isInit) return;
    ESP_LOGI(TAG, "Inicializando enlace E-Sky sobre Wi-Fi...");

    // 1. Inicializar Wi-Fi y Sockets (el código es idéntico al de las otras adaptaciones)
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = { .ap = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD, .authmode = WIFI_AUTH_WPA2_PSK, .max_connection = 1 }};
    strcpy((char*)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.ap.password, WIFI_PASSWORD);
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    esp_wifi_start();

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    // 2. Crear la cola y la tarea de recepción
    rxQueue = xQueueCreate(5, sizeof(CRTPPacket));
    xTaskCreate(eskylinkTask, "eskylinkWifiTask", 4096, NULL, 5, NULL);

    isInit = true;
    ESP_LOGI(TAG, "Enlace Wi-Fi listo. Conéctate a la red '%s'", WIFI_SSID);
}

bool eskylinkTest() {
    return isInit;
}

struct crtpLinkOperations* eskylinkGetLink() {
    return &eskyOp;
}

void eskylinkReInit(void) {
    // Función de compatibilidad, no se requiere acción.
}