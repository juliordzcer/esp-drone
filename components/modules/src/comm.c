/*
 * comm.c - High level communication module for ESP32 with Wi-Fi
 */

#include <stdbool.h>
#include "esp_log.h"
#include "wifilink.h" 
#include "crtp.h"
#include "console.h"
#include "crtpservice.h" // Asegúrate de que este archivo exista.
#include "param.h"
#include "log.h"
#include "comm.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "COMM";

static bool isInit = false;

#define WIFI_CONNECTION_TIMEOUT_MS 15000

void commInit(void)
{
  if (isInit) {
    ESP_LOGW(TAG, "Communication module already initialized.");
    return;
  }

  ESP_LOGI(TAG, "Initializing communication modules...");

  // 1. Esperar a que la conexión Wi-Fi se establezca (o que entre en modo AP).
  // La inicialización de wifilink fue hecha en systemInit().
  ESP_LOGI(TAG, "Waiting for Wi-Fi link ready state (STA or AP)...");
  if (!wifilinkWaitForConnection(pdMS_TO_TICKS(WIFI_CONNECTION_TIMEOUT_MS))) {
      ESP_LOGE(TAG, "❌ Failed to establish Wi-Fi link. Comm init continuing, but link is likely broken.");
      // El commInit continúa para permitir la consola local si fuera necesario, 
      // pero el enlace remoto no funcionará si la espera falla.
  }
  
  // 2. Inicializar el stack del protocolo CRTP.
  ESP_LOGI(TAG, "Debug: Calling crtpInit()...");
  crtpInit();
  ESP_LOGI(TAG, "Debug: crtpInit() done.");

  // 3. Configurar el enlace CRTP para usar las funciones del enlace Wi-Fi.
  ESP_LOGI(TAG, "Debug: Calling crtpSetLink()...");
  crtpSetLink(wifilinkGetLink());
  ESP_LOGI(TAG, "Debug: crtpSetLink() done.");

  // 4. Inicializar otros servicios relacionados con la comunicación.
  ESP_LOGI(TAG, "Debug: Initializing other services...");
  // ----------------------------------------------------
  // 👉 DESCOMENTAR: Asumimos que esta es la intención de tu código.
  crtpserviceInit(); 
  // ----------------------------------------------------
  logInit();
  consoleInit();
  paramInit();
  ESP_LOGI(TAG, "Debug: Other services initialized.");

  isInit = true;
  ESP_LOGI(TAG, "✅ Communication module initialized successfully.");
}

bool commTest(void)
{
  bool pass = isInit;

  // Test the Wi-Fi link
  pass &= wifilinkTest();
  
  // Test the CRTP protocol stack and services
  pass &= crtpTest();
  pass &= crtpserviceTest();
  pass &= consoleTest();
  pass &= paramTest();
  
  return pass;
}