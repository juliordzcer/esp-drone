/*
 * ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 * ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * comm.c - High level communication module for ESP32 with Wi-Fi
 */

#include <stdbool.h>
#include "esp_log.h"
#include "wifilink.h" // Se incluyó esta línea para acceder a las funciones de Wi-Fi.
#include "crtp.h"
#include "console.h"
#include "crtpservice.h"
#include "param.h"
#include "log.h"
#include "uart.h"
#include "comm.h"
#include "freertos/FreeRTOS.h" // Se incluyó para usar pdMS_TO_TICKS

static const char *TAG = "COMM";

static bool isInit = false;

// Definimos un timeout para la conexión Wi-Fi, 15 segundos es un valor seguro.
#define WIFI_CONNECTION_TIMEOUT_MS 15000

void commInit(void)
{
  if (isInit) {
    ESP_LOGW(TAG, "Communication module already initialized.");
    return;
  }

  ESP_LOGI(TAG, "Initializing communication modules...");

  // 1. Inicializar el enlace Wi-Fi para la comunicación CRTP.
  ESP_LOGI(TAG, "Debug: Calling wifilinkInit()...");
  wifilinkInit();
  ESP_LOGI(TAG, "Debug: wifilinkInit() done.");

  // 2. Esperar a que la conexión Wi-Fi se establezca.
  ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
  if (!wifilinkWaitForConnection(pdMS_TO_TICKS(WIFI_CONNECTION_TIMEOUT_MS))) {
      ESP_LOGE(TAG, "❌ Failed to establish Wi-Fi connection. Aborting communication initialization.");
      return;
  }
  
  // 3. Inicializar el stack del protocolo CRTP (solo si la conexión Wi-Fi fue exitosa).
  ESP_LOGI(TAG, "Debug: Calling crtpInit()...");
  crtpInit();
  ESP_LOGI(TAG, "Debug: crtpInit() done.");

  // 4. Configurar el enlace CRTP para usar las funciones del enlace Wi-Fi.
  ESP_LOGI(TAG, "Debug: Calling crtpSetLink()...");
  crtpSetLink(wifilinkGetLink());
  ESP_LOGI(TAG, "Debug: crtpSetLink() done.");

  // 5. Inicializar otros servicios relacionados con la comunicación.
  ESP_LOGI(TAG, "Debug: Initializing other services...");
  crtpserviceInit();
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