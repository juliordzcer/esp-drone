/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * debug.h - Debugging utility functions
 */

#ifndef DEBUG_ESP32_H_
#define DEBUG_ESP32_H_
#include "console.h"
#include "esp_log.h"

// --- Puente de compatibilidad ---
// Define automáticamente la variable TAG que necesita esp_log,
// usando el DEBUG_MODULE que ya tienes definido en tu código.
// #ifdef DEBUG_MODULE
//   static const char *TAG = DEBUG_MODULE;
// #else
//   static const char *TAG = "NO_MODULE"; // Un valor por defecto si se te olvida definir DEBUG_MODULE
// #endif
#define TAG DEBUG_MODULE


// --- Definición de las Macros ---

/**
 * @brief Imprime un mensaje de información (en color verde).
 * Equivalente a `ESP_LOGI`. Para mensajes de depuración menos importantes,
 * puedes cambiarlo a `ESP_LOGD` (y ajustar el nivel en menuconfig).
 */
#define DEBUG_PRINT(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)


/**
 * @brief Evalúa una condición y muestra un mensaje de OK (verde) o FALLO (rojo).
 *
 * @param e La condición a evaluar (ej. `status == true`).
 * @param msgOK El mensaje a mostrar si la condición es verdadera.
 * @param msgFail El mensaje a mostrar si la condición es falsa.
 */
#define TEST_AND_PRINT(e, msgOK, msgFail) \
    do { \
        if(e) { ESP_LOGI(TAG, "%s", msgOK); } \
        else { ESP_LOGE(TAG, "%s", msgFail); } \
    } while(0)


/**
 * @brief Imprime un mensaje de error (en color rojo).
 *
 * @param msg El mensaje de error a mostrar.
 */
#define FAIL_PRINT(msg) ESP_LOGE(TAG, "%s", msg)


#endif /* DEBUG_ESP32_H_ */