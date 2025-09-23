/**
 * ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 * ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * eskylink.h: Adaptación para ESP32 Wi-Fi del enlace E-Sky.
 */

#ifndef __ESKYLINK_WIFI_H__
#define __ESKYLINK_WIFI_H__

#include "crtp.h"

/**
 * @brief Inicializa el enlace de comunicación Wi-Fi para recibir comandos E-Sky.
 */
void eskylinkInit();

/**
 * @brief Comprueba si el enlace Wi-Fi ha sido inicializado.
 */
bool eskylinkTest();

/**
 * @brief Obtiene el puntero a las operaciones del enlace de comunicación.
 */
struct crtpLinkOperations * eskylinkGetLink();

/**
 * @brief Reinicia el enlace (función de compatibilidad, sin acción).
 */
void eskylinkReInit(void);

#endif // __ESKYLINK_WIFI_H__