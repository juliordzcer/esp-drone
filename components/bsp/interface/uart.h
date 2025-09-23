/**
 * ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 * ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * uart.h - Adaptación para ESP32 Wi-Fi del enlace CRTP.
 * Proporciona la misma API que el driver UART original.
 */
#ifndef UART_H_
#define UART_H_

#include <stdbool.h>
#include <stdint.h>
#include "crtp.h"

/**
 * @brief Inicializa el enlace de comunicación Wi-Fi.
 * Crea un Access Point y una tarea para manejar los paquetes UDP.
 */
void uartInit(void);

/**
 * @brief Comprueba si el enlace Wi-Fi ha sido inicializado.
 * @return true si está inicializado.
 */
bool uartTest(void);

/**
 * @brief Obtiene el puntero a las operaciones del enlace de comunicación CRTP.
 * @return Dirección de la estructura crtpLinkOperations.
 */
struct crtpLinkOperations* uartGetLink(void);

// --- Funciones de compatibilidad (Stubs) ---
// Estas funciones se mantienen para compatibilidad pero no son recomendadas
// para un enlace Wi-Fi basado en paquetes.

/**
 * @brief [STUB] Envía datos crudos. No implementado en el enlace Wi-Fi.
 */
void uartSendData(uint32_t size, uint8_t* data);

/**
 * @brief [STUB] Envía un solo carácter. No implementado en el enlace Wi-Fi.
 */
int uartPutchar(int ch);

/**
 * @brief [STUB] Envía datos crudos usando DMA. No implementado en el enlace Wi-Fi.
 */
void uartSendDataDma(uint32_t size, uint8_t* data);

#endif /* UART_H_ */