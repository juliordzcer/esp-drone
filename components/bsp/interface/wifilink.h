#ifndef WIFILINK_H_
#define WIFILINK_H_

#include <stdint.h>
#include <stdbool.h>
#include "crtp.h"
#include "freertos/FreeRTOS.h"

void wifilinkInit(void);
bool wifilinkTest(void);
struct crtpLinkOperations* wifilinkGetLink(void);
void wifilinkReInit(void);

/**
 * @brief Espera a que la conexión Wi-Fi se establezca.
 * @param timeout Tiempo máximo en ticks del sistema para esperar.
 * @return Verdadero si la conexión fue exitosa, falso si falló o hubo timeout.
 */
bool wifilinkWaitForConnection(TickType_t timeout);

#endif /* WIFILINK_H_ */