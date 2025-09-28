#ifndef WIFILINK_H_
#define WIFILINK_H_

#include <stdint.h>
#include <stdbool.h>
#include "crtp.h"
#include "freertos/FreeRTOS.h"

// Estructura para información de conexión
typedef struct {
    char ssid[32];
    char ip[16];
    int port;
} wifilink_info_t;

void wifilinkInit(void);
bool wifilinkTest(void);
struct crtpLinkOperations* wifilinkGetLink(void);
void wifilinkReInit(void);
bool wifilinkIsConnected(void);

/**
 * @brief Espera a que la conexión Wi-Fi se establezca.
 * @param timeout Tiempo máximo en ticks del sistema para esperar.
 * @return Verdadero si la conexión fue exitosa, falso si falló o hubo timeout.
 */
bool wifilinkWaitForConnection(TickType_t timeout);

/**
 * @brief Obtiene información de la conexión actual.
 * @param ssid Buffer para almacenar el SSID (opcional, puede ser NULL)
 * @param ip Buffer para almacenar la IP (opcional, puede ser NULL)
 * @param ip_len Longitud del buffer de IP
 */
void wifilinkGetConnectionInfo(char* ssid, char* ip, int ip_len);

/**
 * @brief Obtiene estadísticas de la conexión.
 * @param packets_received Número de paquetes recibidos (opcional)
 * @param last_packet_time Timestamp del último paquete (opcional)
 */
void wifilinkGetStats(uint32_t* packets_received, uint32_t* last_packet_time);
// En wifilink.h, agrega esta declaración:
int wifilinkSendLogPacket(CRTPPacket *pk);

#endif /* WIFILINK_H_ */