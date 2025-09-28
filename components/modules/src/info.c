#include <string.h>
#include <math.h>
#include <stdbool.h>

/*FreeRtos includes*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "crtp.h"
#include "info.h"
#include "version.h"
#include "pm.h"

// --- ADD THE MISSING ENUMERATION DEFINITIONS HERE ---
typedef enum {
  infoCopterNr = 0x00,
  infoBatteryNr = 0x01,
  infoWarningNr = 0x03
} InfoNbr;

typedef enum {
  infoName = 0x00,
  infoVersion = 0x01,
  infoCpuId = 0x02
} infoId;

typedef enum {
  batteryVoltage = 0x00,
  batteryMin = 0x01,
  batteryMax = 0x02
} batteryId;

typedef enum {
  warningBattery = 0x00
} warningId;
// --- END OF MISSING DEFINITIONS ---

// CPUID access is disabled for the ESP32 platform
// static const unsigned int * CpuId = (unsigned int*)0x1FFFF7E8; 

void infoTask(void *param);

void infoInit() {
    // 1. Inicializar la cola para el puerto 8 (CRTP_PORT_INFO)
    // El puerto 8 es el que estás usando en el script Python (Header: 0x80 -> Port 8, Channel 0)
    crtpInitTaskQueue(CRTP_PORT_INFO); 
    
    // 2. Crear la tarea que recibirá de esa cola
    xTaskCreate(infoTask, "Info",
                configMINIMAL_STACK_SIZE + 1024, NULL, /*priority*/2, NULL);
}

void infoTask(void *param) {
    CRTPPacket p;
    int i;
    static int ctr=0;

    while (true) {
        if (crtpReceivePacketWait(CRTP_PORT_INFO, &p, 1000) == pdTRUE) {
            InfoNbr infoNbr = (p.header >> 4) & 0x0F;

            switch (infoNbr) {
                case infoCopterNr:
                    if (p.data[0] == infoName) {
                        p.data[1] = 0x90;
                        p.data[2] = 0x00;
                        strncpy((char*)&p.data[3], "ESP-Drone", CRTP_MAX_DATA_SIZE - 3);
                        p.size = 3+strlen("ESP-Drone");
                        crtpSendPacket(&p);
                    } else if (p.data[0] == infoVersion) {
                        i=1;
                        strncpy((char*)&p.data[i], V_SLOCAL_REVISION, CRTP_MAX_DATA_SIZE - i);
                        i += strlen(V_SLOCAL_REVISION);
                        if (i<CRTP_MAX_DATA_SIZE) p.data[i++] = ',';
                        strncpy((char*)&p.data[i], V_SREVISION, CRTP_MAX_DATA_SIZE - i);
                        i += strlen(V_SREVISION);
                        if (i<CRTP_MAX_DATA_SIZE) p.data[i++] = ',';
                        strncpy((char*)&p.data[i], V_STAG, CRTP_MAX_DATA_SIZE - i);
                        i += strlen(V_STAG);
                        if (i<CRTP_MAX_DATA_SIZE) p.data[i++] = ',';
                        if (i<CRTP_MAX_DATA_SIZE) p.data[i++] = V_MODIFIED?'M':'C';
                        p.size = (i<CRTP_MAX_DATA_SIZE)?i:CRTP_MAX_DATA_SIZE;
                        crtpSendPacket(&p);
                    } else if (p.data[0] == infoCpuId) {
                        // This part is commented out as it is not compatible with ESP32-S2
                        p.size = 1;
                        crtpSendPacket(&p);
                    }
                    break;
                case infoBatteryNr:
                    if (p.data[0] == batteryVoltage) {
                        float value = pmGetBatteryVoltage();
                        memcpy(&p.data[1], (char*)&value, sizeof(float));
                        p.size = 1 + sizeof(float);
                        crtpSendPacket(&p);
                    } else if (p.data[0] == batteryMax) {
                        float value = pmGetBatteryVoltageMax();
                        memcpy(&p.data[1], (char*)&value, sizeof(float));
                        p.size = 1 + sizeof(float);
                        crtpSendPacket(&p);
                    } else if (p.data[0] == batteryMin) {
                        float value = pmGetBatteryVoltageMin();
                        memcpy(&p.data[1], (char*)&value, sizeof(float));
                        p.size = 1 + sizeof(float);
                        crtpSendPacket(&p);
                    }
                    break;
                default:
                    break;
            }
        }
        if (ctr++>5) {
            ctr=0;
            if (pmGetBatteryVoltageMin() < INFO_BAT_WARNING) {
                float value = pmGetBatteryVoltage();
                p.header = CRTP_HEADER(infoWarningNr, 0);
                p.data[0] = 0;
                memcpy(&p.data[1], (char*)&value, 4);
                p.size = 5;
                crtpSendPacket(&p);
            }
        }
    }
}