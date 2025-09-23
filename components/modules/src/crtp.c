/**
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
 * crtp.c - CrazyRealtimeTransferProtocol stack
 */

#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include "esp_log.h"

/*FreeRtos includes*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "config.h"
#include "crtp.h"
#include "info.h"
#include "cfassert.h"

static bool isInit;

// Funciones nop corregidas
static void nopSetEnable(bool enable) {
    // No hacer nada
}

static int nopSendPacket(CRTPPacket *pk) {
    return -1; // ENETDOWN
}

static int nopReceivePacket(CRTPPacket *pk) {
    return -1; // ENETDOWN
}

static struct crtpLinkOperations nopLink = {
    .setEnable = nopSetEnable,
    .sendPacket = nopSendPacket,
    .receivePacket = nopReceivePacket,
}; 

static struct crtpLinkOperations *link = &nopLink;

static QueueHandle_t  tmpQueue;
static QueueHandle_t  rxQueue;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 20
#define CRTP_RX_QUEUE_SIZE 10

static void crtpTxTask(void *param);
static void crtpRxTask(void *param);

static QueueHandle_t queues[CRTP_NBR_OF_PORTS];
static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];

void crtpInit(void)
{
    if(isInit) {
        return;
    }

    // Inicializar arrays
    for (int i = 0; i < CRTP_NBR_OF_PORTS; i++) {
        queues[i] = NULL;
        callbacks[i] = NULL;
    }

    tmpQueue = xQueueCreate(CRTP_TX_QUEUE_SIZE, sizeof(CRTPPacket));
    if (tmpQueue == NULL) {
        ESP_LOGE("CRTP", "Error creando cola TX");
        return;
    }

    rxQueue = xQueueCreate(CRTP_RX_QUEUE_SIZE, sizeof(CRTPPacket));
    if (rxQueue == NULL) {
        ESP_LOGE("CRTP", "Error creando cola RX");
        vQueueDelete(tmpQueue);
        return;
    }

    /* Start Rx/Tx tasks */
    if (xTaskCreate(crtpTxTask, "CRTP-Tx", configMINIMAL_STACK_SIZE + 1024, 
                   NULL, /*priority*/2, NULL) != pdPASS) {
        ESP_LOGE("CRTP", "Error creando tarea TX");
        vQueueDelete(tmpQueue);
        vQueueDelete(rxQueue);
        return;
    }

    if (xTaskCreate(crtpRxTask, "CRTP-Rx", configMINIMAL_STACK_SIZE + 1024, 
                   NULL, /*priority*/2, NULL) != pdPASS) {
        ESP_LOGE("CRTP", "Error creando tarea RX");
        vQueueDelete(tmpQueue);
        vQueueDelete(rxQueue);
        return;
    }
  
    isInit = true;
    ESP_LOGI("CRTP", "CRTP inicializado correctamente");
}

bool crtpTest(void)
{
    return isInit;
}

void crtpInitTaskQueue(CRTPPort portId)
{
    if (portId >= CRTP_NBR_OF_PORTS) {
        ESP_LOGE("CRTP", "Port ID inválido: %d", portId);
        return;
    }
    
    if (queues[portId] != NULL) {
        ESP_LOGW("CRTP", "Queue para port %d ya existe", portId);
        return;
    }
  
    queues[portId] = xQueueCreate(5, sizeof(CRTPPacket));
    if (queues[portId] == NULL) {
        ESP_LOGE("CRTP", "Error creando queue para port %d", portId);
    }
}

int crtpReceivePacket(CRTPPort portId, CRTPPacket *p)
{
    if (portId >= CRTP_NBR_OF_PORTS || queues[portId] == NULL || p == NULL) {
        return pdFALSE;
    }
    
    return xQueueReceive(queues[portId], p, 0);
}

int crtpReceivePacketBlock(CRTPPort portId, CRTPPacket *p)
{
    if (portId >= CRTP_NBR_OF_PORTS || queues[portId] == NULL || p == NULL) {
        return pdFALSE;
    }
  
    return xQueueReceive(queues[portId], p, portMAX_DELAY);
}

int crtpReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait) {
    if (portId >= CRTP_NBR_OF_PORTS || queues[portId] == NULL || p == NULL) {
        return pdFALSE;
    }
  
    return xQueueReceive(queues[portId], p, pdMS_TO_TICKS(wait));
}

void crtpTxTask(void *param)
{
    CRTPPacket p;
    BaseType_t xResult;

    ESP_LOGI("CRTP-TX", "Tarea TX iniciada");

    while (true) {
        xResult = xQueueReceive(tmpQueue, &p, portMAX_DELAY);
        
        if (xResult == pdTRUE) {
            if (link->sendPacket(&p) != 0) {
                ESP_LOGW("CRTP-TX", "Error enviando paquete");
            }
        }
    }
}

void crtpRxTask(void *param)
{
    CRTPPacket p;
    static unsigned int droppedPacket = 0;

    ESP_LOGI("CRTP-RX", "Tarea RX iniciada");

    while (true) {
        if (link->receivePacket(&p) == 0) {
            // Paquete recibido correctamente
            if (p.port < CRTP_NBR_OF_PORTS && queues[p.port] != NULL) {
                if (xQueueSend(queues[p.port], &p, 0) != pdTRUE) {
                    // Cola llena, descartar paquete más antiguo y enviar el nuevo
                    CRTPPacket old_pkt;
                    xQueueReceive(queues[p.port], &old_pkt, 0);
                    xQueueSend(queues[p.port], &p, 0);
                    droppedPacket++;
                    ESP_LOGW("CRTP-RX", "Cola llena, paquete descartado. Total: %u", droppedPacket);
                }
            } else {
                droppedPacket++;
                ESP_LOGW("CRTP-RX", "Port %d no tiene queue. Paquetes descartados: %u", p.port, droppedPacket);
            }
            
            // Llamar callback si está registrado
            if (p.port < CRTP_NBR_OF_PORTS && callbacks[p.port] != NULL) {
                callbacks[p.port](&p);
            }
        } else {
            // No hay paquetes disponibles, esperar un poco
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void crtpRegisterPortCB(int port, CrtpCallback cb)
{
    if (port >= CRTP_NBR_OF_PORTS) {
        ESP_LOGE("CRTP", "Port ID inválido para callback: %d", port);
        return;
    }
  
    callbacks[port] = cb;
    ESP_LOGI("CRTP", "Callback registrado para port %d", port);
}

int crtpSendPacket(CRTPPacket *p)
{
    if (p == NULL || !isInit) {
        return pdFALSE;
    }

    return xQueueSend(tmpQueue, p, 0);
}

int crtpSendPacketBlock(CRTPPacket *p)
{
    if (p == NULL || !isInit) {
        return pdFALSE;
    }

    return xQueueSend(tmpQueue, p, portMAX_DELAY);
}

void crtpSetLink(struct crtpLinkOperations * lk)
{
    if (link != NULL) {
        link->setEnable(false);
    }

    if (lk != NULL) {
        link = lk;
        ESP_LOGI("CRTP", "Nuevo enlace CRTP configurado");
    } else {
        link = &nopLink;
        ESP_LOGW("CRTP", "Enlace CRTP configurado como nop");
    }

    link->setEnable(true);
}