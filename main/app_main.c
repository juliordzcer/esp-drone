/**
 * Crazyflie control firmware
 * ... (Resto de la cabecera)
 * app_main.c - Main function for the ESP32 port.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include "esp_log.h" // ¡Añadido! Para mensajes de consola

#include "system.h"
#include "usec_time.h"
#include "led.h"       // ¡Añadido! Incluye el driver de LEDs
// Incluye cualquier otro driver o cabecera de inicialización crítica aquí

static const char *TAG = "CF_MAIN";

// ******************************************************************
// FUNCIÓN DE VERIFICACIÓN CRÍTICA (Reemplaza prvClockInit en caso de fallo)
// ******************************************************************
static bool critical_hardware_check(void)
{
    // Aquí es donde harías verificaciones de hardware vitales:
    // - Comprobación de la IMU/Giroscopio
    // - Estado de la alimentación
    // - Estado del Módulo de Radio (si aplica)
    
    // Si la inicialización de algún componente falla, devuelve 'false'.
    // Por ahora, asumimos éxito para que el sistema arranque.
    return true; 
}


// ******************************************************************
// PUNTO DE ENTRADA PRINCIPAL: app_main
// ******************************************************************
void app_main(void)
{
  // 1. Inicialización de utilidades y diagnóstico (DEBE ser lo primero)
  initUsecTimer(); // Inicializa el contador de microsegundos (la 'dummy' del IDF)
  ledInit();       // Inicializa los pines GPIO para los LEDs
  

  // 2. LÓGICA DE VERIFICACIÓN Y FALLA CRÍTICA ("LED de la Muerte")
  if (!critical_hardware_check()) {
      ESP_LOGE(TAG, "FALLA CRITICA: Error en la inicializacion de hardware.");
      
      // Enciende los LEDs Rojo y Verde para indicar el fallo
      ledSet(LED_RED, true);
      ledSet(LED_GREEN, true);
      
      // Bloquea el sistema (equivalente al while(1) del STM32)
      while(1) {
          // El ESP-IDF tiene un Watchdog Timer (WDT) que reiniciaría el chip.
          // Este delay (o un pinza a un tiempo más rápido) evita un bucle de reinicio infinito.
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
  }


  // 3. LANZAMIENTO DEL SISTEMA OPERATIVO Y APLICACIÓN
  
  ESP_LOGI(TAG, "Inicializacion de hardware exitosa. Lanzando tareas del sistema.");
  systemLaunch();
  

  // 4. BUCLE INACTIVO (OPCIONAL)
  // Esta tarea se ejecuta continuamente, manteniendo 'app_main' viva.
  // Es mejor mantenerla limpia y que toda la lógica de la aplicación esté en las tareas lanzadas por 'systemLaunch()'.
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}