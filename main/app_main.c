/**
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
 * app_main.c - Main function for the ESP32 port.
 */

/* Project includes */
// #include "config.h"
#include "system.h"
// #include "led.h"
// Add the required FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h> // Required for uint8_t

// --- FIX CRÍTICO DEL LINKER: FORZAR INCLUSIÓN DE SÍMBOLOS ---
// Estos símbolos marcan el inicio y el fin de las secciones de datos 
// personalizados (.log* y .param*) definidas en 'module_linker_fragment.lf'.
// Se declaran aquí como 'extern const' para que el linker sepa que deben 
// ser preservados y no sean eliminados por el "garbage collector" del linker.
extern const uint8_t _log_start;
extern const uint8_t _log_stop;
extern const uint8_t _param_start;
extern const uint8_t _param_stop;
// --------------------------------------------------------


// The main entry point for the ESP32 application
void app_main(void)
{
  // Launch the system task that will initialize and start everything.
  // The systemLaunch() function itself will create a task to handle all initializations.
  systemLaunch();

  // The FreeRTOS scheduler is already started by ESP-IDF before app_main() is called.
  // This main function can be left empty or used for an idle loop.
  while (1) {
    // This loop is optional. Your application logic should reside in other tasks.
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
