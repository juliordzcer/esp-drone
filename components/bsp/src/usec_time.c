#include "usec_time.h"
#include "esp_timer.h" // Se incluye la biblioteca del temporizador de alta resolución del IDF.


void initUsecTimer(void)
{
}
uint64_t usecTimestamp(void)
{
  return (uint64_t)esp_timer_get_time();
}