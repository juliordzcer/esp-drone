#ifndef USEC_TIME_H_
#define USEC_TIME_H_

#include <stdint.h> // Necesario para uint64_t y uint32_t

/**
 * Inicializa el temporizador de resolución de microsegundos.
 */
void initUsecTimer(void);

/**
 * Obtiene la marca de tiempo de resolución de microsegundos.
 */
uint64_t usecTimestamp(void);

#endif /* USEC_TIME_H_ */