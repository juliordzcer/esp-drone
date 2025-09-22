#include "ledseq.h"

// Includes de ESP-IDF para FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

// Incluimos tu driver de LED ya migrado
#include "led.h"

// --- Secuencias de LED Predefinidas ---
// La lógica de estas secuencias no cambia, es C estándar.

ledseq_t seq_lowbat[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_armed[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(250)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_calibrated[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(450)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_alive[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(1950)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_linkup[] = {
  { true, LEDSEQ_WAITMS(1)},
  {false, LEDSEQ_WAITMS(0)},
  {    0, LEDSEQ_STOP},
};

ledseq_t seq_charged[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_charging[] = {
  { true, LEDSEQ_WAITMS(200)},
  {false, LEDSEQ_WAITMS(800)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_bootloader[] = {
  { true, LEDSEQ_WAITMS(500)},
  {false, LEDSEQ_WAITMS(500)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_testPassed[] = {
  { true, LEDSEQ_WAITMS(50)}, {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, {false, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_STOP},
};

// --- Array de Prioridad de Secuencias ---
// El índice 0 tiene la máxima prioridad.
static ledseq_t * sequences[] = {
  seq_testPassed,
  seq_lowbat,
  seq_charged,
  seq_charging,
  seq_bootloader,
  seq_armed,
  seq_calibrated,
  seq_alive,
  seq_linkup,
};

// --- Variables Internas del Módulo ---
#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

// Prototipos de funciones estáticas
static void runLedseq(TimerHandle_t xTimer);
static int getPrio(ledseq_t *seq);
static void updateActive(led_t led);

// Estado de cada secuencia para cada LED (paso actual o LEDSEQ_STOP)
static int state[LED_NUM][SEQ_NUM];
// Secuencia activa (de mayor prioridad) para cada LED
static int activeSeq[LED_NUM];

// Un temporizador de software por cada LED
static TimerHandle_t timer[LED_NUM];
// Semáforo para proteger el acceso a las variables de estado
static SemaphoreHandle_t ledseqSem;

static bool isInit = false;

void ledseqInit() {
    if (isInit) {
        return;
    }

    ledInit(); // Inicializa el driver de GPIO para los LEDs

    // Inicializa los estados de las secuencias
    for (int i = 0; i < LED_NUM; i++) {
        activeSeq[i] = LEDSEQ_STOP;
        for (int j = 0; j < SEQ_NUM; j++) {
            state[i][j] = LEDSEQ_STOP;
        }
    }

    // Crea los temporizadores de FreeRTOS que manejarán las secuencias
    for (int i = 0; i < LED_NUM; i++) {
        timer[i] = xTimerCreate("ledseqTimer", pdMS_TO_TICKS(1000), pdFALSE, (void*)i, runLedseq);
    }

    // Crea el semáforo binario
    ledseqSem = xSemaphoreCreateBinary();
    xSemaphoreGive(ledseqSem); // El semáforo debe estar disponible inicialmente

    isInit = true;
}

void ledseqRun(led_t led, ledseq_t *sequence) {
    int prio = getPrio(sequence);
    if (prio < 0) return; // Secuencia no encontrada

    xSemaphoreTake(ledseqSem, portMAX_DELAY);
    state[led][prio] = 0;  // Inicia la secuencia en el primer paso (0)
    updateActive(led);
    xSemaphoreGive(ledseqSem);

    // Si la nueva secuencia es ahora la de mayor prioridad, la ejecuta inmediatamente
    if (activeSeq[led] == prio) {
        runLedseq(timer[led]);
    }
}

void ledseqStop(led_t led, ledseq_t *sequence) {
    int prio = getPrio(sequence);
    if (prio < 0) return; // Secuencia no encontrada

    xSemaphoreTake(ledseqSem, portMAX_DELAY);
    state[led][prio] = LEDSEQ_STOP; // Detiene la secuencia
    updateActive(led);
    xSemaphoreGive(ledseqSem);

    // Ejecuta la siguiente secuencia activa (si la hay)
    runLedseq(timer[led]);
}

void ledseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime) {
    sequence[0].action = onTime;
    sequence[1].action = offTime;
}

// --- Motor Principal de Secuencias ---
// Esta función es la devolución de llamada (callback) del temporizador de FreeRTOS
static void runLedseq(TimerHandle_t xTimer) {
    led_t led = (led_t)pvTimerGetTimerID(xTimer);
    ledseq_t *step;
    bool leave = false;

    // Bucle para manejar pasos con tiempo de espera 0 sin reiniciar el timer
    while (!leave) {
        int prio = activeSeq[led];

        if (prio == LEDSEQ_STOP) {
            return; // No hay secuencias activas para este LED
        }

        step = &sequences[prio][state[led][prio]];
        state[led][prio]++; // Avanza al siguiente paso

        xSemaphoreTake(ledseqSem, portMAX_DELAY);
        switch (step->action) {
            case LEDSEQ_LOOP:
                state[led][prio] = 0; // Reinicia la secuencia
                break;
            case LEDSEQ_STOP:
                state[led][prio] = LEDSEQ_STOP; // Marca como detenida
                updateActive(led); // Busca la siguiente de menor prioridad
                break;
            default: // El paso es una acción de LED con un tiempo de espera
                ledSet(led, step->value);
                if (step->action == 0) {
                    // Si el tiempo es 0, continúa el bucle para ejecutar el siguiente paso inmediatamente
                    break;
                }
                // Cambia el período del timer y lo inicia para el siguiente paso
                xTimerChangePeriod(xTimer, pdMS_TO_TICKS(step->action), 0);
                xTimerStart(xTimer, 0);
                leave = true; // Sale del bucle while
                break;
        }
        xSemaphoreGive(ledseqSem);
    }
}

// --- Funciones de Utilidad ---

// Obtiene la prioridad de una secuencia buscando su índice en el array `sequences`
static int getPrio(ledseq_t *seq) {
    for (int prio = 0; prio < SEQ_NUM; prio++) {
        if (sequences[prio] == seq) {
            return prio;
        }
    }
    return -1; // Secuencia no encontrada
}

// Actualiza cuál es la secuencia de mayor prioridad que está activa para un LED
static void updateActive(led_t led) {
    activeSeq[led] = LEDSEQ_STOP;
    ledSet(led, false); // Apaga el LED por defecto al cambiar de secuencia

    for (int prio = 0; prio < SEQ_NUM; prio++) {
        if (state[led][prio] != LEDSEQ_STOP) {
            activeSeq[led] = prio;
            break; // Encontró la de mayor prioridad, termina la búsqueda
        }
    }
}