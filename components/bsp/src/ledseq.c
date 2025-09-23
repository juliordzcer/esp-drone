#include "ledseq.h"

// Includes de ESP-IDF para FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// Incluimos tu driver de LED ya migrado
#include "led.h"

static const char *TAG = "ledseq";

// --- Secuencias de LED Predefinidas ---
ledseq_t seq_lowbat[] = {
  { true, LEDSEQ_WAITMS(200)},
  { false, LEDSEQ_WAITMS(200)},
  { false, LEDSEQ_LOOP},
};

ledseq_t seq_armed[] = {
  { true, LEDSEQ_WAITMS(50)},
  { false, LEDSEQ_WAITMS(250)},
  { false, LEDSEQ_LOOP},
};

ledseq_t seq_calibrated[] = {
  { true, LEDSEQ_WAITMS(50)},
  { false, LEDSEQ_WAITMS(450)},
  { false, LEDSEQ_LOOP},
};

ledseq_t seq_alive[] = {
  { true, LEDSEQ_WAITMS(50)},
  { false, LEDSEQ_WAITMS(1950)},
  { false, LEDSEQ_LOOP},
};

ledseq_t seq_linkup[] = {
  { true, LEDSEQ_WAITMS(10)},
  { false, LEDSEQ_WAITMS(0)},
  { false, LEDSEQ_STOP},
};

ledseq_t seq_charged[] = {
  { true, LEDSEQ_WAITMS(1000)},
  { false, LEDSEQ_LOOP},
};

ledseq_t seq_charging[] = {
  { true, LEDSEQ_WAITMS(200)},
  { false, LEDSEQ_WAITMS(800)},
  { false, LEDSEQ_LOOP},
};

ledseq_t seq_bootloader[] = {
  { true, LEDSEQ_WAITMS(500)},
  { false, LEDSEQ_WAITMS(500)},
  { false, LEDSEQ_LOOP},
};

ledseq_t seq_testPassed[] = {
  { true, LEDSEQ_WAITMS(50)}, { false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, { false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, { false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, { false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, { false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, { false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)}, { false, LEDSEQ_WAITMS(50)},
  { false, LEDSEQ_STOP},
};

// --- Array de Prioridad de Secuencias ---
static ledseq_t *sequences[] = {
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
#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

// --- Variables Internas del Módulo ---
static int state[LED_NUM][SEQ_NUM];
static int activeSeq[LED_NUM];
static TimerHandle_t timer[LED_NUM];
static SemaphoreHandle_t ledseqSem;
static bool isInit = false;

// --- Prototipos de funciones estáticas ---
static void runLedseq(TimerHandle_t xTimer);
static int getPrio(ledseq_t *seq);
static void updateActive(led_t led);

// --- Funciones Públicas ---
void ledseqInit() {
    if (isInit) {
        return;
    }

    ledInit(); // Inicializa el driver de GPIO para los LEDs

    ledseqSem = xSemaphoreCreateMutex();
    
    for (int i = 0; i < LED_NUM; i++) {
        activeSeq[i] = LEDSEQ_STOP;
        for (int j = 0; j < SEQ_NUM; j++) {
            state[i][j] = LEDSEQ_STOP;
        }
        timer[i] = xTimerCreate("ledseqTimer", pdMS_TO_TICKS(100), pdFALSE, (void*)i, runLedseq);
    }
    
    isInit = true;
    ESP_LOGI(TAG, "LED sequence module initialized.");
}

bool ledseqTest(void) {
    ESP_LOGI(TAG, "Iniciando prueba de secuencia de LEDs...");
    
    // 1. Probar la secuencia de "testPassed" en el LED verde
    ESP_LOGI(TAG, "  -> Ejecutando secuencia 'testPassed' en LED VERDE");
    ledseqRun(LED_GREEN, seq_testPassed);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo para ver el efecto
    ledseqStop(LED_GREEN, seq_testPassed);
    
    // 2. Probar una secuencia de "parpadeo" en el LED rojo
    ESP_LOGI(TAG, "  -> Ejecutando secuencia 'bootloader' en LED ROJO");
    ledseqRun(LED_RED, seq_bootloader);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar 5 segundos
    ledseqStop(LED_RED, seq_bootloader);

    // 3. Probar la secuencia 'lowbat' en el LED azul y luego detenerla
    ESP_LOGI(TAG, "  -> Ejecutando secuencia 'lowbat' en LED AZUL");
    ledseqRun(LED_BLUE, seq_lowbat);
    vTaskDelay(pdMS_TO_TICKS(3000)); // Esperar 3 segundos
    ledseqStop(LED_BLUE, seq_lowbat);

    // 4. Probar la función de modificación de tiempos
    ESP_LOGI(TAG, "  -> Modificando y ejecutando secuencia 'lowbat' de nuevo");
    ledseqSetTimes(seq_lowbat, 50, 50); // Parpadeo más rápido
    ledseqRun(LED_BLUE, seq_lowbat);
    vTaskDelay(pdMS_TO_TICKS(3000));
    ledseqStop(LED_BLUE, seq_lowbat);

    // Restaurar los tiempos originales para futuras pruebas si es necesario
    ledseqSetTimes(seq_lowbat, 200, 200);

    ESP_LOGI(TAG, "Prueba de secuencia de LEDs finalizada.");

    return true; // La prueba se completó con éxito
}

void ledseqRun(led_t led, ledseq_t *sequence) {
    int prio = getPrio(sequence);
    if (prio < 0) return; // Secuencia no encontrada

    xSemaphoreTake(ledseqSem, portMAX_DELAY);
    bool wasActive = (activeSeq[led] == prio);
    state[led][prio] = 0; // Inicia la secuencia en el primer paso (0)
    updateActive(led);
    xSemaphoreGive(ledseqSem);
    
    if (activeSeq[led] == prio && !wasActive) {
        // La nueva secuencia es ahora la de mayor prioridad. La ejecuta inmediatamente.
        // Se detiene el timer si estaba en ejecución.
        xTimerStop(timer[led], 0);
        runLedseq(timer[led]);
    }
}

void ledseqStop(led_t led, ledseq_t *sequence) {
    int prio = getPrio(sequence);
    if (prio < 0) return; // Secuencia no encontrada

    xSemaphoreTake(ledseqSem, portMAX_DELAY);
    bool wasActive = (activeSeq[led] == prio);
    state[led][prio] = LEDSEQ_STOP; // Detiene la secuencia
    updateActive(led);
    xSemaphoreGive(ledseqSem);
    
    if (wasActive) {
        // Si la secuencia que se detuvo era la activa, reinicia el motor.
        // Se detiene el timer si estaba en ejecución.
        xTimerStop(timer[led], 0);
        runLedseq(timer[led]);
    }
}

void ledseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime) {
    sequence[0].action = onTime;
    sequence[1].action = offTime;
}

// --- Motor Principal de Secuencias (Callback del Timer) ---
static void runLedseq(TimerHandle_t xTimer) {
    led_t led = (led_t)pvTimerGetTimerID(xTimer);
    ledseq_t *step;
    
    xSemaphoreTake(ledseqSem, portMAX_DELAY);
    int prio = activeSeq[led];
    
    if (prio == LEDSEQ_STOP) {
        ledSet(led, false);
        xSemaphoreGive(ledseqSem);
        return;
    }
    
    int currentStepIndex = state[led][prio];
    step = &sequences[prio][currentStepIndex];
    state[led][prio]++; // Avanza al siguiente paso
    
    xSemaphoreGive(ledseqSem);

    // Ejecuta el paso actual
    switch (step->action) {
        case LEDSEQ_LOOP:
            xSemaphoreTake(ledseqSem, portMAX_DELAY);
            state[led][prio] = 0; // Reinicia la secuencia
            xSemaphoreGive(ledseqSem);
            // Ejecuta el primer paso inmediatamente
            xTimerStart(xTimer, 0); 
            break;
            
        case LEDSEQ_STOP:
            xSemaphoreTake(ledseqSem, portMAX_DELAY);
            state[led][prio] = LEDSEQ_STOP; // Marca como detenida
            updateActive(led); // Busca la siguiente de menor prioridad
            xSemaphoreGive(ledseqSem);
            // El motor se detiene y la siguiente secuencia activa será iniciada por el próximo ciclo de la tarea PM.
            break;
            
        default: // Es una acción de LED con un tiempo de espera
            ledSet(led, step->value);
            // Si el tiempo es 0, inicia el timer para el siguiente paso, pero con un retardo mínimo
            int next_delay = (step->action > 0) ? step->action : 1;
            xTimerChangePeriod(xTimer, pdMS_TO_TICKS(next_delay), 0);
            xTimerStart(xTimer, 0);
            break;
    }
}

// --- Funciones de Utilidad ---
static int getPrio(ledseq_t *seq) {
    for (int prio = 0; prio < SEQ_NUM; prio++) {
        if (sequences[prio] == seq) {
            return prio;
        }
    }
    return -1; // Secuencia no encontrada
}

static void updateActive(led_t led) {
    // Apaga el LED para evitar estados fantasma
    ledSet(led, false); 
    activeSeq[led] = LEDSEQ_STOP;
    
    for (int prio = 0; prio < SEQ_NUM; prio++) {
        if (state[led][prio] != LEDSEQ_STOP) {
            activeSeq[led] = prio;
            break;
        }
    }
}