#include "adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/soc_caps.h"

static const char *TAG = "ADC";

// ====================================================================
// --- CONFIGURACIÓN FINAL Y CALIBRADA ---
// ====================================================================

// 1. CALIBRACIÓN DE PRECISIÓN: Ajustado para que la lectura coincida con el multímetro.
#define VBAT_DIVIDER_RATIO      2.0f

// 2. CONFIGURACIÓN DEL HARDWARE
#define ADC_CHANNEL_VBAT        ADC_CHANNEL_1
#define ADC_ATTENUATION         ADC_ATTEN_DB_11

// 3. CORRECCIÓN DE COMPATIBILIDAD (ESP32-S2): Usar la resolución correcta para el modo continuo.
#define ADC_BITWIDTH            SOC_ADC_DIGI_MAX_BITWIDTH

#define ADC_SAMPLE_FREQ_HZ      (20 * 1024)
#define SAMPLES_PER_BUFFER      256

// ====================================================================
// --- VARIABLES GLOBALES ---
// ====================================================================

static bool isInit = false;
static float current_battery_voltage = 0.0f;
static SemaphoreHandle_t adc_mutex = NULL;
static QueueHandle_t adc_queue = NULL;

static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;

void adcProcessingTask(void *param);

// ====================================================================
// --- IMPLEMENTACIÓN ---
// ====================================================================

// Callback de la interrupción del DMA. Rápido y eficiente.
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    xQueueSendFromISR(adc_queue, &edata->conv_frame_buffer, &mustYield);
    return (mustYield == pdTRUE);
}

// Inicialización del sistema ADC
void adcInit(void)
{
    if (isInit) return;

    adc_mutex = xSemaphoreCreateMutex();
    adc_queue = xQueueCreate(8, sizeof(uint8_t*));

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = SAMPLES_PER_BUFFER,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t dig_pattern_config = {
        .atten = ADC_ATTENUATION,
        .channel = ADC_CHANNEL_VBAT,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH,
    };
    adc_continuous_config_t cont_config = {
        .pattern_num = 1,
        .adc_pattern = &dig_pattern_config,
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &cont_config));
    
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    
    xTaskCreate(adcProcessingTask, "ADC_Processing_Task", 4096, NULL, 5, NULL);
    
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    isInit = true;
    ESP_LOGI(TAG, "ADC en modo continuo, robusto y calibrado inicializado.");
}

bool adcTest(void)
{
    adcInit();

    ESP_LOGI(TAG, "Iniciando prueba de ADC...");

    for (int i = 0; i < 5; i++) {
        float voltage = adcGetBatteryVoltage();
        ESP_LOGI(TAG, "Voltaje de la batería: %.2fV", voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return true;
}

float adcGetBatteryVoltage(void)
{
    float voltage = 0.0f;
    if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(50))) {
        voltage = current_battery_voltage;
        xSemaphoreGive(adc_mutex);
    } else {
        ESP_LOGW(TAG, "No se pudo tomar el mutex del ADC");
    }
    return voltage;
}

// Tarea en segundo plano que procesa los datos del ADC
void adcProcessingTask(void *param)
{
    uint8_t* result_buffer = NULL;
    uint32_t ret_num = SAMPLES_PER_BUFFER; // Corrected to use the expected buffer size
    
    ESP_LOGI(TAG, "Tarea de procesamiento de ADC iniciada.");

    while (1) {
        if (xQueueReceive(adc_queue, &result_buffer, portMAX_DELAY)) {
            
            long sum_mv = 0;
            // The call to adc_continuous_read is removed here!
            uint32_t sample_count = ret_num / SOC_ADC_DIGI_RESULT_BYTES;

            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result_buffer[i];
                int voltage_mv = 0;
                
                adc_cali_raw_to_voltage(cali_handle, p->type1.data, &voltage_mv); 

                voltage_mv *= 2;

                sum_mv += voltage_mv;
            }

            if (sample_count > 0) {
                int avg_mv = sum_mv / sample_count;
                float final_voltage = (float)avg_mv / 1000.0f * VBAT_DIVIDER_RATIO;
                
                if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(50))) {
                    current_battery_voltage = final_voltage;
                    xSemaphoreGive(adc_mutex);
                }
            }
        }
    }
}