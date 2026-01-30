#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <inttypes.h>

#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h" 
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h" 

// Removida a tag TAG, pois os logs estão desabilitados
// static const char *TAG = "ESP32_APP_SERIAL"; 

// --- Configurações da Aplicação ---
#define UART_NUM        UART_NUM_0      
#define SERIAL_BAUD     921600          

#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_3 
#define ADC_ATTEN       ADC_ATTEN_DB_6

#define ENCODER_A_PIN   GPIO_NUM_17
#define ENCODER_B_PIN   GPIO_NUM_16

#define ACQUISITION_FREQUENCY_HZ 40
#define ACQUISITION_PERIOD_MS (1000 / ACQUISITION_FREQUENCY_HZ)
#define BUFFER_SIZE_SAMPLES 4096 

// Estrutura para os dados de cada amostra (Total: 8 bytes)
typedef struct {
    uint32_t timestamp_ms;  // 4 bytes: Tempo desde o início da aquisição (ms)
    uint16_t voltage_raw;   // 2 bytes: Leitura bruta do ADC
    int16_t  encoder_diff;  // 2 bytes: Diferença de pulsos do encoder no período
} AcquisitionDataPoint;

// Estrutura para comunicação via Fila
typedef struct {
    AcquisitionDataPoint* buffer_ptr; 
    uint32_t sample_count;            
} BufferInfo;

// Definição dos buffers e da fila
static AcquisitionDataPoint buffer_a[BUFFER_SIZE_SAMPLES];
static AcquisitionDataPoint buffer_b[BUFFER_SIZE_SAMPLES];
static AcquisitionDataPoint buffer_c[BUFFER_SIZE_SAMPLES];
static AcquisitionDataPoint buffer_d[BUFFER_SIZE_SAMPLES];
static QueueHandle_t stream_queue;
static AcquisitionDataPoint* buffer_pool[] = {buffer_a, buffer_b, buffer_c, buffer_d};
const int NUM_BUFFERS = sizeof(buffer_pool) / sizeof(buffer_pool[0]);


// --- Variáveis Globais ---
static adc_oneshot_unit_handle_t adc_handle;
static EventGroupHandle_t acquisition_event_group;
#define ACQUISITION_RUNNING_BIT BIT0

// Removida g_buffer_drop_error, pois não é utilizada no código sem logs de debug

typedef enum {
    STATE_IDLE,
    STATE_READY_TO_ACQUIRE, 
    STATE_ACQUIRING_DATA
} system_state_t;
volatile system_state_t g_system_state = STATE_IDLE;


// --- Funções de Inicialização e Periféricos ---

void uart_init_serial(void) {
    const uart_config_t uart_config = {
        .baud_rate = SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    g_system_state = STATE_READY_TO_ACQUIRE;
}

void adc_init_voltage() {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT, };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT,};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));
}

pcnt_unit_handle_t pcnt_unit = NULL;
void initialize_encoder_pcnt(void) {
    pcnt_unit_config_t unit_config = { .low_limit = -32768, .high_limit = 32767, };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_chan_config_t chan_a_config = { .edge_gpio_num = ENCODER_A_PIN, .level_gpio_num = ENCODER_B_PIN, };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = { .edge_gpio_num = ENCODER_B_PIN, .level_gpio_num = ENCODER_A_PIN, };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
}


// --- TAREFA: AQUISIÇÃO DE DADOS (PRODUTOR) ---
void high_freq_acquisition_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(ACQUISITION_PERIOD_MS);
    int raw_adc;
    uint32_t sample_index = 0;
    int64_t start_time_ms = 0;
    int buffer_idx = 0;
    esp_err_t err; 
    
    while(1) {
        xEventGroupWaitBits(acquisition_event_group, ACQUISITION_RUNNING_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        
        sample_index = 0;
        buffer_idx = 0;
        AcquisitionDataPoint* current_buffer = buffer_pool[buffer_idx];
        start_time_ms = esp_timer_get_time() / 1000;
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
        xLastWakeTime = xTaskGetTickCount(); 
        
        while((xEventGroupGetBits(acquisition_event_group) & ACQUISITION_RUNNING_BIT) != 0) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            
            // 1. Lê ADC
            err = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_adc);
            if (err != ESP_OK) { ESP_ERROR_CHECK(err); }
            current_buffer[sample_index].voltage_raw = (uint16_t)raw_adc;
            
            // 2. Lê Encoder (diferença)
            int pulses_diff = 0;
            err = pcnt_unit_get_count(pcnt_unit, &pulses_diff);
            if (err != ESP_OK) { ESP_ERROR_CHECK(err); }
            err = pcnt_unit_clear_count(pcnt_unit);
            if (err != ESP_OK) { ESP_ERROR_CHECK(err); }
            current_buffer[sample_index].encoder_diff = (int16_t)pulses_diff;
            
            // 3. Grava Timestamp
            current_buffer[sample_index].timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000 - start_time_ms);
            sample_index++;

            // 4. Se o buffer está cheio, envia para a fila (Produtor)
            if (sample_index >= BUFFER_SIZE_SAMPLES) {
                BufferInfo info_to_send = { .buffer_ptr = current_buffer, .sample_count = BUFFER_SIZE_SAMPLES };
                
                BaseType_t result = xQueueSend(stream_queue, &info_to_send, pdMS_TO_TICKS(100));
                if (result != pdPASS) {
                    // g_buffer_drop_error = true; (Removido, pois a variável foi eliminada)
                    // Se o buffer cair, continuamos, mas não conseguiremos reportar o erro.
                }

                buffer_idx = (buffer_idx + 1) % NUM_BUFFERS;
                current_buffer = buffer_pool[buffer_idx];
                sample_index = 0;
            }
        }

        // Aquisição parada. Envia o buffer residual (o que sobrou)
        if (sample_index > 0) {
            BufferInfo final_info = { .buffer_ptr = current_buffer, .sample_count = sample_index };
            
            BaseType_t result = xQueueSend(stream_queue, &final_info, pdMS_TO_TICKS(100));
            if (result != pdPASS) {
                 // g_buffer_drop_error = true; (Removido, pois a variável foi eliminada)
            }
        }
    }
}


// --- TAREFA: STREAMING DE DADOS VIA SERIAL (CONSUMIDOR) - BINÁRIO ---
void serial_stream_task(void *pvParameters) {
    BufferInfo received_info;
    
    while(1) {
        // CORREÇÃO: Usando a macro correta: portMAX_DELAY
        if(xQueueReceive(stream_queue, &received_info, portMAX_DELAY)) {
            
            if (received_info.buffer_ptr == NULL) {
                // Mensagem de finalização (Texto ASCII)
                const char* final_msg = "OK: Aquisicao concluida.\n";
                uart_write_bytes(UART_NUM, final_msg, strlen(final_msg));
                g_system_state = STATE_READY_TO_ACQUIRE; 
            } 
            else { 
                // Envio BINÁRIO dos dados (8 bytes por amostra)
                int to_write = received_info.sample_count * sizeof(AcquisitionDataPoint);
                int written = uart_write_bytes(UART_NUM, (const char*)received_info.buffer_ptr, to_write);
                
                if (written != to_write) {
                    // Erro de transmissão na UART
                }
            }
        } 
        
        vTaskDelay(pdMS_TO_TICKS(1)); 
    } 
}


// --- TAREFA DE CONTROLE SERIAL (CONTROLADOR) ---
void serial_control_task(void *pvParameters) {
    char rx_buffer[128];
    
    while (1) {
        if (g_system_state != STATE_READY_TO_ACQUIRE && g_system_state != STATE_ACQUIRING_DATA) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int len = uart_read_bytes(UART_NUM, (uint8_t*)rx_buffer, sizeof(rx_buffer) - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            rx_buffer[len] = '\0';
            
            if (g_system_state == STATE_ACQUIRING_DATA) {
                uart_write_bytes(UART_NUM, "ERRO: Aquisição em curso. Espere a finalização.\n", 48);
                continue;
            }

            if (strncmp(rx_buffer, "INICIAR,", 8) == 0) {
                int duration = 0;
                if (sscanf(rx_buffer, "INICIAR,%d", &duration) != 1 || duration <= 0) {
                    uart_write_bytes(UART_NUM, "ERRO: Comando INICIAR mal formatado. Use INICIAR,<segundos>\n", 60);
                } else {
                    // Mensagem de confirmação OK
                    char msg[100];
                    sprintf(msg, "OK: Iniciando aquisição por %d segundos...\n", duration);
                    uart_write_bytes(UART_NUM, msg, strlen(msg)); 
                    
                    // Limpeza crítica da UART antes de iniciar o stream binário
                    uart_flush(UART_NUM); 

                    // g_buffer_drop_error = false; (Removido, pois a variável foi eliminada)
                    xQueueReset(stream_queue); 
                    g_system_state = STATE_ACQUIRING_DATA;
                    xEventGroupSetBits(acquisition_event_group, ACQUISITION_RUNNING_BIT); 
                    
                    vTaskDelay(pdMS_TO_TICKS(duration * 1000));
                    
                    xEventGroupClearBits(acquisition_event_group, ACQUISITION_RUNNING_BIT); 
                    
                    vTaskDelay(pdMS_TO_TICKS(500)); 
                    
                    BufferInfo finalization_msg = { .buffer_ptr = NULL, .sample_count = 0 };
                    xQueueSend(stream_queue, &finalization_msg, pdMS_TO_TICKS(1000)); 
                }
            }
            else if (strncmp(rx_buffer, "STATUS", 6) == 0) {
                uart_write_bytes(UART_NUM, "ESP32 CONECTADO E PRONTO\n", 27);
            }
            else {
                uart_write_bytes(UART_NUM, "ERRO: Comando desconhecido\n", 27);
            }
        }
    }
}


void app_main(void) {
    // DESABILITA LOGS DE INFO E DEBUG DO ESP-IDF (para UART limpa)
    esp_log_level_set("*", ESP_LOG_WARN);
    
    // Inicialização NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializa Periféricos e FreeRTOS
    acquisition_event_group = xEventGroupCreate();
    uart_init_serial(); 
    stream_queue = xQueueCreate(NUM_BUFFERS + 1, sizeof(BufferInfo)); 

    adc_init_voltage();
    initialize_encoder_pcnt();
    
    // Cria as tarefas
    xTaskCreatePinnedToCore(high_freq_acquisition_task, "acquisition_task", 4096, NULL, 10, NULL, 1); 
    xTaskCreatePinnedToCore(serial_stream_task, "serial_stream_task", 4096, NULL, 9, NULL, 0);       
    xTaskCreate(serial_control_task, "serial_control", 4096, NULL, 6, NULL);                          
}