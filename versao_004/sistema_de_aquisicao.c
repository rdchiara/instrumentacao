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

// --- Configurações da Aplicação ---
#define UART_NUM        UART_NUM_0      
#define SERIAL_BAUD     921600          

#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_3 
#define ADC_ATTEN       ADC_ATTEN_DB_6

#define ENCODER_A_PIN   GPIO_NUM_17
#define ENCODER_B_PIN   GPIO_NUM_16

// --- TAXAS DE AQUISIÇÃO ---
#define FREQ_ADC_HZ         1000
#define FREQ_ENCODER_HZ     50

#define PERIOD_ADC_MS       (1000 / FREQ_ADC_HZ)     
#define PERIOD_ENCODER_MS   (1000 / FREQ_ENCODER_HZ) 

#define BUFFER_SIZE_SAMPLES 1024 // Reduzi um pouco para garantir fluidez na memória, mas 4096 funciona

// Estrutura para os dados de cada amostra (Total: 8 bytes)
typedef struct {
    uint32_t timestamp_ms;  // 4 bytes
    uint16_t voltage_raw;   // 2 bytes
    int16_t  encoder_diff;  // 2 bytes
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

// Variáveis de Controle de Fluxo
volatile int16_t g_shared_encoder_diff = 0;
volatile uint32_t g_samples_to_acquire = 0; // META DE AMOSTRAS

typedef enum {
    STATE_IDLE,
    STATE_READY_TO_ACQUIRE, 
    STATE_ACQUIRING_DATA
} system_state_t;
volatile system_state_t g_system_state = STATE_IDLE;


// --- Funções de Inicialização ---

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

// --- TAREFA: LEITURA DO ENCODER (50Hz) ---
void encoder_acquisition_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_ENCODER_MS);
    int pulses_diff = 0;
    esp_err_t err;

    while(1) {
        // Aguarda sinal global
        xEventGroupWaitBits(acquisition_event_group, ACQUISITION_RUNNING_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        // Reset inicial
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
        g_shared_encoder_diff = 0;
        
        xLastWakeTime = xTaskGetTickCount();

        // Loop condicionado ao Bit de Running
        while((xEventGroupGetBits(acquisition_event_group) & ACQUISITION_RUNNING_BIT) != 0) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            err = pcnt_unit_get_count(pcnt_unit, &pulses_diff);
            if (err == ESP_OK) {
                g_shared_encoder_diff = (int16_t)pulses_diff;
                pcnt_unit_clear_count(pcnt_unit);
            }
        }
    }
}

// --- TAREFA: AQUISIÇÃO DE TENSÃO (1000Hz) - CONTROLE PRINCIPAL ---
void adc_acquisition_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 
    
    int raw_adc;
    uint32_t buffer_sample_index = 0;
    uint32_t total_samples_collected = 0; // Contador Geral da Sessão
    int64_t start_time_ms = 0;
    int buffer_idx = 0;
    esp_err_t err; 
    
    while(1) {
        // Aguarda o comando de início
        xEventGroupWaitBits(acquisition_event_group, ACQUISITION_RUNNING_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        
        // Reset das variáveis locais
        buffer_sample_index = 0;
        total_samples_collected = 0;
        buffer_idx = 0;
        AcquisitionDataPoint* current_buffer = buffer_pool[buffer_idx];
        start_time_ms = esp_timer_get_time() / 1000;
        
        // Delay de estabilização
        vTaskDelay(pdMS_TO_TICKS(10)); 

        xLastWakeTime = xTaskGetTickCount(); 
        
        // LOOP PRINCIPAL: Roda até atingir o NÚMERO EXATO DE AMOSTRAS
        // E TAMBÉM verifica o bit de Running (caso o usuário force parada)
        while(total_samples_collected < g_samples_to_acquire && 
             (xEventGroupGetBits(acquisition_event_group) & ACQUISITION_RUNNING_BIT) != 0) {
            
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            
            // 1. Leitura ADC
            err = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_adc);
            if (err != ESP_OK) raw_adc = 0;
            current_buffer[buffer_sample_index].voltage_raw = (uint16_t)raw_adc;
            
            // 2. Leitura Encoder (Variável Compartilhada)
            current_buffer[buffer_sample_index].encoder_diff = g_shared_encoder_diff;
            
            // 3. Timestamp
            current_buffer[buffer_sample_index].timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000 - start_time_ms);
            
            buffer_sample_index++;
            total_samples_collected++;

            // 4. Buffer cheio? Envia.
            if (buffer_sample_index >= BUFFER_SIZE_SAMPLES) {
                BufferInfo info_to_send = { .buffer_ptr = current_buffer, .sample_count = BUFFER_SIZE_SAMPLES };
                
                // Timeout curto. Se falhar, paciência (melhor perder 1 buffer que travar tudo)
                xQueueSend(stream_queue, &info_to_send, pdMS_TO_TICKS(20)); 
                
                buffer_idx = (buffer_idx + 1) % NUM_BUFFERS;
                current_buffer = buffer_pool[buffer_idx];
                buffer_sample_index = 0;
            }
        }

        // FIM DA AQUISIÇÃO (Por Meta Atingida ou Cancelamento)
        
        // Envia o buffer residual
        if (buffer_sample_index > 0) {
            BufferInfo final_info = { .buffer_ptr = current_buffer, .sample_count = buffer_sample_index };
            xQueueSend(stream_queue, &final_info, pdMS_TO_TICKS(100));
        }

        // Avisa que acabou: Limpa o Bit
        xEventGroupClearBits(acquisition_event_group, ACQUISITION_RUNNING_BIT);
        
        // Envia pacote de finalização (NULL) para a task de Serial
        // Damos um delay para garantir que os dados anteriores foram processados
        vTaskDelay(pdMS_TO_TICKS(100));
        BufferInfo finalization_msg = { .buffer_ptr = NULL, .sample_count = 0 };
        xQueueSend(stream_queue, &finalization_msg, pdMS_TO_TICKS(100)); 
    }
}


// --- TAREFA: STREAMING (CONSUMIDOR) ---
void serial_stream_task(void *pvParameters) {
    BufferInfo received_info;
    const char* final_msg = "OK: Aquisicao concluida.\n";

    while(1) {
        if(xQueueReceive(stream_queue, &received_info, portMAX_DELAY)) {
            
            if (received_info.buffer_ptr == NULL) {
                // Finalização
                uart_write_bytes(UART_NUM, final_msg, strlen(final_msg));
                g_system_state = STATE_READY_TO_ACQUIRE; 
            } 
            else { 
                // Envio Binário
                int to_write = received_info.sample_count * sizeof(AcquisitionDataPoint);
                uart_write_bytes(UART_NUM, (const char*)received_info.buffer_ptr, to_write);
            }
        } 
    } 
}


// --- TAREFA DE CONTROLE (CONTROLADOR) ---
void serial_control_task(void *pvParameters) {
    char rx_buffer[128];
    
    while (1) {
        // Se já está adquirindo, não processa comandos (exceto se quiséssemos um STOP, mas simplificamos aqui)
        if (g_system_state == STATE_ACQUIRING_DATA) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int len = uart_read_bytes(UART_NUM, (uint8_t*)rx_buffer, sizeof(rx_buffer) - 1, pdMS_TO_TICKS(50));

        if (len > 0) {
            rx_buffer[len] = '\0';
            
            if (strncmp(rx_buffer, "INICIAR,", 8) == 0) {
                int duration = 0;
                if (sscanf(rx_buffer, "INICIAR,%d", &duration) == 1 && duration > 0) {
                    
                    // 1. Calcula META de amostras
                    g_samples_to_acquire = duration * FREQ_ADC_HZ;
                    
                    char msg[100];
                    sprintf(msg, "OK: Iniciando aquisicao de %ld amostras (%ds)...\n", g_samples_to_acquire, duration);
                    uart_write_bytes(UART_NUM, msg, strlen(msg)); 
                    uart_flush(UART_NUM); 

                    // 2. Prepara Estado
                    xQueueReset(stream_queue); 
                    g_system_state = STATE_ACQUIRING_DATA;
                    
                    // 3. Libera as tasks
                    xEventGroupSetBits(acquisition_event_group, ACQUISITION_RUNNING_BIT); 
                    
                    // 4. AQUI ESTÁ O PULO DO GATO:
                    // Não damos ClearBits aqui após X segundos.
                    // A própria task de aquisição vai contar as amostras e dar ClearBits.
                    // Nós apenas esperamos o estado voltar ao normal.
                } 
                else {
                    uart_write_bytes(UART_NUM, "ERRO: Comando invalido\n", 23);
                }
            }
            else if (strncmp(rx_buffer, "STATUS", 6) == 0) {
                uart_write_bytes(UART_NUM, "ESP32 PRONTO\n", 13);
            }
        }
    }
}


void app_main(void) {
    // IMPORTANTE: Desabilita logs para não corromper o binário com texto
    esp_log_level_set("*", ESP_LOG_NONE);
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    acquisition_event_group = xEventGroupCreate();
    uart_init_serial(); 
    stream_queue = xQueueCreate(NUM_BUFFERS + 2, sizeof(BufferInfo)); 

    adc_init_voltage();
    initialize_encoder_pcnt();
    
    // Core 1 para Aquisição (ADC e Encoder)
    xTaskCreatePinnedToCore(adc_acquisition_task, "adc_task", 4096, NULL, 10, NULL, 1); 
    xTaskCreatePinnedToCore(encoder_acquisition_task, "enc_task", 2048, NULL, 10, NULL, 1);

    // Core 0 para Comunicação
    xTaskCreatePinnedToCore(serial_stream_task, "stream_task", 4096, NULL, 9, NULL, 0);       
    xTaskCreate(serial_control_task, "control_task", 4096, NULL, 6, NULL);                          
}