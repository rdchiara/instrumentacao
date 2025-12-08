/*
 * Código para ESP32 DEVKIT V1 - Coleta de Dados e Stream via TCP
 *
 * HISTÓRICO DE CORREÇÕES:
 * 1. (FIX-1) Adicionada verificação de retorno da fila (xQueueSend) e
 * sinalização de erro (LED Amarelo) para perda de buffers.
 *
 * 2. (FIX-2) Removida condição de corrida (Race Condition) na finalização.
 * A 'tcp_stream_task' agora é a única que envia a msg de conclusão,
 * garantindo a ordem correta dos dados.
 *
 * 3. (FIX-3) Adicionada verificação de erro (ESP_ERROR_CHECK) dentro
 * do loop de aquisição (ADC e PCNT) para diagnosticar travamentos.
 *
 * 4. (FIX-4) Adicionado vTaskDelay(1) na 'tcp_stream_task' (Consumidor)
 * para prevenir "starvation" (inanição) da tarefa de rede (lwIP)
 * que roda no mesmo Core 0.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "nvs_flash.h"
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"

// --- Configurações da Aplicação ---
#define WIFI_SSID       "RoteadorESP32"
#define WIFI_PASS       "teste123"
#define STATIC_IP       "192.168.0.199"
#define GATEWAY         "192.168.0.1"
#define NETMASK         "255.255.255.0"
#define TCP_PORT        4343

#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_3 
#define ADC_ATTEN       ADC_ATTEN_DB_12

#define ENCODER_A_PIN   GPIO_NUM_17
#define ENCODER_B_PIN   GPIO_NUM_16

#define ACQUISITION_FREQUENCY_HZ 1000
#define ACQUISITION_PERIOD_MS (1000 / ACQUISITION_FREQUENCY_HZ)
#define BUFFER_SIZE_SAMPLES 4096

// Estrutura para os dados de cada amostra
typedef struct {
    uint32_t timestamp_ms;
    uint16_t voltage_raw;
    int16_t  encoder_diff;
} AcquisitionDataPoint;

// Estrutura para comunicação via Fila
typedef struct {
    AcquisitionDataPoint* buffer_ptr; // Ponteiro para o buffer OU NULL para msg de finalização
    uint32_t sample_count;            // Número de amostras no buffer
} BufferInfo;

// Definição dos buffers e da fila (usaremos 4 para mais robustez)
static AcquisitionDataPoint buffer_a[BUFFER_SIZE_SAMPLES];
static AcquisitionDataPoint buffer_b[BUFFER_SIZE_SAMPLES];
static AcquisitionDataPoint buffer_c[BUFFER_SIZE_SAMPLES];
static AcquisitionDataPoint buffer_d[BUFFER_SIZE_SAMPLES];
static QueueHandle_t stream_queue;
static AcquisitionDataPoint* buffer_pool[] = {buffer_a, buffer_b, buffer_c, buffer_d};
const int NUM_BUFFERS = sizeof(buffer_pool) / sizeof(buffer_pool[0]);


// --- Variáveis Globais ---
static const char *TAG = "ESP32_APP_STREAM";
static adc_oneshot_unit_handle_t adc_handle;
static EventGroupHandle_t acquisition_event_group;
#define ACQUISITION_RUNNING_BIT BIT0

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

#define LED_GREEN_PIN   GPIO_NUM_25
#define LED_RED_PIN     GPIO_NUM_22
#define LED_YELLOW_PIN  GPIO_NUM_26 // (FIX-1) Usado para sinalizar erro de buffer
#define LED_WIFI_PIN    GPIO_NUM_21
#define BUZZER_PIN      GPIO_NUM_27

volatile int g_active_socket = -1;
volatile bool g_buffer_drop_error = false; // (FIX-1) Flag global de erro de buffer

typedef enum {
    STATE_IDLE,
    STATE_CLIENT_CONNECTED,
    STATE_ACQUIRING_DATA
} system_state_t;
volatile system_state_t g_system_state = STATE_IDLE;

// --- Protótipos de Funções ---
void beep(int duration_ms);
void led_status_task(void *pvParameters);
void wifi_led_task(void *pvParameters);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_init_sta_static_ip(void);
void adc_init_voltage();
pcnt_unit_handle_t pcnt_unit = NULL;
void initialize_encoder_pcnt(void);
void high_freq_acquisition_task(void *pvParameters);
void tcp_stream_task(void *pvParameters);
void tcp_server_task(void *pvParameters);


// --- Funções Auxiliares e de Status ---
void beep(int duration_ms) {
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(BUZZER_PIN, 0);
}

void led_status_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarefa de status dos LEDs iniciada.");
    bool activity_led_state = false;
    while (1) {
        activity_led_state = !activity_led_state;
        switch (g_system_state) {
            case STATE_IDLE:
                gpio_set_level(LED_GREEN_PIN, 0); gpio_set_level(LED_RED_PIN, 0); gpio_set_level(LED_YELLOW_PIN, 0);
                break;
            case STATE_CLIENT_CONNECTED:
                gpio_set_level(LED_GREEN_PIN, 1); gpio_set_level(LED_RED_PIN, 0); gpio_set_level(LED_YELLOW_PIN, 0);
                break;
            case STATE_ACQUIRING_DATA:
                gpio_set_level(LED_GREEN_PIN, 1); 
                gpio_set_level(LED_RED_PIN, activity_led_state); 
                // (FIX-1) LED Amarelo acende se houver erro de perda de buffer
                gpio_set_level(LED_YELLOW_PIN, g_buffer_drop_error); 
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void wifi_led_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarefa do LED de Wi-Fi iniciada.");
    while(1) {
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Wi-Fi conectado, LED azul piscando.");
        while(1) {
            if ((xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) == 0) {
                gpio_set_level(LED_WIFI_PIN, 0);
                ESP_LOGI(TAG, "Wi-Fi desconectado, LED azul apagado.");
                break;
            }
            gpio_set_level(LED_WIFI_PIN, 1); vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_WIFI_PIN, 0); vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_WIFI_PIN, 1); vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_WIFI_PIN, 0); vTaskDelay(pdMS_TO_TICKS(800));
        }
    }
}

// --- Funções de Inicialização ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Desconectado. Tentando reconectar...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta_static_ip(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));
    esp_netif_ip_info_t ip_info;
    inet_pton(AF_INET, STATIC_IP, &ip_info.ip);
    inet_pton(AF_INET, GATEWAY, &ip_info.gw);
    inet_pton(AF_INET, NETMASK, &ip_info.netmask);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS, }, };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi STA iniciado.");
}

void adc_init_voltage() {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT, };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    adc_oneshot_chan_cfg_t chan_cfg = { .atten = ADC_ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT, };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));
}

void initialize_encoder_pcnt(void) {
    ESP_LOGI(TAG, "Inicializando encoder com hardware PCNT...");
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
    ESP_LOGI(TAG, "PCNT iniciado.");
}


// --- TAREFA: AQUISIÇÃO DE DADOS (PRODUTOR) ---
void high_freq_acquisition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarefa de aquisição de alta frequência iniciada.");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(ACQUISITION_PERIOD_MS);
    int raw_adc;
    uint32_t sample_index = 0;
    int64_t start_time_ms = 0;
    int buffer_idx = 0;
    esp_err_t err; // (FIX-3) Variável para checar erros
    
    while(1) {
        // Espera pelo bit de início da aquisição
        xEventGroupWaitBits(acquisition_event_group, ACQUISITION_RUNNING_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        
        ESP_LOGI(TAG, "Aquisição iniciada.");
        sample_index = 0;
        buffer_idx = 0;
        AcquisitionDataPoint* current_buffer = buffer_pool[buffer_idx];
        start_time_ms = esp_timer_get_time() / 1000;
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
        xLastWakeTime = xTaskGetTickCount(); 
        
        while((xEventGroupGetBits(acquisition_event_group) & ACQUISITION_RUNNING_BIT) != 0) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            
            // Lê ADC
            // (FIX-3) Adiciona verificação de erro
            err = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_adc);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Falha ao ler ADC: %s", esp_err_to_name(err));
                // Em um loop de alta freq, podemos optar por continuar ou travar
                // Vamos travar para depuração, como estava acontecendo:
                ESP_ERROR_CHECK(err); 
            }
            current_buffer[sample_index].voltage_raw = (uint16_t)raw_adc;
            
            // Lê Encoder (diferença)
            int pulses_diff = 0;
            // (FIX-3) Adiciona verificação de erro
            err = pcnt_unit_get_count(pcnt_unit, &pulses_diff);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Falha ao ler PCNT: %s", esp_err_to_name(err));
                ESP_ERROR_CHECK(err);
            }
            
            // (FIX-3) Adiciona verificação de erro
            err = pcnt_unit_clear_count(pcnt_unit);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Falha ao limpar PCNT: %s", esp_err_to_name(err));
                ESP_ERROR_CHECK(err);
            }
            current_buffer[sample_index].encoder_diff = (int16_t)pulses_diff;
            
            // Grava Timestamp
            current_buffer[sample_index].timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000 - start_time_ms);
            sample_index++;

            // Se o buffer está cheio, envia para a fila
            if (sample_index >= BUFFER_SIZE_SAMPLES) {
                BufferInfo info_to_send = { .buffer_ptr = current_buffer, .sample_count = BUFFER_SIZE_SAMPLES };
                
                // (FIX-1) Verifica se a fila NÃO está cheia antes de enviar
                BaseType_t result = xQueueSend(stream_queue, &info_to_send, pdMS_TO_TICKS(100));
                if (result != pdPASS) {
                    ESP_LOGE(TAG, "Queue full! Buffer de dados (%" PRIu32 " amostras) perdido!", info_to_send.sample_count);
                    g_buffer_drop_error = true; // Sinaliza o erro
                }

                buffer_idx = (buffer_idx + 1) % NUM_BUFFERS;
                current_buffer = buffer_pool[buffer_idx];
                sample_index = 0;
            }
        }

        // Aquisição parada. Envia o buffer residual (o que sobrou)
        ESP_LOGI(TAG, "Aquisição parada. Enviando buffer residual com %" PRIu32 " amostras.", sample_index);
        if (sample_index > 0) {
            BufferInfo final_info = { .buffer_ptr = current_buffer, .sample_count = sample_index };
            
            // (FIX-1) Verifica também o buffer residual
            BaseType_t result = xQueueSend(stream_queue, &final_info, pdMS_TO_TICKS(100));
            if (result != pdPASS) {
                ESP_LOGE(TAG, "Queue full! Buffer residual (%" PRIu32 " amostras) perdido!", final_info.sample_count);
                g_buffer_drop_error = true;
            }
        }
    }
}


// --- TAREFA: STREAMING DE DADOS VIA TCP (CONSUMIDOR) ---
// (FIX-2) Esta tarefa agora é a ÚNICA que envia dados E mensagens de conclusão.
void tcp_stream_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarefa de streaming TCP iniciada.");
    BufferInfo received_info;
    while(1) {
        // Espera por um item na fila (pode ser um buffer de dados OU uma msg de finalização)
        if(xQueueReceive(stream_queue, &received_info, portMAX_DELAY)) {
            
            // Se o cliente desconectou (g_active_socket = -1), descarta o item da fila
            if (g_active_socket == -1) {
                ESP_LOGW(TAG, "Item da fila recebido, mas sem cliente. Descartando.");
                continue; // Pula para o vTaskDelay no final
            }

            // (FIX-2) Verifica se é uma mensagem de finalização (buffer_ptr == NULL)
            if (received_info.buffer_ptr == NULL) {
                ESP_LOGI(TAG, "Mensagem de finalização recebida. Enviando confirmação final ao cliente.");
                // Todos os dados binários já foram. Agora envia a msg de texto final.
                send(g_active_socket, "OK: Aquisicao concluida.\n", 26, 0);
            } 
            // (FIX-2) Senão, é um buffer de dados normal
            else { 
                int to_write = received_info.sample_count * sizeof(AcquisitionDataPoint);
                int written = 0;
                while (written < to_write) {
                    int ret = send(g_active_socket, (uint8_t*)received_info.buffer_ptr + written, to_write - written, 0);
                    if (ret < 0) {
                        ESP_LOGE(TAG, "Erro durante o envio do stream. Cliente pode ter desconectado.");
                        // O loop principal do servidor (tcp_server_task) vai detectar a desconexão
                        // e setar g_active_socket = -1.
                        break; 
                    }
                    written += ret;
                }
                ESP_LOGD(TAG, "Buffer com %d bytes enviado.", written);
            }
        } // fim do if(xQueueReceive...)

        // (FIX-4) FORÇA UMA TROCA DE CONTEXTO EM CADA LOOP (CRÍTICO!)
        // Esta tarefa (Prio 9) roda no Core 0, assim como o stack TCP/IP (lwIP).
        // Sem este delay, esta tarefa pode ficar 100% ocupada esvaziando a
        // fila e NUNCA dar tempo para a tarefa lwIP (prioridade menor)
        // realmente enviar os pacotes pelo Wi-Fi.
        vTaskDelay(pdMS_TO_TICKS(1)); 
    } // fim do while(1)
}


// --- TAREFA DO SERVIDOR TCP (CONTROLADOR) ---
void tcp_server_task(void *pvParameters) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) { ESP_LOGE(TAG, "Erro ao criar socket. Reiniciando..."); vTaskDelay(pdMS_TO_TICKS(1000)); esp_restart(); }

    struct sockaddr_in dest_addr = { .sin_family = AF_INET, .sin_port = htons(TCP_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) { ESP_LOGE(TAG, "Erro no bind. Reiniciando..."); close(listen_sock); vTaskDelay(pdMS_TO_TICKS(1000)); esp_restart(); }
    if (listen(listen_sock, 1) < 0) { ESP_LOGE(TAG, "Erro no listen. Reiniciando..."); close(listen_sock); vTaskDelay(pdMS_TO_TICKS(1000)); esp_restart(); }

    while (1) {
        g_system_state = STATE_IDLE;
        g_active_socket = -1;
        ESP_LOGI(TAG, "Servidor aguardando nova conexão na porta %d...", TCP_PORT);
        int sock = accept(listen_sock, NULL, NULL);
        if (sock < 0) { ESP_LOGW(TAG, "Accept falhou."); continue; }
        
        ESP_LOGI(TAG, "Cliente conectado!");
        g_active_socket = sock;
        g_system_state = STATE_CLIENT_CONNECTED;

        // Loop de tratamento do cliente conectado
        while (1) {
            char rx_buffer[128];
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len <= 0) {
                ESP_LOGI(TAG, "Cliente desconectou.");
                xEventGroupClearBits(acquisition_event_group, ACQUISITION_RUNNING_BIT);
                break; // Sai do loop do cliente e espera uma nova conexão
            }
            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "Comando recebido: %s", rx_buffer);
            
            if (strncmp(rx_buffer, "INICIAR,", 8) == 0) {
                int duration = 0;
                if (sscanf(rx_buffer, "INICIAR,%d", &duration) != 1 || duration <= 0) {
                    send(sock, "ERRO: Comando INICIAR mal formatado. Use INICIAR,<segundos>\n", 60, 0);
                } else {
                    char msg[100];
                    sprintf(msg, "OK: Iniciando aquisição por %d segundos...\n", duration);
                    send(sock, msg, strlen(msg), 0); // Envia confirmação de início
                    
                    g_buffer_drop_error = false; // (FIX-1) Reseta a flag de erro
                    xQueueReset(stream_queue);   // Limpa a fila de qualquer lixo anterior
                    g_system_state = STATE_ACQUIRING_DATA;
                    xEventGroupSetBits(acquisition_event_group, ACQUISITION_RUNNING_BIT); // Inicia a tarefa de aquisição
                    beep(500); // Beep de início

                    ESP_LOGI(TAG, "Aquisição temporizada iniciada por %d segundos.", duration);
                    vTaskDelay(pdMS_TO_TICKS(duration * 1000));
                    
                    ESP_LOGI(TAG, "Tempo esgotado. Finalizando aquisição...");
                    xEventGroupClearBits(acquisition_event_group, ACQUISITION_RUNNING_BIT); // Para a tarefa de aquisição
                    g_system_state = STATE_CLIENT_CONNECTED;
                    
                    // (FIX-2) Espera um tempo para a 'high_freq_acquisition_task'
                    //         enviar seu buffer residual para a FILA.
                    vTaskDelay(pdMS_TO_TICKS(500)); 
                    
                    // (FIX-2) NÃO envia a msg de conclusão aqui.
                    // send(sock, "OK: Aquisicao concluida.\n", 26, 0); // <-- REMOVIDO
                    
                    // (FIX-2) Envia uma mensagem de finalização (ptr NULL) para a FILA.
                    //         A 'tcp_stream_task' vai ler isso e enviar a msg de texto final.
                    ESP_LOGI(TAG, "Enviando comando de finalização para a fila de stream.");
                    BufferInfo finalization_msg = { .buffer_ptr = NULL, .sample_count = 0 };
                    xQueueSend(stream_queue, &finalization_msg, pdMS_TO_TICKS(1000)); 

                    // Beep de finalização
                    beep(300); vTaskDelay(pdMS_TO_TICKS(50)); beep(300);
                }
            }
            else if (strncmp(rx_buffer, "STATUS", 6) == 0) {
                send(sock, "ESP32 CONECTADO E PRONTO\n", 27, 0);
            }
            else {
                send(sock, "ERRO: Comando desconhecido\n", 27, 0);
            }
        }
        // Fim do loop do cliente
        g_active_socket = -1;
        close(sock);
    }
}


void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();
    acquisition_event_group = xEventGroupCreate();
    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
    // Reconfigura a UART 0
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));


    // (FIX-2) A fila agora precisa de espaço para a msg de finalização (NULL)
    stream_queue = xQueueCreate(NUM_BUFFERS + 1, sizeof(BufferInfo)); 

    adc_init_voltage();
    initialize_encoder_pcnt();
    
    gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_YELLOW_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_WIFI_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    wifi_init_sta_static_ip();
    ESP_LOGI(TAG, "Aguardando conexão Wi-Fi...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi conectado com sucesso!");

    // Cria as tarefas
    xTaskCreatePinnedToCore(high_freq_acquisition_task, "acquisition_task", 4096, NULL, 10, NULL, 1); // Core 1 (Produtor)
    xTaskCreatePinnedToCore(tcp_stream_task, "tcp_stream_task", 4096, NULL, 9, NULL, 0);             // Core 0 (Consumidor)
    xTaskCreate(led_status_task, "led_status_task", 2048, NULL, 5, NULL);
    xTaskCreate(wifi_led_task, "wifi_led_task", 2048, NULL, 5, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 6, NULL);                                 // Core 0 (Controlador)

    ESP_LOGI(TAG, "app_main finalizada. O sistema está rodando.");
}

