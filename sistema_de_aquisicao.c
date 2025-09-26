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

#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_task_wdt.h"


// --- Configurações da Aplicação ---
// Wi-Fi e Rede
#define WIFI_SSID       "RoteadorESP32"
#define WIFI_PASS       "teste123"
#define STATIC_IP       "192.168.0.199"
#define GATEWAY         "192.168.0.1"
#define NETMASK         "255.255.255.0"
#define TCP_PORT        4343

// ADC
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_3
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_SENSE       2.0f // Fator de multiplicação para divisor de tensão (ex: 2.0 para 1/2)

// Encoder
#define ENCODER_A_PIN   GPIO_NUM_17
#define ENCODER_B_PIN   GPIO_NUM_16
#define PPR             12      // Pulsos por revolução do encoder
#define QUADRATURE      4       // Contagem em quadratura completa
#define FILTER_DEPTH 5

volatile float g_filtered_speed_hz = 0.0f; // Variável para comunicação entre as tarefas


// SD Card
#define SD_MOUNT_POINT  "/sdcard"

// --- Variáveis Globais ---
static const char *TAG = "ESP32_APP";
static adc_oneshot_unit_handle_t adc_handle;

// Event Group para sinalizar eventos de Wi-Fi
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Variáveis globais para a leitura do encoder
volatile int32_t g_pulse_count = 0;

// --- DEFINIÇÕES PARA OS LEDs DE STATUS ---
#define LED_GREEN_PIN   GPIO_NUM_25
#define LED_RED_PIN     GPIO_NUM_26
#define LED_YELLOW_PIN  GPIO_NUM_27
#define LED_WIFI_PIN    GPIO_NUM_33

// Enum para deixar nosso código de estados mais legível
typedef enum {
    STATE_IDLE,                 // Ninguém conectado
    STATE_CLIENT_CONNECTED,     // Cliente conectado, aguardando comando
    STATE_ACQUIRING_DATA,       // Gravando dados no SD card
    STATE_SENDING_FILE          // Enviando arquivo para o cliente
} system_state_t;

// Variável global para comunicar o estado entre as tarefas
volatile system_state_t g_system_state = STATE_IDLE;

void led_status_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarefa de status dos LEDs iniciada.");
    bool activity_led_state = false; // Usado para piscar os LEDs de atividade (vermelho/amarelo)

    while (1) {
        activity_led_state = !activity_led_state; // Inverte o estado do pisca-pisca a cada ciclo

        switch (g_system_state) {
            case STATE_IDLE:
                // Nenhum cliente conectado, tudo apagado.
                gpio_set_level(LED_GREEN_PIN, 0);
                gpio_set_level(LED_RED_PIN, 0);
                gpio_set_level(LED_YELLOW_PIN, 0);
                break;

            case STATE_CLIENT_CONNECTED:
                // Cliente conectado e ocioso: Verde fixo.
                gpio_set_level(LED_GREEN_PIN, 1);
                gpio_set_level(LED_RED_PIN, 0);
                gpio_set_level(LED_YELLOW_PIN, 0);
                break;

            case STATE_ACQUIRING_DATA:
                // Conectado (verde fixo) e adquirindo (vermelho piscando).
                gpio_set_level(LED_GREEN_PIN, 1);
                gpio_set_level(LED_RED_PIN, activity_led_state);
                gpio_set_level(LED_YELLOW_PIN, 0);
                break;

            case STATE_SENDING_FILE:
                // Conectado (verde fixo) e enviando (amarelo piscando).
                gpio_set_level(LED_GREEN_PIN, 1);
                gpio_set_level(LED_RED_PIN, 0);
                gpio_set_level(LED_YELLOW_PIN, activity_led_state);
                break;
        }

        // O delay do loop controla a velocidade do "pisca-pisca" dos LEDs de atividade.
        // 250ms resulta em uma piscada a cada meio segundo (2Hz).
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void wifi_led_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarefa do LED de Wi-Fi iniciada.");

    while(1) {
        // A tarefa fica bloqueada aqui esperando o bit de "Wi-Fi Conectado"
        // Ela não consome quase nada de CPU enquanto espera.
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        // Se chegou aqui, o Wi-Fi conectou. Começa a piscar.
        ESP_LOGI(TAG, "Wi-Fi conectado, LED azul piscando.");
        while(1) {
            // Verifica a cada piscada se a conexão ainda está ativa
            if ((xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) == 0) {
                // Se a conexão caiu, apaga o LED e sai do loop de piscar
                gpio_set_level(LED_WIFI_PIN, 0);
                ESP_LOGI(TAG, "Wi-Fi desconectado, LED azul apagado.");
                break;
            }
            // Pisca o LED
            gpio_set_level(LED_WIFI_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_WIFI_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_WIFI_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_WIFI_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}


// --- Funções de Inicialização e Handlers ---

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // >>>>> MUDANÇA IMPORTANTE AQUI <<<<<
        // Limpa o bit para avisar às outras tarefas que a conexão caiu
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Desconectado. Tentando reconectar...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // Sinaliza que a conexão foi bem sucedida
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta_static_ip(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    
    // Garante que o cliente DHCP está parado para usar IP estático
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));

    // Configura o IP estático
    esp_netif_ip_info_t ip_info;
    inet_pton(AF_INET, STATIC_IP, &ip_info.ip);
    inet_pton(AF_INET, GATEWAY, &ip_info.gw);
    inet_pton(AF_INET, NETMASK, &ip_info.netmask);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Registra os handlers de evento
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Wi-Fi STA iniciado, aguardando conexão...");
}

void adc_init_voltage() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));
}

esp_err_t sdcard_init(void) {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_23,
        .miso_io_num = GPIO_NUM_19,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_5;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Falha ao montar SD via SPI: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

// --- Funções de Leitura e Escrita ---

void write_csv_line(FILE *f, const char *fmt, ...) {
    if (!f) return;
    va_list args;
    va_start(args, fmt);
    vfprintf(f, fmt, args);
    va_end(args);
    fprintf(f, "\n");
}

float read_adc_voltage() {
    int raw;
    adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
    // Tensão de referência ~3.3V, resolução de 12-bit (4096 níveis)
    return (float)raw * (3.3f / 4095.0f) * ADC_SENSE;
}


void encoder_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarefa do Encoder iniciada.");

    // Variáveis para o cálculo instantâneo
    int32_t last_pulse_count = 0;
    int64_t last_time_ms = 0;

    // Variáveis para o filtro de média móvel
    // const int FILTER_DEPTH = 5;
    float speed_samples[FILTER_DEPTH] = {0.0f};
    int sample_index = 0;
    
    // Variáveis para a leitura dos pinos
    int last_A = gpio_get_level(ENCODER_A_PIN);
    int last_B = gpio_get_level(ENCODER_B_PIN);

    // Inicializa o tempo para a primeira iteração
    last_time_ms = esp_timer_get_time() / 1000;

    while (1) {
        // --- 1. Lógica de Leitura de Pulsos ---
        int current_A = gpio_get_level(ENCODER_A_PIN);
        int current_B = gpio_get_level(ENCODER_B_PIN);

        if (current_A != last_A || current_B != last_B) {
            if (last_A == 0 && current_A == 1) {
                g_pulse_count += (current_B == 0) ? 1 : -1;
            } else if (last_A == 1 && current_A == 0) {
                g_pulse_count += (current_B == 1) ? 1 : -1;
            }
            last_A = current_A;
            last_B = current_B;
        }

        // --- 2. Lógica de Cálculo de Velocidade INSTANTÂNEA ---
        int64_t now_ms = esp_timer_get_time() / 1000;
        int32_t pulses_diff = g_pulse_count - last_pulse_count;
        int64_t dt_ms = now_ms - last_time_ms;

        // Atualiza o estado para a próxima iteração
        last_pulse_count = g_pulse_count;
        last_time_ms = now_ms;
        
        float current_hz = 0.0f;
        if (dt_ms > 0) {
            float revolutions = (float)pulses_diff / (PPR * QUADRATURE);
            // Calcula a velocidade instantânea (pode ser ruidosa)
            current_hz = fabsf((revolutions * 1000.0f) / dt_ms);
        }

        // --- 3. Adiciona a leitura instantânea ao filtro ---
        speed_samples[sample_index] = current_hz;
        sample_index = (sample_index + 1) % FILTER_DEPTH; // Avança o índice circular
        
        // --- 4. Calcula a média e atualiza a variável global ---
        float average_hz = 0.0f;
        for (int i = 0; i < FILTER_DEPTH; i++) {
            average_hz += speed_samples[i];
        }
        g_filtered_speed_hz = average_hz / FILTER_DEPTH;

        // --- 5. Define a frequência desta tarefa ---
        vTaskDelay(pdMS_TO_TICKS(10)); // Roda a cada 10ms
    }
}




// --- Task Principal do Servidor TCP (VERSÃO COM CONEXÃO PERSISTENTE) ---

void tcp_server_task(void *pvParameters) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Erro ao criar socket. Reiniciando...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE(TAG, "Erro no bind. Reiniciando...");
        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Erro no listen. Reiniciando...");
        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    while (1) {
        g_system_state = STATE_IDLE; // Define o estado inicial como Ocioso
        ESP_LOGI(TAG, "Servidor aguardando nova conexão na porta %d...", TCP_PORT);

        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Erro no accept. Continuando...");
            continue;
        }
        ESP_LOGI(TAG, "Cliente conectado!");
        g_system_state = STATE_CLIENT_CONNECTED;

        // Loop de conversa com o cliente conectado
        while (1) {
            
            char rx_buffer[128];
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            // Detecta se o cliente desconectou
            if (len <= 0) {
                if (len == 0) {
                    ESP_LOGI(TAG, "Cliente desconectou de forma limpa.");
                } else {
                    ESP_LOGE(TAG, "Erro no recv, cliente desconectou de forma abrupta. errno: %d", errno);
                }
                break; // Sai do loop interno para esperar um novo cliente
            }

            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "Comando recebido: %s", rx_buffer);
            
            // --- Comando INICIAR ---
            if (strncmp(rx_buffer, "INICIAR", 7) == 0) {
                g_system_state = STATE_ACQUIRING_DATA;
                int duration = 0;
                // Valida o formato do comando, ex: "INICIAR,10"
                if (sscanf(rx_buffer, "INICIAR,%d", &duration) != 1 || duration <= 0) {
                    send(sock, "ERRO: Comando INICIAR mal formatado\n", 36, 0);
                } else {
                    // Inscreve esta tarefa no watchdog, pois a aquisição é uma operação longa.
                    ESP_LOGI(TAG, "Inscrevendo tarefa no Watchdog para aquisição...");
                    esp_task_wdt_add(NULL);

                    // Envia a primeira confirmação para o cliente Python
                    send(sock, "OK: Comando recebido, iniciando preparativos...\n", 48, 0);
                    ESP_LOGI(TAG, "Preparando para aquisição de %d segundos...", duration);

                    FILE *f = fopen(SD_MOUNT_POINT"/data.csv", "w");
                    if (!f) {
                        ESP_LOGE(TAG, "Falha ao abrir o arquivo no SD card.");
                        // Se falhar, envia um erro como a "segunda resposta"
                        send(sock, "ERRO: Falha ao abrir o arquivo no SD\n", 39, 0);
                    } else {
                        ESP_LOGI(TAG, "Arquivo aberto. Iniciando aquisição.");
                        write_csv_line(f, "tempo_ms,tensao_V,rotacao_hz");
                        int64_t start_ms = esp_timer_get_time() / 1000;
                        
                        // Loop principal da aquisição de dados
                        while ((esp_timer_get_time() / 1000 - start_ms) < (duration * 1000)) {
                            esp_task_wdt_reset(); // Alimenta o watchdog a cada ciclo
                            
                            // --- Aquisição Simplificada ---
                            int64_t t_ms = esp_timer_get_time() / 1000 - start_ms;
                            float v = read_adc_voltage();
                            
                            // Pega o valor de velocidade mais recente, já calculado e filtrado pela encoder_task
                            float r_hz = g_filtered_speed_hz;

                            write_csv_line(f, "%" PRIu64 ",%.3f,%.3f", t_ms, v, r_hz);

                            // Define a taxa de amostragem da gravação no CSV (50Hz)
                            vTaskDelay(pdMS_TO_TICKS(20));
                        }
                        
                        ESP_LOGI(TAG, "Aquisição de dados finalizada. Salvando dados no cartão SD...");
                        esp_task_wdt_reset(); // Alimenta o watchdog antes das operações lentas de arquivo
                        fflush(f);
                        fclose(f);
                        
                        ESP_LOGI(TAG, "Arquivo salvo com sucesso. Enviando confirmação final.");
                        // Envia a segunda e última resposta para o cliente
                        send(sock, "OK: Aquisicao concluida\n", 26, 0);
                    }

                    // Remove a inscrição do watchdog, pois a operação de risco terminou.
                    ESP_LOGI(TAG, "Removendo tarefa do Watchdog.");
                    esp_task_wdt_delete(NULL);
                }
                g_system_state = STATE_CLIENT_CONNECTED;
            }
            // --- Comando STATUS ---
            else if (strncmp(rx_buffer, "STATUS", 6) == 0) {
                send(sock, "ESP32 CONECTADO\n", 18, 0);
            }
            // --- Comando RESET ---
            else if (strncmp(rx_buffer, "RESET", 5) == 0) {
                send(sock, "Resetando...\n", 13, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
            // --- Comando GET_FILE ---
            else if (strncmp(rx_buffer, "GET_FILE", 8) == 0) {
                g_system_state = STATE_SENDING_FILE;
                esp_task_wdt_add(NULL);

                FILE *f = fopen(SD_MOUNT_POINT"/data.csv", "r");
                if (!f) {
                    send(sock, "ERRO: Arquivo nao encontrado\n", 30, 0);
                } else {
                    ESP_LOGI(TAG, "Enviando arquivo para o cliente...");
                    esp_task_wdt_reset();
                    
                    char file_buffer[2048];
                    size_t bytes_read;
                    while ((bytes_read = fread(file_buffer, 1, sizeof(file_buffer), f)) > 0) {
                        if (send(sock, file_buffer, bytes_read, 0) < 0) {
                            ESP_LOGE(TAG, "Erro ao enviar chunk do arquivo.");
                            break;
                        }
                        esp_task_wdt_reset();
                            vTaskDelay(1);
                    }
                    fclose(f);
                    ESP_LOGI(TAG, "Envio de arquivo concluído. Enviando marcador EOF.");

                    // >>>>> MUDANÇA IMPORTANTE: Envia o sinal de Fim de Arquivo <<<<<
                    send(sock, "\nEOF\n", 5, 0);
                }

                esp_task_wdt_delete(NULL);
                g_system_state = STATE_CLIENT_CONNECTED;
            }
            // --- Comando desconhecido ---
            else {
                send(sock, "ERRO: Comando desconhecido\n", 27, 0);
            }
        }
        
        // Fecha o socket do cliente que desconectou
        close(sock);
    }
}

// --- Função Principal ---

void app_main(void) {
    // 1. Inicializa o NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // >>>>> MUDANÇA FINAL E DEFINITIVA <<<<<
    // Reconfigura o Watchdog já existente para ter um timeout mais longo (30 segundos)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 30000,
        .trigger_panic = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));
    ESP_LOGI(TAG, "Task Watchdog reconfigurado para 30 segundos.");


    // 2. Cria o Event Group para gerenciar eventos do Wi-Fi
    s_wifi_event_group = xEventGroupCreate();

    // 3. Inicializa os periféricos
    adc_init_voltage();

    if(sdcard_init() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar SD. Travando a aplicação.");
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_A_PIN) | (1ULL << ENCODER_B_PIN), 
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_config_t io_conf_leds = {
        .pin_bit_mask = (1ULL << LED_GREEN_PIN) | (1ULL << LED_RED_PIN) | (1ULL << LED_YELLOW_PIN) | (1ULL << LED_WIFI_PIN), 
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_leds);
    
    // 4. Inicia o Wi-Fi
    wifi_init_sta_static_ip();

    // 5. AGUARDA a conexão Wi-Fi ser estabelecida antes de continuar
    ESP_LOGI(TAG, "Aguardando conexão Wi-Fi ser estabelecida...");
    xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT,
                        pdFALSE,
                        pdFALSE,
                        portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi conectado com sucesso!");

    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 10, NULL);
    // >>>>> INICIA A NOVA TAREFA DOS LEDS <<<<<
    xTaskCreate(led_status_task, "led_status_task", 2048, NULL, 5, NULL);
    xTaskCreate(wifi_led_task, "wifi_led_task", 2048, NULL, 5, NULL);
    // 6. Agora é seguro criar a task do servidor TCP
    xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 5, NULL);
}
