# Sistema de Aquisição de Dados com ESP32 via Wi-Fi

Este projeto transforma um ESP32 em um robusto dispositivo de aquisição de dados, capaz de medir simultaneamente a tensão de um sensor analógico e a velocidade de rotação de um encoder de quadratura com alta precisão. Os dados coletados são armazenados em um cartão SD e podem ser controlados e transferidos remotamente através de uma conexão Wi-Fi por meio de um servidor TCP.

O sistema é projetado para ser confiável, utilizando FreeRTOS para gerenciamento de tarefas concorrentes e o hardware Pulse Counter (PCNT) do ESP32 para uma leitura precisa do encoder, sem sobrecarregar a CPU principal.

## ✨ Funcionalidades Principais

-   **Aquisição de Dados Simultânea**: Leitura de um canal ADC (tensão) e de um encoder de quadratura (velocidade de rotação).
-   **Leitura de Encoder de Alta Precisão**: Utiliza o periférico de hardware **Pulse Counter (PCNT)** do ESP32 para decodificar o encoder, garantindo precisão mesmo em altas velocidades.
-   **Armazenamento Local**: Salva os dados em formato CSV (`data.csv`) em um cartão MicroSD.
-   **Conectividade Wi-Fi**: Conecta-se a uma rede Wi-Fi com configuração de IP estático para acesso estável.
-   **Servidor TCP para Controle Remoto**: Permite que um cliente (como um script Python) inicie a aquisição, verifique o status e baixe o arquivo de dados.
-   **Feedback Visual e Sonoro**: Utiliza múltiplos LEDs e um buzzer para fornecer feedback claro sobre o estado do sistema (conectado, gravando, enviando dados, etc.).
-   **Arquitetura Multitarefa**: Baseado em FreeRTOS, com tarefas dedicadas para o servidor TCP, leitura do encoder e controle dos LEDs de status.
-   **Proteção com Watchdog**: Implementa o Task Watchdog Timer para reiniciar o sistema caso uma tarefa crítica (como a gravação ou envio de arquivos) trave.

## 🛠️ Hardware Necessário

-   Placa de desenvolvimento ESP32
-   Encoder Rotativo de Quadratura (configurado no código para 600 PPR)
-   Módulo para Cartão MicroSD (interface SPI)
-   Cartão MicroSD
-   Sensor analógico (para medição de tensão)
-   LEDs:
    -   1x Verde (Status de Conexão do Cliente)
    -   1x Vermelho (Status de Aquisição)
    -   1x Amarelo (Status de Envio de Arquivo)
    -   1x Azul (Status da Conexão Wi-Fi)
-   1x Buzzer
-   Resistores apropriados para os LEDs

## 🔌 Pinagem (Conexões)

As conexões de hardware estão definidas no início do código. A configuração padrão é:

| Periférico      | Pino do ESP32 | Descrição                  |
| --------------- | ------------- | -------------------------- |
| **Encoder** | GPIO 17       | Canal A                    |
|                 | GPIO 16       | Canal B                    |
| **ADC** | GPIO 39       | Canal ADC1\_CH3 (Padrão)   |
| **Cartão SD** | GPIO 23       | MOSI                       |
| (SPI)           | GPIO 19       | MISO                       |
|                 | GPIO 18       | SCLK                       |
|                 | GPIO 5        | CS (Chip Select)           |
| **LEDs** | GPIO 25       | LED Verde                  |
|                 | GPIO 26       | LED Vermelho               |
|                 | GPIO 27       | LED Amarelo                |
|                 | GPIO 33       | LED Wi-Fi (Azul)           |
| **Buzzer** | GPIO 14       | Saída para o Buzzer        |

## ⚙️ Configuração e Compilação

Este projeto foi desenvolvido utilizando o framework **ESP-IDF**.

1.  **Clone o repositório:**
    ```bash
    git clone [URL_DO_SEU_REPOSITORIO]
    cd [NOME_DO_DIRETORIO]
    ```

2.  **Configure os parâmetros**:
    Abra o arquivo `main.c` e ajuste as definições na seção `// --- Configurações da Aplicação ---` de acordo com sua necessidade:
    -   **Rede Wi-Fi**: `WIFI_SSID` e `WIFI_PASS`.
    -   **IP Estático**: `STATIC_IP`, `GATEWAY` e `NETMASK`.
    -   **Porta TCP**: `TCP_PORT`.
    -   **Parâmetros do Encoder**: `PPR` (Pulsos Por Revolução).

3.  **Compile e grave no ESP32**:
    Use os comandos do ESP-IDF:
    ```bash
    idf.py set-target esp32
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor
    ```
    (Substitua `/dev/ttyUSB0` pela porta serial correta).

## 📡 Protocolo de Comunicação TCP

Após conectar-se à rede Wi-Fi, o ESP32 iniciará um servidor TCP no IP e porta configurados (padrão: `192.168.0.199:4343`). Você pode se conectar a ele usando um cliente TCP (como Netcat, ou um script Python) para enviar os seguintes comandos:

| Comando                  | Descrição                                                                                                                                                                 | Resposta do ESP32                                                                |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `INICIAR,<segundos>`     | Inicia a aquisição de dados pela duração especificada em `segundos`. Ex: `INICIAR,10` inicia uma gravação de 10 segundos.                                                   | `OK: Comando recebido...` (imediata) e `OK: Aquisicao concluida` (ao finalizar). |
| `GET_FILE`               | Solicita o envio do arquivo `data.csv` armazenado no cartão SD. O final da transmissão é marcado pela string `\nEOF\n`.                                                     | O conteúdo do arquivo `data.csv`, seguido por `\nEOF\n`.                         |
| `STATUS`                 | Verifica se o dispositivo está online e respondendo.                                                                                                                      | `ESP32 CONECTADO\n`                                                              |
| `RESET`                  | Envia uma resposta de confirmação e reinicia o ESP32.                                                                                                                     | `Resetando...\n`                                                                 |
| _(Comando inválido)_     | Se um comando não reconhecido for enviado.                                                                                                                                | `ERRO: Comando desconhecido\n`                                                   |

## 🚦 LEDs de Status e Buzzer

O sistema fornece feedback claro sobre seu estado atual:

-   **LED Wi-Fi (Azul)**: Pisca em um padrão específico quando o ESP32 está conectado ao Wi-Fi. Fica apagado se a conexão cair.
-   **LED Verde**: Fica aceso continuamente enquanto um cliente TCP estiver conectado.
-   **LED Vermelho**: Pisca enquanto a aquisição de dados (`INICIAR`) estiver em andamento.
-   **LED Amarelo**: Pisca enquanto o arquivo de dados (`GET_FILE`) estiver sendo enviado ao cliente.
-   **Buzzer**:
    -   Emite um **bip longo** ao iniciar uma aquisição.
    -   Emite **dois bipes curtos** ao finalizar uma aquisição.
