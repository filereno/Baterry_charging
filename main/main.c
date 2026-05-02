/*
 * ==============================================================================
 * ARQUIVO: main.c
 * PROJETO: TEST_BATERRY — Carregador/Testador de Baterias NiCd
 *
 * COMPONENTES USADOS:
 * - pwm_esp      : Driver PWM síncrono (MCPWM) para o buck converter
 * - controle_pid : Controlador PID de corrente
 *
 * CANAIS ADS1115 (I2C, endereço 0x48):
 * AIN0 → Tensão da bateria (via divisor resistivo)
 * AIN1 → Corrente de carga (via shunt + amplificador)
 * AIN2 → Temperatura NTC 10kΩ (via divisor resistivo)
 * ==============================================================================
 */

#include <stdio.h>               // Biblioteca padrão de entrada e saída (standard I/O).
#include <math.h>                // Biblioteca matemática, necessária para a função logf().
#include <stdbool.h>             // Biblioteca que permite a utilização do tipo de dados booleano (true/false).
#include "freertos/FreeRTOS.h"   // Biblioteca principal do núcleo do sistema operativo FreeRTOS.
#include "freertos/task.h"       // Biblioteca do FreeRTOS para gestão de tarefas e atrasos (como vTaskDelay).
#include "driver/gpio.h"         // Biblioteca de drivers do ESP32 para controlo dos pinos de entrada e saída (GPIO).
#include "driver/i2c.h"          // Biblioteca de drivers do ESP32 para comunicação pelo barramento I2C.
#include "esp_log.h"             // Biblioteca do ESP32 para impressão de mensagens de registo (logs) na consola.

// Componentes 
// customizados                  // (Comentário original mantido).
#include "pwm_esp.h"             // Inclusão do cabeçalho do módulo personalizado de controlo PWM.
#include "controle_pid.h"        // Inclusão do cabeçalho do módulo personalizado de controlo PID.

static const char *TAG = "CARREGADOR"; // Definição de uma string constante para identificar os logs deste ficheiro no terminal.

/* ============================================================================
 * PINOS
 * ============================================================================ */
#define PINO_RELE         GPIO_NUM_18   // Define o pino GPIO 18 para controlar o relé de segurança.
#define PINO_PWM_ALTO     GPIO_NUM_19   // Gate IRF9530 (high-side P-ch) // Define o pino GPIO 19 para o sinal PWM da parte alta.
#define PINO_PWM_BAIXO    GPIO_NUM_5    // Gate IRF640  (low-side  N-ch) // Define o pino GPIO 5 para o sinal PWM da parte baixa.
#define PINO_SDA          GPIO_NUM_21   // Define o pino GPIO 21 como linha de dados (SDA) do barramento I2C.
#define PINO_SCL          GPIO_NUM_22   // Define o pino GPIO 22 como linha de relógio (SCL) do barramento I2C.

/* ============================================================================
 * I2C / ADS1115
 * ============================================================================ */
#define I2C_MASTER_NUM    I2C_NUM_0     // Define a utilização do controlador I2C número 0 do ESP32.
#define I2C_FREQ_HZ       100000        // Define a frequência do barramento I2C para 100 kHz (modo standard).
#define ADS1115_ADDR      0x48          // ADDR → GND         // Define o endereço I2C do conversor analógico-digital ADS1115.

// FSR ±2.048V → 1 LSB = 62.5 µV
#define ADS1115_LSB_V     0.0000625f    // Define o valor em Volts de cada bit do ADC (resolução do sensor).

/* ============================================================================
 * FATORES DE CONVERSÃO DOS SENSORES
 *
 * *** AJUSTE ESTES VALORES CONFORME SEU HARDWARE ***
 *
 * TENSÃO (AIN0):
 * Divisor com R1=10kΩ, R2=10kΩ → V_bat = V_adc × 2.0
 *
 * CORRENTE (AIN1):
 * Shunt de 0.1Ω com amplificador de ganho 10×
 * I = V_adc / (R_shunt × ganho) = V_adc / 1.0
 *
 * TEMPERATURA (AIN2):
 * NTC 10kΩ (Beta=3950) com R_série=10kΩ, VCC=3.3V
 * ============================================================================ */
#define SENSOR_V_DIVISOR    2.0f        // Fator do divisor de tensão // Multiplicador para reverter a divisão de tensão da bateria.
#define SENSOR_SHUNT_OHM    0.1f        // Resistência do shunt (Ω)   // Valor ohmico do resistor de shunt para cálculo de corrente.
#define SENSOR_AMP_GANHO    10.0f       // Ganho do amp de corrente   // Ganho do amplificador operacional acoplado ao shunt.
#define SENSOR_NTC_BETA     3950.0f     // Coeficiente Beta do termistor NTC para a equação de Steinhart-Hart.
#define SENSOR_NTC_R25      10000.0f    // Resistência NTC a 25°C (Ω) // Resistência nominal do NTC à temperatura ambiente padrão.
#define SENSOR_R_SERIE      10000.0f    // Resistor série do divisor NTC (Ω) // Valor do resistor fixo no divisor de tensão do NTC.
#define SENSOR_VCC          3.3f        // Tensão do divisor NTC // Tensão de alimentação do circuito divisor do NTC em Volts.

/* ============================================================================
 * PARÂMETROS DE CARGA NiCd
 * ============================================================================ */
#define CORRENTE_ALVO       0.8f        // Corrente de carga em amperes // Define o limite de corrente (setpoint) que o PID vai tentar alcançar.
#define DELTA_V_THRESHOLD   0.015f      // Queda de tensão para fim de carga (V) // Define a queda de tensão (15mV) que indica que a bateria está cheia.
#define MAX_TEMP_CELSIUS    45.0f       // Temperatura de corte (°C) // Limite térmico máximo permitido antes de o sistema interromper o processo.
#define V_BAT_MINIMA        0.5f        // Tensão mínima para detectar bateria // Tensão abaixo da qual o sistema ignora a bateria.
#define V_BAT_MAXIMA        1.8f        // Tensão máxima para iniciar carga // Tensão limite aceitável de uma única célula para permitir a carga.
#define V_INVERTIDA         -0.1f       // Limiar de polaridade invertida // Tensão negativa que aciona o erro de bateria ligada ao contrário.
#define V_DESCONECTADA      0.2f        // Abaixo disso: bateria removida // Tensão de referência que sinaliza a ausência física da bateria.

/* ============================================================================
 * MÁQUINA DE ESTADOS
 * ============================================================================ */
typedef enum {                          // Criação de um tipo enumerado (enum) para listar os estados possíveis do carregador.
    DESCONECTADO,                       // Estado de repouso, sistema à espera.
    AVALIANDO,                          // Estado transitório de verificação de segurança antes de ligar a corrente.
    CARREGANDO,                         // Estado ativo onde o PWM e o PID estão a enviar energia para a bateria.
    ERRO_INVERTIDA,                     // Estado de proteção bloqueando o sistema devido a bateria invertida.
    CARGA_COMPLETA                      // Estado de conclusão do processo.
} EstadoSistema;                        // Nome dado a este tipo enumerado recém-criado.

static EstadoSistema estadoAtual = DESCONECTADO; // Declaração da variável que guarda o estado atual, começando sempre como DESCONECTADO.

/* Medições globais */
static float tensaoBateria   = 0.0f;    // Variável que guardará a última leitura da tensão da bateria.
static float tensaoMaxima    = 0.0f;    // Variável que registará o pico máximo de tensão (usado para o cálculo do Delta-V).
static float correnteCarga   = 0.0f;    // Variável que guardará a última leitura da corrente de carga calculada.
static float temperaturaAtual = 0.0f;   // Variável que guardará a última leitura da temperatura em graus Celsius.
static float dutyCycleAtual  = 0.0f;    // Variável que guardará o valor atual da potência do PWM (de 0 a 100).

/* ============================================================================
 * I2C / ADS1115
 * ============================================================================ */
static void init_i2c(void) {                    // Início da função que configura e inicializa o barramento I2C.
    i2c_config_t conf = {                       // Declaração de uma estrutura de configuração do I2C.
        .mode             = I2C_MODE_MASTER,    // Define o ESP32 como mestre no barramento I2C.
        .sda_io_num       = PINO_SDA,           // Associa o pino SDA físico à configuração do I2C.
        .scl_io_num       = PINO_SCL,           // Associa o pino SCL físico à configuração do I2C.
        .sda_pullup_en    = GPIO_PULLUP_ENABLE, // Ativa a resistência pull-up interna no pino SDA.
        .scl_pullup_en    = GPIO_PULLUP_ENABLE, // Ativa a resistência pull-up interna no pino SCL.
        .master.clk_speed = I2C_FREQ_HZ,        // Define a velocidade do relógio (clock) usando a macro de 100kHz.
    };                                         
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));                   // Aplica as configurações ao controlador I2C e verifica se houve erros de hardware.
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));    // Instala o driver I2C no sistema operativo do ESP32.
    ESP_LOGI(TAG, "I2C pronto: SDA=%d SCL=%d", PINO_SDA, PINO_SCL);             // Imprime uma mensagem de sucesso no terminal com os números dos pinos.
}   

static int16_t ler_ads1115(uint8_t canal) { // Início da função que lê valores do ADC, recebendo como parâmetro o número do canal desejado.
    uint8_t mux;                            // Declaração de uma variável para armazenar o byte de configuração do multiplexador.
    switch (canal) {                        // Início do bloco condicional para escolher o canal de leitura.
        case 0:  mux = 0xC3; break;         // Se o canal for 0, configura o MUX para ler AIN0 em relação ao GND.
    // AIN0 vs GND, PGA=±2.048V, single-shot 
        case 1:  mux = 0xD3; break;         // Se o canal for 1, configura o MUX para ler AIN1 em relação ao GND.
    // AIN1 vs GND                          
        default: mux = 0xE3; break;         // Para qualquer outro canal, configura o MUX para ler AIN2 em relação ao GND.
    // AIN2 vs GND                          
    }                                       
    uint8_t cfg[3] = { 0x01, mux, 0x83 };   // Cria um array de 3 bytes: Byte 0 (aponta para registo de config), Byte 1 (MUX selecionado), Byte 2 (velocidade).
    // 0x83 = 860 SPS                       // (0x83 define 860 amostras por segundo).

    i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, cfg, 3, pdMS_TO_TICKS(100));   // Envia o comando de configuração (array cfg) pelo I2C para o ADS1115.
    vTaskDelay(pdMS_TO_TICKS(2));                                                           // Pausa a tarefa durante 2 milissegundos para dar tempo físico ao ADC de processar a conversão.
// Aguarda conversão                    

    uint8_t reg = 0x00;                                                                     // Prepara a variável com o endereço 0x00, que é o Registo de Conversão (onde ficam os resultados).
    i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, &reg, 1, pdMS_TO_TICKS(100));  // Envia um byte em branco dizendo ao ADC "quero ler o registo 0x00 a seguir".

    uint8_t data[2] = {0};                                                                  // Cria um array vazio de 2 bytes para armazenar os dados que o ADC vai responder.
    i2c_master_read_from_device(I2C_MASTER_NUM, ADS1115_ADDR, data, 2, pdMS_TO_TICKS(100)); // Solicita e recebe 2 bytes de dados do ADC através do I2C.

    return (int16_t)((data[0] << 8) | data[1]);                                             // Combina o byte alto (deslocando-o 8 bits) e o byte baixo com a operação lógica OR, retornando um número inteiro.
}                                       

/* ============================================================================
 * CONVERSÃO DE SENSORES
 * ============================================================================ */
static float ads_para_tensao(int16_t raw) {         // Função que converte o valor bruto (inteiro) do ADC em Volts reais.
    return raw * ADS1115_LSB_V * SENSOR_V_DIVISOR;  // Multiplica o valor lido pela resolução e pelo fator do divisor resistivo.
}                                       

static float ads_para_corrente(int16_t raw) {                               // Função que converte o valor bruto (inteiro) do ADC em Amperes reais.
    return (raw * ADS1115_LSB_V) / (SENSOR_SHUNT_OHM * SENSOR_AMP_GANHO);   // Aplica a lei de Ohm considerando a tensão, a resistência e o ganho do operacional.
}                                       

static float ads_para_temperatura(int16_t raw) {                                    // Função que converte o valor bruto do ADC em graus Celsius.
    float v = raw * ADS1115_LSB_V;                                                  // Converte a leitura bruta para tensão medida no meio do divisor NTC.
    float den = SENSOR_VCC - v;                                                     // Subtrai a tensão medida da tensão total para calcular a queda no resistor série.
    if (den < 0.001f) return 999.0f;                                                // NTC desconectado // Previne erro de divisão por zero caso o NTC não esteja fisicamente ligado.
    float r_ntc = SENSOR_R_SERIE * (v / den);                                       // Calcula a resistência atual em Ohms do termistor NTC.
    float tk = 1.0f / ((1.0f / (25.0f + 273.15f)) +                                 // Inicia a aplicação da equação de Steinhart-Hart invertida.
                        (1.0f / SENSOR_NTC_BETA) * logf(r_ntc / SENSOR_NTC_R25));   // Completa a equação calculando a temperatura absoluta em Kelvin.
    return tk - 273.15f;                                                            // Converte o resultado de Kelvin para graus Celsius e retorna o valor.
}                                      

/* ============================================================================
 * MÁQUINA DE ESTADOS
 * ============================================================================ */
static const char *nome_estado(EstadoSistema e) {       // Função utilitária que converte o valor da enumeração numa string legível.
    switch (e) {                                        // Início da avaliação da variável de estado recebida como parâmetro.
        case DESCONECTADO:   return "DESCONECTADO";     // Retorna a string respetiva caso o estado seja DESCONECTADO.
        case AVALIANDO:      return "AVALIANDO";        // Retorna a string respetiva caso o estado seja AVALIANDO.
        case CARREGANDO:     return "CARREGANDO";       // Retorna a string respetiva caso o estado seja CARREGANDO.
        case ERRO_INVERTIDA: return "ERRO_INVERTIDA";   // Retorna a string respetiva caso o estado seja ERRO_INVERTIDA.
        case CARGA_COMPLETA: return "CARGA_COMPLETA";   // Retorna a string respetiva caso o estado seja CARGA_COMPLETA.
        default:             return "?";                // Retorna um ponto de interrogação como prevenção de falha, caso o estado seja desconhecido.
    }                                       
}                                       

static void processar_estado(void) {    // Função principal da lógica do programa, responsável por mudar os estados do sistema consoante as leituras.
    switch (estadoAtual) {              // Avalia qual é o estado global em que a máquina se encontra atualmente.

        case DESCONECTADO:                  // Bloco lógico executado continuamente enquanto o sistema estiver no estado DESCONECTADO.
            gpio_set_level(PINO_RELE, 0);   // Força o pino do relé para zero (nível lógico baixo), mantendo a bateria isolada do circuito.
            pwm_ajustar_duty(0.0f);         // Chama a função do módulo PWM para definir a potência a zero (desligando a corrente).
            dutyCycleAtual = 0.0f;          // Zera a variável de rastreio interno do duty cycle para refletir a paragem da potência.

            if (tensaoBateria < V_INVERTIDA) {                              // Verifica se a tensão lida é inferior a -0.1V.
                ESP_LOGW(TAG, "Polaridade invertida!");                     // Imprime um aviso amarelo no terminal alertando o erro de polaridade.
                estadoAtual = ERRO_INVERTIDA;                               // Altera a variável global e atira o sistema para o estado de erro, bloqueando outras ações.
            } else if (tensaoBateria > V_BAT_MINIMA && tensaoBateria < V_BAT_MAXIMA) { // Se não está invertida, verifica se a tensão está entre 0.5V e 1.8V (uma célula NiCd conectada).
                ESP_LOGI(TAG, "Bateria detectada: %.3fV", tensaoBateria);   // Imprime uma mensagem de registo informando que uma bateria válida foi encontrada.
                estadoAtual = AVALIANDO;                                    // Altera o estado do sistema para avançar para a próxima fase.
            }                           
            break;                      
        case ERRO_INVERTIDA:                                                // Bloco lógico executado continuamente enquanto o sistema estiver no estado ERRO_INVERTIDA.
            if (tensaoBateria >= V_INVERTIDA) {                             // Verifica se a tensão subiu e o erro físico da bateria invertida desapareceu.
                ESP_LOGI(TAG, "Polaridade corrigida.");                     // Imprime uma mensagem indicando que a situação foi normalizada.
                estadoAtual = DESCONECTADO;                                 // Recoloca o sistema no estado base (DESCONECTADO) para reiniciar o ciclo em segurança.
            }                                                               
            break;                      
        case AVALIANDO:                                                     // Bloco lógico executado quando o sistema acabou de reconhecer a bateria e vai iniciar a carga.
            if (temperaturaAtual >= MAX_TEMP_CELSIUS) {                     // Verificação prévia de segurança térmica (a temperatura da célula é maior que 45 graus?).
                ESP_LOGW(TAG, "Temperatura alta (%.1f°C). Carga bloqueada.", temperaturaAtual); // Imprime um aviso no terminal recusando o arranque da carga.
                estadoAtual = DESCONECTADO;                                 // Aborta o processo, enviando a máquina de volta para a fase inicial de espera.
                break;                                                      // Sai imediatamente deste "case" porque detetou uma falha de segurança.
            }                          
            gpio_set_level(PINO_RELE, 1);                                   // Se a temperatura estiver boa, ativa o relé de segurança, conectando a placa fisicamente à pilha.
            vTaskDelay(pdMS_TO_TICKS(500));                                 // O sistema adormece meio segundo para garantir que os contactos mecânicos do relé estabilizam (debouncing).
            tensaoMaxima = tensaoBateria;                                   // Copia a leitura atual da tensão e adota este valor como a "tensão máxima inicial" registada.
            pid_resetar();                                                  // Chama a função no módulo de PID para apagar o histórico de memória e integrais de ciclos anteriores.
            ESP_LOGI(TAG, "Iniciando carga. Vbat=%.3fV  Alvo=%.2fA", tensaoBateria, CORRENTE_ALVO); // Imprime as métricas de início de injeção de corrente.
            estadoAtual = CARREGANDO;                                       // Muda oficialmente o estado global para dar início imediato ao funcionamento do conversor buck.
            break;                      
        case CARREGANDO:                                                    // O coração dinâmico do sistema. Executado continuamente enquanto a bateria recebe energia.
            dutyCycleAtual = pid_calcular(correnteCarga);                   // Envia a corrente real lida ao controlador PID, que devolve (retorna) a percentagem de PWM calculada como adequada.
            pwm_ajustar_duty(dutyCycleAtual);                               // Atualiza fisicamente os registos de hardware do ESP32 para alterar a largura do pulso nas saídas.
            if (tensaoBateria > tensaoMaxima) tensaoMaxima = tensaoBateria; // Lógica de deteção de pico: se a tensão subiu num ciclo novo, guarda esse valor na memória de tensão máxima.

            bool delta_v   = (tensaoMaxima - tensaoBateria) >= DELTA_V_THRESHOLD; // Cria uma variável verdadeira/falsa calculando se a tensão caiu 15mV face ao pico anterior registado.
            bool temp_alta = (temperaturaAtual >= MAX_TEMP_CELSIUS); // Cria uma variável verdadeira/falsa calculando se a bateria ultrapassou os 45 graus.
            bool removida  = (tensaoBateria < V_DESCONECTADA);      // Cria uma variável verdadeira/falsa calculando se a tensão caiu drasticamente devido a uma remoção manual.
            if (removida) {                                         // Condição de emergência: se o utilizador puxou a bateria fisicamente com a carga a decorrer.
                ESP_LOGW(TAG, "Bateria removida durante carga.");   // Registo de aviso sobre a desconexão abrupta.
                pwm_ajustar_duty(0.0f);                             // Corta o pulso do PWM imediatamente, protegendo o indutor e MOSFETs do hardware.
                gpio_set_level(PINO_RELE, 0);                       // Abre o contacto do relé mecânico.
                estadoAtual = DESCONECTADO;                         // Reinicia a lógica mandando a máquina procurar uma bateria de novo.
            } else if (delta_v) {                                   // Caso não tenha sido removida, verifica o critério de fim natural (queda química da NiCd atingida).
                ESP_LOGI(TAG, "Delta-V atingido! Vpico=%.3fV  Vatual=%.3fV", tensaoMaxima, tensaoBateria); // Log de sucesso indicando fim de ciclo normal.
                pwm_ajustar_duty(0.0f);                             // Corta o PWM para parar o carregamento.
                gpio_set_level(PINO_RELE, 0);                       // Desconecta o relé físico da placa de circuito impresso para a bateria.
                estadoAtual = CARGA_COMPLETA;                       // Assinala o fim da operação alterando o estado global para completado.
            } else if (temp_alta) {                                 // Caso ainda não tenha havido Delta-V, verifica se a bateria está superaquecendo perigosamente.
                ESP_LOGW(TAG, "Temperatura limite atingida: %.1f°C", temperaturaAtual); // Log indicando que o processo foi cortado precocemente por aquecimento excessivo.
                pwm_ajustar_duty(0.0f);                             // Corta o fornecimento de força imediatamente.
                gpio_set_level(PINO_RELE, 0);                       // Desconecta o relé mecânico de proteção.
                estadoAtual = CARGA_COMPLETA;                       // Encaminha para o estado completado (a pilha precisa de tempo para arrefecer).
            }                                                       // Fim da cadeia de verificações de paragem do sistema (if/else if).
            break;                                                  // Finaliza o "case" de carregamento e permite que a máquina avance.
        case CARGA_COMPLETA:                                        // Bloco lógico de espera após o término (seja por sucesso no Delta-V ou por excesso de calor).
            if (tensaoBateria < V_DESCONECTADA) {                   // O sistema passa o tempo a verificar se alguém já tirou a pilha finalizada (tensão cai a quase zero).
                ESP_LOGI(TAG, "Bateria removida. Reiniciando.");    // Informa que o sistema reconheceu a remoção física.
                estadoAtual = DESCONECTADO;                         // Faz reset total no sistema, enviando-o de volta ao início para esperar nova célula.
            }                                                       // Fim da verificação de remoção.
            break;                                                  // Interrompe o "case" final.
    }                                                               // Fim absoluto do comando condicional "switch" que abrange todos os estados possíveis da máquina.
}                                                                   // Fim da função processar_estado.

/* ============================================================================
 * PONTO DE ENTRADA
 * ============================================================================ */
void app_main(void) {                                           // Função inicial e principal do FreeRTOS. É chamada assim que o ESP32 arranca, funcionando como um setup e loop.
    vTaskDelay(pdMS_TO_TICKS(1000));                            // Ouve um atraso de 1 segundo (1000ms) imposto para permitir que os condensadores da fonte de alimentação da placa física estabilizem completamente.
    ESP_LOGI(TAG, "=== CARREGADOR NiCd — TEST_BATERRY ===");    // Emite o primeiro log de arrabalde, confirmando ao utilizador que o firmware está em execução.

    // Relé                            
    gpio_reset_pin(PINO_RELE);                                          // Executa um reset no pino do relé, limpando qualquer configuração defeituosa que tenha ficado presa no registrador interno do ESP32 aquando do último boot.
    gpio_set_direction(PINO_RELE, GPIO_MODE_OUTPUT);                    // Define de forma expressa que este pino vai funcionar como saída (capaz de conduzir tensão), não como entrada.
    gpio_set_level(PINO_RELE, 0);                                       // Assegura, sem qualquer dúvida, que o pino começa com tensão a zero, garantindo que o relé não liga subitamente a placa.
    // Componentes                         
    pwm_inicializar(PINO_PWM_ALTO, PINO_PWM_BAIXO);                     // Efetua a chamada inicializando os módulos MCPWM internos do ESP32 para o conversor DC-DC, mapeando-os aos pinos High e Low.
    pid_inicializar(CORRENTE_ALVO, PWM_MIN_DUTY_PCT, PWM_MAX_DUTY_PCT); // Invoca o módulo personalizado e configura nele o alvo (0.8A) bem como as barreiras (0% a 95%) em que a malha PI se vai mover.
    init_i2c();                                                         // Arranca o barramento I2C, os pinos SDA/SCL, configurando-o a 100 kHz, preparando a comunicação.

    ESP_LOGI(TAG, "Sistema pronto. Aguardando bateria...");         // Emite a mensagem ao utilizador declarando o fim da inicialização técnica (Setup) em segurança.
    while (1) {                                                     // O ciclo infinito que manterá o microcontrolador a correr perpetuamente (equivalente ao "loop()" do ecossistema Arduino).
        tensaoBateria    = ads_para_tensao(ler_ads1115(0));         // Faz a chamada pela comunicação I2C requisitando do MUX do canal zero os dados puros (0-65535), enviando-os à equação que nos devolve Volts contínuos.
        correnteCarga    = ads_para_corrente(ler_ads1115(1));       // Requisita ao I2C o valor do canal 1, transformando pela leitura diferencial do shunt, os valores reais da corrente em Amperes.
        temperaturaAtual = ads_para_temperatura(ler_ads1115(2));    // Exige do MUX do canal dois o valor bruto do termistor e converte-o matematicamente (Steinhart-Hart) para devolver graus Celsius exatos.

        processar_estado();                                                         // Submete toda as variáveis frescas (Voltagem, Amperagem e Temperatura) ao coração do programa. Onde os parâmetros físicos vão influenciar as transições entre a espera, carga ou as ruturas de segurança e de erro.
        ESP_LOGI(TAG, "[%s] Vbat=%.3fV  I=%.3fA  T=%.1f°C  Duty=%.1f%%",            // Puxa pelo "formatter" de strings para escrever um relatório abrangente na ferramenta série monitora.
                 nome_estado(estadoAtual),                                          // Executa o conversor de tipo string para traduzir o ID numérico do enum para nome e facilitar a depuração a quem monitora.
                 tensaoBateria, correnteCarga, temperaturaAtual, dutyCycleAtual);   // Termina o preenchimento da formatação da string fornecendo a totalidade das referências físicas vitais que guiam a calibração final por parte do construtor.
        vTaskDelay(pdMS_TO_TICKS(500));         
    }                                   
} 