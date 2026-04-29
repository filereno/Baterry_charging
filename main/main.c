/*
 * ==============================================================================
 * ARQUIVO: main.c
 * PROJETO: TEST_BATERRY — Carregador/Testador de Baterias NiCd
 *
 * COMPONENTES USADOS:
 *   - pwm_esp      : Driver PWM síncrono (MCPWM) para o buck converter
 *   - controle_pid : Controlador PID de corrente
 *
 * CANAIS ADS1115 (I2C, endereço 0x48):
 *   AIN0 → Tensão da bateria (via divisor resistivo)
 *   AIN1 → Corrente de carga (via shunt + amplificador)
 *   AIN2 → Temperatura NTC 10kΩ (via divisor resistivo)
 * ==============================================================================
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

// Componentes customizados
#include "pwm_esp.h"
#include "controle_pid.h"

static const char *TAG = "CARREGADOR";

/* ============================================================================
 * PINOS
 * ============================================================================ */
#define PINO_RELE         GPIO_NUM_18
#define PINO_PWM_ALTO     GPIO_NUM_19   // Gate IRF9530 (high-side P-ch)
#define PINO_PWM_BAIXO    GPIO_NUM_5    // Gate IRF640  (low-side  N-ch)
#define PINO_SDA          GPIO_NUM_21
#define PINO_SCL          GPIO_NUM_22

/* ============================================================================
 * I2C / ADS1115
 * ============================================================================ */
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_FREQ_HZ       100000
#define ADS1115_ADDR      0x48          // ADDR → GND

// FSR ±2.048V → 1 LSB = 62.5 µV
#define ADS1115_LSB_V     0.0000625f

/* ============================================================================
 * FATORES DE CONVERSÃO DOS SENSORES
 *
 * *** AJUSTE ESTES VALORES CONFORME SEU HARDWARE ***
 *
 * TENSÃO (AIN0):
 *   Divisor com R1=10kΩ, R2=10kΩ → V_bat = V_adc × 2.0
 *
 * CORRENTE (AIN1):
 *   Shunt de 0.1Ω com amplificador de ganho 10×
 *   I = V_adc / (R_shunt × ganho) = V_adc / 1.0
 *
 * TEMPERATURA (AIN2):
 *   NTC 10kΩ (Beta=3950) com R_série=10kΩ, VCC=3.3V
 * ============================================================================ */
#define SENSOR_V_DIVISOR    2.0f        // Fator do divisor de tensão
#define SENSOR_SHUNT_OHM    0.1f        // Resistência do shunt (Ω)
#define SENSOR_AMP_GANHO    10.0f       // Ganho do amp de corrente
#define SENSOR_NTC_BETA     3950.0f
#define SENSOR_NTC_R25      10000.0f    // Resistência NTC a 25°C (Ω)
#define SENSOR_R_SERIE      10000.0f    // Resistor série do divisor NTC (Ω)
#define SENSOR_VCC          3.3f        // Tensão do divisor NTC

/* ============================================================================
 * PARÂMETROS DE CARGA NiCd
 * ============================================================================ */
#define CORRENTE_ALVO       0.8f        // Corrente de carga em amperes
#define DELTA_V_THRESHOLD   0.015f      // Queda de tensão para fim de carga (V)
#define MAX_TEMP_CELSIUS    45.0f       // Temperatura de corte (°C)
#define V_BAT_MINIMA        0.5f        // Tensão mínima para detectar bateria
#define V_BAT_MAXIMA        1.8f        // Tensão máxima para iniciar carga
#define V_INVERTIDA         -0.1f       // Limiar de polaridade invertida
#define V_DESCONECTADA      0.2f        // Abaixo disso: bateria removida

/* ============================================================================
 * MÁQUINA DE ESTADOS
 * ============================================================================ */
typedef enum {
    DESCONECTADO,
    AVALIANDO,
    CARREGANDO,
    ERRO_INVERTIDA,
    CARGA_COMPLETA
} EstadoSistema;

static EstadoSistema estadoAtual = DESCONECTADO;

/* Medições globais */
static float tensaoBateria   = 0.0f;
static float tensaoMaxima    = 0.0f;
static float correnteCarga   = 0.0f;
static float temperaturaAtual = 0.0f;
static float dutyCycleAtual  = 0.0f;

/* ============================================================================
 * I2C / ADS1115
 * ============================================================================ */
static void init_i2c(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PINO_SDA,
        .scl_io_num       = PINO_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C pronto: SDA=%d SCL=%d", PINO_SDA, PINO_SCL);
}

static int16_t ler_ads1115(uint8_t canal) {
    uint8_t mux;
    switch (canal) {
        case 0:  mux = 0xC3; break;  // AIN0 vs GND, PGA=±2.048V, single-shot
        case 1:  mux = 0xD3; break;  // AIN1 vs GND
        default: mux = 0xE3; break;  // AIN2 vs GND
    }
    uint8_t cfg[3] = { 0x01, mux, 0x83 };  // 0x83 = 860 SPS

    i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, cfg, 3, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(2));  // Aguarda conversão

    uint8_t reg = 0x00;
    i2c_master_write_to_device(I2C_MASTER_NUM, ADS1115_ADDR, &reg, 1, pdMS_TO_TICKS(100));

    uint8_t data[2] = {0};
    i2c_master_read_from_device(I2C_MASTER_NUM, ADS1115_ADDR, data, 2, pdMS_TO_TICKS(100));

    return (int16_t)((data[0] << 8) | data[1]);
}

/* ============================================================================
 * CONVERSÃO DE SENSORES
 * ============================================================================ */
static float ads_para_tensao(int16_t raw) {
    return raw * ADS1115_LSB_V * SENSOR_V_DIVISOR;
}

static float ads_para_corrente(int16_t raw) {
    return (raw * ADS1115_LSB_V) / (SENSOR_SHUNT_OHM * SENSOR_AMP_GANHO);
}

static float ads_para_temperatura(int16_t raw) {
    float v = raw * ADS1115_LSB_V;
    float den = SENSOR_VCC - v;
    if (den < 0.001f) return 999.0f;  // NTC desconectado
    float r_ntc = SENSOR_R_SERIE * (v / den);
    float tk = 1.0f / ((1.0f / (25.0f + 273.15f)) +
                        (1.0f / SENSOR_NTC_BETA) * logf(r_ntc / SENSOR_NTC_R25));
    return tk - 273.15f;
}

/* ============================================================================
 * MÁQUINA DE ESTADOS
 * ============================================================================ */
static const char *nome_estado(EstadoSistema e) {
    switch (e) {
        case DESCONECTADO:   return "DESCONECTADO";
        case AVALIANDO:      return "AVALIANDO";
        case CARREGANDO:     return "CARREGANDO";
        case ERRO_INVERTIDA: return "ERRO_INVERTIDA";
        case CARGA_COMPLETA: return "CARGA_COMPLETA";
        default:             return "?";
    }
}

static void processar_estado(void) {
    switch (estadoAtual) {

        case DESCONECTADO:
            gpio_set_level(PINO_RELE, 0);
            pwm_ajustar_duty(0.0f);
            dutyCycleAtual = 0.0f;

            if (tensaoBateria < V_INVERTIDA) {
                ESP_LOGW(TAG, "Polaridade invertida!");
                estadoAtual = ERRO_INVERTIDA;
            } else if (tensaoBateria > V_BAT_MINIMA && tensaoBateria < V_BAT_MAXIMA) {
                ESP_LOGI(TAG, "Bateria detectada: %.3fV", tensaoBateria);
                estadoAtual = AVALIANDO;
            }
            break;

        case ERRO_INVERTIDA:
            if (tensaoBateria >= V_INVERTIDA) {
                ESP_LOGI(TAG, "Polaridade corrigida.");
                estadoAtual = DESCONECTADO;
            }
            break;

        case AVALIANDO:
            if (temperaturaAtual >= MAX_TEMP_CELSIUS) {
                ESP_LOGW(TAG, "Temperatura alta (%.1f°C). Carga bloqueada.", temperaturaAtual);
                estadoAtual = DESCONECTADO;
                break;
            }
            gpio_set_level(PINO_RELE, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            tensaoMaxima = tensaoBateria;
            pid_resetar();
            ESP_LOGI(TAG, "Iniciando carga. Vbat=%.3fV  Alvo=%.2fA", tensaoBateria, CORRENTE_ALVO);
            estadoAtual = CARREGANDO;
            break;

        case CARREGANDO:
            dutyCycleAtual = pid_calcular(correnteCarga);
            pwm_ajustar_duty(dutyCycleAtual);

            if (tensaoBateria > tensaoMaxima) tensaoMaxima = tensaoBateria;

            bool delta_v   = (tensaoMaxima - tensaoBateria) >= DELTA_V_THRESHOLD;
            bool temp_alta = (temperaturaAtual >= MAX_TEMP_CELSIUS);
            bool removida  = (tensaoBateria < V_DESCONECTADA);

            if (removida) {
                ESP_LOGW(TAG, "Bateria removida durante carga.");
                pwm_ajustar_duty(0.0f);
                gpio_set_level(PINO_RELE, 0);
                estadoAtual = DESCONECTADO;
            } else if (delta_v) {
                ESP_LOGI(TAG, "Delta-V atingido! Vpico=%.3fV  Vatual=%.3fV", tensaoMaxima, tensaoBateria);
                pwm_ajustar_duty(0.0f);
                gpio_set_level(PINO_RELE, 0);
                estadoAtual = CARGA_COMPLETA;
            } else if (temp_alta) {
                ESP_LOGW(TAG, "Temperatura limite atingida: %.1f°C", temperaturaAtual);
                pwm_ajustar_duty(0.0f);
                gpio_set_level(PINO_RELE, 0);
                estadoAtual = CARGA_COMPLETA;
            }
            break;

        case CARGA_COMPLETA:
            if (tensaoBateria < V_DESCONECTADA) {
                ESP_LOGI(TAG, "Bateria removida. Reiniciando.");
                estadoAtual = DESCONECTADO;
            }
            break;
    }
}

/* ============================================================================
 * PONTO DE ENTRADA
 * ============================================================================ */
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "=== CARREGADOR NiCd — TEST_BATERRY ===");

    // Relé
    gpio_reset_pin(PINO_RELE);
    gpio_set_direction(PINO_RELE, GPIO_MODE_OUTPUT);
    gpio_set_level(PINO_RELE, 0);

    // Componentes
    pwm_inicializar(PINO_PWM_ALTO, PINO_PWM_BAIXO);
    pid_inicializar(CORRENTE_ALVO, PWM_MIN_DUTY_PCT, PWM_MAX_DUTY_PCT);
    init_i2c();

    ESP_LOGI(TAG, "Sistema pronto. Aguardando bateria...");

    while (1) {
        tensaoBateria    = ads_para_tensao(ler_ads1115(0));
        correnteCarga    = ads_para_corrente(ler_ads1115(1));
        temperaturaAtual = ads_para_temperatura(ler_ads1115(2));

        processar_estado();

        ESP_LOGI(TAG, "[%s] Vbat=%.3fV  I=%.3fA  T=%.1f°C  Duty=%.1f%%",
                 nome_estado(estadoAtual),
                 tensaoBateria, correnteCarga, temperaturaAtual, dutyCycleAtual);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}