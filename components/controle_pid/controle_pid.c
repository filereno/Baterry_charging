/*
 * ==============================================================================
 * ARQUIVO: controle_pid.c
 * COMPONENTE: controle_pid
 * FUNÇÃO: Wrapper do PID da Espressif para controle de corrente NiCd
 *
 * BIBLIOTECA USADA: espressif/pid_ctrl^0.2.0
 *   Documentação: https://components.espressif.com/components/espressif/pid_ctrl
 *
 * FUNCIONAMENTO:
 *   pid_compute() recebe o ERRO (setpoint - medição), não a medição direta.
 *   A biblioteca usa forma POSICIONAL com anti-windup por clamping do integrador
 *   (definido por max_integral / min_integral na configuração).
 * ==============================================================================
 */

#include "controle_pid.h"
#include "pid_ctrl.h"          // espressif/pid_ctrl
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "PID";

/* Handle interno da biblioteca (opaco para o main.c) */
static pid_ctrl_block_handle_t s_pid_handle = NULL;

/* Setpoint e última saída armazenados localmente */
static float s_setpoint   = 0.0f;
static float s_saida_atual = 0.0f;

/* ============================================================================
 * IMPLEMENTAÇÃO
 * ============================================================================ */

void pid_inicializar(float setpoint, float saida_min, float saida_max) {
    s_setpoint = setpoint;

    /* Parâmetros passados para a biblioteca Espressif */
    pid_ctrl_parameter_t params = {
        .kp           = PID_KP,
        .ki           = PID_KI,
        .kd           = PID_KD,
        .max_output   = saida_max,          // Limite superior do duty cycle (%)
        .min_output   = saida_min,          // Limite inferior do duty cycle (%)
        .max_integral = PID_MAX_INTEGRAL,   // Anti-windup: teto do integrador
        .min_integral = PID_MIN_INTEGRAL,   // Anti-windup: piso do integrador
        .cal_type     = PID_CAL_TYPE_POSITIONAL, // Forma posicional (u = Kp*e + Ki*∫e + Kd*de/dt)
    };

    pid_ctrl_config_t config = {
        .init_param = params,
    };

    /* Se já existia um bloco anterior, destrói antes de criar outro */
    if (s_pid_handle != NULL) {
        pid_del_control_block(s_pid_handle);
        s_pid_handle = NULL;
    }

    esp_err_t err = pid_new_control_block(&config, &s_pid_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar bloco PID: %s", esp_err_to_name(err));
        return;
    }

    s_saida_atual = 0.0f;

    ESP_LOGI(TAG, "PID inicializado (espressif/pid_ctrl) — "
                  "setpoint=%.3fA  Kp=%.2f  Ki=%.3f  Kd=%.4f  "
                  "out=[%.1f, %.1f]%%  int=[%.1f, %.1f]",
             setpoint,
             PID_KP, PID_KI, PID_KD,
             saida_min, saida_max,
             PID_MIN_INTEGRAL, PID_MAX_INTEGRAL);
}

float pid_calcular(float corrente_medida) {
    if (s_pid_handle == NULL) {
        ESP_LOGW(TAG, "pid_calcular chamado antes de pid_inicializar!");
        return 0.0f;
    }

    /* A biblioteca recebe o ERRO, não a medição */
    float erro  = s_setpoint - corrente_medida;
    float saida = 0.0f;

    esp_err_t err = pid_compute(s_pid_handle, erro, &saida);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pid_compute falhou: %s", esp_err_to_name(err));
        return s_saida_atual;  // Mantém o último valor seguro
    }

    s_saida_atual = saida;

    ESP_LOGD(TAG, "I=%.3fA  erro=%+.3f  duty=%.1f%%",
             corrente_medida, erro, saida);

    return saida;
}

void pid_resetar(void) {
    if (s_pid_handle == NULL) return;

    esp_err_t err = pid_reset_ctrl_block(s_pid_handle);
    if (err == ESP_OK) {
        s_saida_atual = 0.0f;
        ESP_LOGI(TAG, "PID resetado.");
    } else {
        ESP_LOGE(TAG, "Falha ao resetar PID: %s", esp_err_to_name(err));
    }
}

float pid_obter_saida(void) {
    return s_saida_atual;
}
