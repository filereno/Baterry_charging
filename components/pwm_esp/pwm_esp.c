/*
 * ==============================================================================
 * ARQUIVO: pwm_esp.c
 * COMPONENTE: pwm_esp
 * FUNÇÃO: Driver PWM Síncrono com Dead-Time para Buck Converter
 *
 * TOPOLOGIA:
 *   VIN(8.5V) ──┬── [IRF9530 P-ch] ──┬── L ──┬── VOUT → Bateria
 *               │   gate: pino_alto   │       │
 *               └── [IRF640  N-ch] ──┘      C === GND
 *                   gate: pino_baixo
 *
 * LÓGICA DE SINAL:
 *   O driver totem-pole inverte o sinal para o IRF9530 (P-ch precisa de gate
 *   LOW para conduzir). O MCPWM envia PWM normal nos dois pinos; o totem-pole
 *   cuida da inversão. O dead-time de hardware protege as transições.
 * ==============================================================================
 */

#include "pwm_esp.h"
#include "esp_log.h"

static const char *TAG = "PWM";

/* Handles do MCPWM (privados ao módulo) */
static mcpwm_timer_handle_t  s_timer      = NULL;
static mcpwm_oper_handle_t   s_oper       = NULL;
static mcpwm_cmpr_handle_t   s_comparator = NULL;
static mcpwm_gen_handle_t    s_gen_high   = NULL;
static mcpwm_gen_handle_t    s_gen_low    = NULL;

/* ============================================================================
 * IMPLEMENTAÇÃO
 * ============================================================================ */

void pwm_inicializar(int pino_alto, int pino_baixo) {
    ESP_LOGI(TAG, "Iniciando MCPWM: %d kHz, dead-time=%d ticks (%d µs)",
             PWM_FREQ_HZ / 1000, PWM_DEADTIME_TICKS, PWM_DEADTIME_TICKS);

    /* --- 1. TIMER --------------------------------------------------------- */
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks  = PWM_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &s_timer));

    /* --- 2. OPERADOR ------------------------------------------------------- */
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_oper, s_timer));

    /* --- 3. COMPARADOR ----------------------------------------------------- */
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = true,  // Atualiza no zero do timer → sem glitch
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_oper, &cmpr_cfg, &s_comparator));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_comparator, 0)); // duty=0% inicial

    /* --- 4. GERADORES (GPIOs) ---------------------------------------------- */
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = pino_alto };
    ESP_ERROR_CHECK(mcpwm_new_generator(s_oper, &gen_cfg, &s_gen_high));

    gen_cfg.gen_gpio_num = pino_baixo;
    ESP_ERROR_CHECK(mcpwm_new_generator(s_oper, &gen_cfg, &s_gen_low));

    /* --- 5. AÇÕES DE COMUTAÇÃO -------------------------------------------- */
    // High-side: HIGH no zero do timer, LOW no comparador
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_gen_high,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_gen_high,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                        s_comparator,
                                        MCPWM_GEN_ACTION_LOW)));

    // Low-side: complementar — LOW no zero, HIGH no comparador
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_gen_low,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_gen_low,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                        s_comparator,
                                        MCPWM_GEN_ACTION_HIGH)));

    /* --- 6. DEAD-TIME ------------------------------------------------------ */
    /*
     * Previne que high-side e low-side conduzam ao mesmo tempo (shoot-through).
     * Atrasa a borda de turn-on de cada canal em PWM_DEADTIME_TICKS.
     *
     * High-side: atrasa rising edge (quando vai ligar)
     */
    mcpwm_dead_time_config_t dt_high = {
        .posedge_delay_ticks = PWM_DEADTIME_TICKS,
        .negedge_delay_ticks = 0,
        .flags.invert_output = false,
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(s_gen_high, s_gen_high, &dt_high));

    /*
     * Low-side: invertido + atraso rising edge
     * (inverte porque o low-side liga no comparador, não no zero)
     */
    mcpwm_dead_time_config_t dt_low = {
        .posedge_delay_ticks = PWM_DEADTIME_TICKS,
        .negedge_delay_ticks = 0,
        .flags.invert_output = true,
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(s_gen_low, s_gen_low, &dt_low));

    /* --- 7. START ---------------------------------------------------------- */
    ESP_ERROR_CHECK(mcpwm_timer_enable(s_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "PWM pronto. GPIO_ALTO=%d  GPIO_BAIXO=%d", pino_alto, pino_baixo);
}

void pwm_ajustar_duty(float porcentagem) {
    if (porcentagem > PWM_MAX_DUTY_PCT) porcentagem = PWM_MAX_DUTY_PCT;
    if (porcentagem < PWM_MIN_DUTY_PCT) porcentagem = PWM_MIN_DUTY_PCT;

    uint32_t ticks = (uint32_t)((porcentagem * (float)PWM_PERIOD_TICKS) / 100.0f);
    mcpwm_comparator_set_compare_value(s_comparator, ticks);
}
