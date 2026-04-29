/*
 * ==============================================================================
 * ARQUIVO: pwm_esp.h
 * COMPONENTE: pwm_esp
 * FUNÇÃO: Interface do Driver PWM Síncrono para Buck Converter
 *
 * HARDWARE ALVO:
 *   High-side: IRF9530 (P-channel MOSFET) — conduz com gate BAIXO
 *   Low-side:  IRF640  (N-channel MOSFET) — conduz com gate ALTO
 *   Driver:    Totem-pole nos gates (lógica complementar)
 *
 * O MCPWM gera dois sinais complementares com dead-time de hardware,
 * prevenindo shoot-through no totem-pole durante as transições.
 * ==============================================================================
 */

#ifndef PWM_ESP_H
#define PWM_ESP_H

#include "driver/mcpwm_prelude.h"

/* ============================================================================
 * PARÂMETROS DE PWM
 * ============================================================================ */
#define PWM_FREQ_HZ          50000      // Frequência de chaveamento: 50 kHz
#define PWM_RESOLUTION_HZ    1000000    // Resolução interna do timer: 1 MHz
#define PWM_PERIOD_TICKS     (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)  // 20 ticks
#define PWM_DEADTIME_TICKS   5          // Dead-time: 5 ticks = 5 µs
#define PWM_MAX_DUTY_PCT     95.0f      // Máx. duty (protege o capacitor bootstrap)
#define PWM_MIN_DUTY_PCT     0.0f       // Mín. duty

/* ============================================================================
 * API
 * ============================================================================ */

/**
 * @brief  Inicializa o MCPWM para controle do buck síncrono.
 *         Configura timer, operador, comparador, geradores e dead-time.
 *         Ambos os pinos já saem com duty = 0% após esta função.
 *
 * @param  pino_alto   GPIO do gate do high-side (IRF9530 via totem-pole)
 * @param  pino_baixo  GPIO do gate do low-side  (IRF640  via totem-pole)
 */
void pwm_inicializar(int pino_alto, int pino_baixo);

/**
 * @brief  Define o duty cycle do conversor buck.
 *         Valores fora do intervalo [0, 95%] são limitados automaticamente.
 *
 * @param  porcentagem  Duty cycle em % (0.0 a 95.0)
 */
void pwm_ajustar_duty(float porcentagem);

#endif // PWM_ESP_H
