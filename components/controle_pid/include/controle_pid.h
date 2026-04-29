/*
 * ==============================================================================
 * ARQUIVO: controle_pid.h
 * COMPONENTE: controle_pid
 * FUNÇÃO: Wrapper do controlador PID da Espressif (espressif/pid_ctrl^0.2.0)
 *
 * A API pública é idêntica à versão anterior — main.c não precisa mudar.
 * Os ganhos e limites são configurados internamente em controle_pid.c.
 * ==============================================================================
 */

#ifndef CONTROLE_PID_H
#define CONTROLE_PID_H

/* ============================================================================
 * PARÂMETROS DO PID
 * Ajuste Kp/Ki/Kd conforme a resposta real do seu circuito.
 * ============================================================================ */
#define PID_KP              8.0f    // Ganho proporcional
#define PID_KI              0.5f    // Ganho integral
#define PID_KD              0.05f   // Ganho derivativo

// Limites do integrador (anti-windup interno da biblioteca)
#define PID_MAX_INTEGRAL    20.0f
#define PID_MIN_INTEGRAL   -20.0f

/* ============================================================================
 * API — mesma interface de antes, main.c não precisa mudar
 * ============================================================================ */

/**
 * @brief  Inicializa o bloco PID da Espressif.
 *         Aloca o handle interno e configura todos os parâmetros.
 *
 * @param  setpoint   Corrente alvo em amperes (ex: 0.8)
 * @param  saida_min  Duty cycle mínimo permitido (normalmente 0.0)
 * @param  saida_max  Duty cycle máximo permitido (normalmente 95.0)
 */
void pid_inicializar(float setpoint, float saida_min, float saida_max);

/**
 * @brief  Calcula o duty cycle a partir da corrente medida.
 *         Internamente calcula o erro e chama pid_compute() da biblioteca.
 *
 * @param  corrente_medida  Corrente lida pelo ADS1115 em amperes
 * @return float            Duty cycle em % no intervalo [saida_min, saida_max]
 */
float pid_calcular(float corrente_medida);

/**
 * @brief  Reseta o estado interno do PID (integrador e histórico).
 *         Chame antes de iniciar cada nova sessão de carga.
 */
void pid_resetar(void);

/**
 * @brief  Retorna o último duty cycle calculado (para log/display).
 * @return float  Último valor de saída em %
 */
float pid_obter_saida(void);

#endif // CONTROLE_PID_H
