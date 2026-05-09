#ifndef ANALISADOR_VCD_H
#define ANALISADOR_VCD_H

/**
 * @brief Trava o FreeRTOS, grava o estado dos pinos do PWM (19 e 5) 
 * na RAM na velocidade máxima e imprime o resultado na serial no formato .vcd.
 */
void capturar_sinais_vcd(void);

#endif // ANALISADOR_VCD_H