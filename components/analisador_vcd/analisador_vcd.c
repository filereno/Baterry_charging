#include "analisador_vcd.h"
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"

#define NUM_AMOSTRAS 5000 
#define ATRASO_MICROSEGUNDOS 2 

static uint8_t buffer_d0[NUM_AMOSTRAS];
static uint8_t buffer_d1[NUM_AMOSTRAS];

void capturar_sinais_vcd(void) {
    printf("\n=== INICIANDO CAPTURA DE SINAIS (Aguarde...) ===\n");
    
    portDISABLE_INTERRUPTS(); 
    
    for (int i = 0; i < NUM_AMOSTRAS; i++) {
        uint32_t pin_states = REG_READ(GPIO_IN_REG); 
        buffer_d0[i] = (pin_states & (1 << 19)) ? 1 : 0; 
        buffer_d1[i] = (pin_states & (1 << 5))  ? 1 : 0; 
        esp_rom_delay_us(ATRASO_MICROSEGUNDOS);
    }
    
    portENABLE_INTERRUPTS(); 

    printf("Captura finalizada. Copie o texto abaixo e salve como arquivo .vcd:\n\n");
    
    printf("$date Today $end\n");
    printf("$timescale 1 us $end\n"); 
    printf("$var wire 1 A D0_Pino19 $end\n");
    printf("$var wire 1 B D1_Pino5 $end\n");
    printf("$enddefinitions $end\n");
    
    uint8_t last_d0 = 255, last_d1 = 255; 
    uint32_t tempo_atual_us = 0;

    for (int i = 0; i < NUM_AMOSTRAS; i++) {
        if (buffer_d0[i] != last_d0 || buffer_d1[i] != last_d1) {
            printf("#%lu\n", tempo_atual_us); 
            if (buffer_d0[i] != last_d0) {
                printf("%dA\n", buffer_d0[i]);
                last_d0 = buffer_d0[i];
            }
            if (buffer_d1[i] != last_d1) {
                printf("%dB\n", buffer_d1[i]);
                last_d1 = buffer_d1[i];
            }
        }
        tempo_atual_us += ATRASO_MICROSEGUNDOS;
    }
    printf("\n=== FIM DO ARQUIVO VCD ===\n\n");
}