#ifndef BLE_ESP_H
#define BLE_ESP_H

/**
 * @brief Inicializa a stack NimBLE e inicia o Advertising.
 */
void ble_inicializar(void);

/**
 * @brief Envia os dados atuais do carregador para o aplicativo conectado.
 */
void ble_enviar_dados(float tensao, float corrente, float temperatura, float duty);

#endif // BLE_ESP_H