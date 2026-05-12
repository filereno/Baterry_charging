#include "ble_esp.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "BLE_ESP";
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_handle;

/* ============================================================================
 * UUIDs (Little-Endian)
 * ============================================================================ */
static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(0x70, 0xb2, 0xd2, 0x68, 0x76, 0xd4, 0x20, 0xa7, 0x12, 0x47, 0x7a, 0x13, 0xb0, 0x21, 0x44, 0xb1);
static const ble_uuid128_t chr_tx_uuid = BLE_UUID128_INIT(0x70, 0xb2, 0xd2, 0x68, 0x76, 0xd4, 0x20, 0xa7, 0x12, 0x47, 0x7a, 0x13, 0xb1, 0x21, 0x44, 0xb1);
static const ble_uuid128_t chr_rx_uuid = BLE_UUID128_INIT(0x70, 0xb2, 0xd2, 0x68, 0x76, 0xd4, 0x20, 0xa7, 0x12, 0x47, 0x7a, 0x13, 0xb2, 0x21, 0x44, 0xb1);

/* ============================================================================
 * CALLBACK UNIFICADA (O erro estava aqui!)
 * O NimBLE exige esta função em todas as características.
 * ============================================================================ */
static int ble_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // Se o App estiver a enviar um comando (RX)
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char rx_buffer[64];
        int len = ctxt->om->om_len;
        if (len >= sizeof(rx_buffer)) len = sizeof(rx_buffer) - 1;
        
        memcpy(rx_buffer, ctxt->om->om_data, len);
        rx_buffer[len] = '\0';
        
        ESP_LOGI(TAG, "Comando do App (RX): %s", rx_buffer);
    }
    return 0;
}

/* ============================================================================
 * TABELA GATT
 * ============================================================================ */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &chr_tx_uuid.u,
                .access_cb = ble_gatt_handler, // <--- FALTAVA ISTO! CAUSAVA O CRASH.
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_handle,
            },
            {
                .uuid = &chr_rx_uuid.u,
                .access_cb = ble_gatt_handler,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {0}
        }
    },
    {0}
};

static int ble_gap_event(struct ble_gap_event *event, void *arg);

/* ============================================================================
 * ADVERTISING (Visibilidade Bluetooth)
 * ============================================================================ */
static void start_advertising(void) {
    uint8_t own_addr_type;
    struct ble_gap_adv_params adv_params = {0};

    ble_hs_id_infer_auto(0, &own_addr_type);

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Flutter App Conectado!");
                conn_handle = event->connect.conn_handle;
            } else {
                start_advertising();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Flutter App Desconectado! A retomar Advertising...");
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
            break;
    }
    return 0;
}

static void ble_app_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    adv_fields.name = (uint8_t *)"ESP32_NiCd_Charger"; 
    adv_fields.name_len = strlen("ESP32_NiCd_Charger");
    adv_fields.name_is_complete = 1;
    
    ble_gap_adv_set_fields(&adv_fields);
    start_advertising();
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ============================================================================
 * APIs PÚBLICAS
 * ============================================================================ */
void ble_inicializar(void) {
    nimble_port_init();

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set("ESP32_NiCd_Charger");
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "NimBLE inicializado de forma limpa. A aguardar App...");
}

void ble_enviar_dados(float tensao, float corrente, float temperatura, float duty) {
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        char payload[64];
        snprintf(payload, sizeof(payload), "V:%.2f,I:%.2f,T:%.1f,D:%.1f", tensao, corrente, temperatura, duty);
        struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, strlen(payload));
        ble_gatts_notify_custom(conn_handle, tx_handle, om);
    }
}