#include <string.h>
#include <nvs_flash.h>
#include <esp_err.h>

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>

#include <esp_ble_mesh_ble_api.h>
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_defs.h"
#include <esp_ble_mesh_provisioning_api.h> // for mesh provision callback function
#include <esp_ble_mesh_networking_api.h>
#include <esp_ble_mesh_config_model_api.h> // for mesh config callback function
#include <esp_ble_mesh_generic_model_api.h> // for mesh config callback function
#include <esp_mac.h>
#include <esp_ble_mesh_local_data_operation_api.h>
#include "esp_gap_bt_api.h"
#include <mesh.h>
#include <ble_mesh_example_nvs.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "board.h"

#define TAG "BLE_MESH"
#define CID_ESP 0x2E25

static uint8_t dev_uuid[8]= {0xdd, 0xdd};

extern led_state board_led_state[3];

static nvs_handle_t NVS_HANDLE;
static const char * NVS_KEY = "onoff_client";

// stores the info for client set
static struct info_store {
    uint16_t net_idx;   /* NetKey Index */
    uint16_t app_idx;   /* AppKey Index */
    uint8_t  onoff;     /* Remote OnOff */
    uint8_t  tid;       /* Message TID */
} __attribute__((packed)) store = {
    .net_idx = ESP_BLE_MESH_KEY_UNUSED,
    .app_idx = ESP_BLE_MESH_KEY_UNUSED,
    .onoff = false,
    .tid = 0x0,
};

// for setting configuration option options in ble_mesh
static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
    #else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    #endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
    #else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    #endif
    .default_ttl = 7,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
}; 

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_1, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_1 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
}; 

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_2, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_2 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
}; 

static esp_ble_mesh_client_t onoff_client;
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 1, ROLE_NODE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_2, &onoff_server_2),
};

static esp_ble_mesh_model_t extended_model_0[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
};
static esp_ble_mesh_model_t extended_model_1[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_1, &onoff_server_1),
};
/* static esp_ble_mesh_model_t extended_model_2[] = { */
/* }; */

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extended_model_0, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extended_model_1, ESP_BLE_MESH_MODEL_NONE),
    /* ESP_BLE_MESH_ELEMENT(0, extended_model_2, ESP_BLE_MESH_MODEL_NONE), */
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
    .output_size = 0,
    .output_actions = 0,
};

static esp_ble_mesh_comp_t composition = {
    .cid  = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};


/* // For checking  whom the recieved message is for ( UNICAST, GROUP, 0XFFFF) */
static void change_led_state(esp_ble_mesh_model_t *model, esp_ble_mesh_msg_ctx_t *ctx, uint8_t onoff) {
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    led_state *led = NULL;
    uint8_t i;

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                ESP_LOGI(TAG, "ESP_BLE_MESH_ADDR_IS_UNICAST");
                led = &board_led_state[i];
                board_led_operation(led->gpio_num, onoff);
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            ESP_LOGI(TAG, "ESP_BLE_MESH_ADDR_IS_GROUP");
            led = &board_led_state[model->element->element_addr - primary_addr];
            board_led_operation(led->gpio_num, onoff);
        }
    } else if (ctx->recv_dst == 0xFFFF) {
        ESP_LOGI(TAG, "ESP_BLE_MESH_ADDR_IS_0xFFFF");
        led = &board_led_state[model->element->element_addr - primary_addr+1];
        board_led_operation(led->gpio_num, onoff);
    } else {}
}


static void mesh_info_store(void)
{
    ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
}

static void mesh_info_restore(void)
{
    esp_err_t err = ESP_OK;
    bool exist = false;

    err = ble_mesh_nvs_restore(NVS_HANDLE, NVS_KEY, &store, sizeof(store), &exist);
    if (err != ESP_OK) {
        return;
    }

    if (exist) {
        ESP_LOGI(TAG, "Restore, net_idx 0x%04x, app_idx 0x%04x, onoff %u, tid 0x%02x",
            store.net_idx, store.app_idx, store.onoff, store.tid);
    }
}

// For sending SET STATE message as client
void ble_mesh_send_gen_onoff_set(void)
{
    esp_ble_mesh_generic_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
    common.model = onoff_client.model;
    common.ctx.net_idx = store.net_idx;
    common.ctx.app_idx = store.app_idx;
    common.ctx.addr = 0xffff;   
    common.ctx.send_ttl = 3;
    common.msg_timeout = 0;     /* 0 indicates that timeout value from menuconfig will be used */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common.msg_role = ROLE_NODE;
#endif

    set.onoff_set.op_en = false;
    set.onoff_set.onoff = store.onoff;
    set.onoff_set.tid = store.tid++;

    err = esp_ble_mesh_generic_client_set_state(&common, &set);
    if (err) {
        ESP_LOGE(TAG, "Send Generic OnOff Set Unack failed");
        return;
    }

    store.onoff = !store.onoff;

    mesh_info_store(); //Store client as
}

// Callback function for  handling provision events
static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param){
    switch (event){
        case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_ENT, err_code %d", param->prov_register_comp.err_code );
            mesh_info_restore(); /* Restore proper mesh  info */
            board_led_operation(LED_B, true);
            break;
        case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT");
            break;
        case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT");
            break;
        case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
            board_led_operation(LED_B, false);
            break;
        case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
            break;
        case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT");
            break;
        default:
            break;
    }
}

// Callback function for handling config server events
static void ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param){
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT){
        switch ( param->ctx.recv_op ){
            case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
                ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
                ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
                board_led_operation(LED_R, true);
                break;
            case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
                ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
                break; 
            case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
                ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
                break;
            default:
                break;
        }
    }
}

// Callback function for handling generic client events
static void ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event, esp_ble_mesh_generic_client_cb_param_t *param)
{
    ESP_LOGI(TAG, "Generic client, event %u, error code %d, opcode is 0x%04" PRIx32,
             event, param->error_code, param->params->opcode);
    switch (event) {
        case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT");
            if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
                ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET, onoff %d", param->status_cb.onoff_status.present_onoff);
            }
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT");
            if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
                ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET, onoff %d", param->status_cb.onoff_status.present_onoff);
            }
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT");
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT");
            if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
                /* If failed to get the response of Generic OnOff Set, resend Generic OnOff Set  */
                ble_mesh_send_gen_onoff_set();
            }
            break;
        default:
            break;
    }
}

// Callback function for  handling generic server events
static void ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event, esp_ble_mesh_generic_server_cb_param_t *param){
    /* esp_ble_mesh_gen_onoff_srv_t *srv; */
    switch (event) {
        case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
            if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
                param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
                ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
                change_led_state(param->model, &param->ctx, param->value.state_change.onoff_set.onoff);
            }
            break;
        case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
            break;
        case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
            break;
        default:
            ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
            break;
    }
}

void ble_mesh_init(){
    ESP_ERROR_CHECK(esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb));
    ESP_ERROR_CHECK(esp_ble_mesh_register_generic_client_callback(ble_mesh_generic_client_cb));
    ESP_ERROR_CHECK(esp_ble_mesh_register_generic_server_callback(ble_mesh_generic_server_cb));
    ESP_ERROR_CHECK(esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb));

    ESP_ERROR_CHECK(esp_ble_mesh_init(&provision, &composition));

    ESP_ERROR_CHECK(esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT)));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    board_init();

    esp_bt_controller_config_t bt_cfg  = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    memcpy(dev_uuid+2, esp_bt_dev_get_address(), BD_ADDR_LEN);

    ble_mesh_nvs_open(&NVS_HANDLE);

    ble_mesh_init();

    // for setting Node name
    char name[8] = {};
    snprintf(name, sizeof(name), "%02x",dev_uuid[7]);
    esp_ble_mesh_set_unprovisioned_device_name(name);
}
