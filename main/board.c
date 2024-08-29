#include "driver/gpio.h"
#include "esp_log.h"

#include "board.h"
#include "iot_button.h"

#define TAG "BOARD"

#define BUTTON_IO_NUM           0
#define BUTTON_ACTIVE_LEVEL     0

extern void ble_mesh_send_gen_onoff_set(void);

led_state board_led_state[3] = { 
    { LED_R, true, false, "red"},
    { LED_G, false, false, "green"},
    { LED_B, false, false, "blue"}
};

void board_led_init(){
    for(int i=0 ; i<3 ; i++){
        gpio_reset_pin(board_led_state[i].gpio_num);
        gpio_set_direction(board_led_state[i].gpio_num, GPIO_MODE_OUTPUT);
    }
}

void board_led_operation( gpio_num_t gpio_num, bool on_off){
    for (int i = 0 ; i < 3 ; i++){
        if(board_led_state[i].gpio_num != gpio_num){
            continue;
        } 
        if(on_off == board_led_state[i].previous){
            ESP_LOGW(TAG, "led %s is already %s", board_led_state[i].name, (on_off ? "on": "off"));
            return;
        }
        gpio_set_level(gpio_num, on_off);
        ESP_LOGI(TAG, "set value complete");
        board_led_state[i].previous = on_off;
        return;
    }

    ESP_LOGE(TAG, "LED is not found");
}

static void button_tap_cb(void* arg)
{
    ESP_LOGI(TAG, "tap cb (%s)", (char *)arg);

    ble_mesh_send_gen_onoff_set();
}

static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
    }
}

void board_init(){
    board_button_init();
    board_led_init();
}
