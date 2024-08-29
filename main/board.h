#ifndef _BOARD_H_
#define _BOARD_H_


#include <driver/gpio.h>

void board_init();
void board_led_operation( gpio_num_t gpio_num, bool on_off);

#define LED_R GPIO_NUM_25
#define LED_G GPIO_NUM_26
#define LED_B GPIO_NUM_27

typedef struct {
    gpio_num_t gpio_num;
    bool current;
    bool previous;
    char* name;
}led_state;

#endif
